#include <stdio.h>
#include <string.h>
#include "esp_log.h"
#include "nvs_flash.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "ble_manager.h"
#include "xbox_controller.h"

static const char *TAG = "APP_MAIN";

// 首次配对时匹配的蓝牙广播名称
#define XBOX_TARGET_DEVICE_NAME "Xbox Wireless Controller"

// 上层业务与蓝牙胶水层全局状态结构体
typedef struct
{
    /* 物理链路连接状态 */
    uint16_t connection_handle; // 当前活跃的连接句柄 (无效时为 0xFFFF)
    bool is_connecting;         // 防止并发重复连接的防抖标志位

    /* 动态发现的 GATT 属性句柄 (实测物理地址) */
    uint16_t hid_service_start_handle;   // 句柄 22: HID 服务起始
    uint16_t hid_service_end_handle;     // 句柄 35: HID 服务结束
    uint16_t hid_control_point_handle;   // 句柄 26: 唤醒控制点
    uint16_t input_report_value_handle;  // 句柄 30: 16字节按键数据输入通道
    uint16_t output_rumble_value_handle; // 句柄 34: 8字节马达震动输出通道
    uint16_t input_cccd_handle;          // 句柄 31: 按键推送订阅开关

    /* 自动回连记忆缓存 */
    bool has_bonded_target;                                // 是否有已配对过的历史手柄
    uint8_t bonded_target_address[6];                      // 历史手柄的 6 字节 MAC 地址
    ble_manager_address_type_t bonded_target_address_type; // 历史手柄的地址类型 (Public 或 Random)

} main_application_context_t;

// 主程序全局静态上下文单例
static main_application_context_t s_application_context;

/* ========================================================================= */
/*  手柄驱动依赖注入 (实现手柄驱动要求的物理发包接口)                           */
/* ========================================================================= */

/**
 * @brief 手柄驱动向底层发包的回调接口
 * @note 负责通过蓝牙把 8 字节震动包打出去。
 */
static int xbox_transport_ble_send(const uint8_t *data, uint16_t length, void *user_context)
{
    if (s_application_context.connection_handle == BLE_MANAGER_INVALID_CONNECTION_HANDLE ||
        s_application_context.output_rumble_value_handle == 0)
    {
        return -1;
    }

    // 将 8 字节控制包通过无应答写入通道 (句柄 34) 发送
    esp_err_t err = ble_manager_central_write(s_application_context.connection_handle,
                                              s_application_context.output_rumble_value_handle,
                                              BLE_MANAGER_WRITE_TYPE_WITHOUT_RESPONSE,
                                              data,
                                              length);
    return (err == ESP_OK) ? 0 : -1;
}

/* ========================================================================= */
/*  Xbox 手柄业务事件中枢                                                     */
/* ========================================================================= */

/**
 * @brief 手柄事件回调函数
 */
static void on_xbox_controller_event(xbox_controller_event_t event,
                                     const xbox_controller_state_t *current_state,
                                     void *user_context)
{
    switch (event)
    {
    case XBOX_CONTROLLER_EVENT_CONNECTED:
        // 连接成功后触发手柄震动
        ESP_LOGW(TAG, "手柄连接成功！");
        xbox_controller_pulse_custom_vibration(0, 0, 40, 40, 50, 0, 0);
#if CONFIG_APP_ENABLE_XBOX_DEVICE_INFORMATION
        ble_manager_central_read(s_application_context.connection_handle, XBOX_HANDLE_SERIAL_NUMBER_VALUE);
#endif
        break;

    case XBOX_CONTROLLER_EVENT_DISCONNECTED:
        ESP_LOGW(TAG, "⚠️ 手柄断开连接！");
        break;

    // 主要业务实现发生事件
    case XBOX_CONTROLLER_EVENT_STATE_CHANGED:
    {
        if (current_state == NULL)
            break;

        /* 1. 静态记忆变量：消抖防刷屏 */
        static int16_t s_last_lx = 0, s_last_ly = 0, s_last_rx = 0, s_last_ry = 0;
        static uint16_t s_last_lt = 0, s_last_rt = 0;
        static xbox_controller_buttons_t s_last_buttons = {0};
        static xbox_controller_dpad_direction_t s_last_dpad = XBOX_DPAD_DIRECTION_CENTERED;
        static uint8_t s_last_sent_lt_intensity = 0, s_last_sent_rt_intensity = 0;

        /* ==================================================================== */
        /*  特殊功能：按下西瓜键 (Xbox 键) 呼出手柄状态仪表盘                     */
        /* ==================================================================== */
        if (current_state->buttons.button_xbox && !s_last_buttons.button_xbox)
        {
            ESP_LOGI("DASHBOARD", "============================================================");
            ESP_LOGI("DASHBOARD", "🎮 >>> XBOX 手柄状态仪表盘 (GUIDE MENU) <<<");

#if CONFIG_APP_ENABLE_XBOX_BATTERY_MONITORING
            uint8_t battery = xbox_controller_get_battery_percentage();
            if (battery <= 100)
                ESP_LOGI("DASHBOARD", "🔋 剩余电池电量: %u%%", battery);
            else
                ESP_LOGW("DASHBOARD", "🔋 剩余电池电量: [未获取/读取中]");
#endif

#if CONFIG_APP_ENABLE_XBOX_DEVICE_INFORMATION
            const char *sn = xbox_controller_get_serial_number();
            if (sn[0] != '\0' && strcmp(sn, "UNKNOWN") != 0)
                ESP_LOGI("DASHBOARD", "📋 出厂硬件 SN:  %s", sn);
            else
                ESP_LOGW("DASHBOARD", "📋 出厂硬件 SN:  [未获取 / 读取中]");

            const char *fw = xbox_controller_get_firmware_version();
            if (fw[0] != '\0' && strcmp(fw, "UNKNOWN") != 0)
                ESP_LOGI("DASHBOARD", "⚙️ 当前固件版本: %s", fw);
            else
                ESP_LOGW("DASHBOARD", "⚙️ 当前固件版本: [未获取 / 读取中]");
#endif
            ESP_LOGI("DASHBOARD", "============================================================");

            // 按下西瓜键时，机身轻震 100ms (10个单位) 提示呼出成功！
            xbox_controller_pulse_custom_vibration(0, 0, 40, 40, 50, 0, 0);
        }

        /* ==================================================================== */
        /*  扳机动态力反馈                                                       */
        /* ==================================================================== */
        uint8_t cur_lt = (uint8_t)(((uint32_t)current_state->trigger_left * 100) / 1023);
        uint8_t cur_rt = (uint8_t)(((uint32_t)current_state->trigger_right * 100) / 1023);

        int d_lt = (int)cur_lt - (int)s_last_sent_lt_intensity;
        int d_rt = (int)cur_rt - (int)s_last_sent_rt_intensity;

        // 强度变化超过 5% 才触发，防止蓝牙被塞爆
        if (d_lt >= 5 || d_lt <= -5 || d_rt >= 5 || d_rt <= -5 ||
            (cur_lt == 0 && s_last_sent_lt_intensity != 0) ||
            (cur_rt == 0 && s_last_sent_rt_intensity != 0))
        {
            s_last_sent_lt_intensity = cur_lt;
            s_last_sent_rt_intensity = cur_rt;

            if (cur_lt > 0 || cur_rt > 0)
            {
                // 随着手指扣动的深浅，扳机震动强度越强
                xbox_controller_set_trigger_motors(cur_lt, cur_rt, 45, 0, 0);
            }
            else
            {
                xbox_controller_stop_vibration(); // 完全松手时立即停止
            }
        }

        /* ==================================================================== */
        /*  动作过滤与控制台打印                                                 */
        /* ==================================================================== */
        int dlx = (int)current_state->joystick_left_x - (int)s_last_lx;
        int dly = (int)current_state->joystick_left_y - (int)s_last_ly;
        int drx = (int)current_state->joystick_right_x - (int)s_last_rx;
        int dry = (int)current_state->joystick_right_y - (int)s_last_ry;
        int dlt = (int)current_state->trigger_left - (int)s_last_lt;
        int drt = (int)current_state->trigger_right - (int)s_last_rt;

        bool stick_moved = (dlx > 1500 || dlx < -1500 || dly > 1500 || dly < -1500 || drx > 1500 || drx < -1500 || dry > 1500 || dry < -1500);
        bool trigger_moved = (dlt > 50 || dlt < -50 || drt > 50 || drt < -50);
        bool button_changed = (memcmp(&current_state->buttons, &s_last_buttons, sizeof(xbox_controller_buttons_t)) != 0);
        bool dpad_changed = (current_state->dpad_direction != s_last_dpad);

        /* 只有发生真正动作时才打印 */
        if (stick_moved || trigger_moved || button_changed || dpad_changed)
        {
            s_last_lx = current_state->joystick_left_x;
            s_last_ly = current_state->joystick_left_y;
            s_last_rx = current_state->joystick_right_x;
            s_last_ry = current_state->joystick_right_y;
            s_last_lt = current_state->trigger_left;
            s_last_rt = current_state->trigger_right;
            s_last_buttons = current_state->buttons;
            s_last_dpad = current_state->dpad_direction;

            char active_info[96] = {0};
            int offset = 0;

            if (current_state->buttons.button_a)
                offset += snprintf(active_info + offset, sizeof(active_info) - offset, "[A] ");
            if (current_state->buttons.button_b)
                offset += snprintf(active_info + offset, sizeof(active_info) - offset, "[B] ");
            if (current_state->buttons.button_x)
                offset += snprintf(active_info + offset, sizeof(active_info) - offset, "[X] ");
            if (current_state->buttons.button_y)
                offset += snprintf(active_info + offset, sizeof(active_info) - offset, "[Y] ");
            if (current_state->buttons.bumper_left)
                offset += snprintf(active_info + offset, sizeof(active_info) - offset, "[LB] ");
            if (current_state->buttons.bumper_right)
                offset += snprintf(active_info + offset, sizeof(active_info) - offset, "[RB] ");
            if (current_state->buttons.thumbstick_click_left)
                offset += snprintf(active_info + offset, sizeof(active_info) - offset, "[L3] ");
            if (current_state->buttons.thumbstick_click_right)
                offset += snprintf(active_info + offset, sizeof(active_info) - offset, "[R3] ");
            if (current_state->buttons.button_view)
                offset += snprintf(active_info + offset, sizeof(active_info) - offset, "[View] ");
            if (current_state->buttons.button_menu)
                offset += snprintf(active_info + offset, sizeof(active_info) - offset, "[Menu] ");
            if (current_state->buttons.button_xbox)
                offset += snprintf(active_info + offset, sizeof(active_info) - offset, "[Xbox] ");
            if (current_state->buttons.button_share)
                offset += snprintf(active_info + offset, sizeof(active_info) - offset, "[Share] ");

            if (current_state->dpad_direction == XBOX_DPAD_DIRECTION_UP)
                offset += snprintf(active_info + offset, sizeof(active_info) - offset, "[Up] ");
            else if (current_state->dpad_direction == XBOX_DPAD_DIRECTION_DOWN)
                offset += snprintf(active_info + offset, sizeof(active_info) - offset, "[Down] ");
            else if (current_state->dpad_direction == XBOX_DPAD_DIRECTION_LEFT)
                offset += snprintf(active_info + offset, sizeof(active_info) - offset, "[Left] ");
            else if (current_state->dpad_direction == XBOX_DPAD_DIRECTION_RIGHT)
                offset += snprintf(active_info + offset, sizeof(active_info) - offset, "[Right] ");

            if (current_state->trigger_left > 50)
                offset += snprintf(active_info + offset, sizeof(active_info) - offset, "[LT:%u%%] ", cur_lt);
            if (current_state->trigger_right > 50)
                offset += snprintf(active_info + offset, sizeof(active_info) - offset, "[RT:%u%%] ", cur_rt);

            if (offset > 0)
            {
                ESP_LOGI("XBOX", "L:(%6d, %6d)  R:(%6d, %6d) | %s",
                         current_state->joystick_left_x, current_state->joystick_left_y,
                         current_state->joystick_right_x, current_state->joystick_right_y,
                         active_info);
            }
            else
            {
                ESP_LOGI("XBOX", "L:(%6d, %6d)  R:(%6d, %6d)",
                         current_state->joystick_left_x, current_state->joystick_left_y,
                         current_state->joystick_right_x, current_state->joystick_right_y);
            }
        }
        break;
    }

#if CONFIG_APP_ENABLE_XBOX_BATTERY_MONITORING
    case XBOX_CONTROLLER_EVENT_BATTERY_LEVEL_UPDATED:
    {
        // 电池电量更新事件
        uint8_t battery = xbox_controller_get_battery_percentage();
        ESP_LOGI(TAG, "🔋 [手柄电量] 当前剩余电量: %u%%", battery);

        // 低于 20% 时震动一下提示充电
        if (battery <= 20)
        {
            ESP_LOGW(TAG, "⚠️手柄电量不足 20%，请及时充电！");
            xbox_controller_pulse_custom_vibration(0, 0, 80, 80, 50, 0, 0); // 双扳机震动提醒
        }
        break;
    }

#endif

#if CONFIG_APP_ENABLE_XBOX_DEVICE_INFORMATION
    case XBOX_CONTROLLER_EVENT_SERIAL_NUMBER_FETCH:
    {
        // 触发获取出厂 SN 序列号
        if (s_application_context.connection_handle != BLE_MANAGER_INVALID_CONNECTION_HANDLE)
        {
            ESP_LOGI(TAG, "📡 [触发获取] 正在向手柄发起出厂 SN 码读取请求...");
            esp_err_t err = ble_manager_central_read(s_application_context.connection_handle,
                                                     XBOX_HANDLE_SERIAL_NUMBER_VALUE);
            if (err != ESP_OK)
            {
                ESP_LOGE(TAG, "发起出厂 SN 码读取失败, 错误码: 0x%x", err);
            }
        }
        else
        {
            ESP_LOGW(TAG, "手柄未连接，无法发起出厂 SN 码获取！");
        }
        break;
    }

    case XBOX_CONTROLLER_EVENT_SERIAL_NUMBER_QUERY:
    {

        const char *sn = xbox_controller_get_serial_number();
        if (sn[0] != '\0' && strcmp(sn, "UNKNOWN") != 0)
        {
            // 读取本地缓存的手柄出厂序列号
            ESP_LOGI(TAG, "📋 [本地查询查看] 手柄出厂序列号 (SN): %s", sn);
        }
        else
        {
            ESP_LOGW(TAG, "📋 [本地查询查看] 出厂 SN 码尚未就绪！");
        }

        break;
    }

    case XBOX_CONTROLLER_EVENT_FIRMWARE_VERSION_FETCH:
    {
        // 触发获取固件版本号
        if (s_application_context.connection_handle != BLE_MANAGER_INVALID_CONNECTION_HANDLE)
        {
            ESP_LOGI(TAG, "📡 [触发获取] 正在向手柄硬件发起固件版本号读取请求...");
            esp_err_t err = ble_manager_central_read(s_application_context.connection_handle,
                                                     XBOX_HANDLE_FIRMWARE_REVISION_VALUE);
            if (err != ESP_OK)
            {
                ESP_LOGE(TAG, "发起固件版本号读取失败, 错误码: 0x%x", err);
            }
        }
        else
        {
            ESP_LOGW(TAG, "手柄未连接，无法发起固件版本号获取!");
        }
        break;
    }

    case XBOX_CONTROLLER_EVENT_FIRMWARE_VERSION_QUERY:
    {
        const char *fw = xbox_controller_get_firmware_version();
        if (fw[0] != '\0' && strcmp(fw, "UNKNOWN") != 0)
        {
            // 读取本地缓存的固件版本号
            ESP_LOGI(TAG, "⚙️ [本地查询查看] 手柄固件版本号: %s", fw);
        }
        else
        {
            ESP_LOGW(TAG, "⚙️ [本地查询查看] 固件版本号尚未就绪!");
        }
        break;
    }
#endif

    default:
        break;
    }
}

/* ========================================================================= */
/*  底层蓝牙协议栈状态机回调                                                   */
/* ========================================================================= */

static void on_ble_manager_event(ble_manager_event_t event,
                                 uint16_t connection_handle,
                                 void *event_data,
                                 void *user_context)
{
    switch (event)
    {
    case BLE_MANAGER_EVENT_READY:
    {
        ESP_LOGI(TAG, "BLE 协议栈就绪！检查历史绑定存储...");

        // 自动回连检测：查询 Flash 中是否有旧手柄记忆
        ble_manager_bonded_device_t bonded_devices[1];
        if (ble_manager_get_bonded_devices(bonded_devices, 1) > 0)
        {
            s_application_context.has_bonded_target = true;
            memcpy(s_application_context.bonded_target_address, bonded_devices[0].address, 6);
            s_application_context.bonded_target_address_type = bonded_devices[0].address_type;

            ESP_LOGW(TAG, ">>> 发现已绑定 Xbox 手柄 [%02X:%02X:%02X:%02X:%02X:%02X],进入定向回连模式！<<<",
                     bonded_devices[0].address[0], bonded_devices[0].address[1], bonded_devices[0].address[2],
                     bonded_devices[0].address[3], bonded_devices[0].address[4], bonded_devices[0].address[5]);
        }
        else
        {
            ESP_LOGI(TAG, "未发现历史手柄，进入全网广播扫描配对模式...");
        }

        ble_manager_central_start_scanning();
        break;
    }

    case BLE_MANAGER_EVENT_DEVICE_DISCOVERED:
    {
        ble_manager_discovered_device_t *device = (ble_manager_discovered_device_t *)event_data;
        if (s_application_context.is_connecting)
            break;

        bool is_match = false;

        // 分支 A: 如果有绑定记忆，专等 MAC 地址匹配 (开机秒连)
        if (s_application_context.has_bonded_target)
        {
            if (memcmp(device->address, s_application_context.bonded_target_address, 6) == 0)
            {
                ESP_LOGI(TAG, "已绑定手柄上线！立刻发起定向连接...");
                is_match = true;
            }
        }
        // 分支 B: 首次配对，匹配设备名称
        else if (device->device_name[0] != '\0' && strcmp(device->device_name, XBOX_TARGET_DEVICE_NAME) == 0)
        {
            ESP_LOGI(TAG, "发现处于配对模式的 Xbox 手柄，立刻连接...");
            is_match = true;
        }

        if (is_match)
        {
            s_application_context.is_connecting = true;
            ble_manager_central_stop_scanning();
            // vTaskDelay(pdMS_TO_TICKS(50)); // 留给底层停止扫描的处理时间

            esp_err_t connect_err = ble_manager_central_connect(device->address, device->address_type);
            if (connect_err != ESP_OK)
            {
                ESP_LOGE(TAG, "发起连接失败: 0x%x，恢复扫描状态", connect_err);
                s_application_context.is_connecting = false;
                ble_manager_central_start_scanning();
            }
        }
        break;
    }

    case BLE_MANAGER_EVENT_CONNECTED:
        s_application_context.connection_handle = connection_handle;
        s_application_context.is_connecting = false;

        // 连上后第一步：寻找主服务
        ble_manager_central_discover_services(connection_handle);
        break;

    case BLE_MANAGER_EVENT_SERVICE_DISCOVERED:
    {
        ble_manager_discovered_service_t *service = (ble_manager_discovered_service_t *)event_data;
        if (service->uuid.type == BLE_MANAGER_UUID_TYPE_16 &&
            service->uuid.value.uuid16 == BLE_SERVICE_HID_UUID16)
        {
            // 抓取 HID 服务区间 (实测 handles [22 - 35])
            s_application_context.hid_service_start_handle = service->start_handle;
            s_application_context.hid_service_end_handle = service->end_handle;
        }
        break;
    }

    case BLE_MANAGER_EVENT_SERVICE_DISCOVERY_COMPLETED:
        if (s_application_context.hid_service_end_handle > 0)
        {
            // 第二步：直接去检索 HID 服务区间内的所有特征值
            ble_manager_central_discover_characteristics(connection_handle, s_application_context.hid_service_start_handle, s_application_context.hid_service_end_handle);
        }
        break;

    case BLE_MANAGER_EVENT_CHARACTERISTIC_DISCOVERED:
    {
        ble_manager_discovered_characteristic_t *characteristic = (ble_manager_discovered_characteristic_t *)event_data;
        if (characteristic->uuid.type == BLE_MANAGER_UUID_TYPE_16)
        {
            // 抓取控制点 (句柄 26，UUID 0x2A4C)
            if (characteristic->uuid.value.uuid16 == BLE_CHAR_HID_CONTROL_POINT_UUID16)
            {
                s_application_context.hid_control_point_handle = characteristic->value_handle;
            }
            // 抓取按键输入通道 (句柄 30，UUID 0x2A4D，具备 Notify 0x10 权限)
            else if (characteristic->uuid.value.uuid16 == BLE_CHAR_HID_REPORT_UUID16 &&
                     (characteristic->properties & 0x10) &&
                     s_application_context.input_report_value_handle == 0)
            {
                s_application_context.input_report_value_handle = characteristic->value_handle;
            }
            // 抓取马达震动输出通道 (句柄 34，UUID 0x2A4D，具备 Write 0x04 权限)
            else if (characteristic->uuid.value.uuid16 == BLE_CHAR_HID_REPORT_UUID16 &&
                     (characteristic->properties & 0x04))
            {
                s_application_context.output_rumble_value_handle = characteristic->value_handle;
            }
        }
        break;
    }

    case BLE_MANAGER_EVENT_CHARACTERISTIC_DISCOVERY_COMPLETED:
        // 第三步：特征值全找到时，立刻触发底层链路加密配对 (Xbox不加密无法控制)
        ble_manager_central_start_security(connection_handle);
        break;

    case BLE_MANAGER_EVENT_ENCRYPTION_COMPLETED:
    {
        ble_manager_async_result_t *result = (ble_manager_async_result_t *)event_data;

        if (result->status == 0)
        {
            ble_manager_bonded_device_t bonded_devices[1];
            if (ble_manager_get_bonded_devices(bonded_devices, 1) > 0)
            {
                s_application_context.has_bonded_target = true;
                memcpy(s_application_context.bonded_target_address, bonded_devices[0].address, 6);
                s_application_context.bonded_target_address_type = bonded_devices[0].address_type;
            }

            // 第四步：配对成功，寻找输入通道 (句柄 30) 下属的 CCCD (0x2902) 描述符
            ble_manager_central_discover_descriptors(connection_handle,
                                                     s_application_context.input_report_value_handle,
                                                     s_application_context.hid_service_end_handle);
        }
        else
        {
            // 密钥已失效，针对失效设备解绑
            ESP_LOGE(TAG, "❌ [加密失败: %d] 密钥已失效，正在针对目标 [%02X:%02X:%02X:%02X:%02X:%02X] 解绑...",
                     result->status,
                     s_application_context.bonded_target_address[0], s_application_context.bonded_target_address[1],
                     s_application_context.bonded_target_address[2], s_application_context.bonded_target_address[3],
                     s_application_context.bonded_target_address[4], s_application_context.bonded_target_address[5]);

            // 从 Flash 中剔除这只坏手柄的密钥，其他手柄不受任何影响（针对多连）
            ble_manager_unpair_device(s_application_context.bonded_target_address,
                                      s_application_context.bonded_target_address_type);

            // 清除应用层对这只手柄的回连记忆，让系统重新退回扫描配对模式
            s_application_context.has_bonded_target = false;
            memset(s_application_context.bonded_target_address, 0, 6);

            // 彻底断开本次异常连接，触发 DISCONNECTED 重新开始全网扫描
            ble_manager_disconnect(connection_handle);
        }
        break;
    }

    case BLE_MANAGER_EVENT_DESCRIPTOR_DISCOVERED:
    {
        ble_manager_discovered_descriptor_t *descriptor = (ble_manager_discovered_descriptor_t *)event_data;
        if (descriptor->uuid.type == BLE_MANAGER_UUID_TYPE_16 &&
            descriptor->uuid.value.uuid16 == BLE_DESCRIPTOR_CCCD_UUID16)
        {
            // 抓取到 CCCD (句柄 31)
            s_application_context.input_cccd_handle = descriptor->descriptor_handle;
        }
        break;
    }

    case BLE_MANAGER_EVENT_DESCRIPTOR_DISCOVERY_COMPLETED:
        // 第五步：写入 CCCD (0x0001)，开启按键数据实时推送！
        ble_manager_central_subscribe(connection_handle, s_application_context.input_cccd_handle, true);
        break;

    case BLE_MANAGER_EVENT_SUBSCRIBE_COMPLETED:
    {
        ble_manager_async_result_t *result = (ble_manager_async_result_t *)event_data;
        if (result->status == 0)
        {
            if (result->attribute_handle == s_application_context.input_cccd_handle)
            {
                /*
                 * 向 HID Control Point (句柄 26) 写入 0x01 (Exit Suspend 退出休眠)，
                 * 手柄硬件正式唤醒，按键引擎全面激活！
                 */
                uint8_t exit_suspend_command = XBOX_COMMAND_EXIT_SUSPEND;
                ble_manager_central_write(connection_handle,
                                          s_application_context.hid_control_point_handle,
                                          BLE_MANAGER_WRITE_TYPE_WITHOUT_RESPONSE,
                                          &exit_suspend_command,
                                          1);

                // 通知手柄驱动：物理连接已就绪
                xbox_controller_notify_connected();

#if CONFIG_APP_ENABLE_XBOX_BATTERY_MONITORING
                ble_manager_central_subscribe(connection_handle, XBOX_HANDLE_BATTERY_LEVEL_CCCD, true);
#endif
            }
#if CONFIG_APP_ENABLE_XBOX_BATTERY_MONITORING
            else if (result->attribute_handle == XBOX_HANDLE_BATTERY_LEVEL_CCCD)
            {
                ESP_LOGI(TAG, "✅ [电池通道] CCCD 订阅成功！等待电量自动推送...");
            }
#endif
        }
        else
        {
            ESP_LOGE(TAG, "❌ 订阅失败，句柄: %u, 错误码: %d", result->attribute_handle, result->status);
        }
        break;
    }

    case BLE_MANAGER_EVENT_NOTIFICATION_RECEIVED:
    {
        ble_manager_received_data_t *received = (ble_manager_received_data_t *)event_data;

        /*
         * 解耦桥梁
         * 蓝牙层只管把收到的 16 字节原始数据就写进手柄驱动，
         * 手柄驱动内部自动完成归一化解析并触发上层业务
         */
        if (received->attribute_handle == s_application_context.input_report_value_handle)
        {
            xbox_controller_feed_input_report(received->data, received->length);
        }

#if CONFIG_APP_ENABLE_XBOX_BATTERY_MONITORING
        else if (received->attribute_handle == XBOX_HANDLE_BATTERY_LEVEL_VALUE)
        {
            if (received->length > 0)
            {
                uint8_t new_battery = received->data[0];
                if (new_battery != xbox_controller_get_battery_percentage())
                {
                    // 保存新电池电量
                    xbox_controller_feed_battery_percentage(new_battery);
                    ESP_LOGI(TAG, "🔋 [手柄电量更新成功] 当前剩余电量: %u%%", new_battery);
                }
            }
        }
#endif

        break;
    }

    case BLE_MANAGER_EVENT_DISCONNECTED:
    {
        s_application_context.connection_handle = BLE_MANAGER_INVALID_CONNECTION_HANDLE;
        s_application_context.is_connecting = false;
        s_application_context.hid_service_start_handle = 0;
        s_application_context.hid_service_end_handle = 0;
        s_application_context.hid_control_point_handle = 0;
        s_application_context.input_report_value_handle = 0;
        s_application_context.output_rumble_value_handle = 0;
        s_application_context.input_cccd_handle = 0;

        // 通知手柄驱动已经掉线
        xbox_controller_notify_disconnected();

        vTaskDelay(pdMS_TO_TICKS(1000));

        // 重新开启扫描等待重连
        ble_manager_central_start_scanning();

        break;
    }

    case BLE_MANAGER_EVENT_READ_COMPLETED:
    {
        ble_manager_read_result_t *read_result = (ble_manager_read_result_t *)event_data;
        if (read_result->status == 0 && read_result->data != NULL && read_result->length > 0)
        {
#if CONFIG_APP_ENABLE_XBOX_DEVICE_INFORMATION
            // 句柄 17: 出厂 SN 码读取完成
            if (read_result->attribute_handle == XBOX_HANDLE_SERIAL_NUMBER_VALUE)
            {
                xbox_controller_feed_serial_number_raw(read_result->data, read_result->length);

                ble_manager_central_read(s_application_context.connection_handle, XBOX_HANDLE_FIRMWARE_REVISION_VALUE);
            }
            // 句柄 15: 固件版本号读取完成
            else if (read_result->attribute_handle == XBOX_HANDLE_FIRMWARE_REVISION_VALUE)
            {
                xbox_controller_feed_firmware_version_raw(read_result->data, read_result->length);
            }
#endif

#if CONFIG_APP_ENABLE_XBOX_BATTERY_MONITORING
            // 句柄 20: 电池电量读取完成
            if (read_result->attribute_handle == XBOX_HANDLE_BATTERY_LEVEL_VALUE)
            {
                xbox_controller_feed_battery_percentage(read_result->data[0]);
                // // 触发电池电量更新事件
                // on_xbox_controller_event(XBOX_CONTROLLER_EVENT_BATTERY_LEVEL_UPDATED, NULL, NULL);
            }
#endif
        }
        else
        {
            ESP_LOGE(TAG, "❌ 读取失败! 句柄: %u, 状态码: %d", read_result->attribute_handle, read_result->status);
        }
        break;
    }

    case BLE_MANAGER_EVENT_ERROR:
    {
        /* 1. 安全解包错误码 */
        int err = (event_data != NULL) ? *(int *)event_data : -1;
        ESP_LOGE(TAG, "BLE Manager Error occurred! Code: %d (0x%04x)", err, err);

        /*
         * 2. 分流判断：这是连接超时/连接失败，还是底层协议栈崩溃
         */

        // 情况 A：属于“超时/连接失败”类错误
        // (13 是 BLE_HS_ETIMEOUT 超时，0x023e / 0x3e 是连接建立失败)
        if (err == 13 || err == 0x023e || err == 0x3e)
        {
            ESP_LOGW(TAG, "手柄未开机、休眠或连接失败，稍后自动重启扫描...");

            // 延时 1000ms 缓冲
            vTaskDelay(pdMS_TO_TICKS(1000));

            // 重新开启扫描
            esp_err_t scan_err = ble_manager_central_start_scanning();
            if (scan_err != ESP_OK)
            {
                ESP_LOGE(TAG, "Restart scanning failed: %s", esp_err_to_name(scan_err));
            }
        }
        // 情况 B：属于“底层控制器异常复位”等严重系统错误
        else
        {
            ESP_LOGE(TAG, "蓝牙底层发生异常复位 (Reset)，请勿手动开扫描，静等 READY 事件自愈...");

            /*
             * 此时不要手动调 start_scanning()
             * 因为底层复位后，NimBLE 重新同步完毕时，会自动抛出 BLE_MANAGER_EVENT_READY 事件！
             * 你只需要在 READY 事件里去启动扫描，系统就能全自动自愈！
             */
        }
        break;
    }

    default:
        break;
    }
}

void app_main(void)
{

    // 将 NimBLE 原生协议栈的日志降到 WARN
    esp_log_level_set("NimBLE", ESP_LOG_WARN);

    // 将底层 BLE_MANAGER 的日志级别降到 WARN (只报严重警告和错误，屏蔽所有日常刷屏)
    esp_log_level_set("BLE_MANAGER", ESP_LOG_WARN);

    ESP_LOGI(TAG, "================ XBOX MODULAR CONTROLLER SYSTEM START ================");

    /* 1. 初始化 NVS 闪存 */
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND)
    {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    /* 2. 启动手柄核心协议引擎 (依赖注入发送函数) */
    xbox_controller_config_t xbox_config = {
        .send_function = xbox_transport_ble_send,   // 注入底层发包函数
        .event_callback = on_xbox_controller_event, // 绑定纯业务控制回调
        .user_context = NULL};
    xbox_controller_init(&xbox_config);

    /* 3. 启动底层低功耗蓝牙通信管理器 */
    ble_manager_config_t ble_config = {
        .role = BLE_MANAGER_ROLE_CENTRAL,
        .device_name = "ESP32_Xbox_Master",
        .gatt_services = NULL,
        .event_callback = on_ble_manager_event,
        .user_context = NULL};
    ESP_ERROR_CHECK(ble_manager_init(&ble_config));
}
