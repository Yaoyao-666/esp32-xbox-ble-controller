#include "xbox_controller.h"
#include <string.h>

/* ========================================================================= */
/* 宏定义与内部默认参数                                                       */
/* ========================================================================= */

// 默认摇杆抗漂移死区阈值 (范围 0~32767，默认取 1800，约占满量程的 5.5%)
#define XBOX_CONTROLLER_DEFAULT_JOYSTICK_DEADZONE 1800

/* ========================================================================= */
/* 驱动内部静态单例上下文结构体                                                */
/* ========================================================================= */

typedef struct
{
    xbox_controller_state_t state;                       // 手柄当前最新姿态数据快照
    xbox_controller_send_bytes_function_t send_function; // 依赖注入的底层发包函数指针
    xbox_controller_event_callback_t event_callback;     // 上层用户事件监听回调函数指针
    void *user_context;                                  // 用户私有上下文透传指针
    uint16_t joystick_deadzone_threshold;                // 摇杆死区阈值
    bool is_initialized;                                 // 驱动初始化标记
} xbox_controller_context_t;

// 驱动全局唯一静态上下文实例
static xbox_controller_context_t s_xbox_controller_context;

// 设备出厂信息与电量存储
#if CONFIG_APP_ENABLE_XBOX_BATTERY_MONITORING
static uint8_t s_xbox_battery_percentage = 255; // 剩余电量百分比 (0~100，默认 255 代表离线或未获取)
#endif

#if CONFIG_APP_ENABLE_XBOX_DEVICE_INFORMATION
static char s_xbox_serial_number[32] = {0};    // 出厂 SN 序列号字符串
static char s_xbox_firmware_version[32] = {0}; // 固件版本号字符串
#endif

/* ========================================================================= */
/* 内部数学辅助函数                                                           */
/* ========================================================================= */

/**
 * @brief 摇杆抗漂移死区滤波算法
 * @note  若摇杆位移在 [-threshold, +threshold] 范围内，强制置 0
 */
static inline int16_t xbox_controller_filter_joystick_deadzone(int32_t normalized_axis_value,
                                                               uint16_t deadzone_threshold)
{
    if (normalized_axis_value > -((int32_t)deadzone_threshold) &&
        normalized_axis_value < ((int32_t)deadzone_threshold))
    {
        return 0;
    }

    // 数值饱和截断保护 (-32768 ~ 32767)
    if (normalized_axis_value > 32767)
    {
        return 32767;
    }
    if (normalized_axis_value < -32768)
    {
        return -32768;
    }

    return (int16_t)normalized_axis_value;
}

/**
 * @brief 内部组装并发送 8 字节微软震动控制报文
 *
 * @param actuator_mask             马达使能掩码 (Bit0:右扳机, Bit1:左扳机, Bit2:右轻马达, Bit3:左重马达)
 * @param left_trigger_percent      左扳机马达强度 (0 ~ 100)
 * @param right_trigger_percent     右扳机马达强度 (0 ~ 100)
 * @param left_main_percent         左握把主大马达强度 (0 ~ 100)
 * @param right_main_percent        右握把副小马达强度 (0 ~ 100)
 * @param duration_in_10ms_units    震动时长 (单位: 10毫秒，0~255)
 * @param delay_in_10ms_units       震动启动延时时间 (单位: 10毫秒，0 ~ 255，0 代表立即起震不延时)
 * @param loop_count                脉冲序列循环次数 (0 为单次，1~254 为重复N次，255 为无限循环(可能会被手柄忽略不详))
 * @return xbox_controller_status_t 状态码
 */
static xbox_controller_status_t xbox_controller_send_rumble_packet(uint8_t actuator_mask,
                                                                   uint8_t left_trigger_percent,
                                                                   uint8_t right_trigger_percent,
                                                                   uint8_t left_main_percent,
                                                                   uint8_t right_main_percent,
                                                                   uint8_t duration_in_10ms_units,
                                                                   uint8_t delay_in_10ms_units,
                                                                   uint8_t loop_count)
{
    /* 1. 状态检查：未连接或底层发包函数为空时立即退出 */
    if (!s_xbox_controller_context.is_initialized || !s_xbox_controller_context.state.is_connected)
    {
        return XBOX_CONTROLLER_STATUS_ERROR_NOT_CONNECTED;
    }

    if (s_xbox_controller_context.send_function == NULL)
    {
        return XBOX_CONTROLLER_STATUS_ERROR_IO;
    }

    /* 2. 强度饱和度限幅保护 (0 ~ 100) */
    uint8_t lt = (left_trigger_percent > 100) ? 100 : left_trigger_percent;
    uint8_t rt = (right_trigger_percent > 100) ? 100 : right_trigger_percent;
    uint8_t lm = (left_main_percent > 100) ? 100 : left_main_percent;
    uint8_t rm = (right_main_percent > 100) ? 100 : right_main_percent;

    /* 3. 组装经过实测验证的 8 字节微软 BLE 震动控制报文 */
    uint8_t rumble_packet[8];
    rumble_packet[0] = actuator_mask;          // 马达选择使能掩码
    rumble_packet[1] = lt;                     // 左扳机马达强度
    rumble_packet[2] = rt;                     // 右扳机马达强度
    rumble_packet[3] = lm;                     // 左主握把重马达强度
    rumble_packet[4] = rm;                     // 右主握把轻马达强度
    rumble_packet[5] = duration_in_10ms_units; // 震动持续时间
    rumble_packet[6] = delay_in_10ms_units;    // 触发延迟 (0 代表立即起震)
    rumble_packet[7] = loop_count;             // 循环次数

    /* 4. 调用依赖注入的底层发包函数 */
    int result = s_xbox_controller_context.send_function(rumble_packet,
                                                         sizeof(rumble_packet),
                                                         s_xbox_controller_context.user_context);
    if (result != 0)
    {
        return XBOX_CONTROLLER_STATUS_ERROR_IO;
    }

    return XBOX_CONTROLLER_STATUS_OK;
}

/* ========================================================================= */
/*  一、辅助管理与解耦数据 (Auxiliary)                                        */
/* ========================================================================= */

xbox_controller_status_t xbox_controller_init(const xbox_controller_config_t *config)
{
    /* 1. 入参检查：配置指针及依赖注入的发包函数必须有效 */
    if (config == NULL || config->send_function == NULL)
    {
        return XBOX_CONTROLLER_STATUS_ERROR_INVALID_ARG;
    }

    /* 2. 安全重置整个上下文结构体为零状态 */
    memset(&s_xbox_controller_context, 0, sizeof(xbox_controller_context_t));

    /* 3. 装配用户配置与函数指针 */
    s_xbox_controller_context.send_function = config->send_function;
    s_xbox_controller_context.event_callback = config->event_callback;
    s_xbox_controller_context.user_context = config->user_context;

    /* 4. 加载默认摇杆抗漂移死区并标记就绪 */
    s_xbox_controller_context.joystick_deadzone_threshold = XBOX_CONTROLLER_DEFAULT_JOYSTICK_DEADZONE;
    s_xbox_controller_context.is_initialized = true;

    return XBOX_CONTROLLER_STATUS_OK;
}

xbox_controller_status_t xbox_controller_deinit(void)
{
    /* 幂等清理：直接将所有状态、指针、数据清零，退回未初始化状态 */
    memset(&s_xbox_controller_context, 0, sizeof(xbox_controller_context_t));

    return XBOX_CONTROLLER_STATUS_OK;
}

void xbox_controller_feed_input_report(const uint8_t *raw_data, uint16_t length)
{
    /* 1. 防呆检查：未初始化或报文长度不足 16 字节直接丢弃 */
    if (!s_xbox_controller_context.is_initialized || raw_data == NULL || length < 16)
    {
        return;
    }

    /*
     * 2. 摇杆原始数据提取 (16位无符号小端序: 0 ~ 65535, 自然居中约 32768)
     */
    uint16_t raw_left_x = (uint16_t)(raw_data[0] | (raw_data[1] << 8));
    uint16_t raw_left_y = (uint16_t)(raw_data[2] | (raw_data[3] << 8));
    uint16_t raw_right_x = (uint16_t)(raw_data[4] | (raw_data[5] << 8));
    uint16_t raw_right_y = (uint16_t)(raw_data[6] | (raw_data[7] << 8));

    /*
     * 3. 数学归一化处理与抗漂移死区滤波：
     * 将 0 ~ 65535 减去 32768，转换为 -32768 ~ 32767，居中为 0！
     */
    int32_t normalized_left_x = (int32_t)raw_left_x - 32768;
    int32_t normalized_left_y = (int32_t)raw_left_y - 32768;
    int32_t normalized_right_x = (int32_t)raw_right_x - 32768;
    int32_t normalized_right_y = (int32_t)raw_right_y - 32768;

    uint16_t deadzone = s_xbox_controller_context.joystick_deadzone_threshold;

    s_xbox_controller_context.state.joystick_left_x = xbox_controller_filter_joystick_deadzone(normalized_left_x, deadzone);
    s_xbox_controller_context.state.joystick_left_y = xbox_controller_filter_joystick_deadzone(normalized_left_y, deadzone);
    s_xbox_controller_context.state.joystick_right_x = xbox_controller_filter_joystick_deadzone(normalized_right_x, deadzone);
    s_xbox_controller_context.state.joystick_right_y = xbox_controller_filter_joystick_deadzone(normalized_right_y, deadzone);

    /*
     * 4. 模拟量线性扳机键提取 (Byte 8~11: 10-bit 线性模拟量, 范围 0 ~ 1023)
     */
    s_xbox_controller_context.state.trigger_left = (uint16_t)(raw_data[8] | (raw_data[9] << 8));
    s_xbox_controller_context.state.trigger_right = (uint16_t)(raw_data[10] | (raw_data[11] << 8));

    /*
     * 5. 十字方向键 D-Pad 提取 (Byte 12: 8方向离散帽开关, 范围 0 ~ 8)
     */
    uint8_t raw_dpad = raw_data[12];
    if (raw_dpad <= 8)
    {
        s_xbox_controller_context.state.dpad_direction = (xbox_controller_dpad_direction_t)raw_dpad;
    }
    else
    {
        s_xbox_controller_context.state.dpad_direction = XBOX_DPAD_DIRECTION_CENTERED;
    }

    /*
     * 6. 核心动作按键提取 (Byte 13: 按实测位掩码提取)
     */
    uint8_t action_buttons_byte = raw_data[13];
    s_xbox_controller_context.state.buttons.button_a = (action_buttons_byte & (1 << 0)) != 0;     // Bit 0: A键
    s_xbox_controller_context.state.buttons.button_b = (action_buttons_byte & (1 << 1)) != 0;     // Bit 1: B键
    s_xbox_controller_context.state.buttons.button_x = (action_buttons_byte & (1 << 3)) != 0;     // Bit 3: X键
    s_xbox_controller_context.state.buttons.button_y = (action_buttons_byte & (1 << 4)) != 0;     // Bit 4: Y键
    s_xbox_controller_context.state.buttons.bumper_left = (action_buttons_byte & (1 << 6)) != 0;  // Bit 6: LB键
    s_xbox_controller_context.state.buttons.bumper_right = (action_buttons_byte & (1 << 7)) != 0; // Bit 7: RB键

    /*
     * 7. 系统辅助按键提取 (Byte 14、Byte 15: 按实测位掩码提取)
     */
    uint8_t system_buttons_byte1 = raw_data[14];
    s_xbox_controller_context.state.buttons.button_view = (system_buttons_byte1 & (1 << 2)) != 0;            // Bit 2: View键
    s_xbox_controller_context.state.buttons.button_menu = (system_buttons_byte1 & (1 << 3)) != 0;            // Bit 3: Menu键
    s_xbox_controller_context.state.buttons.button_xbox = (system_buttons_byte1 & (1 << 4)) != 0;            // Bit 4: 西瓜键
    s_xbox_controller_context.state.buttons.thumbstick_click_left = (system_buttons_byte1 & (1 << 5)) != 0;  // Bit 5: L3键
    s_xbox_controller_context.state.buttons.thumbstick_click_right = (system_buttons_byte1 & (1 << 6)) != 0; // Bit 6: R3键
                                                                                                             //
    uint8_t system_buttons_byte2 = raw_data[15];                                                             //
    s_xbox_controller_context.state.buttons.button_share = (system_buttons_byte2 & (1 << 0)) != 0;           // Bit0 : Share键

    /* 8. 确保连接标志有效，并触发姿态改变事件回调 */
    s_xbox_controller_context.state.is_connected = true;

    if (s_xbox_controller_context.event_callback != NULL)
    {
        s_xbox_controller_context.event_callback(XBOX_CONTROLLER_EVENT_STATE_CHANGED,
                                                 &s_xbox_controller_context.state,
                                                 s_xbox_controller_context.user_context);
    }
}

void xbox_controller_notify_connected(void)
{
    if (!s_xbox_controller_context.is_initialized)
    {
        return;
    }

    s_xbox_controller_context.state.is_connected = true;

    // 触发连接成功
    if (s_xbox_controller_context.event_callback != NULL)
    {
        s_xbox_controller_context.event_callback(XBOX_CONTROLLER_EVENT_CONNECTED,
                                                 &s_xbox_controller_context.state,
                                                 s_xbox_controller_context.user_context);
    }
}

void xbox_controller_notify_disconnected(void)
{
    if (!s_xbox_controller_context.is_initialized)
    {
        return;
    }

    // 掉线后安全将整个手柄姿态数据清零(防止掉线前持续按住导致失控)
    memset(&s_xbox_controller_context.state, 0, sizeof(xbox_controller_state_t));
    s_xbox_controller_context.state.is_connected = false;

#if CONFIG_APP_ENABLE_XBOX_BATTERY_MONITORING
    s_xbox_battery_percentage = 255;
#endif
#if CONFIG_APP_ENABLE_XBOX_DEVICE_INFORMATION
    memset(s_xbox_serial_number, 0, sizeof(s_xbox_serial_number));
    memset(s_xbox_firmware_version, 0, sizeof(s_xbox_firmware_version));
#endif

    // 触发上层断开事件通知
    if (s_xbox_controller_context.event_callback != NULL)
    {
        s_xbox_controller_context.event_callback(XBOX_CONTROLLER_EVENT_DISCONNECTED,
                                                 &s_xbox_controller_context.state,
                                                 s_xbox_controller_context.user_context);
    }
}

xbox_controller_status_t xbox_controller_set_joystick_deadzone(uint16_t deadzone_threshold)
{
    if (!s_xbox_controller_context.is_initialized)
    {
        return XBOX_CONTROLLER_STATUS_ERROR_NOT_CONNECTED;
    }

    // 限制死区阈值不得超过最大行程的一半 (16384)
    if (deadzone_threshold > 16384)
    {
        return XBOX_CONTROLLER_STATUS_ERROR_INVALID_ARG;
    }

    s_xbox_controller_context.joystick_deadzone_threshold = deadzone_threshold;
    return XBOX_CONTROLLER_STATUS_OK;
}

/* ========================================================================= */
/*  二、控制输入与姿态查询接口 (Inputs)                                       */
/* ========================================================================= */

bool xbox_controller_is_button_pressed(xbox_controller_button_t button)
{
    /* 手柄未初始化或未连接时，安全返回 false */
    if (!s_xbox_controller_context.is_initialized || !s_xbox_controller_context.state.is_connected)
    {
        return false;
    }

    switch (button)
    {
    case XBOX_CONTROLLER_BUTTON_A:
        return s_xbox_controller_context.state.buttons.button_a;

    case XBOX_CONTROLLER_BUTTON_B:
        return s_xbox_controller_context.state.buttons.button_b;

    case XBOX_CONTROLLER_BUTTON_X:
        return s_xbox_controller_context.state.buttons.button_x;

    case XBOX_CONTROLLER_BUTTON_Y:
        return s_xbox_controller_context.state.buttons.button_y;

    case XBOX_CONTROLLER_BUTTON_BUMPER_LEFT:
        return s_xbox_controller_context.state.buttons.bumper_left;

    case XBOX_CONTROLLER_BUTTON_BUMPER_RIGHT:
        return s_xbox_controller_context.state.buttons.bumper_right;

    case XBOX_CONTROLLER_BUTTON_THUMBSTICK_LEFT:
        return s_xbox_controller_context.state.buttons.thumbstick_click_left;

    case XBOX_CONTROLLER_BUTTON_THUMBSTICK_RIGHT:
        return s_xbox_controller_context.state.buttons.thumbstick_click_right;

    case XBOX_CONTROLLER_BUTTON_VIEW:
        return s_xbox_controller_context.state.buttons.button_view;

    case XBOX_CONTROLLER_BUTTON_MENU:
        return s_xbox_controller_context.state.buttons.button_menu;

    case XBOX_CONTROLLER_BUTTON_XBOX:
        return s_xbox_controller_context.state.buttons.button_xbox;

    case XBOX_CONTROLLER_BUTTON_SHARE:
        return s_xbox_controller_context.state.buttons.button_share;

    default:
        return false;
    }
}

xbox_controller_status_t xbox_controller_get_joystick_left(int16_t *out_x, int16_t *out_y)
{
    /* 1. 指针防呆 */
    if (out_x == NULL || out_y == NULL)
    {
        return XBOX_CONTROLLER_STATUS_ERROR_INVALID_ARG;
    }

    /* 2. 状态检查：未连接时安全赋 0 并返回错误 */
    if (!s_xbox_controller_context.is_initialized || !s_xbox_controller_context.state.is_connected)
    {
        *out_x = 0;
        *out_y = 0;
        return XBOX_CONTROLLER_STATUS_ERROR_NOT_CONNECTED;
    }

    /* 3. 输出已经过抗漂移死区滤波的坐标 */
    *out_x = s_xbox_controller_context.state.joystick_left_x;
    *out_y = s_xbox_controller_context.state.joystick_left_y;

    return XBOX_CONTROLLER_STATUS_OK;
}

xbox_controller_status_t xbox_controller_get_joystick_right(int16_t *out_x, int16_t *out_y)
{
    /* 1. 指针防呆 */
    if (out_x == NULL || out_y == NULL)
    {
        return XBOX_CONTROLLER_STATUS_ERROR_INVALID_ARG;
    }

    /* 2. 状态检查：未连接时安全赋 0 并返回错误 */
    if (!s_xbox_controller_context.is_initialized || !s_xbox_controller_context.state.is_connected)
    {
        *out_x = 0;
        *out_y = 0;
        return XBOX_CONTROLLER_STATUS_ERROR_NOT_CONNECTED;
    }

    /* 3. 输出已经过抗漂移死区滤波的坐标 */
    *out_x = s_xbox_controller_context.state.joystick_right_x;
    *out_y = s_xbox_controller_context.state.joystick_right_y;

    return XBOX_CONTROLLER_STATUS_OK;
}

uint16_t xbox_controller_get_trigger_left(void)
{
    /* 手柄未初始化或未连接时，安全返回 0 (代表完全松开) */
    if (!s_xbox_controller_context.is_initialized || !s_xbox_controller_context.state.is_connected)
    {
        return 0;
    }

    return s_xbox_controller_context.state.trigger_left;
}

uint16_t xbox_controller_get_trigger_right(void)
{
    /* 手柄未初始化或未连接时，安全返回 0 */
    if (!s_xbox_controller_context.is_initialized || !s_xbox_controller_context.state.is_connected)
    {
        return 0;
    }

    return s_xbox_controller_context.state.trigger_right;
}

xbox_controller_dpad_direction_t xbox_controller_get_dpad_direction(void)
{
    /* 手柄未初始化或未连接时，安全返回居中无动作状态 */
    if (!s_xbox_controller_context.is_initialized || !s_xbox_controller_context.state.is_connected)
    {
        return XBOX_DPAD_DIRECTION_CENTERED;
    }

    return s_xbox_controller_context.state.dpad_direction;
}

xbox_controller_status_t xbox_controller_get_state(xbox_controller_state_t *out_state)
{
    /* 1. 指针防呆 */
    if (out_state == NULL)
    {
        return XBOX_CONTROLLER_STATUS_ERROR_INVALID_ARG;
    }

    /* 2. 状态检查：未连接时安全清零整个结构体，并返回未连接错误 */
    if (!s_xbox_controller_context.is_initialized || !s_xbox_controller_context.state.is_connected)
    {
        memset(out_state, 0, sizeof(xbox_controller_state_t));
        return XBOX_CONTROLLER_STATUS_ERROR_NOT_CONNECTED;
    }

    /* 3. 拷贝当前最新姿态快照 */
    memcpy(out_state, &s_xbox_controller_context.state, sizeof(xbox_controller_state_t));

    return XBOX_CONTROLLER_STATUS_OK;
}

/* ========================================================================= */
/* 三、马达震动与强度控制接口 (Outputs)                                        */
/* ========================================================================= */

xbox_controller_status_t xbox_controller_set_main_motors(uint8_t left_intensity,
                                                         uint8_t right_intensity,
                                                         uint8_t duration_in_10ms_units,
                                                         uint8_t delay_in_10ms_units,
                                                         uint8_t loop_count)
{

    return xbox_controller_send_rumble_packet(0x0F,
                                              0, 0,
                                              left_intensity,
                                              right_intensity,
                                              duration_in_10ms_units,
                                              delay_in_10ms_units,
                                              loop_count);
}

xbox_controller_status_t xbox_controller_set_trigger_motors(uint8_t left_trigger_intensity,
                                                            uint8_t right_trigger_intensity,
                                                            uint8_t duration_in_10ms_units,
                                                            uint8_t delay_in_10ms_units,
                                                            uint8_t loop_count)
{

    return xbox_controller_send_rumble_packet(0x0F,
                                              left_trigger_intensity,
                                              right_trigger_intensity,
                                              0, 0,
                                              duration_in_10ms_units,
                                              delay_in_10ms_units,
                                              loop_count);
}

xbox_controller_status_t xbox_controller_pulse_custom_vibration(
    uint8_t left_main_percent,
    uint8_t right_main_percent,
    uint8_t left_trigger_percent,
    uint8_t right_trigger_percent,
    uint8_t duration_in_10ms_units,
    uint8_t delay_in_10ms_units,
    uint8_t loop_count)
{

    return xbox_controller_send_rumble_packet(0x0F,
                                              left_trigger_percent,
                                              right_trigger_percent,
                                              left_main_percent,
                                              right_main_percent,
                                              duration_in_10ms_units,
                                              delay_in_10ms_units,
                                              loop_count);
}

xbox_controller_status_t xbox_controller_stop_vibration(void)
{
    /* 掩码 0x0F: 四颗马达全部强制熄火归零 */
    return xbox_controller_send_rumble_packet(0x0F, 0, 0, 0, 0, 0, 0, 0);
}

/* ========================================================================= */
/* 四、手柄设备信息获取与查询接口 (Information)                                */
/* ========================================================================= */

bool xbox_controller_is_connected(void)
{
    return s_xbox_controller_context.is_initialized && s_xbox_controller_context.state.is_connected;
}

#if CONFIG_APP_ENABLE_XBOX_BATTERY_MONITORING

uint8_t xbox_controller_get_battery_percentage(void)
{
    if (!xbox_controller_is_connected())
        return 255;
    return s_xbox_battery_percentage;
}

void xbox_controller_feed_battery_percentage(uint8_t battery_percentage)
{
    s_xbox_battery_percentage = battery_percentage;
}
#endif

#if CONFIG_APP_ENABLE_XBOX_DEVICE_INFORMATION

const char *xbox_controller_get_serial_number(void)
{
    if (!xbox_controller_is_connected() || s_xbox_serial_number[0] == '\0')
    {
        return "UNKNOWN";
    }
    return s_xbox_serial_number;
}

const char *xbox_controller_get_firmware_version(void)
{
    if (!xbox_controller_is_connected() || s_xbox_firmware_version[0] == '\0')
    {
        return "UNKNOWN";
    }
    return s_xbox_firmware_version;
}

void xbox_controller_feed_serial_number_raw(const uint8_t *raw_data, uint16_t length)
{
    if (raw_data == NULL || length == 0)
        return;

    size_t copy_length = (length < sizeof(s_xbox_serial_number) - 1) ? length : sizeof(s_xbox_serial_number) - 1;
    memcpy(s_xbox_serial_number, raw_data, copy_length);
    s_xbox_serial_number[copy_length] = '\0';
}

void xbox_controller_feed_firmware_version_raw(const uint8_t *raw_data, uint16_t length)
{
    if (raw_data == NULL || length == 0)
        return;

    size_t copy_length = (length < sizeof(s_xbox_firmware_version) - 1) ? length : sizeof(s_xbox_firmware_version) - 1;
    memcpy(s_xbox_firmware_version, raw_data, copy_length);
    s_xbox_firmware_version[copy_length] = '\0';
}
#endif

/* ========================================================================= */
/* 五、耳机音频接口 (Audio Interface - 架构预留)                              */
/* ========================================================================= */

xbox_controller_status_t xbox_controller_audio_send_speaker_pcm(
    const uint8_t *pcm_data,
    uint16_t length);

xbox_controller_status_t xbox_controller_audio_register_microphone_callback(
    xbox_controller_audio_microphone_callback_t callback,
    void *user_context);
