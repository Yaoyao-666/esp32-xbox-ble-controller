#include "ble_manager.h"

#include <string.h>
#include "esp_log.h"
#include "nvs_flash.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"

/* NimBLE 核心头文件 */
#include "nimble/nimble_port.h"
#include "nimble/nimble_port_freertos.h"
#include "store/config/ble_store_config.h"
#include "host/ble_hs.h"
#include "host/ble_gap.h"
#include "host/ble_att.h"
#include "host/ble_gatt.h"
#include "host/ble_store.h"
#include "services/gap/ble_svc_gap.h"

static const char *TAG = "BLE_MANAGER";

/* ========================================================================= */
/* 内部状态单例定义                                                           */
/* ========================================================================= */

/**
 * @brief 管理器内部核心状态机生命周期枚举
 */
typedef enum
{
    BLE_MANAGER_STATE_UNINITIALIZED = 0, // 未初始化：内存未分配或已被注销
    BLE_MANAGER_STATE_INITIALIZED,       // 已初始化：底层资源就绪，等待 Host 与 Controller 完成同步 (Sync)
    BLE_MANAGER_STATE_RUNNING            // 运行中：协议栈完全就绪，允许发起广播、扫描或数据收发
} ble_manager_internal_state_t;

/**
 * @brief 独立连接槽位信息结构体 (维护单一物理链路的连接态与专属 MTU)
 */
typedef struct
{
    uint16_t connection_handle; // 链路连接句柄 (为 BLE_MANAGER_INVALID_CONNECTION_HANDLE 时表示槽位空闲)
    uint16_t current_mtu;       // 当前链路协商生效的 ATT MTU 大小 (默认 23 字节)
} ble_manager_connection_slot_t;

/**
 * @brief BLE 管理器全局控制块单例 (管理本地设备状态、连接池及互斥锁)
 */
typedef struct
{
    ble_manager_internal_state_t state;                                     // 当前全局运行状态
    ble_manager_role_t role;                                                // 当前设备工作角色 (外围设备或中央设备)
    char device_name[32];                                                   // 本机设备名称 (用作本地 GAP 0x2A00 特性及广播名称)
    ble_manager_event_callback_t event_callback;                            // 上层业务注册的全局事件回调指针
    void *user_context;                                                     // 上层业务私有上下文指针
    ble_manager_connection_slot_t connections[BLE_MANAGER_MAX_CONNECTIONS]; // 多连接管理槽位数组
    uint8_t active_connection_count;                                        // 当前处于活跃状态的连接总数
    SemaphoreHandle_t lock;                                                 // 控制块互斥锁 (保护状态流转与槽位访问安全)

} ble_manager_control_block_t;

static ble_manager_control_block_t s_manager;

/* ========================================================================= */
/* 内部安全辅助函数：派发事件给业务层                                          */
/* ========================================================================= */

/**
 * @brief 线程安全的事件派发中枢
 * @note 采用“锁内快速拷贝函数指针、释放锁后再执行调用”的原则，彻底杜绝用户回调执行耗时操作引发的死锁。
 */
static void ble_manager_dispatch_event(ble_manager_event_t event,
                                       uint16_t connection_handle,
                                       void *event_data)
{
    ble_manager_event_callback_t cb = NULL;
    void *ctx = NULL;

    /* 在锁内快速复制回调句柄与上下文，随后立即释放锁，防止用户回调耗时引发死锁 */
    if (s_manager.lock != NULL)
    {
        xSemaphoreTake(s_manager.lock, portMAX_DELAY);
        cb = s_manager.event_callback;
        ctx = s_manager.user_context;
        xSemaphoreGive(s_manager.lock);
    }

    if (cb != NULL)
    {
        cb(event, connection_handle, event_data, ctx);
    }
}

/* ========================================================================= */
/* NimBLE GAP 事件中枢 F(统一管理广播、扫描、连接与数据链路状态)                */
/* ========================================================================= */

static int ble_manager_gap_event_handler(struct ble_gap_event *event, void *arg)
{
    switch (event->type)
    {
    case BLE_GAP_EVENT_CONNECT:
        if (event->connect.status == 0)
        {
            bool accepted = false;
            uint8_t current_active_count = 0; // 活动连接数快照

            xSemaphoreTake(s_manager.lock, portMAX_DELAY);
            for (int i = 0; i < BLE_MANAGER_MAX_CONNECTIONS; i++)
            {
                if (s_manager.connections[i].connection_handle == BLE_MANAGER_INVALID_CONNECTION_HANDLE)
                {
                    s_manager.connections[i].connection_handle = event->connect.conn_handle;
                    s_manager.connections[i].current_mtu = BLE_ATT_MTU_DFLT;
                    s_manager.active_connection_count++;
                    current_active_count = s_manager.active_connection_count; // 锁内拷贝
                    accepted = true;
                    break;
                }
            }
            xSemaphoreGive(s_manager.lock);

            if (accepted)
            {
                // 使用局部变量打印，防止多线程读写竞态
                ESP_LOGI(TAG, "Connected successfully [handle: %d, active: %u/%u]",
                         event->connect.conn_handle, current_active_count, BLE_MANAGER_MAX_CONNECTIONS);
                ble_manager_dispatch_event(BLE_MANAGER_EVENT_CONNECTED,
                                           event->connect.conn_handle,
                                           NULL);
            }
            else
            {
                ESP_LOGW(TAG, "Connection limit reached (%d), rejecting handle: %u",
                         BLE_MANAGER_MAX_CONNECTIONS, event->connect.conn_handle);
                ble_gap_terminate(event->connect.conn_handle, BLE_ERR_REM_USER_CONN_TERM);
            }
        }
        else
        {
            ESP_LOGW(TAG, "Connection failed, status: %d", event->connect.status);
            int error_code = event->connect.status;
            ble_manager_dispatch_event(BLE_MANAGER_EVENT_ERROR,
                                       BLE_MANAGER_INVALID_CONNECTION_HANDLE,
                                       &error_code);
        }
        return 0;

    case BLE_GAP_EVENT_DISCONNECT:
    {
        uint16_t dis_handle = event->disconnect.conn.conn_handle;
        ESP_LOGI(TAG, "Disconnected, handle: %d, reason: 0x%02x", dis_handle, event->disconnect.reason);

        ble_manager_event_disconnect_t dis_data = {
            .reason = event->disconnect.reason};

        xSemaphoreTake(s_manager.lock, portMAX_DELAY);

        // 根据断开句柄精准查找并释放对应的管理槽位
        for (int i = 0; i < BLE_MANAGER_MAX_CONNECTIONS; i++)
        {
            if (s_manager.connections[i].connection_handle == dis_handle)
            {
                s_manager.connections[i].connection_handle = BLE_MANAGER_INVALID_CONNECTION_HANDLE;
                s_manager.connections[i].current_mtu = BLE_ATT_MTU_DFLT;
                if (s_manager.active_connection_count > 0)
                {
                    s_manager.active_connection_count--;
                }
                break;
            }
        }
        xSemaphoreGive(s_manager.lock);

        // 包含正常链路断开与 0x023e 握手流产，确保上层状态机始终闭环 (如自动重启扫描)
        ble_manager_dispatch_event(BLE_MANAGER_EVENT_DISCONNECTED,
                                   dis_handle,
                                   &dis_data);

        return 0;
    }

    case BLE_GAP_EVENT_MTU:
    {
        uint16_t conn_handle = event->mtu.conn_handle;
        uint16_t updated_mtu = event->mtu.value;
        bool need_notify = false;

        xSemaphoreTake(s_manager.lock, portMAX_DELAY);

        // 更新目标连接槽位的专属 MTU 缓存
        for (int i = 0; i < BLE_MANAGER_MAX_CONNECTIONS; i++)
        {
            if (s_manager.connections[i].connection_handle == conn_handle)
            {
                s_manager.connections[i].current_mtu = updated_mtu;
                need_notify = true;
                break;
            }
        }
        xSemaphoreGive(s_manager.lock);

        if (need_notify)
        {
            ble_manager_dispatch_event(BLE_MANAGER_EVENT_MTU_UPDATED,
                                       conn_handle,
                                       &updated_mtu);
        }
        return 0;
    }

    case BLE_GAP_EVENT_CONN_UPDATE:
    {
        ESP_LOGI(TAG, "Connection update event on handle %u, status: %d",
                 event->conn_update.conn_handle,
                 event->conn_update.status);

        ble_manager_connection_parameters_t parameters;
        memset(&parameters, 0, sizeof(parameters));
        parameters.status = event->conn_update.status;

        /* 若协商成功，通过 NimBLE 官方 API 查询当前连接实际生效的物理参数 */
        if (event->conn_update.status == 0)
        {
            struct ble_gap_conn_desc connection_description;
            int find_rc = ble_gap_conn_find(event->conn_update.conn_handle, &connection_description);
            if (find_rc == 0)
            {
                parameters.connection_interval = connection_description.conn_itvl;
                parameters.peripheral_latency = connection_description.conn_latency;
                parameters.supervision_timeout = connection_description.supervision_timeout;

                ESP_LOGI(TAG, "Effective parameters [interval: %.2f ms, latency: %u, timeout: %u ms]",
                         connection_description.conn_itvl * 1.25f,         // 协商生效的连接间隔 (单位: 1.25ms)
                         connection_description.conn_latency,              // 从机潜伏次数
                         connection_description.supervision_timeout * 10); // 协商生效的超时时间 (单位: 10ms)
            }
            else
            {
                ESP_LOGE(TAG, "Failed to find connection description, rc: %d", find_rc);
            }
        }

        ble_manager_dispatch_event(BLE_MANAGER_EVENT_CONNECTION_PARAMETERS_UPDATED,
                                   event->conn_update.conn_handle,
                                   &parameters);
        return 0;
    }

    case BLE_GAP_EVENT_ENC_CHANGE:
    {
        ESP_LOGI(TAG, "Encryption change event on handle %u, status: %d",
                 event->enc_change.conn_handle, event->enc_change.status);

        ble_manager_async_result_t enc_result;
        memset(&enc_result, 0, sizeof(enc_result));
        enc_result.status = event->enc_change.status;
        enc_result.attribute_handle = 0;

        ble_manager_dispatch_event(BLE_MANAGER_EVENT_ENCRYPTION_COMPLETED,
                                   event->enc_change.conn_handle,
                                   &enc_result);
        return 0;
    }

    case BLE_GAP_EVENT_ADV_COMPLETE:

        ESP_LOGI(TAG, "Advertising completed/stopped, reason: %d", event->adv_complete.reason);
        ble_manager_dispatch_event(BLE_MANAGER_EVENT_ADVERTISING_STOPPED,
                                   BLE_MANAGER_INVALID_CONNECTION_HANDLE,
                                   NULL);
        return 0;

    case BLE_GAP_EVENT_NOTIFY_RX:
    {
        ble_manager_received_data_t received_data;
        memset(&received_data, 0, sizeof(received_data));

        received_data.attribute_handle = event->notify_rx.attr_handle;

        //  BLE 规范中单次 ATT PDU 最大为 512 字节载荷 + 5 字节 L2CAP/ATT 报头，故设 517 字节安全上限]
        uint8_t notify_buffer[517];
        uint16_t data_len = OS_MBUF_PKTLEN(event->notify_rx.om);
        if (data_len > sizeof(notify_buffer))
        {
            ESP_LOGW(TAG, "Notification truncated: incoming %u bytes, buffer limit %u bytes",
                     data_len, (unsigned int)sizeof(notify_buffer));
            data_len = sizeof(notify_buffer);
        }

        int copy_rc = os_mbuf_copydata(event->notify_rx.om, 0, data_len, notify_buffer);
        if (copy_rc != 0)
        {
            ESP_LOGE(TAG, "Failed to copy notification data from mbuf, rc: %d", copy_rc);
            return 0;
        }
        received_data.data = notify_buffer;
        received_data.length = data_len;

        ESP_LOGI(TAG, "Notification received: handle=%d, attr=0x%04x, len=%d",
                 event->notify_rx.conn_handle, received_data.attribute_handle, data_len);

        // 派发通知数据接收事件给应用层
        ble_manager_dispatch_event(BLE_MANAGER_EVENT_NOTIFICATION_RECEIVED,
                                   event->notify_rx.conn_handle,
                                   &received_data);
        return 0;
    }

    case BLE_GAP_EVENT_SUBSCRIBE:

        // 从机角色下，远端 Central 写入本端 CCCD (0x2902) 订阅时在此捕获]
        ESP_LOGI(TAG, "Subscribe event on conn %d, attr %d (notify: %d, indicate: %d)",
                 event->subscribe.conn_handle,
                 event->subscribe.attr_handle,
                 event->subscribe.cur_notify,
                 event->subscribe.cur_indicate);
        return 0;

    case BLE_GAP_EVENT_DISC:
    {
        ble_manager_discovered_device_t dev;
        memset(&dev, 0, sizeof(dev));

        // 1. 提取对端设备 MAC 地址与地址类型 (公有或随机)
        memcpy(dev.address, event->disc.addr.val, 6);
        dev.address_type = (event->disc.addr.type == BLE_ADDR_PUBLIC)
                               ? BLE_MANAGER_ADDRESS_TYPE_PUBLIC
                               : BLE_MANAGER_ADDRESS_TYPE_RANDOM;
        dev.rssi = event->disc.rssi;

        // 2. 解析原始广播与扫描响应字段 (提取设备本地全名)
        struct ble_hs_adv_fields fields;
        int rc = ble_hs_adv_parse_fields(&fields, event->disc.data, event->disc.length_data);
        if (rc == 0 && fields.name != NULL && fields.name_len > 0)
        {
            size_t copy_len = fields.name_len;
            if (copy_len >= sizeof(dev.device_name))
            {
                copy_len = sizeof(dev.device_name) - 1; // 严格保留末尾 '\0' 空间，防止越界
            }
            memcpy(dev.device_name, fields.name, copy_len);
            dev.device_name[copy_len] = '\0';
        }

        // 3. 同步上报应用层 (注意：栈变量指针仅在当前回调期间合法)
        ble_manager_dispatch_event(BLE_MANAGER_EVENT_DEVICE_DISCOVERED,
                                   BLE_MANAGER_INVALID_CONNECTION_HANDLE,
                                   &dev);
        return 0;
    }

    case BLE_GAP_EVENT_DISC_COMPLETE:

        // 扫描生命周期结束：无论是超时自然结束，还是主动调用 cancel 均在此闭环
        ESP_LOGI(TAG, "Scanning stopped/completed, reason: %d", event->disc_complete.reason);
        ble_manager_dispatch_event(BLE_MANAGER_EVENT_SCANNING_STOPPED,
                                   BLE_MANAGER_INVALID_CONNECTION_HANDLE,
                                   NULL);
        return 0;

    default:
        break;
    }

    return 0;
}

/* ========================================================================= */
/* NimBLE Host 生命周期回调                                                  */
/* ========================================================================= */

/**
 * @brief NimBLE Host 事件主循环后台任务
 */
static void ble_manager_host_task(void *param)
{
    ESP_LOGI(TAG, "NimBLE host task started");
    nimble_port_run();             // 阻塞运行，直到 nimble_port_stop 被触发退出
    nimble_port_freertos_deinit(); // 官方端口注销清理，内部会自动销毁当前任务
}

/**
 * @brief 协议栈与控制器完成握手同步回调 (Stack Sync)
 */
static void ble_manager_on_sync(void)
{
    ESP_LOGI(TAG, "NimBLE host synced, stack is READY");

    xSemaphoreTake(s_manager.lock, portMAX_DELAY);
    s_manager.state = BLE_MANAGER_STATE_RUNNING;
    xSemaphoreGive(s_manager.lock);

    ble_manager_dispatch_event(BLE_MANAGER_EVENT_READY,
                               BLE_MANAGER_INVALID_CONNECTION_HANDLE,
                               NULL);
}

/**
 * @brief 协议栈控制器异常复位回调 (Controller Reset)
 */
static void ble_manager_on_reset(int reason)
{

    ESP_LOGE(TAG, "NimBLE host reset, reason: %d", reason);

    xSemaphoreTake(s_manager.lock, portMAX_DELAY);
    for (int i = 0; i < BLE_MANAGER_MAX_CONNECTIONS; i++)
    {
        s_manager.connections[i].connection_handle = BLE_MANAGER_INVALID_CONNECTION_HANDLE;
        s_manager.connections[i].current_mtu = BLE_ATT_MTU_DFLT;
    }
    s_manager.active_connection_count = 0;
    s_manager.state = BLE_MANAGER_STATE_INITIALIZED;
    xSemaphoreGive(s_manager.lock);

    int error_code = reason;
    ble_manager_dispatch_event(BLE_MANAGER_EVENT_ERROR,
                               BLE_MANAGER_INVALID_CONNECTION_HANDLE,
                               &error_code);
}

/* ========================================================================= */
/* 核心接口实现: ble_manager_init                                             */
/* ========================================================================= */

// 确保在未开启 CONFIG_BT_NIMBLE_NVS_PERSIST 宏时依然兼容编译
void ble_store_config_init(void);

esp_err_t ble_manager_init(const ble_manager_config_t *config)
{
    /* 1. 校验全局生命周期状态 */
    if (s_manager.state != BLE_MANAGER_STATE_UNINITIALIZED)
    {
        ESP_LOGE(TAG, "Init failed: Manager already initialized");
        return ESP_ERR_INVALID_STATE;
    }

    /* 2. 输入参数校验 */
    if (config == NULL || config->device_name == NULL || config->event_callback == NULL)
    {
        ESP_LOGE(TAG, "Init failed: Invalid NULL arguments");
        return ESP_ERR_INVALID_ARG;
    }

    if (config->role != BLE_MANAGER_ROLE_PERIPHERAL &&
        config->role != BLE_MANAGER_ROLE_CENTRAL)
    {
        ESP_LOGE(TAG, "Init failed: Invalid role %d", config->role);
        return ESP_ERR_INVALID_ARG;
    }

    size_t name_len = strlen(config->device_name);
    if (name_len == 0 || name_len >= sizeof(s_manager.device_name))
    {
        ESP_LOGE(TAG, "Init failed: Device name length invalid");
        return ESP_ERR_INVALID_ARG;
    }

    esp_err_t err = ESP_OK;
    int rc = 0;

    // 3. 创建状态保护互斥锁
    s_manager.lock = xSemaphoreCreateMutex();
    if (s_manager.lock == NULL)
    {
        ESP_LOGE(TAG, "Init failed: Mutex creation failed");
        return ESP_ERR_NO_MEM;
    }

    // 4. 保存全局配置与重置槽位表
    s_manager.role = config->role;
    s_manager.event_callback = config->event_callback;
    s_manager.user_context = config->user_context;
    strncpy(s_manager.device_name, config->device_name, sizeof(s_manager.device_name) - 1);
    s_manager.device_name[sizeof(s_manager.device_name) - 1] = '\0';

    for (int i = 0; i < BLE_MANAGER_MAX_CONNECTIONS; i++)
    {
        s_manager.connections[i].connection_handle = BLE_MANAGER_INVALID_CONNECTION_HANDLE;
        s_manager.connections[i].current_mtu = BLE_ATT_MTU_DFLT;
    }
    s_manager.active_connection_count = 0;

    // 5. 初始化 NimBLE 端口
    err = nimble_port_init();
    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "nimble_port_init failed: %s", esp_err_to_name(err));
        goto fail_clean_all;
    }

    ble_hs_cfg.sync_cb = ble_manager_on_sync;
    ble_hs_cfg.reset_cb = ble_manager_on_reset;

    // 6. 初始化默认 GAP 服务并设置设备名称
    ble_svc_gap_init();
    rc = ble_svc_gap_device_name_set(s_manager.device_name);
    if (rc != 0)
    {
        ESP_LOGE(TAG, "ble_svc_gap_device_name_set failed, rc: %d", rc);
        err = ESP_FAIL;
        goto fail_deinit_port;
    }

    // 7. 配置安全管理器协议 (SMP) 与密钥持久化机制:
#if CONFIG_BT_NIMBLE_NVS_PERSIST
    ble_store_config_init(); // 初始化 NVS 密钥存储，保证重连时自动恢复 Bonding 记录
#endif

    ble_hs_cfg.sm_io_cap = BLE_SM_IO_CAP_NO_IO;               // 无显示无按键输入能力 (自动进入 Just Works 无感配对)
    ble_hs_cfg.sm_bonding = 1;                                // 开启绑定支持 (生成并存储长期密钥 LTK)
    ble_hs_cfg.sm_sc = 1;                                     // 开启 LE 安全连接 (使用基于 ECDH 的强加密机制)
    ble_hs_cfg.sm_our_key_dist |= BLE_SM_PAIR_KEY_DIST_ENC;   // 配对时分发本端的加密密钥 (LTK)
    ble_hs_cfg.sm_their_key_dist |= BLE_SM_PAIR_KEY_DIST_ENC; // 配对时接收并存储对端的加密密钥 (LTK)

    // 8. 从机模式下，安全注册业务 GATT 服务表 (必须在启动 Host 任务前完成)
    if (s_manager.role == BLE_MANAGER_ROLE_PERIPHERAL && config->gatt_services != NULL)
    {
        rc = ble_gatts_count_cfg(config->gatt_services);
        if (rc != 0)
        {
            ESP_LOGE(TAG, "ble_gatts_count_cfg failed, rc: %d", rc);
            err = ESP_FAIL;
            goto fail_deinit_port;
        }

        rc = ble_gatts_add_svcs(config->gatt_services);
        if (rc != 0)
        {
            ESP_LOGE(TAG, "ble_gatts_add_svcs failed, rc: %d", rc);
            err = ESP_FAIL;
            goto fail_deinit_port;
        }
    }

    // 9. 状态流转并拉起 Host 调度任务
    s_manager.state = BLE_MANAGER_STATE_INITIALIZED;
    nimble_port_freertos_init(ble_manager_host_task);

    ESP_LOGI(TAG, "BLE Manager initialized successfully (Max Connections: %d)", BLE_MANAGER_MAX_CONNECTIONS);
    return ESP_OK;

fail_deinit_port:
    nimble_port_deinit();
fail_clean_all:
    if (s_manager.lock != NULL)
    {
        vSemaphoreDelete(s_manager.lock);
        s_manager.lock = NULL;
    }
    s_manager.event_callback = NULL;
    s_manager.user_context = NULL;
    memset(s_manager.device_name, 0, sizeof(s_manager.device_name));
    s_manager.state = BLE_MANAGER_STATE_UNINITIALIZED;
    return err;
}

/* ========================================================================= */
/* 核心接口实现: ble_manager_deinit (与 init 对称设计)                         */
/* ========================================================================= */

esp_err_t ble_manager_deinit(void)
{
    if (s_manager.state == BLE_MANAGER_STATE_UNINITIALIZED || s_manager.lock == NULL)
    {
        return ESP_OK;
    }

    ESP_LOGI(TAG, "Deinitializing BLE Manager...");

    int rc = nimble_port_stop();
    if (rc != 0)
    {
        ESP_LOGE(TAG, "nimble_port_stop failed, rc: %d", rc);
        return ESP_FAIL;
    }

    nimble_port_deinit();

    if (s_manager.lock != NULL)
    {
        vSemaphoreDelete(s_manager.lock);
        s_manager.lock = NULL;
    }

    s_manager.state = BLE_MANAGER_STATE_UNINITIALIZED;
    for (int i = 0; i < BLE_MANAGER_MAX_CONNECTIONS; i++)
    {
        s_manager.connections[i].connection_handle = BLE_MANAGER_INVALID_CONNECTION_HANDLE;
        s_manager.connections[i].current_mtu = BLE_ATT_MTU_DFLT;
    }
    s_manager.active_connection_count = 0;
    s_manager.event_callback = NULL;
    s_manager.user_context = NULL;
    memset(s_manager.device_name, 0, sizeof(s_manager.device_name));

    ESP_LOGI(TAG, "BLE Manager deinitialized safely");
    return ESP_OK;
}

/* ========================================================================= */
/* 基础控制与查询接口                                                         */
/* ========================================================================= */

esp_err_t ble_manager_disconnect(uint16_t connection_handle)
{
    if (s_manager.lock == NULL)
    {
        return ESP_ERR_INVALID_STATE;
    }

    bool handle_found = false;

    xSemaphoreTake(s_manager.lock, portMAX_DELAY);
    if (s_manager.state != BLE_MANAGER_STATE_RUNNING)
    {
        xSemaphoreGive(s_manager.lock);
        return ESP_ERR_INVALID_STATE;
    }

    // 在已录入的槽位中检索目标句柄
    if (connection_handle != BLE_MANAGER_INVALID_CONNECTION_HANDLE)
    {
        for (int i = 0; i < BLE_MANAGER_MAX_CONNECTIONS; i++)
        {
            if (s_manager.connections[i].connection_handle == connection_handle)
            {
                handle_found = true;
                break;
            }
        }
    }
    xSemaphoreGive(s_manager.lock);

    if (!handle_found)
    {
        ESP_LOGW(TAG, "Disconnect failed: Connection handle %u not found", connection_handle);
        return ESP_ERR_NOT_FOUND;
    }

    /*
     * 释放锁后提交 HCI 终止请求:
     * 错误码 0x13 (BLE_ERR_REM_USER_CONN_TERM) 代表远端用户主动挂断
     */
    int rc = ble_gap_terminate(connection_handle, BLE_ERR_REM_USER_CONN_TERM);
    if (rc != 0)
    {
        ESP_LOGE(TAG, "ble_gap_terminate failed, rc: %d", rc);
        return ESP_FAIL;
    }

    ESP_LOGI(TAG, "Disconnect request submitted for handle: %u", (unsigned int)connection_handle);
    return ESP_OK;
}

uint16_t ble_manager_get_connection_handle(void)
{
    uint16_t handle = BLE_MANAGER_INVALID_CONNECTION_HANDLE;
    if (s_manager.lock == NULL)
        return handle;

    xSemaphoreTake(s_manager.lock, portMAX_DELAY);

    // 遍历返回首个活动的物理连接句柄 (向后兼容单连接逻辑)
    for (int i = 0; i < BLE_MANAGER_MAX_CONNECTIONS; i++)
    {
        if (s_manager.connections[i].connection_handle != BLE_MANAGER_INVALID_CONNECTION_HANDLE)
        {
            handle = s_manager.connections[i].connection_handle;
            break;
        }
    }
    xSemaphoreGive(s_manager.lock);

    return handle;
}

uint8_t ble_manager_get_active_connections(uint16_t *handles, uint8_t max_count)
{
    if (handles == NULL || max_count == 0 || s_manager.lock == NULL)
    {
        return 0;
    }

    uint8_t copied = 0;
    xSemaphoreTake(s_manager.lock, portMAX_DELAY);
    for (int i = 0; i < BLE_MANAGER_MAX_CONNECTIONS && copied < max_count; i++)
    {
        if (s_manager.connections[i].connection_handle != BLE_MANAGER_INVALID_CONNECTION_HANDLE)
        {
            handles[copied++] = s_manager.connections[i].connection_handle;
        }
    }
    xSemaphoreGive(s_manager.lock);

    return copied;
}

bool ble_manager_is_connected(void)
{
    bool connected = false;
    if (s_manager.lock == NULL)
        return false;

    xSemaphoreTake(s_manager.lock, portMAX_DELAY);

    // 多连接模型：活动连接数大于 0 即视为已连接状态
    connected = (s_manager.state == BLE_MANAGER_STATE_RUNNING) &&
                (s_manager.active_connection_count > 0);
    xSemaphoreGive(s_manager.lock);

    return connected;
}

uint16_t ble_manager_get_mtu(uint16_t connection_handle)
{
    uint16_t mtu = 0;
    if (s_manager.lock == NULL || connection_handle == BLE_MANAGER_INVALID_CONNECTION_HANDLE)
    {
        return 0;
    }

    xSemaphoreTake(s_manager.lock, portMAX_DELAY);
    if (s_manager.state == BLE_MANAGER_STATE_RUNNING)
    {
        // 针对指定句柄的槽位检索其专属协商生效的 MTU 尺寸
        for (int i = 0; i < BLE_MANAGER_MAX_CONNECTIONS; i++)
        {
            if (s_manager.connections[i].connection_handle == connection_handle)
            {
                mtu = s_manager.connections[i].current_mtu;
                break;
            }
        }
    }
    xSemaphoreGive(s_manager.lock);

    return mtu;
}

uint8_t ble_manager_get_bonded_devices(ble_manager_bonded_device_t *bonded_devices,
                                       uint8_t maximum_count)
{
    if (bonded_devices == NULL || maximum_count == 0)
    {
        return 0;
    }

    if (s_manager.lock == NULL)
    {
        return 0;
    }

    xSemaphoreTake(s_manager.lock, portMAX_DELAY);
    if (s_manager.state != BLE_MANAGER_STATE_RUNNING)
    {
        xSemaphoreGive(s_manager.lock);
        return 0;
    }
    xSemaphoreGive(s_manager.lock);

    /*
     * 临时分配栈空间数组接收 NimBLE 底层存储的绑定信息，
     * 数量上限受限于宏 BLE_MANAGER_MAX_CONNECTIONS
     */
    ble_addr_t peer_addresses[BLE_MANAGER_MAX_CONNECTIONS];
    int query_limit = maximum_count;
    if (query_limit > BLE_MANAGER_MAX_CONNECTIONS)
    {
        query_limit = BLE_MANAGER_MAX_CONNECTIONS;
    }

    int actual_count = 0;
    int rc = ble_store_util_bonded_peers(peer_addresses, &actual_count, query_limit);
    if (rc != 0)
    {
        ESP_LOGE(TAG, "Failed to retrieve bonded peers from store, rc: %d", rc);
        return 0;
    }

    // 将 NimBLE 的 ble_addr_t 结构转换为统一的对外数据格式
    for (int i = 0; i < actual_count; i++)
    {
        memcpy(bonded_devices[i].address, peer_addresses[i].val, 6);
        bonded_devices[i].address_type = (peer_addresses[i].type == BLE_ADDR_PUBLIC)
                                             ? BLE_MANAGER_ADDRESS_TYPE_PUBLIC
                                             : BLE_MANAGER_ADDRESS_TYPE_RANDOM;
    }

    ESP_LOGI(TAG, "Retrieved %d bonded device(s) from storage", actual_count);
    return (uint8_t)actual_count;
}

esp_err_t ble_manager_clear_all_bonds(void)
{
    if (s_manager.lock == NULL)
    {
        return ESP_ERR_INVALID_STATE;
    }

    xSemaphoreTake(s_manager.lock, portMAX_DELAY);
    if (s_manager.state != BLE_MANAGER_STATE_RUNNING)
    {
        xSemaphoreGive(s_manager.lock);
        return ESP_ERR_INVALID_STATE;
    }
    xSemaphoreGive(s_manager.lock);

    // 清空 NVS 及内存中的全部配对密钥记录，恢复出厂未绑定状态
    int rc = ble_store_clear();
    if (rc != 0)
    {
        ESP_LOGE(TAG, "ble_store_clear failed, rc: %d", rc);
        return ESP_FAIL;
    }

    ESP_LOGI(TAG, "All bonding keys and credentials cleared successfully (Factory Reset)");
    return ESP_OK;
}

esp_err_t ble_manager_unpair_device(const uint8_t target_address[6],
                                    ble_manager_address_type_t address_type)
{
    if (target_address == NULL)
    {
        return ESP_ERR_INVALID_ARG;
    }

    if (s_manager.lock == NULL)
    {
        return ESP_ERR_INVALID_STATE;
    }

    xSemaphoreTake(s_manager.lock, portMAX_DELAY);
    if (s_manager.state != BLE_MANAGER_STATE_RUNNING)
    {
        xSemaphoreGive(s_manager.lock);
        return ESP_ERR_INVALID_STATE;
    }
    xSemaphoreGive(s_manager.lock);

    // 组装对端目标地址结构体
    ble_addr_t peer_address;
    memcpy(peer_address.val, target_address, 6);
    peer_address.type = (address_type == BLE_MANAGER_ADDRESS_TYPE_PUBLIC)
                            ? BLE_ADDR_PUBLIC
                            : BLE_ADDR_RANDOM;

    // 从底层安全数据库中解绑并删除该设备的长期密钥 (LTK)
    int rc = ble_gap_unpair(&peer_address);
    if (rc != 0)
    {
        ESP_LOGE(TAG, "ble_gap_unpair failed for target, rc: %d", rc);
        return ESP_FAIL;
    }

    ESP_LOGI(TAG, "Unpaired device [%02X:%02X:%02X:%02X:%02X:%02X] successfully",
             target_address[0], target_address[1], target_address[2],
             target_address[3], target_address[4], target_address[5]);

    return ESP_OK;
}

/* ========================================================================= */
/*  外围设备专属接口 (Role == BLE_MANAGER_ROLE_PERIPHERAL 时调用)             */
/* ========================================================================= */

// esp_err_t ble_manager_peripheral_register_services(const struct ble_gatt_svc_def *gatt_services)
// {
// }

esp_err_t ble_manager_peripheral_start_advertising(void)
{
    if (s_manager.lock == NULL)
    {
        return ESP_ERR_INVALID_STATE;
    }

    char name_buffer[sizeof(s_manager.device_name)];
    uint8_t current_active_count = 0; // 锁内原子快照，用于锁外无竞态日志打印

    // 1. 锁内校验系统状态与多连接配额
    xSemaphoreTake(s_manager.lock, portMAX_DELAY);
    if (s_manager.state != BLE_MANAGER_STATE_RUNNING)
    {
        xSemaphoreGive(s_manager.lock);
        ESP_LOGE(TAG, "Start advertising failed: Stack not running");
        return ESP_ERR_INVALID_STATE;
    }

    if (s_manager.role != BLE_MANAGER_ROLE_PERIPHERAL)
    {
        xSemaphoreGive(s_manager.lock);
        ESP_LOGE(TAG, "Start advertising failed: Device role is not peripheral");
        return ESP_ERR_NOT_SUPPORTED;
    }

    current_active_count = s_manager.active_connection_count;
    if (current_active_count >= BLE_MANAGER_MAX_CONNECTIONS)
    {
        xSemaphoreGive(s_manager.lock);
        ESP_LOGW(TAG, "Start advertising ignored: Max connections reached (%u/%u)",
                 current_active_count, BLE_MANAGER_MAX_CONNECTIONS);
        return ESP_ERR_INVALID_STATE;
    }

    strncpy(name_buffer, s_manager.device_name, sizeof(name_buffer));
    xSemaphoreGive(s_manager.lock);

    // 2. 检查广播是否已经在运行
    if (ble_gap_adv_active())
    {
        ESP_LOGW(TAG, "Advertising is already active");
        return ESP_OK;
    }

    /*
     * 3. 组装主广播包 (Adv Data):
     * 主广播包只放 3 字节的 Flags，为后续广播自定义服务 UUID 留出充足空间
     */
    struct ble_hs_adv_fields adv_fields;
    memset(&adv_fields, 0, sizeof(adv_fields));

    adv_fields.flags = BLE_HS_ADV_F_DISC_GEN | BLE_HS_ADV_F_BREDR_UNSUP;

    int rc = ble_gap_adv_set_fields(&adv_fields);
    if (rc != 0)
    {
        ESP_LOGE(TAG, "ble_gap_adv_set_fields failed, rc: %d", rc);
        return ESP_FAIL;
    }

    /*
     * 4. 组装扫描响应包 (Scan Response):
     * 标注官方规范 AD Type：
     *  <=29 字节采用 Complete Local Name (AD Type 0x09);
     *  >29 字节截断并采用 Shortened Local Name (AD Type 0x08)
     */
    struct ble_hs_adv_fields response_fields;
    memset(&response_fields, 0, sizeof(response_fields));

    size_t name_length = strlen(name_buffer);
    if (name_length > 29)
    {
        name_length = 29;
        response_fields.name_is_complete = 0; // 0: 简称 (Shortened Local Name, AD Type 0x08)
    }
    else
    {
        response_fields.name_is_complete = 1; // 全称 (Complete Local Name, AD Type 0x09)
    }
    response_fields.name = (uint8_t *)name_buffer;
    response_fields.name_len = name_length;

    rc = ble_gap_adv_rsp_set_fields(&response_fields);
    if (rc != 0)
    {
        ESP_LOGE(TAG, "ble_gap_adv_rsp_set_fields failed, rc: %d", rc);
        return ESP_FAIL;
    }

    /*
     * 5. 广播参数配置:
     *  CONN_MODE_UND: 发射标准 ADV_IND 报文，允许任意设备发起连接;
     *  DISC_MODE_GEN: 通用可发现模式;
     *  FAST_INTERVAL1: 范围 0x0030~0x0060，对应 30ms ~ 60ms 快速广播，步长 0.625ms
     */
    struct ble_gap_adv_params adv_params;
    memset(&adv_params, 0, sizeof(adv_params));
    adv_params.conn_mode = BLE_GAP_CONN_MODE_UND;         // 通用非定向可连接模式 (发射 ADV_IND，允许任意设备发起连接)
    adv_params.disc_mode = BLE_GAP_DISC_MODE_GEN;         // 通用可发现模式 (持续广播，允许被所有设备扫描发现)
    adv_params.itvl_min = BLE_GAP_ADV_FAST_INTERVAL1_MIN; // 最小广播间隔：30ms (0x0030，步长 0.625ms)
    adv_params.itvl_max = BLE_GAP_ADV_FAST_INTERVAL1_MAX; // 最大广播间隔：60ms (0x0060，步长 0.625ms)

    // 自动推导当前芯片配置的本端地址类型 (Public 或 Random)
    uint8_t own_addr_type;
    rc = ble_hs_id_infer_auto(0, &own_addr_type);
    if (rc != 0)
    {
        ESP_LOGE(TAG, "ble_hs_id_infer_auto failed, rc: %d", rc);
        return ESP_FAIL;
    }

    // 启动广播，并将 GAP 事件中枢绑定传入
    rc = ble_gap_adv_start(own_addr_type, NULL, BLE_HS_FOREVER, &adv_params,
                           ble_manager_gap_event_handler, NULL);
    if (rc != 0)
    {
        ESP_LOGE(TAG, "ble_gap_adv_start failed, rc: %d", rc);
        return ESP_FAIL;
    }

    ESP_LOGI(TAG, "Peripheral advertising started successfully");
    ble_manager_dispatch_event(BLE_MANAGER_EVENT_ADVERTISING_STARTED,
                               BLE_MANAGER_INVALID_CONNECTION_HANDLE,
                               NULL);
    return ESP_OK;
}

esp_err_t ble_manager_peripheral_stop_advertising(void)
{
    if (s_manager.lock == NULL)
    {
        return ESP_ERR_INVALID_STATE;
    }

    xSemaphoreTake(s_manager.lock, portMAX_DELAY);
    if (s_manager.state != BLE_MANAGER_STATE_RUNNING)
    {
        xSemaphoreGive(s_manager.lock);
        return ESP_ERR_INVALID_STATE;
    }
    xSemaphoreGive(s_manager.lock);

    // 幂等检查：未在广播则直接返回成功
    if (!ble_gap_adv_active())
    {
        ESP_LOGI(TAG, "Advertising is not active");
        return ESP_OK;
    }

    int rc = ble_gap_adv_stop();
    if (rc != 0)
    {
        ESP_LOGE(TAG, "ble_gap_adv_stop failed, rc: %d", rc);
        return ESP_FAIL;
    }

    ESP_LOGI(TAG, "Peripheral advertising stopped");
    ble_manager_dispatch_event(BLE_MANAGER_EVENT_ADVERTISING_STOPPED,
                               BLE_MANAGER_INVALID_CONNECTION_HANDLE,
                               NULL);
    return ESP_OK;
}

esp_err_t ble_manager_peripheral_notify(uint16_t connection_handle,
                                        uint16_t value_handle,
                                        const uint8_t *data,
                                        uint16_t length)
{
    if (data == NULL || length == 0)
    {
        return ESP_ERR_INVALID_ARG;
    }

    if (s_manager.lock == NULL)
    {
        return ESP_ERR_INVALID_STATE;
    }

    uint16_t target_mtu = 0;
    bool handle_valid = false;

    // 1. 锁内遍历槽位，验证目标句柄的活跃状态并抓取其专属 MTU
    xSemaphoreTake(s_manager.lock, portMAX_DELAY);
    if (s_manager.state != BLE_MANAGER_STATE_RUNNING)
    {
        xSemaphoreGive(s_manager.lock);
        return ESP_ERR_INVALID_STATE;
    }

    for (int i = 0; i < BLE_MANAGER_MAX_CONNECTIONS; i++)
    {
        if (s_manager.connections[i].connection_handle == connection_handle &&
            connection_handle != BLE_MANAGER_INVALID_CONNECTION_HANDLE)
        {
            target_mtu = s_manager.connections[i].current_mtu;
            handle_valid = true;
            break;
        }
    }
    xSemaphoreGive(s_manager.lock);

    if (!handle_valid)
    {
        ESP_LOGW(TAG, "Notify failed: Connection handle %u is not active", connection_handle);
        return ESP_ERR_NOT_FOUND;
    }

    /*
     *  单包 ATT Notification 格式为 Opcode(1B) + Handle(2B) + Value(变长)，
     *  故有效数据载荷最大不得超过当前连接的 MTU - 3 字节]
     */
    uint16_t max_payload = (target_mtu > 3) ? (target_mtu - 3) : 0;
    if (length > max_payload)
    {
        ESP_LOGE(TAG, "Notify failed: Payload length %u exceeds target MTU limit %u (MTU: %u)",
                 length, max_payload, target_mtu);
        return ESP_ERR_INVALID_SIZE;
    }

    // 2. 将平坦内存封装至 NimBLE 的网络内存缓冲链表 os_mbuf
    struct os_mbuf *om = ble_hs_mbuf_from_flat(data, length);
    if (om == NULL)
    {
        ESP_LOGE(TAG, "Notify failed: Out of memory for os_mbuf");
        return ESP_ERR_NO_MEM;
    }

    /*
     * 关键内存所有权法则：
     * ble_gatts_notify_custom 内部会无条件消费并释放传入的 om（无论调用成功还是失败）。
     * 外部绝不能手动执行 os_mbuf_free_chain(om)，否则必触发 Double Free 内存崩溃。
     */
    int rc = ble_gatts_notify_custom(connection_handle, value_handle, om);
    if (rc != 0)
    {
        ESP_LOGE(TAG, "ble_gatts_notify_custom failed, rc: %d", rc);
        return ESP_FAIL;
    }

    return ESP_OK;
}

/* ========================================================================= */
/*  中央设备专属接口 (Role == BLE_MANAGER_ROLE_CENTRAL 时调用)                */
/* ========================================================================= */

esp_err_t ble_manager_central_start_scanning(void)
{
    if (s_manager.lock == NULL)
    {
        return ESP_ERR_INVALID_STATE;
    }

    // 1. 锁内校验协议栈运行状态与角色
    xSemaphoreTake(s_manager.lock, portMAX_DELAY);
    if (s_manager.state != BLE_MANAGER_STATE_RUNNING)
    {
        xSemaphoreGive(s_manager.lock);
        ESP_LOGE(TAG, "Start scanning failed: Stack not running");
        return ESP_ERR_INVALID_STATE;
    }

    if (s_manager.role != BLE_MANAGER_ROLE_CENTRAL)
    {
        xSemaphoreGive(s_manager.lock);
        ESP_LOGE(TAG, "Start scanning failed: Device role is not central");
        return ESP_ERR_NOT_SUPPORTED;
    }
    xSemaphoreGive(s_manager.lock);

    // 2. 幂等检查：若协议栈已经在扫描，直接返回成功
    if (ble_gap_disc_active())
    {
        ESP_LOGI(TAG, "Scanning is already active");
        return ESP_OK;
    }

    // 3. 配置扫描参数
    struct ble_gap_disc_params discovery_parameters;
    memset(&discovery_parameters, 0, sizeof(discovery_parameters));

    discovery_parameters.passive = 0;           // 主动扫描：发送 Scan Request 获取对端的扫描响应包 (包含设备名)
    discovery_parameters.filter_duplicates = 1; // 硬件去重：过滤同一设备的重复广播包，防止回调频繁触发
    discovery_parameters.filter_policy = 0;     // 0: 接受所有广播，不使用白名单过滤
    discovery_parameters.limited = 0;           // 0: 扫描所有可发现模式的设备

    /*
     * 快速扫描配置：单位为 0.625ms
     * itvl = 48 (30ms), window = 48 (30ms)，占空比 100% 满负荷连续侦听
     */
    discovery_parameters.itvl = BLE_GAP_SCAN_FAST_INTERVAL_MIN;
    discovery_parameters.window = BLE_GAP_SCAN_FAST_WINDOW;

    uint8_t own_addr_type;
    int rc = ble_hs_id_infer_auto(0, &own_addr_type);
    if (rc != 0)
    {
        ESP_LOGE(TAG, "ble_hs_id_infer_auto failed, rc: %d", rc);
        return ESP_FAIL;
    }

    // 4. 向 NimBLE 提交扫描指令
    rc = ble_gap_disc(own_addr_type, BLE_HS_FOREVER, &discovery_parameters,
                      ble_manager_gap_event_handler, NULL);

    if (rc == BLE_HS_EALREADY)
    {
        ESP_LOGI(TAG, "Scanning already started by another task");
        return ESP_OK;
    }
    else if (rc != 0)
    {
        ESP_LOGE(TAG, "ble_gap_disc failed, rc: %d", rc);
        return ESP_FAIL;
    }

    ESP_LOGI(TAG, "Central scanning started successfully");
    ble_manager_dispatch_event(BLE_MANAGER_EVENT_SCANNING_STARTED,
                               BLE_MANAGER_INVALID_CONNECTION_HANDLE,
                               NULL);
    return ESP_OK;
}

esp_err_t ble_manager_central_stop_scanning(void)
{
    if (s_manager.lock == NULL)
    {
        return ESP_ERR_INVALID_STATE;
    }

    xSemaphoreTake(s_manager.lock, portMAX_DELAY);
    if (s_manager.state != BLE_MANAGER_STATE_RUNNING)
    {
        xSemaphoreGive(s_manager.lock);
        return ESP_ERR_INVALID_STATE;
    }
    xSemaphoreGive(s_manager.lock);

    // 若本来就没在扫描，直接幂等返回成功
    if (!ble_gap_disc_active())
    {
        ESP_LOGI(TAG, "Scanning is not active");
        return ESP_OK;
    }

    // 调用取消扫描 API
    int rc = ble_gap_disc_cancel();
    if (rc != 0)
    {
        ESP_LOGE(TAG, "ble_gap_disc_cancel failed, rc: %d", rc);
        return ESP_FAIL;
    }

    ESP_LOGI(TAG, "Central scanning cancel requested");

    /* 最终扫描停止事件在 GAP_EVENT_DISC_COMPLETE 中统一派发，保证时序闭环 */
    return ESP_OK;
}

esp_err_t ble_manager_central_connect(const uint8_t target_address[6],
                                      ble_manager_address_type_t address_type)
{
    // 1. 入参校验
    if (target_address == NULL)
    {
        return ESP_ERR_INVALID_ARG;
    }

    if (address_type != BLE_MANAGER_ADDRESS_TYPE_PUBLIC &&
        address_type != BLE_MANAGER_ADDRESS_TYPE_RANDOM)
    {
        ESP_LOGE(TAG, "Connect failed: Invalid address type %d", address_type);
        return ESP_ERR_INVALID_ARG;
    }

    if (s_manager.lock == NULL)
    {
        return ESP_ERR_INVALID_STATE;
    }

    // 2. 状态与配额校验
    xSemaphoreTake(s_manager.lock, portMAX_DELAY);
    if (s_manager.state != BLE_MANAGER_STATE_RUNNING)
    {
        xSemaphoreGive(s_manager.lock);
        ESP_LOGE(TAG, "Connect failed: Stack not running");
        return ESP_ERR_INVALID_STATE;
    }

    if (s_manager.role != BLE_MANAGER_ROLE_CENTRAL)
    {
        xSemaphoreGive(s_manager.lock);
        ESP_LOGE(TAG, "Connect failed: Device role is not central");
        return ESP_ERR_NOT_SUPPORTED;
    }

    if (s_manager.active_connection_count >= BLE_MANAGER_MAX_CONNECTIONS)
    {
        xSemaphoreGive(s_manager.lock);
        ESP_LOGW(TAG, "Connect failed: Max connections reached (%u/%u)",
                 s_manager.active_connection_count, BLE_MANAGER_MAX_CONNECTIONS);
        return ESP_ERR_INVALID_STATE;
    }
    xSemaphoreGive(s_manager.lock);

    /*
     *  明确拒绝在底层隐式偷停扫描。要求调用方显式先调用 stop_scanning，
     *  保证扫描和连接两个异步状态机边界清晰，彻底消除硬件重叠竞态
     */
    if (ble_gap_disc_active())
    {
        ESP_LOGE(TAG, "Connect failed: Scanning is active. Please call ble_manager_central_stop_scanning() first!");
        return ESP_ERR_INVALID_STATE;
    }

    // 3. 组装目标地址结构体
    ble_addr_t peer_address;
    memcpy(peer_address.val, target_address, 6);
    peer_address.type = (address_type == BLE_MANAGER_ADDRESS_TYPE_PUBLIC)
                            ? BLE_ADDR_PUBLIC
                            : BLE_ADDR_RANDOM;

    /*
     * 4. 配置物理连接参数:
     *  物理单位换算：
     *  scan_itvl / scan_window: 步长 0.625ms (48 -> 30ms 快速扫描);
     *  itvl_min / itvl_max: 步长 1.25ms (24 -> 30ms, 40 -> 50ms 初始连接间隔);
     *  supervision_timeout: 步长 10ms (256 -> 2560ms = 2.56s 监督超时)]
     */
    struct ble_gap_conn_params connection_parameters;
    memset(&connection_parameters, 0, sizeof(connection_parameters));
    connection_parameters.scan_itvl = BLE_GAP_SCAN_FAST_INTERVAL_MIN;                // 30ms
    connection_parameters.scan_window = BLE_GAP_SCAN_FAST_WINDOW;                    // 30ms
    connection_parameters.itvl_min = BLE_GAP_INITIAL_CONN_ITVL_MIN;                  // 30ms
    connection_parameters.itvl_max = BLE_GAP_INITIAL_CONN_ITVL_MAX;                  // 50ms
    connection_parameters.latency = 0;                                               // 不允许从机延迟
    connection_parameters.supervision_timeout = BLE_GAP_INITIAL_SUPERVISION_TIMEOUT; // 2.56s

    uint8_t own_addr_type;
    int rc = ble_hs_id_infer_auto(0, &own_addr_type);
    if (rc != 0)
    {
        ESP_LOGE(TAG, "ble_hs_id_infer_auto failed, rc: %d", rc);
        return ESP_FAIL;
    }

    // 5. 发起连接：
    rc = ble_gap_connect(own_addr_type, &peer_address, 5000, &connection_parameters,
                         ble_manager_gap_event_handler, NULL);
    if (rc != 0)
    {
        ESP_LOGE(TAG, "ble_gap_connect failed, rc: %d", rc);
        return (rc == BLE_HS_EALREADY) ? ESP_ERR_INVALID_STATE : ESP_FAIL;
    }

    ESP_LOGI(TAG, "Connection initiated to [%02x:%02x:%02x:%02x:%02x:%02x]",
             target_address[0], target_address[1], target_address[2],
             target_address[3], target_address[4], target_address[5]);

    return ESP_OK;
}

/**
 * @brief GATT MTU 协商完成异步回调
 */
static int ble_manager_on_gatt_mtu_exchange_callback(uint16_t connection_handle,
                                                     const struct ble_gatt_error *error,
                                                     uint16_t mtu,
                                                     void *arg)
{
    if (error == NULL)
    {
        ESP_LOGE(TAG, "MTU exchange callback: error struct is NULL");
        return BLE_HS_EUNKNOWN;
    }

    if (error->status == 0)
    {
        ESP_LOGI(TAG, "GATT MTU exchange successful: handle=%u, negotiated_mtu=%u", connection_handle, mtu);
    }
    else
    {
        ESP_LOGW(TAG, "GATT MTU exchange failed: handle=%u, status=%d", connection_handle, error->status);
    }

    /*
     * 架构设计规范：
     * 全局槽位更新与 BLE_MANAGER_EVENT_MTU_UPDATED 事件
     * 已经在底层的 GAP_EVENT_MTU 中统一处理并派发，此处切勿重复派发！
     */
    return 0;
}

esp_err_t ble_manager_central_exchange_mtu(uint16_t connection_handle)
{
    if (connection_handle == BLE_MANAGER_INVALID_CONNECTION_HANDLE)
    {
        return ESP_ERR_INVALID_ARG;
    }

    if (s_manager.lock == NULL)
    {
        return ESP_ERR_INVALID_STATE;
    }

    bool handle_valid = false;

    // 1. 锁内校验系统状态、角色与槽位合法性
    xSemaphoreTake(s_manager.lock, portMAX_DELAY);
    if (s_manager.state != BLE_MANAGER_STATE_RUNNING)
    {
        xSemaphoreGive(s_manager.lock);
        ESP_LOGE(TAG, "Exchange MTU failed: Stack not running");
        return ESP_ERR_INVALID_STATE;
    }

    if (s_manager.role != BLE_MANAGER_ROLE_CENTRAL)
    {
        xSemaphoreGive(s_manager.lock);
        ESP_LOGE(TAG, "Exchange MTU failed: Device role is not central");
        return ESP_ERR_NOT_SUPPORTED;
    }

    for (int i = 0; i < BLE_MANAGER_MAX_CONNECTIONS; i++)
    {
        if (s_manager.connections[i].connection_handle == connection_handle)
        {
            handle_valid = true;
            break;
        }
    }
    xSemaphoreGive(s_manager.lock);

    if (!handle_valid)
    {
        ESP_LOGW(TAG, "Exchange MTU failed: Connection handle %u not found", connection_handle);
        return ESP_ERR_NOT_FOUND;
    }

    // 2. 释放锁后调用 NimBLE 原生发起 MTU 协商 API
    int rc = ble_gattc_exchange_mtu(connection_handle, ble_manager_on_gatt_mtu_exchange_callback, NULL);

    // 3. 精细化处理错误码，避免统一粗暴返回 ESP_FAIL
    if (rc == BLE_HS_EALREADY)
    {
        /*
         * 蓝牙核心规范规定：一条连接上 MTU 协商只能执行一次。
         * 若对端已先一步发起或当前正在协商中，返回 INVALID_STATE，避免上层判定为严重故障。
         */
        ESP_LOGW(TAG, "MTU exchange already in progress or already completed for handle: %u", connection_handle);
        return ESP_ERR_INVALID_STATE;
    }
    else if (rc == BLE_HS_ENOTCONN)
    {
        ESP_LOGW(TAG, "Connection handle %u no longer exists", connection_handle);
        return ESP_ERR_NOT_FOUND;
    }
    else if (rc != 0)
    {
        ESP_LOGE(TAG, "ble_gattc_exchange_mtu failed on handle %u, rc: %d", connection_handle, rc);
        return ESP_FAIL;
    }

    ESP_LOGI(TAG, "MTU exchange initiated successfully on handle %u", connection_handle);
    return ESP_OK;
}

esp_err_t ble_manager_central_update_connection_parameters(uint16_t connection_handle,
                                                           uint16_t interval_minimum,
                                                           uint16_t interval_maximum,
                                                           uint16_t latency,
                                                           uint16_t supervision_timeout)
{
    if (connection_handle == BLE_MANAGER_INVALID_CONNECTION_HANDLE)
    {
        return ESP_ERR_INVALID_ARG;
    }

    /*
     * 蓝牙 Core 规范数值有效性防呆校验：
     * interval 范围在 6 (7.5ms) ~ 3200 (4.0s)
     * timeout 范围在 10 (100ms) ~ 3200 (32.0s)
     * interval_maximum 必须 >= interval_minimum
     */
    if (interval_minimum < 6 || interval_maximum > 3200 || interval_minimum > interval_maximum ||
        supervision_timeout < 10 || supervision_timeout > 3200)
    {
        ESP_LOGE(TAG, "Update connection parameters failed: Invalid parameters range");
        return ESP_ERR_INVALID_ARG;
    }

    if (s_manager.lock == NULL)
    {
        return ESP_ERR_INVALID_STATE;
    }

    bool handle_valid = false;

    // 1. 锁内校验系统状态、角色与槽位有效性
    xSemaphoreTake(s_manager.lock, portMAX_DELAY);
    if (s_manager.state != BLE_MANAGER_STATE_RUNNING)
    {
        xSemaphoreGive(s_manager.lock);
        ESP_LOGE(TAG, "Update connection parameters failed: Stack not running");
        return ESP_ERR_INVALID_STATE;
    }

    if (s_manager.role != BLE_MANAGER_ROLE_CENTRAL)
    {
        xSemaphoreGive(s_manager.lock);
        ESP_LOGE(TAG, "Update connection parameters failed: Device role is not central");
        return ESP_ERR_NOT_SUPPORTED;
    }

    for (int i = 0; i < BLE_MANAGER_MAX_CONNECTIONS; i++)
    {
        if (s_manager.connections[i].connection_handle == connection_handle)
        {
            handle_valid = true;
            break;
        }
    }
    xSemaphoreGive(s_manager.lock);

    if (!handle_valid)
    {
        ESP_LOGW(TAG, "Update connection parameters failed: Connection handle %u not found", connection_handle);
        return ESP_ERR_NOT_FOUND;
    }

    // 2. 组装 NimBLE GAP 参数更新结构体
    struct ble_gap_upd_params update_parameters;
    memset(&update_parameters, 0, sizeof(update_parameters));
    update_parameters.itvl_min = interval_minimum;
    update_parameters.itvl_max = interval_maximum;
    update_parameters.latency = latency;
    update_parameters.supervision_timeout = supervision_timeout;
    update_parameters.min_ce_len = 0;
    update_parameters.max_ce_len = 0;

    // 3. 释放锁后提交连接参数更新请求
    int rc = ble_gap_update_params(connection_handle, &update_parameters);
    if (rc != 0)
    {
        ESP_LOGE(TAG, "ble_gap_update_params failed on handle %u, rc: %d", connection_handle, rc);
        return ESP_FAIL;
    }

    ESP_LOGI(TAG, "Connection parameters update initiated on handle %u [interval: %.2f~%.2f ms, latency: %u, timeout: %u ms]",
             connection_handle, interval_minimum * 1.25f, interval_maximum * 1.25f, latency, supervision_timeout * 10);

    return ESP_OK;
}

esp_err_t ble_manager_central_start_security(uint16_t connection_handle)
{
    if (connection_handle == BLE_MANAGER_INVALID_CONNECTION_HANDLE)
    {
        return ESP_ERR_INVALID_ARG;
    }

    if (s_manager.lock == NULL)
    {
        return ESP_ERR_INVALID_STATE;
    }

    bool handle_valid = false;

    // 1. 锁内校验系统状态、角色与槽位有效性
    xSemaphoreTake(s_manager.lock, portMAX_DELAY);
    if (s_manager.state != BLE_MANAGER_STATE_RUNNING)
    {
        xSemaphoreGive(s_manager.lock);
        ESP_LOGE(TAG, "Start security failed: Stack not running");
        return ESP_ERR_INVALID_STATE;
    }

    if (s_manager.role != BLE_MANAGER_ROLE_CENTRAL)
    {
        xSemaphoreGive(s_manager.lock);
        ESP_LOGE(TAG, "Start security failed: Device role is not central");
        return ESP_ERR_NOT_SUPPORTED;
    }

    for (int i = 0; i < BLE_MANAGER_MAX_CONNECTIONS; i++)
    {
        if (s_manager.connections[i].connection_handle == connection_handle)
        {
            handle_valid = true;
            break;
        }
    }
    xSemaphoreGive(s_manager.lock);

    if (!handle_valid)
    {
        ESP_LOGW(TAG, "Start security failed: Connection handle %u not found", connection_handle);
        return ESP_ERR_NOT_FOUND;
    }

    // 2. 释放锁后调用 GAP 原生发起安全请求 API
    int rc = ble_gap_security_initiate(connection_handle);

    if (rc == BLE_HS_EALREADY)
    {
        ESP_LOGW(TAG, "Security procedure already in progress for handle: %u", connection_handle);
        return ESP_ERR_INVALID_STATE;
    }
    else if (rc == BLE_HS_ENOTCONN)
    {
        ESP_LOGW(TAG, "Connection handle %u no longer exists", connection_handle);
        return ESP_ERR_NOT_FOUND;
    }
    else if (rc != 0)
    {
        ESP_LOGE(TAG, "ble_gap_security_initiate failed on handle %u, rc: %d", connection_handle, rc);
        return ESP_FAIL;
    }

    ESP_LOGI(TAG, "Security initiation requested successfully on handle %u", connection_handle);
    return ESP_OK;
}

/* ========================================================================= */
/* GATT 客户端发现服务异步回调                                                */
/* ========================================================================= */

static int ble_manager_on_discovery_service_callback(uint16_t connection_handle,
                                                     const struct ble_gatt_error *error,
                                                     const struct ble_gatt_svc *service,
                                                     void *arg)
{
    // 1. 防御性判空
    if (error == NULL)
    {
        ESP_LOGE(TAG, "Service discovery callback: error struct is NULL");
        return BLE_HS_EUNKNOWN;
    }

    // 2. 发现单个主服务：service 指针有效且 status 为 0
    if (error->status == 0 && service != NULL)
    {
        ble_manager_discovered_service_t discovered_service;
        memset(&discovered_service, 0, sizeof(discovered_service));

        discovered_service.start_handle = service->start_handle;
        discovered_service.end_handle = service->end_handle;

        // 解析服务 UUID (区分 16 位标准短码与 128 位厂商长码)
        if (service->uuid.u.type == BLE_UUID_TYPE_16)
        {
            discovered_service.uuid.type = BLE_MANAGER_UUID_TYPE_16;
            discovered_service.uuid.value.uuid16 = service->uuid.u16.value;
        }
        else if (service->uuid.u.type == BLE_UUID_TYPE_128)
        {
            discovered_service.uuid.type = BLE_MANAGER_UUID_TYPE_128;
            memcpy(discovered_service.uuid.value.uuid128, service->uuid.u128.value, 16);
        }
        else
        {
            // 过滤非 16位/128位 的异常 UUID 类型 (如 32位 UUID)，记录告警并忽略
            ESP_LOGW(TAG, "Skip unsupported service UUID type: %u", service->uuid.u.type);
            return 0;
        }

        // 派发单服务发现事件
        ble_manager_dispatch_event(BLE_MANAGER_EVENT_SERVICE_DISCOVERED,
                                   connection_handle,
                                   &discovered_service);
        return 0;
    }

    /*
     * 3. 发现流程结束 (此时 service 指针为 NULL)：
     *  NimBLE 在搜索完所有主服务后返回状态码 BLE_HS_EDONE (14)，
     *  这是标准流程结束标志，必须映射为通用成功码 ESP_OK (0)，避免上层误判为通信故障
     */
    ble_manager_async_result_t result;
    memset(&result, 0, sizeof(result));

    if (error->status == BLE_HS_EDONE || (error->status == 0 && service == NULL))
    {
        result.status = ESP_OK; // 正常搜寻完毕
    }
    else
    {
        result.status = error->status; //  链路异常中断错误码
    }
    result.attribute_handle = 0;

    ESP_LOGI(TAG, "Service discovery finished on handle %u, status: %d (NimBLE raw: %d)",
             connection_handle, result.status, error->status);

    ble_manager_dispatch_event(BLE_MANAGER_EVENT_SERVICE_DISCOVERY_COMPLETED,
                               connection_handle,
                               &result);
    return 0;
}

esp_err_t ble_manager_central_discover_services(uint16_t connection_handle)
{
    if (connection_handle == BLE_MANAGER_INVALID_CONNECTION_HANDLE)
    {
        return ESP_ERR_INVALID_ARG;
    }

    if (s_manager.lock == NULL)
    {
        return ESP_ERR_INVALID_STATE;
    }

    bool handle_valid = false;

    // 1. 锁内校验系统状态、角色及连接有效性
    xSemaphoreTake(s_manager.lock, portMAX_DELAY);
    if (s_manager.state != BLE_MANAGER_STATE_RUNNING)
    {
        xSemaphoreGive(s_manager.lock);
        ESP_LOGE(TAG, "Discover services failed: Stack not running");
        return ESP_ERR_INVALID_STATE;
    }

    if (s_manager.role != BLE_MANAGER_ROLE_CENTRAL)
    {
        xSemaphoreGive(s_manager.lock);
        ESP_LOGE(TAG, "Discover services failed: Device role is not central");
        return ESP_ERR_NOT_SUPPORTED;
    }

    for (int i = 0; i < BLE_MANAGER_MAX_CONNECTIONS; i++)
    {
        if (s_manager.connections[i].connection_handle == connection_handle)
        {
            handle_valid = true;
            break;
        }
    }
    xSemaphoreGive(s_manager.lock);

    if (!handle_valid)
    {
        ESP_LOGW(TAG, "Discover services failed: Connection handle %u not found", connection_handle);
        return ESP_ERR_NOT_FOUND;
    }

    // 2. 释放锁后调用 NimBLE 原生主服务发现 API (底层发射 ATT Read By Group Type Request)
    int rc = ble_gattc_disc_all_svcs(connection_handle,
                                     ble_manager_on_discovery_service_callback,
                                     NULL);
    if (rc != 0)
    {
        ESP_LOGE(TAG, "ble_gattc_disc_all_svcs failed on handle %u, rc: %d", connection_handle, rc);
        return ESP_FAIL;
    }

    ESP_LOGI(TAG, "Discovering all primary services on handle %u...", connection_handle);
    return ESP_OK;
}

/* ========================================================================= */
/* GATT 客户端发现特性异步回调                                                */
/* ========================================================================= */

static int ble_manager_on_discovery_characteristic_callback(uint16_t connection_handle,
                                                            const struct ble_gatt_error *error,
                                                            const struct ble_gatt_chr *characteristic,
                                                            void *arg)
{
    // 1. 防御性判空
    if (error == NULL)
    {
        ESP_LOGE(TAG, "Characteristic discovery callback: error struct is NULL");
        return BLE_HS_EUNKNOWN;
    }

    // 2. 发现单个特性：characteristic 指针有效且 status 为 0
    if (error->status == 0 && characteristic != NULL)
    {
        ble_manager_discovered_characteristic_t discovered_characteristic;
        memset(&discovered_characteristic, 0, sizeof(discovered_characteristic));

        // 提取 0x2803 声明句柄、数值句柄 (Value Handle) 与性质标志位 (Properties)
        discovered_characteristic.declaration_handle = characteristic->def_handle;
        discovered_characteristic.value_handle = characteristic->val_handle;
        discovered_characteristic.properties = characteristic->properties;

        // 解析特性 UUID
        if (characteristic->uuid.u.type == BLE_UUID_TYPE_16)
        {
            discovered_characteristic.uuid.type = BLE_MANAGER_UUID_TYPE_16;
            discovered_characteristic.uuid.value.uuid16 = characteristic->uuid.u16.value;
        }
        else if (characteristic->uuid.u.type == BLE_UUID_TYPE_128)
        {
            discovered_characteristic.uuid.type = BLE_MANAGER_UUID_TYPE_128;
            memcpy(discovered_characteristic.uuid.value.uuid128, characteristic->uuid.u128.value, 16);
        }
        else
        {
            ESP_LOGW(TAG, "Skip unsupported characteristic UUID type: %u", characteristic->uuid.u.type);
            return 0;
        }

        // 派发单特性发现事件
        ble_manager_dispatch_event(BLE_MANAGER_EVENT_CHARACTERISTIC_DISCOVERED,
                                   connection_handle,
                                   &discovered_characteristic);
        return 0;
    }

    // 3. 特性发现流程结束 (characteristic == NULL)
    ble_manager_async_result_t result;
    memset(&result, 0, sizeof(result));

    if (error->status == BLE_HS_EDONE || (error->status == 0 && characteristic == NULL))
    {
        result.status = ESP_OK; // 正常搜寻结束
    }
    else
    {
        result.status = error->status; // 链路异常报错
    }
    result.attribute_handle = 0;

    ESP_LOGI(TAG, "Characteristic discovery finished on handle %u, status: %d (NimBLE raw: %d)",
             connection_handle, result.status, error->status);

    // 派发全部特性发现完成事件
    ble_manager_dispatch_event(BLE_MANAGER_EVENT_CHARACTERISTIC_DISCOVERY_COMPLETED,
                               connection_handle,
                               &result);
    return 0;
}

esp_err_t ble_manager_central_discover_characteristics(uint16_t connection_handle,
                                                       uint16_t start_handle,
                                                       uint16_t end_handle)
{
    if (connection_handle == BLE_MANAGER_INVALID_CONNECTION_HANDLE)
    {
        return ESP_ERR_INVALID_ARG;
    }

    /*
     *  属性句柄合法区间为 0x0001~0xFFFF，
     *  起始句柄必须严格 <= 结束句柄，否则底层协议栈会直接报参数错误
     */
    if (start_handle == 0 || start_handle > end_handle)
    {
        ESP_LOGE(TAG, "Discover characteristics failed: Invalid handle range [0x%04x - 0x%04x]",
                 start_handle, end_handle);
        return ESP_ERR_INVALID_ARG;
    }

    if (s_manager.lock == NULL)
    {
        return ESP_ERR_INVALID_STATE;
    }

    bool handle_valid = false;

    // 1. 锁内校验系统状态、角色与槽位有效性
    xSemaphoreTake(s_manager.lock, portMAX_DELAY);
    if (s_manager.state != BLE_MANAGER_STATE_RUNNING)
    {
        xSemaphoreGive(s_manager.lock);
        ESP_LOGE(TAG, "Discover characteristics failed: Stack not running");
        return ESP_ERR_INVALID_STATE;
    }

    if (s_manager.role != BLE_MANAGER_ROLE_CENTRAL)
    {
        xSemaphoreGive(s_manager.lock);
        ESP_LOGE(TAG, "Discover characteristics failed: Device role is not central");
        return ESP_ERR_NOT_SUPPORTED;
    }

    for (int i = 0; i < BLE_MANAGER_MAX_CONNECTIONS; i++)
    {
        if (s_manager.connections[i].connection_handle == connection_handle)
        {
            handle_valid = true;
            break;
        }
    }
    xSemaphoreGive(s_manager.lock);

    if (!handle_valid)
    {
        ESP_LOGW(TAG, "Discover characteristics failed: Connection handle %u not found", connection_handle);
        return ESP_ERR_NOT_FOUND;
    }

    // 2. 释放锁后调用 NimBLE 原生特性发现 API (底层发射 ATT Read By Type Request，搜寻 0x2803)
    int rc = ble_gattc_disc_all_chrs(connection_handle,
                                     start_handle,
                                     end_handle,
                                     ble_manager_on_discovery_characteristic_callback,
                                     NULL);
    if (rc != 0)
    {
        ESP_LOGE(TAG, "ble_gattc_disc_all_chrs failed on handle %u, rc: %d", connection_handle, rc);
        return ESP_FAIL;
    }

    ESP_LOGI(TAG, "Discovering characteristics on handle %u, range: [0x%04x - 0x%04x]...",
             connection_handle, start_handle, end_handle);
    return ESP_OK;
}

/* ========================================================================= */
/* GATT 客户端发现描述符异步回调与实现接口                                     */
/* ========================================================================= */

static int ble_manager_on_discovery_descriptor_callback(uint16_t connection_handle,
                                                        const struct ble_gatt_error *error,
                                                        uint16_t characteristic_value_handle,
                                                        const struct ble_gatt_dsc *descriptor,
                                                        void *arg)
{
    // 1. 防御性判空
    if (error == NULL)
    {
        ESP_LOGE(TAG, "Descriptor discovery callback: error struct is NULL");
        return BLE_HS_EUNKNOWN;
    }

    // 2. 发现单个描述符：descriptor 指针有效且 status 为 0
    if (error->status == 0 && descriptor != NULL)
    {
        ble_manager_discovered_descriptor_t discovered_descriptor;
        memset(&discovered_descriptor, 0, sizeof(discovered_descriptor));

        discovered_descriptor.descriptor_handle = descriptor->handle;

        // 解析描述符 UUID (16位 或 128位)
        if (descriptor->uuid.u.type == BLE_UUID_TYPE_16)
        {
            discovered_descriptor.uuid.type = BLE_MANAGER_UUID_TYPE_16;
            discovered_descriptor.uuid.value.uuid16 = descriptor->uuid.u16.value;
        }
        else if (descriptor->uuid.u.type == BLE_UUID_TYPE_128)
        {
            discovered_descriptor.uuid.type = BLE_MANAGER_UUID_TYPE_128;
            memcpy(discovered_descriptor.uuid.value.uuid128, descriptor->uuid.u128.value, 16);
        }
        else
        {
            ESP_LOGW(TAG, "Skip unsupported descriptor UUID type: %u", descriptor->uuid.u.type);
            return 0;
        }

        // 派发单描述符发现事件
        ble_manager_dispatch_event(BLE_MANAGER_EVENT_DESCRIPTOR_DISCOVERED,
                                   connection_handle,
                                   &discovered_descriptor);
        return 0;
    }

    // 3. 描述符发现流程结束
    ble_manager_async_result_t result;
    memset(&result, 0, sizeof(result));

    result.status = (error->status == BLE_HS_EDONE) ? ESP_OK : error->status;
    result.attribute_handle = characteristic_value_handle; // 明确回传所属的特性数值句柄

    ESP_LOGI(TAG, "Descriptor discovery finished on handle %u, status: %d (NimBLE raw: %d)",
             connection_handle, result.status, error->status);

    // 派发全部描述符发现完成事件
    ble_manager_dispatch_event(BLE_MANAGER_EVENT_DESCRIPTOR_DISCOVERY_COMPLETED,
                               connection_handle,
                               &result);
    return 0;
}

esp_err_t ble_manager_central_discover_descriptors(uint16_t connection_handle,
                                                   uint16_t characteristic_value_handle,
                                                   uint16_t characteristic_end_handle)
{
    if (connection_handle == BLE_MANAGER_INVALID_CONNECTION_HANDLE)
    {
        return ESP_ERR_INVALID_ARG;
    }

    // 描述符必定位于特性数值之后，若起始句柄 >= 结束句柄说明无搜索空间
    if (characteristic_value_handle == 0 || characteristic_value_handle >= characteristic_end_handle)
    {
        ESP_LOGE(TAG, "Discover descriptors failed: Invalid range [value: 0x%04x >= end: 0x%04x]",
                 characteristic_value_handle, characteristic_end_handle);
        return ESP_ERR_INVALID_ARG;
    }

    if (s_manager.lock == NULL)
    {
        return ESP_ERR_INVALID_STATE;
    }

    bool handle_valid = false;

    // 2. 锁内校验系统状态、角色与槽位有效性
    xSemaphoreTake(s_manager.lock, portMAX_DELAY);
    if (s_manager.state != BLE_MANAGER_STATE_RUNNING)
    {
        xSemaphoreGive(s_manager.lock);
        ESP_LOGE(TAG, "Discover descriptors failed: Stack not running");
        return ESP_ERR_INVALID_STATE;
    }

    if (s_manager.role != BLE_MANAGER_ROLE_CENTRAL)
    {
        xSemaphoreGive(s_manager.lock);
        ESP_LOGE(TAG, "Discover descriptors failed: Device role is not central");
        return ESP_ERR_NOT_SUPPORTED;
    }

    for (int i = 0; i < BLE_MANAGER_MAX_CONNECTIONS; i++)
    {
        if (s_manager.connections[i].connection_handle == connection_handle)
        {
            handle_valid = true;
            break;
        }
    }
    xSemaphoreGive(s_manager.lock);

    if (!handle_valid)
    {
        ESP_LOGW(TAG, "Discover descriptors failed: Connection handle %u not found", connection_handle);
        return ESP_ERR_NOT_FOUND;
    }

    // 3. 释放锁后调用 NimBLE 原生描述符检索 API (底层发射 ATT Find Information Request)
    int rc = ble_gattc_disc_all_dscs(connection_handle,
                                     characteristic_value_handle,
                                     characteristic_end_handle,
                                     ble_manager_on_discovery_descriptor_callback,
                                     NULL);
    if (rc != 0)
    {
        ESP_LOGE(TAG, "ble_gattc_disc_all_dscs failed on handle %u, rc: %d", connection_handle, rc);
        return ESP_FAIL;
    }

    ESP_LOGI(TAG, "Discovering descriptors on handle %u, characteristic_value_handle: 0x%04x, characteristic_end_handle: 0x%04x...",
             connection_handle, characteristic_value_handle, characteristic_end_handle);
    return ESP_OK;
}

/* ========================================================================= */
/* GATT 客户端读/写异步回调与业务接口                                          */
/* ========================================================================= */

/**
 * @brief GATT 读取特性值底层异步回调
 */
static int ble_manager_on_gatt_read_callback(uint16_t connection_handle,
                                             const struct ble_gatt_error *error,
                                             struct ble_gatt_attr *attr,
                                             void *arg)
{
    if (error == NULL)
    {
        ESP_LOGE(TAG, "GATT read callback: error struct is NULL");
        return BLE_HS_EUNKNOWN;
    }

    ble_manager_read_result_t read_result;
    memset(&read_result, 0, sizeof(read_result));

    // 517 字节安全上限：满足 BLE 4.2+ 最大的 ATT MTU 规格 (512 字节载荷 + 5 字节报头)
    uint8_t payload_buffer[517];
    uint16_t data_length = 0;

    if (error->status == 0 && attr != NULL && attr->om != NULL)
    {
        read_result.attribute_handle = attr->handle;
        data_length = OS_MBUF_PKTLEN(attr->om);

        if (data_length > sizeof(payload_buffer))
        {
            ESP_LOGE(TAG, "Read data too large: %u bytes", data_length);
            read_result.status = BLE_HS_EMSGSIZE; // 数据尺寸超限错误 (超过 517B 本地缓冲区)
            read_result.data = NULL;
            read_result.length = 0;
        }
        else
        {
            int copy_rc = os_mbuf_copydata(attr->om, 0, data_length, payload_buffer);
            if (copy_rc != 0)
            {
                ESP_LOGE(TAG, "Failed to copy read data from mbuf, rc: %d", copy_rc);
                read_result.status = BLE_HS_EUNKNOWN; // 未知内部错误 (mbuf 网络缓冲链表拷贝失败)
                read_result.data = NULL;
                read_result.length = 0;
            }
            else
            {
                read_result.status = 0; // 0 代表读取成功
                read_result.data = payload_buffer;
                read_result.length = data_length;
            }
        }
    }
    else
    {
        read_result.status = (error->status != 0) ? error->status : BLE_HS_EUNKNOWN;
        read_result.attribute_handle = (attr != NULL) ? attr->handle : 0;
        read_result.data = NULL;
        read_result.length = 0;
    }

    ESP_LOGI(TAG, "GATT read finished: handle=%u, attr=0x%04x, status=%d, len=%u",
             connection_handle, read_result.attribute_handle, read_result.status, read_result.length);

    // 同步派发读取结果 (注意：read_result.data 仅在当前回调执行周期内有效)
    ble_manager_dispatch_event(BLE_MANAGER_EVENT_READ_COMPLETED,
                               connection_handle,
                               &read_result);
    return 0;
}

/**
 * @brief GATT 带应答写入操作异步回调 (Write with response)
 */
static int ble_manager_on_gatt_write_callback(uint16_t connection_handle,
                                              const struct ble_gatt_error *error,
                                              struct ble_gatt_attr *attr,
                                              void *arg)
{
    if (error == NULL)
    {
        ESP_LOGE(TAG, "GATT write callback: error struct is NULL");
        return BLE_HS_EUNKNOWN;
    }

    ble_manager_async_result_t write_result;
    memset(&write_result, 0, sizeof(write_result));

    write_result.status = error->status;
    write_result.attribute_handle = (attr != NULL) ? attr->handle : 0;

    ESP_LOGI(TAG, "GATT write response: handle=%u, attr=0x%04x, status=%d",
             connection_handle, write_result.attribute_handle, error->status);

    // 派发写入完成事件
    ble_manager_dispatch_event(BLE_MANAGER_EVENT_WRITE_COMPLETED,
                               connection_handle,
                               &write_result);
    return 0;
}

esp_err_t ble_manager_central_read(uint16_t connection_handle, uint16_t attribute_handle)
{
    // 1. 基础入参校验
    if (connection_handle == BLE_MANAGER_INVALID_CONNECTION_HANDLE || attribute_handle == 0)
    {
        return ESP_ERR_INVALID_ARG;
    }

    if (s_manager.lock == NULL)
    {
        return ESP_ERR_INVALID_STATE;
    }

    bool handle_valid = false;

    // 2. 锁内校验系统状态、角色与槽位有效性
    xSemaphoreTake(s_manager.lock, portMAX_DELAY);
    if (s_manager.state != BLE_MANAGER_STATE_RUNNING)
    {
        xSemaphoreGive(s_manager.lock);
        ESP_LOGE(TAG, "Read failed: Stack not running");
        return ESP_ERR_INVALID_STATE;
    }

    if (s_manager.role != BLE_MANAGER_ROLE_CENTRAL)
    {
        xSemaphoreGive(s_manager.lock);
        ESP_LOGE(TAG, "Read failed: Device role is not central");
        return ESP_ERR_NOT_SUPPORTED;
    }

    for (int i = 0; i < BLE_MANAGER_MAX_CONNECTIONS; i++)
    {
        if (s_manager.connections[i].connection_handle == connection_handle)
        {
            handle_valid = true;
            break;
        }
    }
    xSemaphoreGive(s_manager.lock);

    if (!handle_valid)
    {
        ESP_LOGW(TAG, "Read failed: Connection handle %u not found", connection_handle);
        return ESP_ERR_NOT_FOUND;
    }

    // 3. 释放锁后调用 NimBLE 原生异步读取 API (底层发射 ATT Read Request)
    int rc = ble_gattc_read(connection_handle, attribute_handle,
                            ble_manager_on_gatt_read_callback, NULL);
    if (rc != 0)
    {
        ESP_LOGE(TAG, "ble_gattc_read failed on handle %u, attr 0x%04x, rc: %d",
                 connection_handle, attribute_handle, rc);
        return ESP_FAIL;
    }

    return ESP_OK;
}

esp_err_t ble_manager_central_write(uint16_t connection_handle,
                                    uint16_t attribute_handle,
                                    ble_manager_write_type_t write_type,
                                    const uint8_t *data,
                                    uint16_t length)
{
    // 1. 基础入参与枚举类型前置强校验
    if (write_type != BLE_MANAGER_WRITE_TYPE_WITH_RESPONSE &&
        write_type != BLE_MANAGER_WRITE_TYPE_WITHOUT_RESPONSE)
    {
        ESP_LOGE(TAG, "Write failed: Invalid write type %d", write_type);
        return ESP_ERR_INVALID_ARG;
    }

    if (connection_handle == BLE_MANAGER_INVALID_CONNECTION_HANDLE ||
        attribute_handle == 0 ||
        data == NULL || length == 0)
    {
        return ESP_ERR_INVALID_ARG;
    }

    if (s_manager.lock == NULL)
    {
        return ESP_ERR_INVALID_STATE;
    }

    bool handle_valid = false;
    uint16_t target_mtu = 0;

    // 2. 锁内校验系统状态、角色，并获取该连接当前的 MTU 容量上限
    xSemaphoreTake(s_manager.lock, portMAX_DELAY);
    if (s_manager.state != BLE_MANAGER_STATE_RUNNING)
    {
        xSemaphoreGive(s_manager.lock);
        ESP_LOGE(TAG, "Write failed: Stack not running");
        return ESP_ERR_INVALID_STATE;
    }

    if (s_manager.role != BLE_MANAGER_ROLE_CENTRAL)
    {
        xSemaphoreGive(s_manager.lock);
        ESP_LOGE(TAG, "Write failed: Device role is not central");
        return ESP_ERR_NOT_SUPPORTED;
    }

    for (int i = 0; i < BLE_MANAGER_MAX_CONNECTIONS; i++)
    {
        if (s_manager.connections[i].connection_handle == connection_handle)
        {
            target_mtu = s_manager.connections[i].current_mtu;
            handle_valid = true;
            break;
        }
    }
    xSemaphoreGive(s_manager.lock);

    if (!handle_valid)
    {
        ESP_LOGW(TAG, "Write failed: Connection handle %u not found", connection_handle);
        return ESP_ERR_NOT_FOUND;
    }

    // 单包 ATT 写入操作有效载荷不得超过 MTU - 3 字节 (扣除 1B 操作码 + 2B 句柄)
    uint16_t max_payload = (target_mtu > 3) ? (target_mtu - 3) : 0;
    if (length > max_payload)
    {
        ESP_LOGE(TAG, "Write failed: Length %u exceeds target MTU capacity %u (MTU: %u)",
                 length, max_payload, target_mtu);
        return ESP_ERR_INVALID_SIZE;
    }

    int rc = 0;

    // 3. 区分写入类型发起底层调用
    if (write_type == BLE_MANAGER_WRITE_TYPE_WITH_RESPONSE)
    {
        // 带应答写入 (ATT Write Request): 等待对端回复 Write Response 后触发异步回调
        rc = ble_gattc_write_flat(connection_handle, attribute_handle,
                                  data, length,
                                  ble_manager_on_gatt_write_callback, NULL);
    }
    else
    {
        /*
         * 无应答写命令 (ATT Write Command): 高速发送，无空中应答。
         * 底层入队成功即视为指令提交完毕，本地立即派发完成事件
         */
        rc = ble_gattc_write_no_rsp_flat(connection_handle, attribute_handle,
                                         data, length);
        if (rc == 0)
        {
            ble_manager_async_result_t write_result = {
                .status = 0,
                .attribute_handle = attribute_handle};

            ble_manager_dispatch_event(BLE_MANAGER_EVENT_WRITE_COMPLETED,
                                       connection_handle,
                                       &write_result);
        }
    }

    if (rc != 0)
    {
        ESP_LOGE(TAG, "GATT write failed on handle %u, attr 0x%04x, type %d, rc: %d",
                 connection_handle, attribute_handle, write_type, rc);
        return ESP_FAIL;
    }

    return ESP_OK;
}

/* ========================================================================= */
/* GATT 客户端订阅 CCCD 异步回调与业务接口                                     */
/* ========================================================================= */

/**
 * @brief GATT 客户端订阅 CCCD 写入响应底层回调
 */
static int ble_manager_on_gatt_subscribe_callback(uint16_t connection_handle,
                                                  const struct ble_gatt_error *error,
                                                  struct ble_gatt_attr *attr,
                                                  void *arg)
{
    if (error == NULL)
    {
        ESP_LOGE(TAG, "GATT subscribe callback: error struct is NULL");
        return BLE_HS_EUNKNOWN;
    }

    ble_manager_async_result_t subscribe_result;
    memset(&subscribe_result, 0, sizeof(subscribe_result));

    // 状态码规范：0 代表成功，非 0 代表错误码
    subscribe_result.status = error->status;
    subscribe_result.attribute_handle = (attr != NULL) ? attr->handle : 0;

    ESP_LOGI(TAG, "GATT subscribe response on handle %u, cccd=0x%04x, status=%d",
             connection_handle, subscribe_result.attribute_handle, error->status);

    // 派发订阅配置完成事件
    ble_manager_dispatch_event(BLE_MANAGER_EVENT_SUBSCRIBE_COMPLETED,
                               connection_handle,
                               &subscribe_result);
    return 0;
}

esp_err_t ble_manager_central_subscribe(uint16_t connection_handle,
                                        uint16_t cccd_handle,
                                        bool enable)
{
    // 1. 基础入参校验 (0x0000 为 ATT 规范非法句柄)
    if (connection_handle == BLE_MANAGER_INVALID_CONNECTION_HANDLE || cccd_handle == 0)
    {
        return ESP_ERR_INVALID_ARG;
    }

    if (s_manager.lock == NULL)
    {
        return ESP_ERR_INVALID_STATE;
    }

    bool handle_valid = false;

    // 2. 锁内校验系统状态、角色及连接有效性
    xSemaphoreTake(s_manager.lock, portMAX_DELAY);
    if (s_manager.state != BLE_MANAGER_STATE_RUNNING)
    {
        xSemaphoreGive(s_manager.lock);
        ESP_LOGE(TAG, "Subscribe failed: Stack not running");
        return ESP_ERR_INVALID_STATE;
    }

    if (s_manager.role != BLE_MANAGER_ROLE_CENTRAL)
    {
        xSemaphoreGive(s_manager.lock);
        ESP_LOGE(TAG, "Subscribe failed: Device role is not central");
        return ESP_ERR_NOT_SUPPORTED;
    }

    for (int i = 0; i < BLE_MANAGER_MAX_CONNECTIONS; i++)
    {
        if (s_manager.connections[i].connection_handle == connection_handle)
        {
            handle_valid = true;
            break;
        }
    }
    xSemaphoreGive(s_manager.lock);

    if (!handle_valid)
    {
        ESP_LOGW(TAG, "Subscribe failed: Connection handle %u not found", connection_handle);
        return ESP_ERR_NOT_FOUND;
    }

    /*
     * 3. 准备 CCCD 写入数据 (小端序 16 位整数):
     * 开启 Notification 写入 0x0001, 关闭写入 0x0000
     */
    uint8_t cccd_value[2];
    cccd_value[0] = enable ? 0x01 : 0x00;
    cccd_value[1] = 0x00;

    /*
     * 4. 向远端 CCCD 描述符发起带应答写入 (ATT Write Request):
     * 直接对通过描述符发现流程获取到的确凿 cccd_handle 发起写入，杜绝硬编码加 1 错位风险。
     */
    int rc = ble_gattc_write_flat(connection_handle,
                                  cccd_handle,
                                  cccd_value,
                                  sizeof(cccd_value),
                                  ble_manager_on_gatt_subscribe_callback,
                                  NULL);
    if (rc != 0)
    {
        ESP_LOGE(TAG, "ble_gattc_write_flat to CCCD (0x%04x) failed, rc: %d", cccd_handle, rc);
        return ESP_FAIL;
    }

    ESP_LOGI(TAG, "Subscribe request submitted [handle: %u, cccd: 0x%04x, enable: %d]",
             connection_handle, cccd_handle, enable);

    return ESP_OK;
}
