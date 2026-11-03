#ifndef BLE_MANAGER_H
#define BLE_MANAGER_H

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#include "esp_err.h"

#ifdef __cplusplus
extern "C"
{
#endif

#define BLE_MANAGER_INVALID_CONNECTION_HANDLE 0xFFFF

// 多连接最大并发数。若 sdkconfig 配置了则继承，否则默认支持 3 路连接
#ifndef BLE_MANAGER_MAX_CONNECTIONS
#ifdef CONFIG_BT_NIMBLE_MAX_CONNECTIONS
#define BLE_MANAGER_MAX_CONNECTIONS CONFIG_BT_NIMBLE_MAX_CONNECTIONS
#else
#define BLE_MANAGER_MAX_CONNECTIONS 3
#endif
#endif

    /* NimBLE GATT 服务定义结构体的前置声明 */
    struct ble_gatt_svc_def;

    /* ========================================================================= */
    /* 1. 基础数据类型与枚举定义                                                  */
    /* ========================================================================= */

    // 设备工作角色
    typedef enum
    {
        BLE_MANAGER_ROLE_PERIPHERAL, // 外围设备 (从机: 广播、等待连接、提供服务)
        BLE_MANAGER_ROLE_CENTRAL     // 中央设备 (主机: 扫描、主动连接、操作外部服务)
    } ble_manager_role_t;

    // 蓝牙地址类型
    typedef enum
    {
        BLE_MANAGER_ADDRESS_TYPE_PUBLIC = 0, // 公开硬件 MAC 地址
        BLE_MANAGER_ADDRESS_TYPE_RANDOM = 1  // 随机地址
    } ble_manager_address_type_t;

    // 通用 UUID 类型定义
    typedef enum
    {
        BLE_MANAGER_UUID_TYPE_16,
        BLE_MANAGER_UUID_TYPE_128
    } ble_manager_uuid_type_t;

    typedef struct
    {
        ble_manager_uuid_type_t type;
        union
        {
            uint16_t uuid16;
            uint8_t uuid128[16];
        } value;
    } ble_manager_uuid_t;

    // GATT 写入类型
    typedef enum
    {
        BLE_MANAGER_WRITE_TYPE_WITH_RESPONSE,   // Write Request (远端返回 ATT 响应)
        BLE_MANAGER_WRITE_TYPE_WITHOUT_RESPONSE // Write Command (无应答，高速透传)
    } ble_manager_write_type_t;

    /* ========================================================================= */
    /* 2. 事件系统定义                                                           */
    /* ========================================================================= */

    // 通用事件枚举
    typedef enum
    {
        // 公共事件
        BLE_MANAGER_EVENT_READY,        // 协议栈与硬件同步就绪
        BLE_MANAGER_EVENT_CONNECTED,    // 连接建立
        BLE_MANAGER_EVENT_DISCONNECTED, // 连接断开
        BLE_MANAGER_EVENT_MTU_UPDATED,  // MTU 协商更新
        BLE_MANAGER_EVENT_ERROR,        // 内部错误

        BLE_MANAGER_EVENT_ENCRYPTION_COMPLETED, // 链路加密/配对流程完成 (event_data: ble_manager_async_result_t*)

        // 外围设备 (从机) 专属事件
        BLE_MANAGER_EVENT_ADVERTISING_STARTED, // 广播已启动
        BLE_MANAGER_EVENT_ADVERTISING_STOPPED, // 广播已停止

        // 中央设备 (主机) 专属事件
        BLE_MANAGER_EVENT_SCANNING_STARTED,              // 扫描启动完成
        BLE_MANAGER_EVENT_SCANNING_STOPPED,              // 扫描停止完成
        BLE_MANAGER_EVENT_DEVICE_DISCOVERED,             // 扫描到设备 (event_data: ble_manager_discovered_device_t)
        BLE_MANAGER_EVENT_CONNECTION_PARAMETERS_UPDATED, // 连接参数更新生效 (event_data: ble_manager_connection_parameters_t*)

        // GATT 发现与操作事件
        BLE_MANAGER_EVENT_SERVICE_DISCOVERED,                 // 发现单个服务 (event_data: ble_manager_discovered_service_t*)
        BLE_MANAGER_EVENT_SERVICE_DISCOVERY_COMPLETED,        // 服务发现全部完成 (event_data: ble_manager_async_result_t*)
        BLE_MANAGER_EVENT_CHARACTERISTIC_DISCOVERED,          // 发现单个特性 (event_data: ble_manager_discovered_characteristic_t*)
        BLE_MANAGER_EVENT_CHARACTERISTIC_DISCOVERY_COMPLETED, // 特性发现全部完成 (event_data: ble_manager_async_result_t*)

        BLE_MANAGER_EVENT_DESCRIPTOR_DISCOVERED,          // 发现单个描述符 (event_data: ble_manager_discovered_descriptor_t*)
        BLE_MANAGER_EVENT_DESCRIPTOR_DISCOVERY_COMPLETED, // 描述符发现全部完成 (event_data: ble_manager_async_result_t*)

        BLE_MANAGER_EVENT_READ_COMPLETED,       // 读取完成 (event_data: ble_manager_read_result_t*)
        BLE_MANAGER_EVENT_WRITE_COMPLETED,      // 写入完成 (event_data: ble_manager_async_result_t*)
        BLE_MANAGER_EVENT_SUBSCRIBE_COMPLETED,  // 订阅配置完成 (event_data: ble_manager_async_result_t*)
        BLE_MANAGER_EVENT_NOTIFICATION_RECEIVED // 收到远端通知推送 (event_data: ble_manager_received_data_t*)
    } ble_manager_event_t;

    // 扫描发现外部设备的负载结构体
    typedef struct
    {
        uint8_t address[6];                      // 目标设备 MAC 地址
        ble_manager_address_type_t address_type; // 地址类型 (0: Public 公开, 1: Random 随机)
        int8_t rssi;                             // 信号强度
        char device_name[32];                    // 广播中的设备名称
    } ble_manager_discovered_device_t;

    // 已绑定设备的信息负载结构体
    typedef struct
    {
        uint8_t address[6];                      // 已配对外设的 6 字节 MAC 地址
        ble_manager_address_type_t address_type; // 地址类型 (Public 或 Random)
    } ble_manager_bonded_device_t;

    // 连接参数更新结果负载结构体
    typedef struct
    {
        int status;                   // 0 代表更新成功，非 0 为协商失败错误码
        uint16_t connection_interval; // 协商生效的连接间隔 (单位: 1.25ms)
        uint16_t peripheral_latency;  // 协商生效的从机延迟周期
        uint16_t supervision_timeout; // 协商生效的超时时间 (单位: 10ms)
    } ble_manager_connection_parameters_t;

    // 发现远端服务的负载结构体
    typedef struct
    {
        uint16_t start_handle;   // 服务的起始句柄
        uint16_t end_handle;     // 服务的结束句柄
        ble_manager_uuid_t uuid; // 完整支持 16/128 位
    } ble_manager_discovered_service_t;

    // 发现远端特性的负载结构体
    typedef struct
    {
        uint16_t declaration_handle; // 特性声明句柄
        uint16_t value_handle;       // 特性数值句柄 (真正读写使用的句柄)
        uint8_t properties;          // 特性性质位掩码 (0x02: 只读, 0x04: 无应答写, 0x08: 带应答写, 0x10: 通知等)
        ble_manager_uuid_t uuid;     // 支持 16/128 位 UUID
    } ble_manager_discovered_characteristic_t;

    // 发现远端描述符的负载结构体
    typedef struct
    {
        uint16_t descriptor_handle; // 描述符的实际属性句柄
        ble_manager_uuid_t uuid;    // 描述符 UUID (例如: 0x2902, 0x2908)
    } ble_manager_discovered_descriptor_t;

    // 通用异步操作完成负载 (用于服务发现完成、特性发现完成、写入完成、订阅完成)
    typedef struct
    {
        int status;                // 0 代表成功，非 0 为错误码
        uint16_t attribute_handle; // 产生操作的属性句柄
    } ble_manager_async_result_t;

    // 读取远端特征值结果负载结构体
    typedef struct
    {
        uint16_t attribute_handle; // 被读取的属性句柄
        int status;                // 0 代表成功，非 0 为错误码
        const uint8_t *data;       // 读取到的数据指针 (仅在当前回调周期内有效)
        uint16_t length;           // 读取到的数据长度
    } ble_manager_read_result_t;

    // 收到外部通知的负载结构体
    typedef struct
    {
        uint16_t attribute_handle; // 产生推送的属性句柄
        const uint8_t *data;       // 数据指针(仅在当前事件回调执行期间有效)
        uint16_t length;           // 数据长度
    } ble_manager_received_data_t;

    // 断开连接事件负载
    typedef struct
    {
        int reason; // 断开错误码
    } ble_manager_event_disconnect_t;

    /**
     * @brief 全局事件回调函数指针原型
     * @param event              事件类型
     * @param connection_handle  发生事件的目标连接句柄 (若事件无关特定连接则为 0xFFFF)
     * @param event_data         事件携带的专用数据负载结构体指针 (强转为对应事件的类型，离开回调后失效)
     * @param user_context       初始化时传入的用户上下文指针
     */
    typedef void (*ble_manager_event_callback_t)(
        ble_manager_event_t event,
        uint16_t connection_handle,
        void *event_data,
        void *user_context);

    // 管理器初始化配置项
    typedef struct
    {
        ble_manager_role_t role;                     // 指定当前工程的工作角色
        const char *device_name;                     // 设备名 (外围设为广播名，中央设备可用作过滤)
        ble_manager_event_callback_t event_callback; // 全局事件回调
        void *user_context;                          // 用户上下文指针

        /* 外围设备服务表（必须是静态或全局生命周期，不可为局部栈变量） */
        const struct ble_gatt_svc_def *gatt_services;
    } ble_manager_config_t;

    /* ========================================================================= */
    /* 3. 通用基础生命周期接口 (两端通用)                                     */
    /* ========================================================================= */

    /**
     * @brief 初始化 BLE 管理器
     * @note 1. 调用前应用层必须已在 app_main 中统一初始化 NVS (nvs_flash_init)。
     *
     *       2. 本接口与 ble_manager_deinit 必须由单个应用任务串行调用，禁止并发。
     */
    esp_err_t ble_manager_init(const ble_manager_config_t *config);

    /**
     * @brief 安全反初始化 BLE 管理器
     * @note 调用此接口期间，应用层应停止调用其他 BLE 接口。
     */
    esp_err_t ble_manager_deinit(void);

    /**
     * @brief 主动断开指定连接
     * @param connection_handle 目标连接句柄
     */
    esp_err_t ble_manager_disconnect(uint16_t connection_handle);

    /**
     * @brief 获取当前首个活动连接的句柄 (向后兼容单连接使用)
     * @return uint16_t 若无连接返回 BLE_MANAGER_INVALID_CONNECTION_HANDLE
     */
    uint16_t ble_manager_get_connection_handle(void);

    /**
     * @brief 批量获取当前所有活动的连接句柄列表 (多连接专用)
     * @param handles 接收句柄的外部数组
     * @param max_count 数组最大容量
     * @return uint8_t 实际成功拷贝的活动连接数
     */
    uint8_t ble_manager_get_active_connections(uint16_t *handles, uint8_t max_count);

    /**
     * @brief 查询当前是否存在至少一条活动连接
     */
    bool ble_manager_is_connected(void);

    /**
     * @brief 查询指定连接当前实际生效的 ATT MTU 大小
     * @param connection_handle 目标连接句柄
     * @return uint16_t 有效 MTU 大小 (若连接失效返回 0)
     */
    uint16_t ble_manager_get_mtu(uint16_t connection_handle);

    /**
     * @brief 从底层 NVS 中读取当前所有已配对绑定的设备记录 (供开机回连使用)
     * @param bonded_devices 接收输出的已绑定设备数组
     * @param maximum_count 数组最大容纳数量
     * @return uint8_t 实际查询到的已绑定设备数量
     */
    uint8_t ble_manager_get_bonded_devices(ble_manager_bonded_device_t *bonded_devices,
                                           uint8_t maximum_count);

    /**
     * @brief 清除底层存储中的所有配对记录与长期密钥 (恢复出厂设置)
     */
    esp_err_t ble_manager_clear_all_bonds(void);

    /**
     * @brief 遗忘/解绑指定 MAC 地址的外设
     *
     * @param target_address 目标设备的 6 字节 MAC 地址
     * @param address_type   目标设备的地址类型
     * @return esp_err_t     ESP_OK 代表解绑成功
     */
    esp_err_t ble_manager_unpair_device(const uint8_t target_address[6],
                                        ble_manager_address_type_t address_type);

    /* ========================================================================= */
    /* 3. 外围设备专属接口 (Role == BLE_MANAGER_ROLE_PERIPHERAL 时调用)          */
    /* ========================================================================= */

    // /**
    //  * @brief 注册本地 GATT 服务表
    //  * @param gatt_services 以 NULL 结尾的 NimBLE GATT 服务定义数组
    //  */
    // esp_err_t ble_manager_peripheral_register_services(const struct ble_gatt_svc_def *gatt_services);

    /**
     * @brief 启动外围设备广播
     * @note 内部自动配置 Flags，并将长设备名放进进扫描响应包 (Scan Response)
     */
    esp_err_t ble_manager_peripheral_start_advertising(void);

    /**
     * @brief 停止外围设备广播
     */
    esp_err_t ble_manager_peripheral_stop_advertising(void);

    /**
     * @brief 主动向已连接的中央设备推送数据 (GATT Notification)
     * @param connection_handle 目标连接句柄
     * @param value_handle 目标特性的数值句柄 (Value Handle，必须对应具备 Notify 属性的特性)
     * @param data 待推送的原始数据指针
     * @param length 数据长度 (单包必须 <= 当前连接的 MTU - 3 字节)
     */
    esp_err_t ble_manager_peripheral_notify(uint16_t connection_handle,
                                            uint16_t value_handle,
                                            const uint8_t *data,
                                            uint16_t length);

    /* ========================================================================= */
    /* 4. 中央设备专属接口 (Role == BLE_MANAGER_ROLE_CENTRAL 时调用)             */
    /* ========================================================================= */

    /**
     * @brief 启动中央设备主动扫描
     */
    esp_err_t ble_manager_central_start_scanning(void);

    /**
     * @brief 停止中央设备扫描
     */
    esp_err_t ble_manager_central_stop_scanning(void);

    /**
     * @brief 主动向指定外设发起物理连接
     * @param target_address 目标外设的 6 字节 MAC 地址
     * @param address_type   目标外设的地址类型 (公有或随机)
     * @return esp_err_t     ESP_OK 代表请求成功提交 (最终结果由 CONNECTED 或 ERROR 事件异步通知)
     */
    esp_err_t ble_manager_central_connect(const uint8_t target_address[6],
                                          ble_manager_address_type_t address_type);

    /**
     * @brief 主动发起 ATT MTU 交换请求 (协商扩大单包传输载荷)
     * @param connection_handle 目标连接句柄
     * @return esp_err_t 请求提交状态；最终生效值由 BLE_MANAGER_EVENT_MTU_UPDATED 事件异步派发
     */
    esp_err_t ble_manager_central_exchange_mtu(uint16_t connection_handle);

    /**
     * @brief 中央设备主动更新物理链路连接参数 (调节通信延迟与功耗)
     *
     * @param connection_handle   目标连接句柄
     * @param interval_minimum    最小连接间隔 (范围 6~3200, 对应 7.5ms ~ 4000ms, 单位 1.25ms)
     * @param interval_maximum    最大连接间隔 (必须 >= interval_minimum)
     * @param latency             从机潜伏延迟周期 (Slave Latency, 允许从机跳过的连接事件次数)
     * @param supervision_timeout 链路监控超时时间 (范围 10~3200, 对应 100ms ~ 32000ms, 单位 10ms)
     * @return esp_err_t          ESP_OK: 请求已提交; 最终协商参数由 BLE_MANAGER_EVENT_CONNECTION_PARAMETERS_UPDATED 通知
     */
    esp_err_t ble_manager_central_update_connection_parameters(uint16_t connection_handle,
                                                               uint16_t interval_minimum,
                                                               uint16_t interval_maximum,
                                                               uint16_t latency,
                                                               uint16_t supervision_timeout);

    /**
     * @brief 中央设备主动启动指定连接的链路安全与加密流程
     *
     * @note 根据双方已有的绑定信息，底层会自动选择执行全新配对 (Pairing) 或恢复长期密钥加密。
     *       调用返回 ESP_OK 仅代表安全请求已成功提交协议栈，并不代表加密已完成。
     *       最终加密结果将通过 BLE_MANAGER_EVENT_ENCRYPTION_COMPLETED 异步事件派发。
     *
     * @param connection_handle 目标连接句柄
     * @return esp_err_t        ESP_OK: 安全流程已成功启动
     *
     *                          ESP_ERR_INVALID_STATE: 该连接当前已在执行安全流程
     *
     *                          ESP_ERR_NOT_FOUND: 连接已失效或不存在
     *
     *                          ESP_FAIL: 协议栈底层报错
     */
    esp_err_t ble_manager_central_start_security(uint16_t connection_handle);

    /**
     * @brief 发现指定连接的所有主服务 (Primary Services)
     *
     * @param connection_handle 目标连接句柄
     * @return esp_err_t        ESP_OK: 请求已成功提交至底层队列；非 0: 提交失败
     * @note 异步响应事件链 (通过全局事件回调派发)：
     *
     *       - 每发现一个主服务触发：
     *
     *         event:      BLE_MANAGER_EVENT_SERVICE_DISCOVERED
     *
     *         event_data: const ble_manager_discovered_service_t*
     *
     *       - 整个服务发现流程结束时触发：
     *
     *         event:      BLE_MANAGER_EVENT_SERVICE_DISCOVERY_COMPLETED
     *
     *         event_data: const ble_manager_async_result_t* (status 为 0 代表成功)
     */
    esp_err_t ble_manager_central_discover_services(uint16_t connection_handle);

    /**
     * @brief 发现指定句柄范围内的所有特性 (Characteristics)
     *
     * @param connection_handle 目标连接句柄
     * @param start_handle      搜索起始句柄 (通常为服务的 start_handle)
     * @param end_handle        搜索结束句柄 (通常为服务的 end_handle)
     * @return esp_err_t        ESP_OK: 请求已成功提交至底层队列；非 0: 提交失败
     * @note 异步响应事件链 (通过全局事件回调派发)：
     *
     *       - 每发现一个特性触发：
     *
     *         event:      BLE_MANAGER_EVENT_CHARACTERISTIC_DISCOVERED
     *
     *         event_data: const ble_manager_discovered_characteristic_t*
     *
     *       - 指定区间内特性全部检索完毕时触发：
     *
     *         event:      BLE_MANAGER_EVENT_CHARACTERISTIC_DISCOVERY_COMPLETED
     *
     *         event_data: const ble_manager_async_result_t* (status 为 0 代表成功)
     */
    esp_err_t ble_manager_central_discover_characteristics(uint16_t connection_handle,
                                                           uint16_t start_handle,
                                                           uint16_t end_handle);

    /**
     * @brief 发现指定特征所属的所有描述符 (用于获取 CCCD 等)
     * @param connection_handle           目标连接句柄
     * @param characteristic_value_handle 所属特征的数值句柄 (NimBLE 将从此句柄 +1 处开始查找)
     * @param characteristic_end_handle   所属特征的结束句柄 (通常为下一个特征的声明句柄 - 1 或所属服务的结束句柄)
     * @return esp_err_t                  ESP_OK 代表指令已提交协议栈
     * @note 异步响应事件链 (通过全局事件回调派发)：
     *
     *       - 每发现一个描述符触发：
     *
     *         event:      BLE_MANAGER_EVENT_DESCRIPTOR_DISCOVERED
     *
     *         event_data: const ble_manager_discovered_descriptor_t*
     *
     *       - 该特性下描述符全部检索完毕时触发：
     *
     *         event:      BLE_MANAGER_EVENT_DESCRIPTOR_DISCOVERY_COMPLETED
     *
     *         event_data: const ble_manager_async_result_t* (status 为 0 代表成功)
     */
    esp_err_t ble_manager_central_discover_descriptors(uint16_t connection_handle,
                                                       uint16_t characteristic_value_handle,
                                                       uint16_t characteristic_end_handle);

    /**
     * @brief 异步读取远端特性的当前值
     *
     * @param connection_handle 目标连接句柄
     * @param attribute_handle  特性的数值句柄 (Value Handle)
     * @return esp_err_t        ESP_OK: 读取请求已发出；非 0: 发送失败
     * @note 异步响应事件 (通过全局事件回调派发)：
     *
     *       - 读取完成时触发：
     *
     *         event:      BLE_MANAGER_EVENT_READ_COMPLETED
     *
     *         event_data: const ble_manager_read_result_t* (包含读取状态 status、数据指针 data 和长度 length)
     *
     *         注意：payload 里的 data 指针仅在回调执行期间有效，严禁跨任务长期持有！
     */
    esp_err_t ble_manager_central_read(uint16_t connection_handle, uint16_t attribute_handle);

    /**
     * @brief 向远端特性写入数据
     * @param connection_handle 目标连接句柄
     * @param attribute_handle 特性的数值句柄 (Value Handle)
     * @param write_type 写入类型 (带应答 Write Request 或无应答 Write Command)
     * @param data 待写入数据的指针
     * @param length 数据字节数 (单包必须 <= MTU - 3)
     * @note 异步响应事件 (通过全局事件回调派发)：
     *
     *       - 操作结束时统一触发：
     *
     *         event:      BLE_MANAGER_EVENT_WRITE_COMPLETED
     *
     *         event_data: const ble_manager_async_result_t*
     *
     *         - 若为 WITH_RESPONSE: status 为 0 代表对端外设已成功接收并回复确认 (ATT Write Response)；
     *
     *         - 若为 WITHOUT_RESPONSE: status 为 0 仅代表数据包已被本地协议栈成功推入空中发射队列。
     */
    esp_err_t ble_manager_central_write(uint16_t connection_handle,
                                        uint16_t attribute_handle,
                                        ble_manager_write_type_t write_type,
                                        const uint8_t *data,
                                        uint16_t length);

    /**
     * @brief 配置远端特性的通知/指示订阅
     * @param connection_handle 目标连接句柄
     * @param cccd_handle 目标特性所对应的 CCCD 描述符句柄 (严禁传错成数值句柄)
     * @param enable true 为开启通知订阅 (写入 0x0001)，false 为退订关闭 (写入 0x0000)
     * @note 异步响应事件 (通过全局事件回调派发)：
     *
     *       - 对端外设确认配置后触发：
     *
     *         event:      BLE_MANAGER_EVENT_SUBSCRIBE_COMPLETED
     *
     *         event_data: const ble_manager_async_result_t* (status 为 0 代表订阅/退订成功)
     *
     *       - 订阅成功后，后续远端推送的数据将持续通过以下事件上报：
     *
     *         event:      BLE_MANAGER_EVENT_NOTIFICATION_RECEIVED
     *
     *         event_data: const ble_manager_received_data_t*
     */
    esp_err_t ble_manager_central_subscribe(uint16_t connection_handle,
                                            uint16_t cccd_handle,
                                            bool enable);

#ifdef __cplusplus
}
#endif

#endif
