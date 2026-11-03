#ifndef XBOX_CONTROLLER_H
#define XBOX_CONTROLLER_H

/*
 * =========================================================================================
 *  Xbox 无线手柄驱动 (Xbox Wireless Controller Driver)
 * =========================================================================================
 *
 * 核心架构特色：
 * 1. 跨平台解耦：仅依赖标准 C 库，可移植到 STM32、nRF52、树莓派、Linux 等平台。
 * 2. 纯粹性：只负责协议解析与状态维护，不绑定具体蓝牙协议栈或硬件介质。
 * 3. 全拼命名：杜绝缩写，函数名即功能说明书。
 *
 * =========================================================================================
 * 附录：Xbox 无线手柄 BLE 协议技术文档 (实测标定全量版)
 * =========================================================================================
 *
 * 表一：核心 GATT 服务与通道句柄速查表
 * -----------------------------------------------------------------------------------------
 * 职能类别        归属服务 (Service)   特征 UUID   通信方向         传输权限 / 方式
 *
 * [1. 人机交互核心控制 - HID Service: 0x1812]
 * 手柄激活唤醒    HID 服务 (0x1812)    0x2A4C      主机->手柄      Write No Rsp (写入 0x01退出休眠)
 * 按键输入通道    HID 服务 (0x1812)    0x2A4D      手柄->主机      Read / Notify (16字节报文)
 * 按键订阅开关    HID 服务 (0x1812)    0x2902      主机->手柄      Write Req (CCCD 写入 0x0001订阅通知推送)
 * 马达震动输出    HID 服务 (0x1812)    0x2A4D      主机->手柄      Write No Rsp (8字节控制包)
 *
 * [2. 电池状态 - Battery Service: 0x180F]
 * 电池电量读取    电池服务 (0x180F)    0x2A19      手柄->主机      Read / Notify (返回 0~100%)
 * 电量订阅开关    电池服务 (0x180F)    0x2902      主机->手柄      Write Req (CCCD 写入 0x0001订阅通知推送)
 *
 * [3. 设备出厂信息 - Device Information Service: 0x180A]
 * 制造商名称      设备信息 (0x180A)    0x2A29      手柄->主机      Read (只读，返回 "Microsoft")
 * PnP 硬件标识    设备信息 (0x180A)    0x2A50      手柄->主机      Read (只读，返回 VID/PID 字节)
 * 固件版本号      设备信息 (0x180A)    0x2A26      手柄->主机      Read (只读，如 "5.23.6.0")
 * 序列号(SN)      设备信息 (0x180A)    0x2A25      手柄->主机      Read (只读，如 "09VX0609...")
 *
 * 表二：16 字节按键输入报文 (Input Report) 字节映射表 (UUID: 0x2A4D)
 * -----------------------------------------------------------------------------------------
 * 字节偏移      物理部件                  数据格式与掩码定义                归一化建议范围
 * Byte 0~1      左摇杆水平 X 轴           16位小端无符号 (居中约 32768)     -32768 ~ 32767
 * Byte 2~3      左摇杆垂直 Y 轴           16位小端无符号 (居中约 32768)     -32768 ~ 32767
 * Byte 4~5      右摇杆水平 X 轴           16位小端无符号 (居中约 32768)     -32768 ~ 32767
 * Byte 6~7      右摇杆垂直 Y 轴           16位小端无符号 (居中约 32768)     -32768 ~ 32767
 * Byte 8~9      左模拟扳机 (LT)           10位线性模拟量                    0 ~ 1023
 * Byte 10~11    右模拟扳机 (RT)           10位线性模拟量                    0 ~ 1023
 * Byte 12       十字方向键 (D-Pad)        8向帽开关: 0=松开                 0 ~ 8 ( 1~8 顺时针代表 上,右上,右,右下,下,左下,左,左上)
 * Byte 13       核心动作按键组 (位掩码)   Bit0=A, Bit1=B, Bit3=X, Bit4=Y, Bit6=LB, Bit7=RB
 * Byte 14       系统辅助按键组1 (位掩码)  Bit2=View, Bit3=Menu, Bit4=Xbox, Bit5=L3, Bit6=R3
 * Byte 15       系统辅助按键组2 (位掩码)  Bit0=Share
 *
 * 表三：8 字节马达震动输出报文 (Output Report) 字节映射表 (UUID: 0x2A4D)
 * -----------------------------------------------------------------------------------------
 * 字节偏移      物理意义                  取值说明
 * Byte 0        马达使能掩码              固定 0x0F (全开，Bit0~3置1)
 * Byte 1        左扳机脉冲马达            0~100 (%) 阻尼、机械触感
 * Byte 2        右扳机脉冲马达            0~100 (%) 油门颠簸感、刹车阻尼感
 * Byte 3        左握把主大马达            0~100 (%) 重低音沉浸感轰鸣
 * Byte 4        右握把副小马达            0~100 (%) 高频清脆微震
 * Byte 5        震动持续时间              0~255 (单位 10ms，定义每一波震动转动多久。如 30 = 300 毫秒)
 * Byte 6        延迟起震时间              0~255 (单位 10ms，定义两波震动之间的停顿时间)
 * Byte 7        脉冲循环次数              0~255 实际执行总次数为 (loop_count + 1) 次:
 *                                        ▶ 模式一：平滑超长连续震动 (即延时 Byte 6 填 0 时)
 *                                           手柄无停顿连续震动，总震动时间 = (loop_count + 1) * duration。
 *                                           例如: duration=100 (1秒), delay=0, loop_count=2 -> 实现连续 3 秒长震动！
 *
 *                                        ▶ 模式二：机械节奏断续脉冲 (即延时 Byte 6 大于 0 时)
 *                                        手柄执行 [停顿 ->震动 -> 停顿 -> 震动 -> 停顿...] 的节奏序列。
 *                                        例如: duration=10 (100ms), delay=10 (100ms), loop_count=2 -> 实现为
 *                                             停顿 ->震动 -> 停顿 -> 震动 -> 停顿 ->震动
 * =========================================================================================
 */

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>

/* ========================================================================= */
/* 功能裁剪宏开关 (0: 关闭剔除, 1: 开启使能)                                 */
/* ========================================================================= */

// 是否启用手柄电池电量存储功能
#define CONFIG_APP_ENABLE_XBOX_BATTERY_MONITORING 1

// 是否开启手柄出厂 SN 码、固件版本号数据存储功能
#define CONFIG_APP_ENABLE_XBOX_DEVICE_INFORMATION 1

/* ========================================================================= */
/* 硬件协议与 GATT 字典常量 (Xbox Wireless Controller Profile)               */
/* ========================================================================= */

// 手柄控制指令
#define XBOX_COMMAND_EXIT_SUSPEND 0x01 // 唤醒手柄指令 (向控制点写入 0x01 退出休眠)
//
// 蓝牙标准 16 位服务 UUID (用于动态搜索比对)
#define BLE_SERVICE_HID_UUID16 0x1812         // 人机交互服务 (Human Interface Device)
#define BLE_SERVICE_BATTERY_UUID16 0x180F     // 电池电量服务 (Battery Service)
#define BLE_SERVICE_DEVICE_INFO_UUID16 0x180A // 设备出厂信息服务 (Device Information)
//
// 蓝牙标准 16 位特征值与描述符 UUID
#define BLE_CHAR_HID_CONTROL_POINT_UUID16 0x2A4C // 挂起/唤醒控制点
#define BLE_CHAR_HID_REPORT_UUID16 0x2A4D        // 数据报文通道 (包含按键输入与马达输出)
#define BLE_CHAR_BATTERY_LEVEL_UUID16 0x2A19     // 电池电量百分比
#define BLE_CHAR_SERIAL_NUMBER_UUID16 0x2A25     // 出厂序列号 SN 码
#define BLE_CHAR_FIRMWARE_REVISION_UUID16 0x2A26 // 固件版本号
#define BLE_DESCRIPTOR_CCCD_UUID16 0x2902        // 客户端特性配置描述符 (CCCD 订阅开关)

// Xbox 手柄静态标定属性句柄 (GATT Caching: 零无线电开销，直通硬件)
#define XBOX_HANDLE_FIRMWARE_REVISION_VALUE 15 // 固件版本号数据句柄 (UUID: 0x2A26) (实测)
#define XBOX_HANDLE_SERIAL_NUMBER_VALUE 17     // 出厂 SN 码数据句柄 (UUID: 0x2A25) (实测)
#define XBOX_HANDLE_BATTERY_LEVEL_VALUE 20     // 电池电量数据句柄 (UUID: 0x2A19) (实测)
#define XBOX_HANDLE_BATTERY_LEVEL_CCCD 21      // 电池电量推送订阅开关句柄 (UUID:0x2902)

#ifdef __cplusplus
extern "C"
{
#endif

    /* ========================================================================= */
    /* 1. 跨平台标准状态与错误码                                                 */
    /* ========================================================================= */

    // 函数执行返回值枚举
    typedef enum
    {
        XBOX_CONTROLLER_STATUS_OK = 0,                   // 成功执行
        XBOX_CONTROLLER_STATUS_ERROR_INVALID_ARG = -1,   // 传入了非法参数 (如空指针)
        XBOX_CONTROLLER_STATUS_ERROR_NOT_CONNECTED = -2, // 手柄未连接，无法执行操作
        XBOX_CONTROLLER_STATUS_ERROR_IO = -3,            // 底层物理通道发送失败
        XBOX_CONTROLLER_STATUS_ERROR_NO_MEMORY = -4      // 系统内存不足
    } xbox_controller_status_t;

    /* ========================================================================= */
    /* 2. 手柄物理按键与姿态数据模型                                             */
    /* ========================================================================= */

    /**
     * @brief 十字方向键 (D-Pad) 离散朝向枚举
     * @note  对应手柄左下方的十字按键，包含 8 个离散方向以及居中松开状态
     */
    typedef enum
    {
        XBOX_DPAD_DIRECTION_CENTERED = 0, // 居中松开 (无方向按下)
        XBOX_DPAD_DIRECTION_UP,           // 向上
        XBOX_DPAD_DIRECTION_UP_RIGHT,     // 右上 (同时按住上和右)
        XBOX_DPAD_DIRECTION_RIGHT,        // 向右
        XBOX_DPAD_DIRECTION_DOWN_RIGHT,   // 右下 (同时按住下和右)
        XBOX_DPAD_DIRECTION_DOWN,         // 向下
        XBOX_DPAD_DIRECTION_DOWN_LEFT,    // 左下 (同时按住下和左)
        XBOX_DPAD_DIRECTION_LEFT,         // 向左
        XBOX_DPAD_DIRECTION_UP_LEFT       // 左上 (同时按住上和左)
    } xbox_controller_dpad_direction_t;

    // 单个独立按键标识枚举 (用于查询指定按键是否按下)
    typedef enum
    {
        XBOX_CONTROLLER_BUTTON_A,                // 动作 A 键 (绿色)
        XBOX_CONTROLLER_BUTTON_B,                // 动作 B 键 (红色)
        XBOX_CONTROLLER_BUTTON_X,                // 动作 X 键 (蓝色)
        XBOX_CONTROLLER_BUTTON_Y,                // 动作 Y 键 (黄色)
        XBOX_CONTROLLER_BUTTON_BUMPER_LEFT,      // LB 左肩键
        XBOX_CONTROLLER_BUTTON_BUMPER_RIGHT,     // RB 右肩键
        XBOX_CONTROLLER_BUTTON_THUMBSTICK_LEFT,  // L3 左摇杆下压点击
        XBOX_CONTROLLER_BUTTON_THUMBSTICK_RIGHT, // R3 右摇杆下压点击
        XBOX_CONTROLLER_BUTTON_VIEW,             // View 视图键 (旧称 Back / Select)
        XBOX_CONTROLLER_BUTTON_MENU,             // Menu 菜单键 (旧称 Start)
        XBOX_CONTROLLER_BUTTON_XBOX,             // 西瓜键 (Guide 导航键)
        XBOX_CONTROLLER_BUTTON_SHARE             // Share 分享键 (新款手柄具备)
    } xbox_controller_button_t;

    /**
     * @brief 手柄上所有离散数字按钮的布尔状态集合
     * @note  true 代表当前按键处于被按下状态，false 代表已松开
     */
    typedef struct
    {
        // 手柄右侧的经典四按键
        bool button_a; // A 键 (绿色)
        bool button_b; // B 键 (红色)
        bool button_x; // X 键 (蓝色)
        bool button_y; // Y 键 (黄色)

        // 顶部肩键 (Bumpers)
        bool bumper_left;  // LB 键 (左肩键，位于左扳机上方)
        bool bumper_right; // RB 键 (右肩键，位于右扳机上方)

        // 摇杆下压点击按键 (Thumbstick Click / 俗称 L3 和 R3)
        bool thumbstick_click_left;  // LSB 键 (把左摇杆当按键垂直用力往下按)
        bool thumbstick_click_right; // RSB 键 (把右摇杆当按键垂直用力往下按)

        // 系统与菜单辅助按键
        bool button_view;  // View 键 (双窗口图标)
        bool button_menu;  // Menu 键 (三道杠图标)
        bool button_xbox;  // 西瓜键 (Xbox 标志键，俗称 Guide 键)
        bool button_share; // Share 键 (正中间下方的小分享键)
    } xbox_controller_buttons_t;

    /**
     * @brief Xbox 手柄在某一个瞬间的完整物理姿态快照
     * @note  包含双摇杆坐标、双线性扳机量、十字键以及所有按键的最新状态
     */
    typedef struct
    {
        /*
         * 左模拟摇杆坐标 (-32768 ~ 32767)：
         * 驱动已完成数学居中校准！
         * 自然居中时数值约为 0；向左推为负，向右推为正；向下推为负，向上推为正。
         */
        int16_t joystick_left_x; // 左摇杆左右位移 (-32768: 极左, +32767: 极右)
        int16_t joystick_left_y; // 左摇杆上下位移 (-32768: 极下, +32767: 极上)

        /* 右模拟摇杆坐标 (-32768 ~ 32767)：标准与左摇杆完全一致 */
        int16_t joystick_right_x; // 右摇杆左右位移
        int16_t joystick_right_y; // 右摇杆上下位移

        /*
         * 模拟线性扳机键深度 (0 ~ 1023)：
         * 手指完全松开时为 0，完全死死按到底时为 1023。
         * 可用于控制赛车油门、刹车线性减速、或机械臂无级开合。
         */
        uint16_t trigger_left;  // 左扳机键 (LT 键)
        uint16_t trigger_right; // 右扳机键 (RT 键)

        // 十字键离散朝向
        xbox_controller_dpad_direction_t dpad_direction;

        // 数字按键状态合集
        xbox_controller_buttons_t buttons;

        // 连接有效性标志 (true 代表手柄物理在线)
        bool is_connected;
    } xbox_controller_state_t;

    /* ========================================================================= */
    /* 3. 跨平台传输抽象与事件系统                                                */
    /* ========================================================================= */

    /**
     * @brief 底层物理发包函数指针原型 (由主程序实现并注入手柄驱动)
     * @note  手柄驱动要发马达震动等指令时，自动调用此函数指针。
     *        在 ESP32 上对应 ble_manager_central_write。
     */
    typedef int (*xbox_controller_send_bytes_function_t)(
        const uint8_t *data,
        uint16_t length,
        void *user_context);

    typedef enum
    {
        XBOX_CONTROLLER_EVENT_CONNECTED,              // 手柄已连接并唤醒就绪 (指示灯常亮)
        XBOX_CONTROLLER_EVENT_DISCONNECTED,           // 手柄断开连接
        XBOX_CONTROLLER_EVENT_STATE_CHANGED,          // 手柄姿态发生改变 (按键被按/摇杆被推)
                                                      //
        XBOX_CONTROLLER_EVENT_BATTERY_LEVEL_UPDATED,  // 电池电量更新事件 (手柄推送电量变化时反复调用触发)
        XBOX_CONTROLLER_EVENT_SERIAL_NUMBER_FETCH,    // 向手柄获取出厂序列号事件 (初始化时触发：通知底层去向手柄硬件读取 SN 码)
        XBOX_CONTROLLER_EVENT_SERIAL_NUMBER_QUERY,    // 查询出厂序列号事件 (调用 get 函数读取已存好的 SN)
        XBOX_CONTROLLER_EVENT_FIRMWARE_VERSION_FETCH, // 向手柄获取固件版本号事件
        XBOX_CONTROLLER_EVENT_FIRMWARE_VERSION_QUERY  // 查询固件版本号事件
    } xbox_controller_event_t;

    typedef void (*xbox_controller_event_callback_t)(
        xbox_controller_event_t event,
        const xbox_controller_state_t *current_state,
        void *user_context);

    // 驱动配置项
    typedef struct
    {
        xbox_controller_send_bytes_function_t send_function; // 必须提供：底层发包函数
        xbox_controller_event_callback_t event_callback;     // 可选提供：事件监听回调
        void *user_context;                                  // 可选提供：用户上下文指针
    } xbox_controller_config_t;

    /* ========================================================================= */
    /*  一、辅助管理与解耦数据 (Auxiliary)                                        */
    /* ========================================================================= */

    /**
     * @brief 初始化手柄协议核心
     */
    xbox_controller_status_t xbox_controller_init(const xbox_controller_config_t *config);

    /**
     * @brief 注销并重置手柄协议核心内部所有状态
     */
    xbox_controller_status_t xbox_controller_deinit(void);

    /**
     * @brief 【核心解耦数据入口】：将底层收到的原始报文“喂”入协议引擎
     * @note  无论底层是 ESP32 BLE、STM32 串口无线透传模块、还是 USB 线，
     *        底层一旦收到了手柄推过来的 16 字节原始报文，直接调用此函数！
     *        内部会自动完成数学归一化、位掩码提取，并通知上层业务。
     *
     * @param raw_data 原始报文字节指针
     * @param length   数据长度 (手柄标准为 16 字节)
     */
    void xbox_controller_feed_input_report(const uint8_t *raw_data, uint16_t length);

    /**
     * @brief 通知手柄协议引擎：物理连接已建立
     */
    void xbox_controller_notify_connected(void);

    /**
     * @brief 通知手柄协议引擎：物理连接已断开
     */
    void xbox_controller_notify_disconnected(void);

    /**
     * @brief 设置摇杆死区阈值 (抗漂移利器)
     * @note  手柄用久了，自然松手时摇杆可能不会完美回到 0。设置死区后，位移小于该阈值的
     *        微小晃动将被强制当作 0 处理，杜绝小车或机械臂原地偷跑！
     *
     * @param deadzone_threshold 死区阈值 (建议 1000 ~ 3000，默认出厂推荐设为 1500)
     */
    xbox_controller_status_t xbox_controller_set_joystick_deadzone(uint16_t deadzone_threshold);

    /* ========================================================================= */
    /*  二、控制输入与姿态查询接口 (Inputs)                                       */
    /* ========================================================================= */

    /**
     * @brief 查询某一颗指定按键当前是否处于按下状态
     * @example if (xbox_controller_is_button_pressed(XBOX_CONTROLLER_BUTTON_A)) { ... }
     *
     * @param button 目标按键枚举 (如 XBOX_CONTROLLER_BUTTON_A)
     * @return true 被按下, false 松开或手柄未连接
     */
    bool xbox_controller_is_button_pressed(xbox_controller_button_t button);

    /**
     * @brief 获取左模拟摇杆的归一化坐标 (-32768 ~ 32767，居中为 0)
     */
    xbox_controller_status_t xbox_controller_get_joystick_left(int16_t *out_x, int16_t *out_y);

    /**
     * @brief 获取右模拟摇杆的归一化坐标 (-32768 ~ 32767，居中为 0)
     */
    xbox_controller_status_t xbox_controller_get_joystick_right(int16_t *out_x, int16_t *out_y);

    /**
     * @brief 获取左模拟扳机 (LT) 的线性行程深度 (0 ~ 1023)
     * @note  手指完全松开为 0，完全按死为 1023；未连接时安全返回 0 (完全松开)
     */
    uint16_t xbox_controller_get_trigger_left(void);

    /**
     * @brief 获取右模拟扳机 (RT) 的线性行程深度 (0 ~ 1023)
     * @note  手指完全松开为 0，完全按死为 1023；未连接时安全返回 0 (完全松开)
     */
    uint16_t xbox_controller_get_trigger_right(void);

    /**
     * @brief 获取十字方向键 (D-Pad) 的离散朝向
     * @note  未连接时安全返回 XBOX_DPAD_DIRECTION_CENTERED (居中松开状态)
     */
    xbox_controller_dpad_direction_t xbox_controller_get_dpad_direction(void);

    /**
     * @brief 获取手柄当前完整的物理姿态快照 (供主程序 while(1) 轮询使用)
     */
    xbox_controller_status_t xbox_controller_get_state(xbox_controller_state_t *out_state);

    /* ========================================================================= */
    /* 三、马达震动与强度控制接口 (Outputs)                                        */
    /* ========================================================================= */
    /**
     * @brief 独立控制机身两颗主握把马达 (左主大马达 + 右副小马达)
     *
     * @param left_intensity         左握把主大马达强度 (0 ~ 100)
     * @param right_intensity        右握把副小马达强度 (0 ~ 100)
     * @param duration_in_10ms_units 震动持续时间 (单位: 10ms，0 ~ 255，如 20 代表 200ms)
     * @param delay_in_10ms_units    启动延时时间 (单位: 10ms，0 ~ 255，0 代表立即起震)
     * @param loop_count             脉冲循环次数 (0 为单次执行不循环，1~254 为重复N次)
     */
    xbox_controller_status_t xbox_controller_set_main_motors(
        uint8_t left_intensity,
        uint8_t right_intensity,
        uint8_t duration_in_10ms_units,
        uint8_t delay_in_10ms_units,
        uint8_t loop_count);

    /**
     * @brief 独立控制左右两颗扳机微型脉冲马达 (LT/RT 扳机)
     *
     * @param left_trigger_intensity  左扳机马达强度 (0 ~ 100)
     * @param right_trigger_intensity 右扳机马达强度 (0 ~ 100)
     * @param duration_in_10ms_units  震动持续时间 (单位: 10ms，0 ~ 255，如 20 代表 200ms)
     * @param delay_in_10ms_units     启动延时时间 (单位: 10ms，0 ~ 255，0 代表立即起震)
     * @param loop_count              脉冲循环次数 (0 为单次执行不循环，1~254 为重复N次)
     */
    xbox_controller_status_t xbox_controller_set_trigger_motors(
        uint8_t left_trigger_intensity,
        uint8_t right_trigger_intensity,
        uint8_t duration_in_10ms_units,
        uint8_t delay_in_10ms_units,
        uint8_t loop_count);

    /**
     * @brief 触发全功能自定义马达震动 (4颗马达独立强度 + 持续时间 + 启动延时 + 循环次数)
     *
     * @param left_main_percent      左握把主大马达强度 (0 ~ 100 百分比)
     * @param right_main_percent     右握把副小马达强度 (0 ~ 100 百分比)
     * @param left_trigger_percent   左扳机脉冲马达强度 (0 ~ 100 百分比)
     * @param right_trigger_percent  右扳机脉冲马达强度 (0 ~ 100 百分比)
     * @param duration_in_10ms_units 震动持续时间，以 10ms 为一个步进单位 (取值 0 ~ 255，例如 30 代表 300ms)
     * @param delay_in_10ms_units    启动延时时间，以 10ms 为一个步进单位 (取值 0 ~ 255，0 代表立即起震不延时)
     * @param loop_count             脉冲循环次数 (0 为单次执行不循环，1~254 为重复N次，255 为无限循环持续震动)
     * @return xbox_controller_status_t 执行状态码
     */
    xbox_controller_status_t xbox_controller_pulse_custom_vibration(
        uint8_t left_main_percent,
        uint8_t right_main_percent,
        uint8_t left_trigger_percent,
        uint8_t right_trigger_percent,
        uint8_t duration_in_10ms_units,
        uint8_t delay_in_10ms_units,
        uint8_t loop_count);

    /**
     * @brief 立即急停所有马达的震动 (将所有马达强度瞬间清零)
     */
    xbox_controller_status_t xbox_controller_stop_vibration(void);

    /* ========================================================================= */
    /* 四、手柄设备信息获取与查询接口 (Information)                                */
    /* ========================================================================= */

    /**
     * @brief 查询手柄当前是否处于已连接在线状态
     */
    bool xbox_controller_is_connected(void);

    // 获取手柄设备信息原始数据接口(供上层把读回来的蓝牙原始字节数据)
#if CONFIG_APP_ENABLE_XBOX_BATTERY_MONITORING
    void xbox_controller_feed_battery_percentage(uint8_t battery_percentage);
#endif

#if CONFIG_APP_ENABLE_XBOX_DEVICE_INFORMATION
    void xbox_controller_feed_serial_number_raw(const uint8_t *raw_data, uint16_t length);
    void xbox_controller_feed_firmware_version_raw(const uint8_t *raw_data, uint16_t length);
#endif

#if CONFIG_APP_ENABLE_XBOX_BATTERY_MONITORING
    /**
     * @brief 获取当前手柄剩余电量百分比
     * @return 0 ~ 100 代表电量百分比，返回 255 代表尚未获取或手柄离线
     */
    uint8_t xbox_controller_get_battery_percentage(void);
#endif

#if CONFIG_APP_ENABLE_XBOX_DEVICE_INFORMATION
    /**
     * @brief 获取手柄出厂硬件唯一序列号 (只读字符串)
     * @return 字符串指针 (如 "09VX0609661150")，手柄未连接或未就绪时返回 "UNKNOWN"
     */
    const char *xbox_controller_get_serial_number(void);

    /**
     * @brief 获取手柄当前固件版本号 (只读字符串)
     * @return 字符串指针 (如 "5.23.6.0")，手柄未连接或未就绪时返回 "UNKNOWN"
     */
    const char *xbox_controller_get_firmware_version(void);
#endif

    /* ========================================================================= */
    /* 五、耳机音频接口 (Audio Interface - 架构预留)                              */
    /* ========================================================================= */

    /**
     * @brief 向手柄 3.5mm 耳机孔推送扬声器音频 PCM (Pulse-Code Modulation) 数据 (架构预留)
     * @note  重要硬件限制提示：由于低功耗蓝牙 (BLE) 物理带宽限制，微软手柄固件在蓝牙模式下
     *        会强制关闭耳机音频芯片。此接口供未来移植到 USB 或 2.4G 接收器场景时使用。
     */
    xbox_controller_status_t xbox_controller_audio_send_speaker_pcm(
        const uint8_t *pcm_data,
        uint16_t length);

    /**
     * @brief 注册耳机麦克风语音采集回调 (架构预留)
     */
    typedef void (*xbox_controller_audio_microphone_callback_t)(
        const uint8_t *pcm_data,
        uint16_t length,
        void *user_context);

    xbox_controller_status_t xbox_controller_audio_register_microphone_callback(
        xbox_controller_audio_microphone_callback_t callback,
        void *user_context);

#ifdef __cplusplus
}
#endif

#endif
