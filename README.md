# 基于 ESP32 与 NimBLE 的 Xbox 无线手柄驱动及 BLE 通信框架
## 项目简介
这是一个基于 ESP32 与 Apache NimBLE 开发的 Xbox 无线手柄 BLE 驱动框架，提供手柄按键与摇杆输入解析、四通道独立振动控制、设备配对绑定及自动回连等功能。
采用模块化设计，将手柄协议处理与 BLE 底层通信分离，方便开发者将驱动移植到其他支持 BLE 的平台，并快速集成 Xbox 手柄控制功能。

## 1. 所需硬件与开发环境

### 1.1 硬件要求

- **硬件要求：** ESP32 / ESP32-S3 等支持 BLE 的开发板。
- **手柄硬件：** 支持 Bluetooth BLE 连接的 Xbox 无线手柄，项目主要针对 Xbox Series X|S 手柄开发。
- **连接方式：** 主控作为 BLE Central，手柄作为 BLE Peripheral。
>注意：不同批次及型号的 Xbox 手柄蓝牙兼容性可能存在差异，其他型号需要自行验证。
### 1.2 开发环境

- **开发框架：** ESP-IDF 5.0 及以上版本。
- **蓝牙协议栈：** Apache NimBLE。
- **当前测试环境：** ESP-IDF v5.5.x。

---

## 2. 关键工程配置

### 2.1 menuconfig 配置

根据当前 ESP-IDF 版本，在 NimBLE Options 中确认以下配置：

**1. 开启 BLE Bonding 信息持久化**

启用将 BLE 配对绑定密钥保存到 NVS 的选项，使设备重启后能够恢复历史绑定关系，支持后续安全重连。

配置位置：

```text
Component config
  -> Bluetooth
    -> NimBLE Options
      -> Persist BLE bonding keys in NVS
```

**2. 配置 NimBLE Host 任务栈大小**

将 NimBLE Host task stack size 设置为：

```text
6144
```

可根据实际任务调用深度和运行情况调整，避免任务栈空间不足。

> 注：不同 ESP-IDF 版本的菜单名称可能略有区别，请以当前版本 menuconfig 中的实际选项为准。

### 2.2 驱动功能宏配置

在 `components/xbox_controller/include/xbox_controller.h` 中，根据项目需求配置以下功能：

| 宏定义 | 作用 | 配置值 |
|---|---|------|
| `define CONFIG_APP_ENABLE_XBOX_BATTERY_MONITORING` | 电池电量读取与通知功能 | 0 / 1 |
| `define CONFIG_APP_ENABLE_XBOX_DEVICE_INFORMATION` | 设备序列号及固件版本读取功能 | 0 / 1 |

- `0`：关闭对应功能。
- `1`：启用对应功能。

---

## 3. 项目目录结构

```text
├── components/
│   │
│   ├── bluetooth_low_energy_manager/
│   │   ├── include/
│   │   │   └── ble_manager.h
│   │   └── src/
│   │       └── ble_manager.c
│   │
│   └── xbox_controller/
│       ├── include/
│       │   └── xbox_controller.h
│       └── src/
│           └── xbox_controller.c
│
└── main/
    ├── CMakeLists.txt
    └── main.c
```

### 模块职责

| 模块 | 主要职责 |
|---|---|
| `define bluetooth_low_energy_manager` | BLE 连接管理、GATT 操作、通知订阅及 SMP 安全管理。 |
| `define xbox_controller` | Xbox 手柄协议解析、输入状态管理及振动控制报文构造。 |
| `main` | 应用层入口，负责模块初始化、通信接口对接及业务逻辑。 |

**移植说明：**

- `bluetooth_low_energy_manager`：需要根据目标平台的 BLE 协议栈适配底层通信实现。
- `xbox_controller`：驱动协议核心采用纯标准 C 语言实现，不直接依赖 ESP-IDF 和 FreeRTOS，可作为独立模块移植。
- `main`：根据实际应用需求实现初始化、事件回调及业务控制逻辑。

---

## 附录：Xbox 无线手柄 BLE 协议技术文档 (实测标定版)

### 表格一、 GATT 服务与特征参数表

| 服务 | 类型 | UUID | 实测 Handle | 参数 | 通信方向 | 传输权限 | 职能说明 |
|---|---|---|---:|---|---|---|---|
| HID | Service | `0x1812` | `22–35` | — | — | — | HID 服务范围 |
| HID | Characteristic | `0x2A4C` | `26` | `写入0x01` | ESP32 → 手柄 | Write | 手柄激活唤醒 |
| HID | Characteristic | `0x2A4D` | `30` | 16 Bytes | 手柄 → ESP32 | Read / Notify | 按键输入数据通道 |
| HID | Descriptor | `0x2902` | `31` | `写入0x0001` | ESP32 → 手柄 | Write | 按键订阅开关 |
| HID | Characteristic | `0x2A4D` | `34` | 8 Bytes | ESP32 → 手柄 | Write Without Response | 马达震动控制 |
| Battery | Service | `0x180F` | — | — | — | — | 电池服务 |
| Battery | Characteristic | `0x2A19` | `20` | 0–100 | 手柄 → ESP32 | Read / Notify | 电池电量信息 |
| Battery | Descriptor | `0x2902` | `21` | `写入0x0001` | ESP32 → 手柄 | Write | 电量订阅开关 |
| Device Information | Service | `0x180A` | — | — | — | — | 设备信息服务 |
| Device Information | Characteristic | `0x2A29` | 未记录 | 字符串 | 手柄 → ESP32 | Read | 厂商名称 |
| Device Information | Characteristic | `0x2A50` | 未记录 | PnP ID | 手柄 → ESP32 | Read | 厂商 ID、产品 ID、产品版本 |
| Device Information | Characteristic | `0x2A26` | `15` | 字符串 | 手柄 → ESP32 | Read | 固件版本 |
| Device Information | Characteristic | `0x2A25` | `17` | 字符串 | 手柄 → ESP32 | Read | 设备序列号 |

### 表格二、 HID Input Report 输入报文映射表

输入报文 UUID：`0x2A4D`  
数据长度：`16 Bytes`  
通信方向：Xbox 手柄 → ESP32（Notification）

| 字节偏移 | 控制部件 | 数据格式 | 位定义 / 编码 | 解析结果 |
|---|---|---|---|---|
| Byte 0–1 | 左摇杆 X 轴 | 16 位无符号小端 | `uint16_t`，中心值约 `32768` | `-32768 ~ 32767` |
| Byte 2–3 | 左摇杆 Y 轴 | 16 位无符号小端 | `uint16_t`，中心值约 `32768` | `-32768 ~ 32767` |
| Byte 4–5 | 右摇杆 X 轴 | 16 位无符号小端 | `uint16_t`，中心值约 `32768` | `-32768 ~ 32767` |
| Byte 6–7 | 右摇杆 Y 轴 | 16 位无符号小端 | `uint16_t`，中心值约 `32768` | `-32768 ~ 32767` |
| Byte 8–9 | 左模拟扳机（LT） | 10 位模拟量 | `0 ~ 1023` | `0 ~ 1023` |
| Byte 10–11 | 右模拟扳机（RT） | 10 位模拟量 | `0 ~ 1023` | `0 ~ 1023` |
| Byte 12 | 十字方向键（D-Pad） | 8 向帽开关 | `0`：释放；`1–8`：顺时针方向编码 | 上、右上、右、右下、下、左下、左、左上 |
| Byte 13 | 核心动作按键 | 8 位位掩码 | `Bit0=A`、`Bit1=B`、`Bit3=X`、`Bit4=Y`、`Bit6=LB`、`Bit7=RB` | A、B、X、Y、LB、RB 按键状态 |
| Byte 14 | 系统辅助按键 1 | 8 位位掩码 | `Bit2=View`、`Bit3=Menu`、`Bit4=Xbox`、`Bit5=L3`、`Bit6=R3` | View、Menu、Xbox、L3、R3 按键状态 |
| Byte 15 | 系统辅助按键 2 | 8 位位掩码 | `Bit0=Share` | Share 按键状态 |

### 表格三、 HID Output Report 震动输出报文映射表

UUID：`0x2A4D`  
数据长度：`8 Bytes`  
通信方向：ESP32 → Xbox 手柄  
传输方式：Write Without Response

| 字节偏移 | 控制参数 | 数据格式 / 取值范围 | 控制效果 |
|---|---|---|---|
| Byte 0 | 马达使能掩码 | 固定 `0x0F` | 使能四个振动通道 |
| Byte 1 | 左扳机脉冲马达 | `0–100`（%） | 控制左扳机触觉反馈 |
| Byte 2 | 右扳机脉冲马达 | `0–100`（%） | 控制右扳机触觉反馈 |
| Byte 3 | 左握把主大马达 | `0–100`（%） | 产生低频、强烈的握把震动 |
| Byte 4 | 右握把副小马达 | `0–100`（%） | 产生高频、细腻的握把震动 |
| Byte 5 | 震动持续时间（Duration） | `0–255`（单位：10 ms） | 定义每一波震动转动多久。如 30 = 300 毫秒 |
| Byte 6 | 延迟起震时间（Delay） | `0–255`（单位：10 ms） | 定义两波震动之间的停顿时间 |
| Byte 7 | 脉冲循环次数（Loop Count） | `0–255`（实际执行次数：`Loop Count + 1`） | 控制震动脉冲的重复次数 |

 #### 震动控制说明
Byte 5、Byte 6 和 Byte 7 共同决定震动的持续时间、间隔及重复次数。根据当前测试手柄的实测结果，分为以下两种模式：

**模式一：连续震动（Delay = 0）**
当 Byte 6 设置为 0 时，多个振动周期之间没有停顿，形成连续振动效果。
- Duration = 100（每次 1 秒）
- Delay = 0
- Loop Count = 2
- 实际效果：连续震动约 3 秒。

**模式二：间歇脉冲震动（Delay > 0）**
  当 Byte 6 大于 0 时，振动按照持续时间与间隔时间重复执行，形成机械节奏式的脉冲效果。
- Duration = 10（每次 100 ms）
- Delay = 10（间隔 100 ms）
- Loop Count = 2
- 实际效果(间隔震动3次)：停顿 ->震动 -> 停顿 -> 震动 -> 停顿 ->震动




