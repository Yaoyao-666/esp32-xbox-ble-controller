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

