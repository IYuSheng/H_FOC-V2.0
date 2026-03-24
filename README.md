# H_FOC V2.0

<p align="center">
  <img src="./H_FOC_V2.0_image.jpg" alt="H_FOC V2.0 board render" width="920" />
</p>

<p align="center">
  面向机器人关节驱动与双关节机械臂实验平台的软硬件一体化项目
</p>

<p align="center">
  <img src="https://img.shields.io/badge/Hardware-H__FOC_V2.0-C96A2B?style=flat-square" alt="Hardware Badge" />
  <img src="https://img.shields.io/badge/MCU-STM32G431RBT6-0F766E?style=flat-square" alt="MCU Badge" />
  <img src="https://img.shields.io/badge/Control-FOC_20kHz-1D4ED8?style=flat-square" alt="Control Badge" />
  <img src="https://img.shields.io/badge/Bus-CAN_FD_1M%2F2M-374151?style=flat-square" alt="Bus Badge" />
</p>

<p align="center">
  <a href="#项目概览">项目概览</a> |
  <a href="#系统架构">系统架构</a> |
  <a href="#快速开始">快速开始</a> |
  <a href="#通信协议">通信协议</a> |
  <a href="#设计资料">设计资料</a>
</p>

## 项目概览

`H_FOC V2.0` 面向机器人关节电机控制场景，核心目标是构建一块可用于关节驱动、实验验证和上位机联调的 FOC 控制板，并围绕它形成一套可持续迭代的软件工具链。

从当前仓库内容来看，这个项目已经覆盖了以下几条主线：

- 基于 `STM32G431RBT6` 的电机控制固件
- 基于 `AS5048A` 的编码器闭环采样
- 基于 `CAN FD` 的主从关节通信
- 面向双关节平面机械臂的运动学、示教与重力补偿
- 基于 `Qt6` 的桌面上位机
- PCB 资料、交互式 BOM、结构模型和芯片手册归档

## 亮点特性

- `20 kHz` PWM / 电流环控制，使用 `CORDIC`、`FMAC` 等 STM32G4 外设辅助运算。
- 编码器经 `SPI2` 读取角度，并在固件中完成机械角、电角度、速度和加速度观测。
- 内置过压、欠压、过流、过温、编码器异常、采样异常等保护状态机。
- 支持 `CAN FD` 状态上报与控制指令下发，协议字段简洁清晰，适合继续扩展。
- MIT 风格控制命令已接入：目标电流、目标角度、目标速度、目标加速度、目标 jerk、刚度、阻尼、控制模式。
- `S-curve` 轨迹规划已在固件侧接入，`trapezoidal` 模式接口已预留。
- Qt 上位机已经实现双关节机械臂工作区显示、点击示教、控制模式切换和实时状态展示。
- 仓库同时保留了结构设计、BOM 和芯片资料，适合继续做硬件迭代和文档整理。

## 仓库结构

```text
H_FOC_V2.0/
|-- Code/
|   |-- H_FOC V2.1/                # 历史固件版本
|   |-- H_FOC V2.2/                # 当前主固件版本
|   |-- H_FOC_Robot_App/           # Qt6 机器人上位机
|   |-- control.py                 # 串口版控制/可视化脚本
|   `-- control_can.py             # CAN FD 版控制/可视化脚本
|-- Control_GUI/
|   `-- arm_ui/
|       `-- main.py                # RC + Serial + CAN(SLCAN) 调试界面
|-- 结构/                           # SolidWorks 结构文件
|-- 相关芯片手册/                    # 芯片资料归档
|-- InteractiveBOM_PCB1_2025-12-18.html
|-- H_FOC V2.0.docx
|-- H_FOC_V2.0_image.jpg
`-- README.md
```

### 模块索引

| 模块 | 路径 | 说明 |
| --- | --- | --- |
| 主固件 | [`Code/H_FOC V2.2`](./Code/H_FOC%20V2.2) | 推荐从这里开始，包含 FOC、编码器、CAN FD、轨迹规划与保护逻辑 |
| 旧版固件 | [`Code/H_FOC V2.1`](./Code/H_FOC%20V2.1) | 保留的历史版本，可用于对照差异 |
| Qt 上位机 | [`Code/H_FOC_Robot_App/H_FOC_Robot_App`](./Code/H_FOC_Robot_App/H_FOC_Robot_App) | 双关节机械臂界面、CAN FD 通信、运动学与控制参数配置 |
| Python CAN 工具 | [`Code/control_can.py`](./Code/control_can.py) | CAN FD 监控、示教、仿真与控制 |
| Python 串口工具 | [`Code/control.py`](./Code/control.py) | 基于 UART 文本协议的控制与可视化 |
| 调试 GUI | [`Control_GUI/arm_ui/main.py`](./Control_GUI/arm_ui/main.py) | RC + Serial + SLCAN 联调工具 |
| 交互式 BOM | [`InteractiveBOM_PCB1_2025-12-18.html`](./InteractiveBOM_PCB1_2025-12-18.html) | 浏览器可直接打开 |
| 结构模型 | [`结构`](./结构) | 结构件、座子和测试台架模型 |
| 芯片资料 | [`相关芯片手册`](./相关芯片手册) | MCU、编码器等资料归档 |
| 设计说明 | [`H_FOC V2.0.docx`](./H_FOC%20V2.0.docx) | 板卡引脚与说明文档 |

## 固件概览

主固件位于 [`Code/H_FOC V2.2`](./Code/H_FOC%20V2.2)，从代码可以确认它的关键特征如下：

| 项目 | 内容 |
| --- | --- |
| 主控 MCU | `STM32G431RBT6` |
| 系统主频 | `170 MHz` |
| PWM 频率 | `20 kHz` |
| 编码器 | `AS5048A` |
| 总线 | `FDCAN1` |
| 调试串口 | `USART3` |
| 工程文件 | `MDK-ARM/H_FOC V2.0.uvprojx` |
| CubeMX 文件 | `H_FOC V2.0.ioc` |

默认参数中可读到以下安全和控制边界：

| 项目 | 默认值 |
| --- | --- |
| 母线电压限制 | `30.0 V` |
| 最大相电流 | `5.5 A` |
| 欠压阈值 | `9.0 V` |
| 过温阈值 | `60.0 °C` |
| 电机极对数 | `11` |
| 机械角度保护范围 | `-170° ~ 70°` |

### 固件主入口

- 参数配置：[`Code/H_FOC V2.2/APP/Config.h`](./Code/H_FOC%20V2.2/APP/Config.h)
- 系统入口：[`Code/H_FOC V2.2/Core/Src/main.c`](./Code/H_FOC%20V2.2/Core/Src/main.c)
- 初始化：[`Code/H_FOC V2.2/APP/FOC_Init.c`](./Code/H_FOC%20V2.2/APP/FOC_Init.c)
- 编码器驱动：[`Code/H_FOC V2.2/APP/foc_encoder.c`](./Code/H_FOC%20V2.2/APP/foc_encoder.c)
- 控制主逻辑：[`Code/H_FOC V2.2/APP/foc_control.c`](./Code/H_FOC%20V2.2/APP/foc_control.c)
- 通信封装：[`Code/H_FOC V2.2/MID/foc_communication.c`](./Code/H_FOC%20V2.2/MID/foc_communication.c)
- CAN FD 协议：[`Code/H_FOC V2.2/Core/Inc/fdcan.h`](./Code/H_FOC%20V2.2/Core/Inc/fdcan.h)

## 上位机与工具链

### Qt6 机器人上位机

路径：[`Code/H_FOC_Robot_App/H_FOC_Robot_App`](./Code/H_FOC_Robot_App/H_FOC_Robot_App)

当前功能：

- `Qt 6.5+`
- `Qt Widgets + Qt SerialPort`
- 双关节机械臂工作区可视化
- 点击画布设置末端目标点
- 实时显示目标位置、当前姿态、关节角速度、电流与通信状态
- 提供 `normal / trapezoidal / s_curve` 控制模式选项
- 当前固件侧重点接入的是 `normal / s_curve`
- 可配置刚度、阻尼、速度、加速度、jerk

关键配置入口：

- [`Code/H_FOC_Robot_App/H_FOC_Robot_App/core/appconfig.h`](./Code/H_FOC_Robot_App/H_FOC_Robot_App/core/appconfig.h)

## 通信协议

### CAN FD 协议摘要

当前固件与上位机使用的协议要点如下：

- 标准 `11-bit CAN ID`
- ID 组织方式：`(src << 8) | (dst << 4) | type`
- 主节点 ID：`0`
- 默认关节节点：`1 / 2`
- 广播地址：`0x0F`
- 仲裁段速率：`1 Mbps`
- 数据段速率：`2 Mbps`
- 定点缩放比例：`1e-5`

### 状态帧 `STATUS`

| 字段 | 类型 |
| --- | --- |
| position | `int32` |
| velocity | `int32` |
| current | `int32` |
| temperature | `int32` |
| acceleration | `int32` |

### 控制帧 `CONTROL`

| 字段 | 类型 |
| --- | --- |
| target_current | `int32` |
| target_angle | `int32` |
| target_velocity | `int32` |
| target_acceleration | `int32` |
| target_jerk | `int32` |
| stiffness | `int32` |
| damping | `int32` |
| control_mode | `uint8` |

### UART 文本命令

固件串口解析逻辑当前可识别以下文本命令：

- `target_position:<float>`
- `target_speed:<float>`
- `current_q:<float>`
- `target_q:<float>`

这使得在没有 CAN FD 的情况下，仍然可以通过串口完成基础联调。

## 快速开始

### 环境准备

建议准备以下工具链：

- `Keil MDK` 或可兼容当前工程的 ARM 编译环境
- `STM32CubeMX`，用于查看或再生成外设配置
- `Qt 6.5+`
- CAN FD 转接设备，或支持 `SLCAN` 的串口桥接设备

### 固件构建

1. 打开 [`Code/H_FOC V2.2/MDK-ARM/H_FOC V2.0.uvprojx`](./Code/H_FOC%20V2.2/MDK-ARM/H_FOC%20V2.0.uvprojx)
2. 根据硬件修改 [`Code/H_FOC V2.2/APP/Config.h`](./Code/H_FOC%20V2.2/APP/Config.h)
3. 编译并下载到 `STM32G431RBT6`
4. 如需查看外设配置，可打开 [`Code/H_FOC V2.2/H_FOC V2.0.ioc`](./Code/H_FOC%20V2.2/H_FOC%20V2.0.ioc)

### Qt 上位机构建

```powershell
cd "Code/H_FOC_Robot_App/H_FOC_Robot_App"
cmake -S . -B build
cmake --build build --config Release
```

依赖：

- `Qt 6.5+`
- `Qt SerialPort`

## 配置入口

如果你要改默认参数，通常会从这些文件开始：

| 场景 | 文件 |
| --- | --- |
| 固件控制参数 | [`Code/H_FOC V2.2/APP/Config.h`](./Code/H_FOC%20V2.2/APP/Config.h) |
| Qt 上位机默认通道和控制参数 | [`Code/H_FOC_Robot_App/H_FOC_Robot_App/core/appconfig.h`](./Code/H_FOC_Robot_App/H_FOC_Robot_App/core/appconfig.h) |

## 设计资料

保留了较完整的研发资料入口：

- 板卡外观图：[`H_FOC_V2.0_image.jpg`](./H_FOC_V2.0_image.jpg)
- 交互式 BOM：[`InteractiveBOM_PCB1_2025-12-18.html`](./InteractiveBOM_PCB1_2025-12-18.html)
- 结构模型：[`结构`](./结构)
- 芯片手册：[`相关芯片手册`](./相关芯片手册)
- 设计说明：[`H_FOC V2.0.docx`](./H_FOC%20V2.0.docx)

<details>
<summary>展开查看板卡主要引脚摘要</summary>

- ADC 采样：`PC0 / PC1 / PC2 / PC3` = `VA / VB / VC / VBUS`
- 电流采样：`PA0 / PA1 / PA2` = `IA / IB / IC`
- 温度采样：`PC4`
- 三相驱动高端：`PC6 / PC7 / PC8`
- 三相驱动低端：`PC10 / PC11 / PC12`
- 编码器：`PB12 / PB13 / PB14 / PB15`
- 调试 DAC：`PA4 / PA5`
- 调试串口：`PB10 / PB11`
- 屏幕接口：`PB3 / PB5 / PB6 / PB7 / PA15`
- CAN：`PB8 / PB9`
- 按键：`PA8 / PA9`
- 三色灯：`PB0 / PB1 / PB2`

</details>

## 安全提示

涉及电机、功率板和机械臂时，建议至少注意以下几点：

- 首次上电时使用受限电源或降低电压。
- 先确认编码器方向、零位和关节限位是否正确。
- 在低速、空载或脱开机械结构的状态下先验证闭环。
- 修改电流、电压和刚度参数前，先确认硬件承受范围。
- 在真实机械臂联调时，避免直接在满量程和无软限位保护下运行。