# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## 项目概述

WHEELTEC C50X 是基于 STM32F407VE 微控制器的智能小车嵌入式固件项目，使用 FreeRTOS 实时操作系统。

**目标芯片**: STM32F407VE (Cortex-M4, 120MHz)
**Flash**: 512KB
**RAM**: 128KB

## 构建系统

项目使用 Keil MDK-ARM (uVision) 进行开发。

### 编译步骤

1. 打开 `USER/WHEELTEC.uvprojx` 工程文件
2. 在 Project 菜单中选择目标车型（见下方车型配置）
3. 点击 Build 按钮编译
4. 编译完成后自动生成 `.hex` 和 `.bin` 文件到根目录

### 车型配置

通过 Keil 工程的 Target 选项选择不同车型，通过预定义宏实现条件编译：

| 宏定义 | 车型 | 目标名称 | 车型数量 |
|--------|------|----------|----------|
| `AKM_CAR` | 阿克曼车 | Akm_Car | 10 |
| `DIFF_CAR` | 差速车 | Diff_Car | 9 |
| `MEC_CAR` | 麦轮车 | Mec_Car | 11 |
| `_4WD_CAR` | 四驱车 | 4WD_Car | 13 |
| `OMNI_CAR` | 全向车 | Omni_Car | 7 |

**重要**: 系统会在编译时检查，确保同时只能定义一种车型（见 `system.h:67-69`）

## 项目架构

### 目录结构

```
USER/           # 主程序入口、系统初始化
CORE/           # STM32 HAL 核心头文件
FWLIB/          # STM32 固件库
FreeRTOS/       # FreeRTOS 实时操作系统
HARDWARE/       # 硬件驱动层
  ├─ Inc/       # 硬件驱动头文件
  ├─ ICM20948/  # ICM20948 IMU 传感器驱动 (V1.1 硬件)
  └─ MPU6050/   # MPU6050 IMU 传感器驱动 (V1.0 硬件)
MiddleWares/    # USB Host 中间件
BALANCE/        # 运动控制核心逻辑
CarType/        # 车型配置和初始化
USB_HOST/       # USB 应用层
```

### FreeRTOS 任务架构

主程序启动后创建以下任务（见 `USER/main.c:55-69`）：

| 任务名称 | 优先级 | 功能说明 |
|----------|--------|----------|
| `start_task` | 1 | 启动任务，创建其他任务后自删除 |
| `Balance_task` | - | 小车运动控制核心任务 |
| `show_task` | - | 数据显示用户交互任务 |
| `led_task` | - | LED 闪烁任务 |
| `data_task` | - | 数据发送到各接口任务 |
| `IMU_task` | - | IMU 数据采集任务 |
| `PS2_task` | - | PS2 手柄读取任务（仅 V1.0 硬件） |

硬件版本通过 `SysVal.HardWare_Ver` 区分：
- `V1_0`: 使用 MPU6050 + PS2 接口
- `V1_1`: 使用 ICM20948 + USB 手柄接口

## 核心数据结构

### ROBOT_t 机器人结构体 (`CarType/Inc/robot_init.h`)

```c
typedef struct {
    Robot_Parament_InitTypeDef HardwareParam;  // 硬件结构参数
    Moto_parameter MOTOR_A/B/C/D;              // 电机参数
    Moto_parameter SERVO;                      // 舵机参数（仅阿克曼车型）
    uint8_t type;                              // 车型号
    float voltage;                             // 电池电压
    int V_KP, V_KI;                           // 速度控制 PID 参数
} ROBOT_t;
```

### 机器人硬件参数 (`Robot_Parament_InitTypeDef`)

- `WheelSpacing`: 轮距（麦轮/四驱为半轮距）
- `AxleSpacing`: 轴距（麦轮/四驱为半轴距）
- `Wheel_Circ`: 主动轮周长
- `GearRatio`: 电机减速比
- `Encoder_precision`: 车轮转一圈对应的编码器数值

### 电机参数 (`Moto_parameter`)

- `Target`: 电机目标速度
- `Encoder`: 编码器实时读数
- `Output`: 输出 PWM 值

## 控制系统

### 运动控制

主要控制逻辑在 `BALANCE/` 目录下：

- `control.h/c`: 运动控制核心，包含 PID 算法、PWM 限幅、运动学分析
- `balance_task.h/c`: 平衡控制任务（100Hz 执行频率）

### 运动学解算

不同车型的运动学正逆解在 `CarType/Src/` 对应的初始化文件中实现：
- `akm_robot_init.c`: 阿克曼运动学
- `diff_robot_init.c`: 差速运动学
- `mec_robot_init.c`: 麦轮运动学
- `4wd_robot_init.c`: 四驱运动学
- `omni_robot_init.c`: 全向运动学

### 控制接口

小车支持多种控制方式：
- **串口控制** (`uartx_callback.h/c`)
- **CAN 控制** (`can_callback.h/c`)
- **PS2 手柄** (V1.0 硬件)
- **USB 手柄** (V1.1 硬件，`USB_HOST/`)
- **航模遥控** (`remote.h/c`)

## 硬件版本兼容

通过 `SysVal.HardWare_Ver` 区分：
- **V1.0**: MPU6050 IMU + PS2 手柄接口
- **V1.1**: ICM20948 IMU + USB 手柄接口

## 重要配置参数

### RTOS 任务频率定义 (`system.h`)

预定义了常用的任务频率宏：
```c
#define RATE_1_HZ      1
#define RATE_5_HZ      5
#define RATE_10_HZ     10
// ... 到 RATE_1000_HZ
```

### 控制延迟

小车启动后有 `CONTROL_DELAY` (默认 1000ms，即 10秒 @ 100Hz) 的延迟时间后才允许控制运动。

### 编码器倍频

`EncoderMultiples = 4`（见 `robot_init.h:88`），编码器使用 4 倍频。

## 关键文件说明

| 文件 | 说明 |
|------|------|
| `USER/main.c` | 主程序入口，任务创建 |
| `USER/system.h/c` | 系统初始化 |
| `CarType/Inc/robot_init.h` | 车型参数定义和结构体 |
| `CarType/Src/*_robot_init.c` | 各车型初始化和运动学 |
| `BALANCE/Inc/control.h` | 运动控制核心接口 |
| `HARDWARE/Inc/motor.h` | 电机驱动 |
| `HARDWARE/Inc/encoder.h` | 编码器读取 |
| `HARDWARE/Inc/auto_recharge.h` | 自动回充功能 |
| `更新记录.txt` | 项目版本更新历史 |

## 版本历史

项目版本记录在根目录 `更新记录.txt` 中，重要更新包括：

- 2023.07.25: 修复 CAN 通信问题
- 2023.11.24: 新增自动回充功能
- 2024.11.25: 合并 5 款车型源码
- 2024.11.28: 新增直线偏移可调参数
- 2024.12.24: 新增 USB 手柄支持
- 2025.05.08: 新增 PID 参数掉电保存功能
- 2025.08.07: 修复阿克曼车型 7 调试问题
