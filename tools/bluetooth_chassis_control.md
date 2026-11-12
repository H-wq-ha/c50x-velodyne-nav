# 蓝牙控制 WHEELTEC 底盘

工具：`tools/bluetooth_chassis_control.py`。直接使用当前 STM32 固件的 **UART4 蓝牙 APP 协议**，无需启动 ROS2，无需修改或重新烧录底盘固件。

航模入口需要真实的四路 PWM 输出硬件；普通电脑无法仅靠 Python 和普通串口产生这些信号。本工具采用蓝牙模块透传 APP 指令的路径：

```text
电脑 Python → 蓝牙 BLE 或 SPP → 底盘蓝牙模块 → UART4 → APP 控制逻辑 → 电机
```

## 连接条件

- 电脑有可用的蓝牙适配器，底盘有连接 UART4 的蓝牙串口透传模块。
- BLE 使用 Linux BlueZ，适配实测 `WHEELTEC-IOS` 的 FFE0 服务、FFE1 透传特征；不向 FFE2 特征发送任何数据。SPP 使用虚拟串口或 RFCOMM。
- SPP 先在操作系统中配对；BLE 按地址直接连接，不要求预先配对。关闭占用同一连接的手机 APP。运动时停止其他串口控制、航模遥控和自动回充控制，避免它们切换控制模式。
- 底盘上电约 13 秒后再运行运动命令；固件前约 10 秒忽略 APP 命令，随后还会自检。
- 首次验证运动时将驱动轮架空。BLE 状态接收已使用实车验证，运动发送仅经过离线模拟验证，没有实际驱动车辆。

## 当前车辆：优先使用 BLE

实测可用设备为 `WHEELTEC-IOS`，地址 `2A:A2:19:08:05:45`。当前主机的系统 Python 已提供 D-Bus 和 GLib 支持，直接使用 `/usr/bin/python3`，不需要安装 bleak。普通 `python3` 可能指向另一个缺少这些组件的环境。

以下命令在 `/home/y/arm_car/lidar` 执行。

只读状态，不发送速度、方向或模式切换指令：

```bash
/usr/bin/python3 tools/bluetooth_chassis_control.py --ble 2A:A2:19:08:05:45 status --duration 5
```

键盘控制普通四驱底盘，遥控速度基准 0.1 m/s：

```bash
/usr/bin/python3 tools/bluetooth_chassis_control.py --ble 2A:A2:19:08:05:45 --chassis 4wd keyboard --speed 0.1
```

定时前进 2 秒后停车：

```bash
/usr/bin/python3 tools/bluetooth_chassis_control.py --ble 2A:A2:19:08:05:45 --chassis 4wd move forward --speed 0.1 --duration 2
```

发送停车：

```bash
/usr/bin/python3 tools/bluetooth_chassis_control.py --ble 2A:A2:19:08:05:45 stop
```

若设备尚未被发现，工具先扫描该地址，然后连接并开启 FFE1 通知；默认每个连接阶段最多等待 15 秒，可用 `--connect-timeout` 调整。退出时释放自己的通知、扫描和连接；设备被本机其他会话占用时拒绝复用。BLE 不需要设置波特率。其他 Linux 主机如缺少组件，可安装发行版的 `python3-dbus`、`python3-gi` 和 BlueZ。

同一套 APP 指令用于 BLE 和 SPP。BLE 写入使用 GATT Write Request，确认的只是蓝牙层收到数据，运动或停车结果仍无专用确认帧。实现依据：[BlueZ GATT API](https://bluez.readthedocs.io/en/latest/gatt-api/)、[BlueZ 适配器 API](https://bluez.readthedocs.io/en/latest/adapter-api/)。

## 其他设备：SPP 连接

安装串口依赖：

```bash
python3 -m pip install pyserial
```

Windows 可使用配对后提供的出站串口，例如 `COM6`；Linux 可使用已经建立绑定的 `/dev/rfcomm0`。不要使用普通上位机的 `/dev/wheeltec_chassis`：它一般对应 USART3，协议不同，工具会因收不到 APP 参数回读而拒绝发送运动动作。

Linux 如提供 BlueZ 的 `sdptool` / `rfcomm`，可以先查询模块的 Serial Port 服务通道，再绑定串口。以下地址是示例，必须替换；通道 `1` 也必须以查询结果为准：

```bash
sdptool browse AA:BB:CC:DD:EE:FF
sudo rfcomm bind 0 AA:BB:CC:DD:EE:FF 1
```

该绑定不等于已经完成配对，串口访问仍需要相应权限。用完可执行 `sudo rfcomm release 0`。命令依据：[BlueZ rfcomm 手册](https://man.archlinux.org/man/rfcomm.1.en)、[BlueZ sdptool 手册](https://man.archlinux.org/man/sdptool.1.en)。

如果当前 Python 提供蓝牙 socket，也可以使用 `--address AA:BB:CC:DD:EE:FF --channel 1` 直接连接，无需虚拟串口。不支持蓝牙 socket 的 Python 会提示改用 `--port`，依据：[Python socket 文档](https://docs.python.org/3/library/socket.html)。

## 使用

以下命令在 `/home/y/arm_car/lidar` 执行。连接参数和车型参数放在子命令前。

先预览协议字节，不打开蓝牙或串口：

```bash
python3 tools/bluetooth_chassis_control.py --dry-run move forward --speed 0.1 --duration 2
```

连接后只查看 APP 回传，不发送任何指令：

```bash
python3 tools/bluetooth_chassis_control.py --port /dev/rfcomm0 status --duration 5
```

四驱底盘以 0.1 m/s 的遥控速度基准前进 2 秒，然后发送停车：

```bash
python3 tools/bluetooth_chassis_control.py --port /dev/rfcomm0 --chassis 4wd move forward --speed 0.1 --duration 2
```

后退、左转、沿右弧线前进：

```bash
python3 tools/bluetooth_chassis_control.py --port /dev/rfcomm0 move backward --speed 0.1 --duration 2
python3 tools/bluetooth_chassis_control.py --port /dev/rfcomm0 move left --speed 0.1 --duration 1
python3 tools/bluetooth_chassis_control.py --port /dev/rfcomm0 move forward-right --speed 0.1 --duration 2
```

停车（不执行模式激活）：

```bash
python3 tools/bluetooth_chassis_control.py --port /dev/rfcomm0 stop
```

键盘控制（Linux/macOS 的交互终端）：

```bash
python3 tools/bluetooth_chassis_control.py --port /dev/rfcomm0 --chassis 4wd keyboard --speed 0.1
```

| 按键 | 动作 |
|---|---|
| W / S | 前进 / 后退 |
| A / D | 左转 / 右转，阿克曼车型禁用原地旋转 |
| Q / E | 前进并向左 / 右转；全向车型为斜向平移 |
| Z / C | 后退并向左 / 右转；全向车型为斜向平移 |
| J / L | 左 / 右横移，仅麦轮、全向车型 |
| + / - | 将速度基准增加 / 减少 0.05 m/s，调速时先停车 |
| 空格 | 发送停车 |
| Ctrl+C | 退出并尝试停车 |

终端无法直接收到物理按键松开事件，工具通过按键重复维持动作，超过 0.35 秒未收到方向键就发送停车。系统首次按键重复的延迟可能使开始运动出现短暂停顿。方向键不叠加，组合动作使用 Q/E/Z/C。

`--chassis` 可选 `4wd`（默认）、`diff`、`ackermann`、`mecanum`、`omni`。阿克曼使用 `forward-left` / `forward-right` 等弧线动作；麦轮、全向可使用 `strafe-left` / `strafe-right`。该选项只选择 APP 指令页，不会更改底盘固件车型。

`--speed` 是固件中的遥控速度基准，范围 0～1 m/s，默认 0.1。旋转速度、阿克曼转角及斜向动作由固件按此基准解算，不能分别指定任意 `Vx/Vy/Vz`。斜向平移时每个分量都使用该速度，合速度可能更大。默认发送频率 20 Hz，可通过 `--rate` 设置 5～50 Hz。

`--baudrate` 默认 9600。SPP 空中链路不使用这个值配置底盘 UART；底盘模块与 UART4 的实际速率必须已经匹配。源码在 `VECT_TAB_OFFSET=0` 时初始化 UART4 为 9600，在 `0x10000` 时为 230400。有需要可在虚拟串口连接时指定 `--baudrate 230400`。

## 协议与初始化

所有指令都是原始字节，不附加换行，不是 USART1/3 的 11 字节控制帧。

| 字节 / 文本 | 当前固件含义 |
|---|---|
| `I` | 普通摇杆页 |
| `K` | 转向页，同时清除方向与旋转标志 |
| `A` / `E` | 前进 / 后退 |
| `B` / `H` | 前进右 / 左；全向底盘为斜移 |
| `D` / `F` | 后退右 / 左；全向底盘为斜移 |
| `C` / `G` | 普通底盘右 / 左转；全向底盘在普通页横移、转向页旋转 |
| `00`（十六进制） | 清除当前页方向；工具组合发送 `4B 00 49 00` 清除两种状态 |
| `{0:100}` | 遥控速度基准设为 100 mm/s，不保存到 Flash |
| `{0:P}` | 请求参数回读，返回 `{C...}$` |
| 连续两次 `A` | 切换到 APP 控制模式，必须先确认零速 |

运动前依次执行：

1. 清除 APP 方向与旋转标志。
2. 设置速度为零，请求参数回读，确认 `{C...}$ 中速度为零。
3. 发送连续 `AA` 切换 APP 模式，立即清除方向。此时速度已确认是零。
4. 设置所需速度并再次核对回读，然后按固定频率发送方向指令。
5. 到时、Ctrl+C、SIGTERM 或发生错误时，尝试重复发送停车三次，再把遥控速度设为零。

切换运动方向时先发送停车，等待 30 ms 让 100 Hz 控制任务处理，再发送新方向，避免全向车型从旋转切换平移时残留旋转速度。

初始化未收到正确参数回读时，不发送 `AA` 或运动动作。`status` 完全只读。固件未提供模式切换、方向指令或停车的确认帧，因此参数回读不能证明底盘已切换模式或已停止。

回传显示使用 APP 文本帧：`A` 包含两路轮速显示值、电量显示值和陀螺仪 Z 原始值；`B` 包含三轴陀螺仪原始值；`C` 包含速度和控制参数。它们不同于上位机 24 字节状态帧，轮速显示值没有方向，电量是固件的估计值。

**当前固件的蓝牙 APP 控制没有可靠的失联自动停车机制。** 工具可在反馈超时时尝试停车，但蓝牙已经断开、电脑掉电或程序被强制杀死时，停车指令可能无法送达。连续发送指令不能替代底盘端看门狗；实际运行须保留可用的硬件急停。工具的“停车”只是零运动命令。

控制结束会把全局遥控速度基准留在零，不会写入 Flash；后续若改用手柄或手机 APP，需要重新设置速度。只读 `status` 不更改速度。工具不会切换自动回充状态。

依据：`BALANCE/uartx_callback.c` 的 `UART4_IRQHandler()`、`BALANCE/balance_task.c` 的 `Get_APPcmd()`、`BALANCE/show_task.c` 的 `APP_ShowTask()`。

## 离线验证

```bash
python3 -m unittest discover -s tools/tests -p 'test_bluetooth_chassis_control.py' -v
```

测试覆盖协议字节、车型指令页、分片回传、参数回读与零速激活、BLE 特征选择、通知、断线与资源释放，以及模拟串口上的发送、超时、只读和退出停车。测试不会访问真实底盘。
