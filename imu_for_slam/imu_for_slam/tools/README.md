# IMU 串口工具

使用已安装依赖的环境，例如 `conda activate stm`。

在项目目录中运行，无需指定串口：

```bash
python tools/serial_receive.py --print-every 200
python tools/imu_live_plot.py
```

两个程序应分别运行，同一时刻只运行一个串口程序。也可以使用 `imu_monitor.py` 接收。
所有工具共用 `serial_ports.py`：

- 唯一 USB 串口：自动选择并显示设备名。
- 多个 USB 串口：交互终端列出候选设备并输入序号；非交互运行要求显式指定 `--port`。
- 没有 USB 串口：报告连接问题；内置 UART 等其他串口可用 `--port` 指定。
- `--list` 查看所有串口；`--port /dev/ttyACM0` 或 `--port COM5` 手动选择。

自动选择依据 USB 枚举信息，不会扫描打开所有设备，也不保证候选设备一定是 IMU。
选定后，接收程序按既有协议校验数据。默认波特率为 460800；若设备固件不同，用 `--baud` 指定。

标定项目的 `stm32_imu_calibration.py record` 也复用此选择器，可省略 `--port`：

```bash
cd ImuCalibration
python stm32_imu_calibration.py record --output data/session03.csv
```

## calibration03 实时校正

`serial_receive.py`、`imu_monitor.py` 和 `imu_live_plot.py` 默认加载脚本旁的
`imu_calibration.json`，这是 `ImuCalibration/data/calibration03/params.json` 的固定副本。
终端显示校正后的 SI 值；实时绘图同时显示原始值和校正值。
校正包括零偏、比例因子及轴间误差矩阵；温度不变。
参数适用于本次标定的同一块 IMU、同一量程和滤波配置。重新标定后用
`--calibration /绝对路径/params.json` 指定新参数，或更新默认副本。

```bash
# 自动识别串口，显示校正后的值
python tools/serial_receive.py --print-every 200
# 关闭校正，观察原始值
python tools/serial_receive.py --raw --print-every 200
# 同一批样本同时保存原始值与对照值，约 60 秒后退出（200 Hz）
python tools/serial_receive.py --count 12001 --print-every 200 \
    --csv raw.csv --corrected-csv paired.csv
```

`--csv` 始终保存原始 SI 数据。`--corrected-csv` 保存同样的原始列，另加
`corrected_ax_mps2`、`corrected_ay_mps2`、`corrected_az_mps2`、
`corrected_gx_rads`、`corrected_gy_rads`、`corrected_gz_rads` 六列，
两组数据共用序号、MCU 时间与电脑接收时间。文件均不覆盖已有文件。
`--raw` 与 `--corrected-csv` 不能同时使用。参数缺失或格式错误时停止并提示。

校正公式（与原标定程序相同）：

- 加速度：`a_corrected = T_a @ diag(scale_a) @ (a_raw + additive_bias)`。
- 角速度：`w_corrected = T_g @ diag(scale_g) @ (w_raw - subtractive_bias)`。

串口协议解码器 `StreamDecoder` 始终返回原始值。标定采集程序继续采集原始值，
不会重复校正。其他调用解码器的 SLAM 节点需要在解包之后调用
`Calibration(path).apply(sample)`；只改 PC 显示工具不会改变 MCU 发出的数据。
静止时检查加速度向量模长是否接近重力；板子倾斜时各轴不要求为 `(0, 0, g)`。
单次静止检查不能独立验证陀螺仪比例因子或全姿态加速度标定精度。

## 实时曲线对照

```bash
python tools/imu_live_plot.py
```

保留原来的六轴总图及六个单轴图，每个轴增加校正后的曲线：

- 总图：颜色区分 AX、AY、AZ、GX、GY、GZ；虚线表示原始值，实线表示校正值。
- 单轴图：灰色虚线是原始值，彩色实线是校正值；右上角显示两个当前数值。
- 两条曲线来自同一条串口样本，共用 MCU 时间戳，同步滚动、裁剪和处理板子复位。
- 纵轴自动适配两条曲线的范围，避免原始偏置较大时曲线被裁掉。
- `--raw` 仅显示原始曲线；`--calibration /路径/params.json` 指定另一组参数。

修改脚本后，需要关闭旧绘图窗口并重新启动程序才能显示新曲线。
