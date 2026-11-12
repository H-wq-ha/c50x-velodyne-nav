# IMU 单独内参标定使用教程

雷达与 IMU 的联合外参标定使用 [OA-LICalib 教程](../tool/oa_calibration/README.md)。本页入口配套独立 `imu_for_slam` 固件，不能直接解析联合授时固件的串口协议。当前 OA/GRIL 使用的加速度校正配置保存在 `../tool/calibration/accel_calibration03.json`；历史采集数据及内参结果已清理。


本工具适配 `/home/y/arm_car/imu_for_slam/imu_for_slam` 中的现有固件和串口解析器。
固件配置为 USART1、460800 baud、8N1，默认 200 Hz；HEX 数据带 CRC 校验。
传感器读数只做标称单位转换，没有移除重力或零偏。加速度单位为 m/s²，角速度为 rad/s。
当前源码设置加速度计量程 1 档（±4g，8192 LSB/g）、陀螺仪量程 3 档（±2000°/s），
并设置加速度计和陀螺仪低通为 3 档（MPU6050：44/42 Hz；MPU6500：41/41 Hz）。
修改配置后需要重新烧录固件，再用新配置采集并标定。
驱动从 `0x3B` 一次连续读取 14 字节，再解析加速度、温度和角速度，以保证同一帧及高低字节的一致性。
读取失败时保留上一份数据并报告错误。修改后仍需烧录并做静止对照采集，验证实际尖峰是否消失。

## 环境和启动

```bash
conda activate stm
cd /home/y/arm_car/lidar/ImuCalibration
python stm32_imu_calibration.py --help
```

## 采集

先连接当前 STM32 固件使用的串口，列出串口设备：

```bash
python /home/y/arm_car/imu_for_slam/imu_for_slam/tools/serial_receive.py --list
```

接线按当前固件：STM32 PA9/TX 接 USB-TTL 的 RX，PA10/RX 接其 TX，并共地，使用 3.3V 电平。
运行时关闭占用同一串口的绘图程序、串口助手和 `serial_receive.py`。
采集工具会自己发送 `@STOP`、`@HEX`、`@START`；无需 Arduino 的初始化握手。

使用实际串口名；下面以 `/dev/ttyACM0` 为例：

```bash
python stm32_imu_calibration.py record \
  --port /dev/ttyACM0 \
  --output data/new_session01.csv
```

`record` 也支持省略 `--port` 自动选择：唯一 USB 串口直接使用，多个 USB 串口时在终端输入序号。
接收工具 `serial_receive.py`、`imu_monitor.py` 和绘图工具 `imu_live_plot.py` 使用相同的选择器。

1. 上电预热，等温度和读数稳定；保持量程与采样配置不变。
2. 按 Enter 开始，第一姿态保持静止 30 秒，用于估计陀螺仪零偏和静止噪声。
3. 按提示，用 4 秒平稳转到下一个姿态，再静置 6 秒。
4. 共采集 40 个姿态，默认约 7 分钟。前 6 个姿态建议依次让 Z+、Z-、X+、X-、Y+、Y- 方向朝上，然后增加不同倾斜姿态。每个姿态均要真正停稳。
5. 全程连续记录转动过程。不要只保存静止均值，也不要减去重力；陀螺仪比例因子计算需要转动数据。

姿态数量和时长可调整，例如：

```bash
python stm32_imu_calibration.py record --port /dev/ttyACM0 \
  --output data/new_session02.csv --positions 40 --t-init 30 --move 4 --hold 6
```

生成 `new_session01.csv`（保留序号、MCU 时间、六轴 SI 读数及温度）与
`new_session01.csv.session.json`（采集时长、完整性和检测窗口配置）。默认检测窗口 `--t-wait 3` 秒，
它与每个姿态静置时长 `--hold 6` 秒不同。
Ctrl+C 可中断并保存部分数据；未完成的会话不会进入标定。结束时向板端发送 STOP。
工具拒绝覆盖已有采集文件。

## 标定

可以先检查采集质量：

```bash
python stm32_imu_calibration.py inspect --input data/new_session01.csv
```

工具报告静止段数量、约有多少种不同姿态及缺少的重力方向。本工具的完整内参标定要求
至少 10 个明显不同的稳定姿态，并在三个轴的正负方向都有足够的重力分量（归一化后至少 0.65）。
这是用于避免姿态覆盖狭窄而得到不可靠参数的质量门槛，不是声称所有标定方法都必须如此采集。
数据不足时会保存诊断报告，并明确要求补充采集。传感器是否饱和应按烧录固件的实际量程判断；
不能将超过 2g 的瞬态加速度自动判断为饱和。

```bash
python stm32_imu_calibration.py calibrate \
  --input data/new_session01.csv \
  --output data/calibration01 --plots
```

标定会检查序号和 MCU 时间，拒绝丢包、复位、大间隔及采样率不符的数据。
对连续样本的轻微时间抖动，按 MCU 时间插值到默认 200 Hz 的均匀网格。
若固件采样率确实已改动，传入对应的 `--fs`。
SI 输入的加速度计和陀螺仪比例因子初值都是 1；原工具陀螺仪默认初值针对 ADC 原始计数，不能直接用于这里的 rad/s 数据。

如果已经用现有接收工具记录了数据：

```bash
python /home/y/arm_car/imu_for_slam/imu_for_slam/tools/serial_receive.py \
  --port /dev/ttyACM0 --csv data/existing.csv --print-every 200
```

必须按同样要求保留开始的静止段和连续转动/静置数据。Ctrl+C 结束后显式给出开头真实静止时长：

```bash
python stm32_imu_calibration.py calibrate --input data/existing.csv \
  --t-init 30 --t-wait 3 --output data/calibration_existing
```

## 结果和应用

结果目录中包含 `params.json`、`params_acc.csv`、`params_gyro.csv`、
`calibration_input.csv`（原工具的文件结构，内容仍为 SI 单位）及 `calibrated.csv`（校正后六轴数据）。
这些结果仅由本机标定程序生成，不会自动写入固件。

注意加速度计和陀螺仪的偏置符号约定不同，应按工具的实际模型应用：

```python
acc_corrected = imu.apply_accel_calibration(params_acc, accel_mps2)
gyro_corrected = imu.apply_gyro_calibration(
    params_gyro[:9], gyro_rads - params_gyro[9:]
)
```

加速度计模型为 `T_a @ K_a @ (a + b_a)`，所以参数最后三项是加性修正偏置。
陀螺仪模型为 `T_g @ K_g @ (w - b_g)`，最后三项是需要减掉的静止零偏。
SI 输入的比例因子通常接近 1，且为无量纲；加速度偏置为 m/s²，陀螺仪偏置为 rad/s。
需要确认静止校正后的加速度模长接近重力、角速度接近零，并用另一份数据检查效果。
标定结果还取决于姿态覆盖、是否真正静止、转动数据和温度稳定性；示例运行成功不能代表真实传感器已经标定。
