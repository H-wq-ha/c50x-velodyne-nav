# IMU 与 VLP-16 接收冲激图

`imu_lidar_scope.py` 同时接收 STM32 的 v2 串口协议和当前雷达驱动的 `/velodyne_packets`，使用 matplotlib 实时绘图并保存原始记录。`lidar_receive.py` 可单独记录雷达；`imu_receive.py` 保留原有 IMU 接收与 ROS 发布功能。

## 运行

本机使用系统 Python，其环境已安装 pyserial、matplotlib、ROS 2 Humble 和 velodyne_msgs。先加载正在运行的雷达工程消息定义：

```bash
cd tools/fake_gps/fake_gps
source /opt/ros/humble/setup.bash
source ../../../install/setup.bash
/usr/bin/python3 tools/imu_lidar_scope.py --port /dev/ttyACM0
```

默认 460800/8N1，显示最近 1 秒；关闭窗口或 Ctrl+C 结束并保存图片。仅发现一个 USB 串口时可省略 `--port`；也可用 `--port` 指定本机的稳定设备路径。同一串口同时只运行一个接收程序。

无窗口、限时记录：

```bash
/usr/bin/python3 tools/imu_lidar_scope.py --port /dev/ttyACM0 \
  --headless --duration 60 --output output/reception_validated
```

`--window 5` 显示最近 5 秒，`--window 0` 显示内存保留的全部事件。默认向 STM32 发 `@START` 和 `@STATS`，使其输出数据和状态；不改变采样率、时钟或 GPS。`--no-start` 可关闭这两条命令。

单独测试雷达：

```bash
/usr/bin/python3 tools/lidar_receive.py --duration 10 --csv output/lidar.csv
```

默认 ROS 模式不占用 UDP 2368，适合当前雷达驱动已启动的情况。一条冲激代表收到一条扫描消息，检查其中所有原始包的长度、块头、角度范围和小时内时间戳。当前每次扫描约有 76 个原始包，完整扫描频率约 10Hz。需要逐个原始 UDP 包记录时，可在没有其他程序占用接收端口的情况下使用 `--lidar-source udp`（单独程序使用 `--source udp`）；原始包频率与完整扫描频率不同。

## 坐标与成功条件

- 默认横轴是电脑应用程序实际观察到接收完成的时间，刻度为相对图下方北京时间起点的秒数。串口一次读取包含多帧时，它们可能具有相同的接收时间，不人为均匀分配。
- 纵轴 1 表示有效数据：IMU 通过 CRC、时间标志及有限数值检查；雷达通过包结构检查。每秒设备状态包写入 CSV，不画成 IMU 采样冲激。
- 纵轴 0 的红叉表示检测到 CRC 错误、IMU 序号缺失、设备计时重启、无效内容或接收超时。IMU 超时阈值 100ms，雷达 500ms。缺失帧只能在后续数据或超时检测时标记，不能推断其真实到达时间。
- 设备时间图使用 IMU 与雷达各自的 `hour_us`，按电脑邻近小时补齐日期和小时，与现有驱动策略一致。它与电脑接收时间图有不同含义；模拟 GPS 的绝对日期不代表真实 UTC。CSV 同时保留设备原始计时和接收时间。
- 两路冲激图用于观察接收与时间分布；零失败事件不证明网络绝无丢包，也不代表已测得传感器物理同步精度。ROS 扫描的设备时间选其最后一个原始包的时间。

## 输出

默认目录为 `output/reception_年月日_时分秒/`，也可使用 `--output` 指定：

| 文件 | 内容 |
| --- | --- |
| `events.csv` | 所有样本、状态、失败事件；纳秒接收时间、设备时间、序号及原始字段 |
| `summary.json` | 两路实收频率、成功/失败计数、CRC、缺失序号和最新设备状态；每秒更新 |
| `receive_impulses.png` | 最后一个显示窗口的接收时间冲激图 |
| `hardware_impulses.png` | 最后一个显示窗口的设备时间冲激图 |
| `receive_overview.png` | 内存保留事件的接收时间总览 |
| `hardware_overview.png` | 内存保留事件的设备时间总览 |

内存最多保留最近 50000 个绘图事件，CSV 持续保留全部记录。没有可靠设备时间的失败事件仅出现在接收时间图，避免伪造硬件时间。

自动检查：`/usr/bin/python3 tests/test_reception_scope.py`。覆盖伪串口分片/CRC/序号缺失/超时、雷达坏包、CSV 与两类绘图；实机结果应以对应输出目录的 `summary.json` 为准。

## 本次实机记录

2026-10-18 连续接收 60 秒，结果保存在 `output/reception_validated/`：IMU 有效样本 12017 条，实收 200.3126Hz；雷达完整扫描 594 次，实收 9.9148Hz。两路均无检测到的失败或超时，IMU CRC 错误及序号缺失均为 0。CSV 共 12671 条记录，另含 60 条设备状态；样本数与汇总一致，两路硬件时间递增。

设备上报 MPU6500（WHO_AM_I=0x70）、系统时钟 72MHz，采样丢弃、I²C 错误、样本队列丢弃、发送丢弃均为 0，历史最长读取耗时 1273μs。此记录验证接收与时间连续性，不替代外部示波器同步精度测量。
