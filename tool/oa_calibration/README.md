# OA-LICalib：采集与离线标定教程

流程：**准备硬件 → 静止初始化 → 三轴运动采集 → Ctrl+C 保存 → 离线求解 → 检查结果**。本入口自动录包，停止后再求解。

## 1. 准备

- 雷达与 IMU 固定在同一刚性支架上，采集期间相对安装不变。
- IMU 预热至温度相对稳定。
- 使用 STM32 联合授时固件；雷达网页显示 PPS `Locked`，持续接收 GPRMC。
- IMU INT 接 PA0，SCL/SDA 接 PB10/PB11，PC 串口使用 PA2/PA3；PPS 从 PA8 输出，GPRMC 从 PA9 经当前反相接口接雷达，所有设备共地。接线细节以联合授时固件工程说明为准。
- 上位机与 VLP-16 网络连通，默认雷达地址 `192.168.1.201`、UDP 2368。
- 关闭占用同一 IMU 串口或雷达 UDP 端口的其他程序。
- 每轮可能新增数 GB 数据，启动前检查磁盘空间。

当前设备为 MPU6500，入口也接受 MPU6050。加速度使用 `tool/calibration/accel_calibration03.json` 中的参数；角速度零偏在每轮启动时重新估计。历史内参只适用于同一颗 IMU、相同安装轴向和采集配置。

## 2. 启动采集

本机已安装并编译，可以直接运行：

```bash
cd /home/y/arm_car/lidar
./tool/oa_calibration/run.sh
```

它会启动雷达驱动、点云解码、硬件输入和录包，并打印本轮目录及离线处理命令。目录自动创建在 `tool/runs/oa_capture_时间_微秒/`。

如果已有雷达驱动运行：

```bash
./tool/oa_calibration/run.sh start_lidar:=false
```

如需固定一个新目录，便于后续复制命令：

```bash
./tool/oa_calibration/run.sh \
  output:=/home/y/arm_car/lidar/tool/runs/oa_next_round
```

指定目录必须尚不存在。下一轮换一个新名字，或直接使用自动目录。

可选参数：

| 参数 | 默认值 | 含义 |
|---|---|---|
| `port` | `/dev/serial/by-id/usb-FIREPHX_USB_SER_FX2348N-if00` | IMU 串口，460800 baud |
| `start_lidar` | `true` | 是否启动雷达驱动 |
| `duration` | `0` | 总运行秒数；0 表示手动 Ctrl+C，计时包含静止初始化 |
| `min_valid_seconds` | `90` | 建议结束所需的有效数据时长 |
| `min_clouds` | `600` | 建议结束所需的合格点云数量 |

例如希望至少 120 秒、900 帧才提示结束，可加 `min_valid_seconds:=120 min_clouds:=900`。固定 `duration` 可能早于条件满足，一般使用手动结束。

## 3. 完成静止与运动

1. **最初 10 秒完全静止。** 整套传感器和支架都不要动。
2. 等待“10 秒静止检查通过”，再继续静止约 5 秒。
3. 整体做绕 X/Y/Z 三轴的平稳往复转动，转动方向与速度应有变化，同时加入前后、左右、上下平移和加减速。
4. 选择能看到地面、墙面和其他静态结构的环境，避免猛烈甩动、碰撞、严重遮挡和大面积运动物体。
5. 查看每约 5 秒输出的有效时长、点云数、三轴进度和缺少的条件。
6. 出现 **“【可以结束采集】”** 后，可再静止数秒，按一次 Ctrl+C。
7. 等待所有进程退出，录包保存完成后再处理。

结束提示默认检查 90 秒有效数据、600 帧点云、三轴往复转动、平移加速度、场景结构、输入频率及质量。它是采集建议，不是完整可观测性证明或真实精度验收。

## 4. 输入容错与时间规则

OA 入口会过滤 CRC 坏帧、非有限测量、重复帧、坏扫描和坏点云。少量跳号保留为真实时间缺口，不补造采样、不插值点云、不改写硬件采样时间。

默认相邻有效 IMU 间隔不超过 50 ms；时间倒退、设备重启、硬件时基异常、长缺口或持续断流会停止采集。累计和最近约 10 秒的异常比例需要不超过 1%，才会给出结束建议。静止初始化失败仍需重新启动。

IMU 使用 STM32 硬件捕获时刻；雷达使用包内小时计数，并由同一 STM32 时基补齐小时。固件模拟日期只用于共同相对时基，不代表真实 UTC。

加速度模型为 `T_a @ diag(scale) @ (a_raw + additive_bias)`；角速度先扣除本轮 10 秒静止均值。OA 后端还会估计残余零偏。录包已经包含校正后的输入，离线转换不会再次扣零偏。

## 5. 停止后执行离线标定

对第 2 步固定目录的例子，执行：

```bash
cd /home/y/arm_car/lidar
./tool/oa_calibration/calibrate.sh \
  /home/y/arm_car/lidar/tool/runs/oa_next_round
```

使用自动目录时，将末尾路径换成启动时打印的本轮目录。

脚本依次检查采集状态、转换 ROS2 录包、核对前端发布数与录包实收数、按本轮硬件时间自动生成配置、启动独立 ROS1 容器求解，最后生成诊断和报告。默认首轮批量优化加四轮细化。耗时通常为数分钟至十几分钟，取决于数据长度、场景和计算负载。

无需继续连接或移动硬件。终端会打印结果路径，默认为新的 `output/oa_licalib_时间_微秒/`。

可选参数：

```bash
# 指定新的项目内结果目录
./tool/oa_calibration/calibrate.sh \
  /home/y/arm_car/lidar/tool/runs/oa_next_round \
  --output /home/y/arm_car/lidar/output/oa_next_round

# 只检查并转换，不求解
./tool/oa_calibration/calibrate.sh \
  /home/y/arm_car/lidar/tool/runs/oa_next_round --prepare-only
```

以上是不同运行方式，选择需要的一条。已有结果目录拒绝覆盖。`--iterations 8` 可增加求解轮数；`--knot 0.05` 指定样条节点间隔。日常先用默认设置。

存在 `capture_failure.json` 的采集轮次会被拒绝，应排除问题后重新采集。正常 Ctrl+C 时偶尔出现 rosout 发布器已关闭的警告，可结合进程正常退出、`metadata.yaml` 和发布/实收数量判断保存状态。

## 6. 查看与使用结果

新一轮输出文件：

| 文件 | 用途 |
|---|---|
| `report.md` | 中文摘要与诊断图 |
| `evaluation.json` | 最终矩阵、四元数、RPY 和各轮变化 |
| `verification.json` | 输入数量、数值有限性和求解收敛检查 |
| `config.yaml` | 本轮实际配置 |
| `run.log` | 完整求解日志 |
| `dataset/refined_map-iter4-seg0.pcd` | 默认设置下的最后细化地图 |

优先使用 `evaluation.json` 中 `final.T_imu_lidar`：

```text
p_imu = R_lidar_to_imu * p_lidar + t_lidar_in_imu
t_imu = t_lidar + time_offset_s
```

RPY 使用 ROS 常用约定 `R = Rz(yaw) * Ry(pitch) * Rx(roll)`。上游部分控制台欧拉角及 CSV 的方向不同，应以这里的矩阵或四元数为准。

最后两轮变化只说明同一份数据上的数值稳定性。保持安装不变，至少完成三轮独立采集，比较旋转、平移和地图表现后再采用。程序不会自动改写导航 TF。

历史采集数据、标定结果及 rosbag（包括源码附带的仿真录包）已清理。后续新一轮仍会正常生成上述输出。

## 7. 维护与重编译

通用参数模板位于 `tool/oa_calibration/config.yaml`，录包路径和时长由程序自动填写。

修改 OA 源码后：

```bash
cd /home/y/arm_car/lidar
./tool/build_oa_licalib.sh
```

本机运行环境为 `oa-licalib:noetic` Docker 镜像，使用项目内已编译的 ROS1 工作区；宿主机采集使用 ROS2 Humble。转换环境为 `tool/oa_licalib_venv/`。

如果删除了转换环境，可重建：

```bash
cd /home/y/arm_car/lidar
/usr/bin/python3 -m venv tool/oa_licalib_venv
./tool/oa_licalib_venv/bin/pip install rosbags python-lzf
```

宿主机结果分析需要系统 Python 的 numpy、scipy、matplotlib 和 PyYAML。换机器时还需要准备 ROS2 Humble、Velodyne 驱动与解码器、Docker、pyserial，以及对应源码和子模块。

OA 源码位于 `tool/OA-LICalib/`，使用 leo-drive 分支，提交 `fa78b8fd4f716b1b2048f37d055d0549ad85de9a`；依赖源码与构建产物保留在 `tool/`。版权和许可证位于各源码目录。
