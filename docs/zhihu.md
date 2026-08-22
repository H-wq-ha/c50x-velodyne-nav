# WHEELTEC C50X + VLP-16 激光雷达 SLAM 与自主导航 — 架构图版

> 说明：本文所有图表均为纯文本 ASCII 绘制，知乎编辑器直接粘贴即可正常显示（mermaid 图在知乎会退化为源码，故改用本格式）。

## 1. 系统分层架构总图

```
            感知/状态上行 (↑)                        控制/指令下行 (↓)

╔════════════════════════════════════════════════════════════════════╗
║  L6  应用交互层                                                      ║
║        RViz2 可视化  │  rtabmap_viz 建图窗口  │  keyboard_teleop 遥控 ║
╚════════════════════════════╤═══════════════════════════════════════╝
                             │ ① 导航目标 (2D Nav Goal / 路点)
╔════════════════════════════╪════════════════════════════════════════╗
║  L5  地图与导航决策层 (Nav2)                                           ║
║        map_server ──> planner ──> controller ──> velocity_smoother  ║
║         (静态地图)      (全局路径)      (DWB局部)     (平滑→/cmd_vel)   ║
╚═════════╤══════════════════╪════════════════════════════╤═══════════╝
          │                  │                            │
   ⑤ map→odom TF     ② /cmd_vel 控制下行           ④ 从 maps/*.yaml
    (来自 L4)           (直达底盘驱动)                加载静态地图
          │                  │
╔═════════╪══════════════════╪════════════════════════════════════════╗
║  L4  里程计与定位层                                                   ║
║        icp_odometry                     rtabmap (定位/SLAM 双模式)   ║
║        (ICP 里程计)                     (回环检测/图优化/重定位)        ║
║        发布 /odom + odom→base_link      发布 map→odom + /map         ║
╚═════════╪══════════════════╪════════════════════════════════════════╝
          │                  │
   ③ 去畸变点云         /map (定位模式下为静态)
    /velodyne_points/deskewed
          │                  │
╔═════════╪══════════════════╪════════════════════════════════════════╗
║  L3  感知预处理层                                                     ║
║        imu_filter_madgwick → imu_to_tf → lidar_deskewing            ║
║        (IMU 滤波 gain=0.01)  (稳定帧TF)   (扫描运动去畸变)              ║
╚═════════╪══════════════════╪════════════════════════════════════════╝
          │                  │
    /imu_raw        /velodyne_points
          │                  │
╔═════════╪══════════════════╪════════════════════════════════════════╗
║  L2  驱动接入层                                                       ║
║        chassis_node (car_bringup)     velodyne_transform_node       ║
║        串口 460800 · 断线重连          packet → PointCloud2           ║
╚═════════╪══════════════════╪════════════════════════════════════════╝
          │ ⑥ 24B状态帧↑ / 11B控制帧↓
╔═════════╪═══════════════════════════════════════════════════════════╗
║  L1  固件控制层 (STM32F407VE + FreeRTOS)                              ║
║        Balance_task (运动学逆解 + PID @100Hz)  ←─ 编码器反馈闭环        ║
║        data_task (状态帧发送 @200Hz)  IMU_task (IMU采集)              ║
╚═════════╪═══════════════════════════════════════════════════════════╝
          │
╔═════════╪══════════════════════════════════════════════════════════╗
║  L0  硬件物理层                                                      ║
║        4轮差速电机+编码器  │  IMU(ICM20948/MPU6050)  │  电池ADC       ║
╚════════════════════════════════════════════════════════════════════╝
```

## 2. 数据流图例

| 编号 | 流名 | 完整路径 | 方向 |
|---|---|---|---|
| ① | 导航目标 | `用户 → RViz2 Nav2 Goal → bt_navigator` | 下行 |
| ② | 控制指令 | `controller → cmd_vel_nav → velocity_smoother → /cmd_vel → chassis_node → 串口 → STM32 → 电机` | 下行 |
| ③ | 感知数据 | `VLP-16 → /velodyne_packets → /velodyne_points → 去畸变 → /velodyne_points/deskewed → {ICP, rtabmap, local_costmap}` | 上行 |
| ④ | 静态地图 | `maps/rtabmap_nav2_latest.yaml → map_server → /static_map → global costmap` | 下行 |
| ⑤ | 全局定位 | `rtabmap (加载 latest.db) → map→odom TF` | 上行 |
| ⑥ | 底盘状态 | `STM32 (编码器+IMU+电压) → 24B状态帧@200Hz → chassis_node → /imu_raw /chassis_vel /battery` | 上行 |

两个闭环：

```
闭环A (固件内, 100Hz):  编码器 ──> PID ──> PWM ──> 电机 ──> 编码器
闭环B (ROS 内):        点云 ──> ICP ──> RTAB-Map ──> costmap ──> controller ──> 新速度
```

## 3. TF 树

```
map
 ├──> odom            (rtabmap 定位 或 amcl 发布, 互斥)
      └──> base_link  (icp_odometry 发布, 唯一局部位姿源)
           ├──> velodyne             (静态: z=0.3m, pitch≈5°)
           ├──> imu_link             (静态: 单位阵)
           └──> base_link_stabilized (imu_to_tf 发布, IMU 姿态稳定帧, 供去畸变)
```

要点：

- 底盘不发布 TF（无编码器里程计），`odom→base_link` 完全来自雷达 ICP
- `base_link_stabilized` 是去畸变专用帧，由 200Hz IMU 姿态生成
- 两套定位方案（RTAB-Map / AMCL）发布同一个 `map→odom`，互斥启动

## 4. 关键话题

| 话题 | 类型 | 发布者 | 作用 |
|---|---|---|---|
| `/velodyne_points` | PointCloud2 | velodyne_transform_node | 标定后点云 (min_range=0.5) |
| `/velodyne_points/deskewed` | PointCloud2 | lidar_deskewing | 运动去畸变点云 |
| `/imu_raw` | Imu | chassis_node | 原始 IMU (无磁力计) |
| `/imu/data` | Imu | imu_filter_madgwick | 滤波后姿态 (gain=0.01) |
| `/odom` | Odometry | icp_odometry | ICP 里程计 |
| `/map` | OccupancyGrid | rtabmap | 全局地图 (定位模式静态) |
| `/static_map` | OccupancyGrid | map_server | 静态地图 (避免与 /map 冲突) |
| `/cmd_vel` | Twist | velocity_smoother | 最终底盘速度指令 |
| `cmd_vel_nav` | Twist | controller_server | Nav2 内部未平滑速度 |

## 5. 快速开始

```bash
# 1. 建图
ros2 launch velodyne_bringup rtabmap_slam.launch.py

# 2. 键盘遥控巡游
ros2 run car_bringup keyboard_teleop

# 3. 保存地图
./scripts/save_rtabmap_map.sh

# 4. 定位导航
ros2 launch velodyne_bringup rtabmap_nav_complete.launch.py
```

导航成功标志（日志）：`controller_server: Reached the goal!` + `bt_navigator: Goal succeeded`
