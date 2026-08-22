# RTAB-Map 建图参数说明

本文档记录当前 `ros2 launch velodyne_bringup rtabmap_slam.launch.py` 建图流程中使用的主要参数。

相关配置文件：

- `src/velodyne_bringup/launch/rtabmap_slam.launch.py`
- `src/velodyne_bringup/launch/lidar3d.launch.py`

## 启动链路参数

| 参数 | 当前值 | 作用 |
|---|---:|---|
| `lidar_topic` | `/velodyne_points` | 原始 VLP-16 点云输入话题。 |
| 实际建图点云 | `/velodyne_points/deskewed` | 开启去畸变后，RTAB-Map 和 ICP 实际使用的点云。 |
| `deskewing` | `true` | 启用点云去畸变，减少运动时一帧点云被拉弯。 |
| `frame_id` | `base_link` | 机器人基坐标系。 |
| `imu_topic` | `/imu/data` | 用 IMU 姿态辅助点云去畸变。 |
| `expected_update_rate` | `15.0` | 期望雷达更新频率。 |
| `qos` | `2` | best effort QoS，适合高频点云流。 |
| `voxel_size` | `0.05` | 默认体素尺寸，影响 RTAB-Map 点云处理精度。 |
| `min_loop_closure_overlap` | `0.2` | 回环或图匹配接受的最小点云重叠比例。 |
| `use_sim_time` | `false` | 使用真实时间。 |
| `database_path` | 空 | 建图模式不指定数据库路径。 |
| `-d` | 启用 | 建图启动时删除旧 `~/.ros/rtabmap.db`，重新建图。 |

## 雷达与 IMU 参数

| 参数 | 当前值 | 作用 |
|---|---:|---|
| `timestamp_first_packet` | `true` | 点云时间戳使用一帧第一个 packet 的时间。 |
| `calibration` | `VLP16db.yaml` | VLP-16 雷达标定文件。 |
| `velodyne_transform min_range` | `0.5` | 雷达点云转换阶段过滤 0.5m 内点。 |
| `base_link -> velodyne` | `x=0, y=0, z=0.3, roll=0, pitch=0.0873, yaw=0` | 雷达相对底盘的静态 TF。 |
| `base_link -> imu_link` | `x=0, y=0, z=0, roll=0, pitch=0, yaw=0` | IMU 相对底盘的静态 TF。 |
| `imu_filter use_mag` | `false` | 不使用磁力计。 |
| `imu_filter publish_tf` | `false` | IMU 滤波器不发布 TF。 |
| `imu_filter world_frame` | `enu` | IMU 世界坐标约定。 |
| `imu_filter gain` | `0.01` | 姿态滤波增益；越小越平滑，延迟略增。 |
| `imu_filter zeta` | `0.001` | 陀螺零偏跟踪参数。 |
| `remove_gravity_vector` | `false` | 保留加速度中的重力分量。 |

## ICP 里程计参数

| 参数 | 当前值 | 作用 |
|---|---:|---|
| `odom_frame_id` | `odom` | ICP 发布 `odom -> base_link` 里程计 TF。 |
| `guess_frame_id` | `base_link_stabilized` | 使用 IMU 稳定帧辅助 ICP 初始估计。 |
| `deskewing_slerp` | `true` | 使用快速球面插值进行去畸变。 |
| `wait_imu_to_init` | `false` | 不等待 IMU 初始化后才启动里程计。 |
| `always_check_imu_tf` | `false` | IMU 固定安装，不反复检查 IMU TF。 |
| `Icp/PointToPlane` | `true` | 使用点到面 ICP。 |
| `Icp/Iterations` | `10` | ICP 最大迭代次数。 |
| `Icp/VoxelSize` | `0.10` | ICP 点云降采样体素尺寸。 |
| `Icp/Epsilon` | `0.001` | ICP 收敛阈值。 |
| `Icp/PointToPlaneK` | `20` | 估算法向量的近邻点数量。 |
| `Icp/PointToPlaneRadius` | `0` | 点到面法向量搜索半径；`0` 表示使用 K 近邻。 |
| `Icp/PointToPlaneGroundNormalsUp` | `0.5` | 修正环形雷达地面法向量方向。 |
| `Icp/MaxTranslation` | `3` | 单次匹配允许的最大平移。 |
| `Icp/MaxCorrespondenceDistance` | `0.5` | ICP 最大对应点距离，由 `voxel_size * 10` 得到。 |
| `Icp/Strategy` | `1` | ICP 匹配策略。 |
| `Icp/RangeMin` | `0.5` | ICP 忽略 0.5m 内点，减少车体反射。 |
| `Icp/RangeMax` | `40.0` | ICP 忽略 40m 外稀疏远点。 |
| `Icp/OutlierRatio` | `0.7` | 允许剔除较多离群点。 |
| `Icp/CorrespondenceRatio` | `0.01` | ICP 里程计最低对应比例。 |
| `Odom/ScanKeyFrameThr` | `0.4` | 新关键帧触发阈值。 |
| `OdomF2M/ScanSubtractRadius` | `0.05` | 局部地图点云去重半径。 |
| `OdomF2M/ScanMaxSize` | `30000` | 局部 ICP 地图最大点数。 |
| `OdomF2M/BundleAdjustment` | `false` | ICP 里程计不做 BA 优化。 |

## RTAB-Map 建图参数

| 参数 | 当前值 | 作用 |
|---|---:|---|
| `subscribe_scan_cloud` | `true` | 使用 3D 点云建图。 |
| `subscribe_depth` | `false` | 不使用深度图。 |
| `subscribe_rgb` | `false` | 不使用 RGB 图像。 |
| `subscribe_odom_info` | `true` | 订阅 ICP 里程计信息。 |
| `map_frame_id` | `map` | 全局地图坐标系。 |
| `odom_frame_id` | `odom` | 里程计坐标系。 |
| `odom_sensor_sync` | `true` | 点云与里程计同步。 |
| `RGBD/CreateOccupancyGrid` | `true` | 生成 2D 占据栅格，可保存为 Nav2 地图。 |
| `RGBD/AngularUpdate` | `0.05` | 角度变化超过阈值才更新图。 |
| `RGBD/LinearUpdate` | `0.05` | 位移超过阈值才更新图。 |
| `RGBD/ProximityMaxGraphDepth` | `0` | 关闭近邻图深度扩展。 |
| `RGBD/ProximityPathMaxNeighbors` | `1` | 限制近邻路径数量。 |
| `Reg/Strategy` | `1` | 使用 ICP 配准策略。 |
| `Mem/STMSize` | `30` | 短期记忆节点数量。 |
| `Mem/NotLinkedNodesKept` | `false` | 不保留未连接节点。 |
| `Icp/CorrespondenceRatio` | `0.2` | RTAB-Map 图匹配或回环接受的最低对应比例。 |

## 占据栅格与障碍过滤参数

| 参数 | 当前值 | 作用 |
|---|---:|---|
| `Grid/3D` | `true` | 使用 3D OctoMap/点云栅格生成地图。 |
| `Grid/CellSize` | `0.05` | 地图栅格分辨率。 |
| `Grid/RangeMin` | `0.5` | 建图忽略 0.5m 内点，减少车体反射。 |
| `Grid/RangeMax` | `5.0` | 建图只使用 5m 内点，减少远端弧形噪声。 |
| `Grid/NormalsSegmentation` | `true` | 使用法向量分割地面和障碍。 |
| `Grid/NormalK` | `30` | 法向量计算近邻数量。 |
| `Grid/GroundIsObstacle` | `false` | 地面不作为障碍。 |
| `Grid/MaxGroundAngle` | `15` | 地面法向最大角度阈值。 |
| `Grid/FlatObstacleDetected` | `true` | 检测平面障碍物。 |
| `Grid/MaxObstacleHeight` | `0.8` | 建图障碍物最大高度。 |
| `Grid/PreVoxelFiltering` | `true` | 建图前先做体素滤波。 |
| `Grid/ClusterRadius` | `0.2` | 障碍聚类半径。 |
| `Grid/NoiseFilteringRadius` | `0.3` | 噪声过滤搜索半径。 |
| `Grid/NoiseFilteringMinNeighbors` | `8` | 半径内至少 8 个邻居才保留。 |
| `Grid/MinClusterSize` | `40` | 小于 40 点的簇会被过滤。 |
| `Grid/RayTracing` | `true` | 用射线追踪清除自由空间。 |
| `Grid/OctoMapOccupancyThr` | `0.6` | OctoMap 占据概率阈值。 |
| `GridGlobal/FloodFillDepth` | `8` | 全局栅格填充深度。 |
| `GridGlobal/OccupancyThr` | `0.5` | 全局占据判断阈值。 |

## 地图保存

建图完成后使用：

```bash
cd $PROJECT_ROOT
source /opt/ros/humble/setup.bash
source install/setup.bash
./scripts/save_rtabmap_map.sh
```

保存结果固定为：

| 文件 | 作用 |
|---|---|
| `maps/rtabmap_nav2_latest.yaml` | Nav2/AMCL 静态地图元数据。 |
| `maps/rtabmap_nav2_latest.pgm` | Nav2/AMCL 静态地图图像。 |
| `maps/rtabmap_nav2_latest.db` | RTAB-Map 定位使用的最新数据库备份。 |
