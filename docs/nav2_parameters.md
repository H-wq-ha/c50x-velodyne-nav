# Nav2 导航参数说明

本文档记录当前 Nav2 导航相关参数。主要配置来自：

- `src/velodyne_bringup/config/nav2_params.yaml`
- `src/velodyne_bringup/config/amcl_params.yaml`
- `src/velodyne_bringup/launch/nav2_bringup.launch.py`
- `src/velodyne_bringup/launch/nav2_navigation.launch.py`
- `src/velodyne_bringup/launch/rtabmap_nav_complete.launch.py`

## 启动模式

| 模式 | 启动文件 | 定位来源 | Nav2 地图 | 说明 |
|---|---|---|---|---|
| AMCL 导航 | `nav2_bringup.launch.py` | AMCL 使用 `/scan` 发布 `map -> odom` | `maps/rtabmap_nav2_latest.yaml` | 启动 VLP-16、点云转 `/scan`、ICP 里程计、AMCL 和完整 Nav2。 |
| RTAB-Map 定位导航 | `rtabmap_nav_complete.launch.py` | RTAB-Map 使用 `maps/rtabmap_nav2_latest.db` 发布定位 TF | `maps/rtabmap_nav2_latest.yaml` | 启动 RTAB-Map 定位，再包含 `nav2_navigation.launch.py` 启动 Nav2。 |
| Nav2-only | `nav2_navigation.launch.py` | 外部定位必须已经提供 `map -> odom` | `maps/rtabmap_nav2_latest.yaml` | 只启动 Nav2 栈，不启动传感器和定位。 |

## 启动参数

| 参数 | 当前值 | 作用 |
|---|---:|---|
| `map` | `$PROJECT_ROOT/maps/rtabmap_nav2_latest.yaml` | Nav2 静态地图 YAML 路径。 |
| `params_file` | `config/nav2_params.yaml` | Nav2 主参数文件。 |
| `rviz_config` | `config/nav2_wheeltec.rviz` | RViz2 配置文件。 |
| `use_rviz` | `true` | 是否启动 RViz2。 |
| `autostart` | `true` | 是否自动激活 Nav2 生命周期节点。 |

## AMCL 导航传感器链路

| 参数/节点 | 当前值 | 作用 |
|---|---:|---|
| `velodyne_transform min_range` | `0.5` | VLP-16 点云转换阶段过滤 0.5m 内点。 |
| `base_link -> velodyne` | `x=0, y=0, z=0.3, roll=0, pitch=0.0873, yaw=0` | 雷达相对底盘的静态 TF。 |
| `base_link -> imu_link` | `0, 0, 0, 0, 0, 0` | IMU 相对底盘的静态 TF。 |
| `velodyne_laserscan ring` | `-1` | 从 VLP-16 点云生成 2D `/scan`，`-1` 表示使用配置默认/多线处理方式。 |
| `velodyne_laserscan resolution` | `0.007` | `/scan` 角分辨率。 |
| `lidar_deskewing input_cloud` | `/velodyne_points` | 去畸变输入点云。 |
| `lidar_deskewing output` | `/velodyne_points/deskewed` | 去畸变点云，供 ICP 和 local costmap 使用。 |
| `imu_to_tf fixed_frame_id` | `base_link_stabilized` | 基于 IMU 生成稳定帧，辅助点云去畸变。 |

## 车体外形

| 参数 | 当前值 | 作用 |
|---|---:|---|
| `footprint` | `[[0.31, 0.325], [0.31, -0.325], [-0.31, -0.325], [-0.31, 0.325]]` | 机器人矩形轮廓，来自前后 0.62m、左右 0.65m。 |
| 前后半长 | `0.31 m` | 碰撞检测中机器人前后边界。 |
| 左右半宽 | `0.325 m` | 碰撞检测中机器人左右边界。 |

## BT Navigator

| 参数 | 当前值 | 作用 |
|---|---:|---|
| `global_frame` | `map` | 行为树导航使用的全局坐标系。 |
| `robot_base_frame` | `base_link` | 机器人基坐标系。 |
| `odom_topic` | `/odom` | 里程计话题。 |
| `bt_loop_duration` | `10` | 行为树循环周期，单位毫秒。 |
| `default_server_timeout` | `20` | 默认 action/service 等待超时。 |
| `wait_for_service_timeout` | `1000` | 等待服务超时，单位毫秒。 |
| `plugin_lib_names` | 多个 Nav2 BT 插件 | 提供规划、跟踪、恢复、等待、目标更新等行为树节点。 |

## Controller Server

| 参数 | 当前值 | 作用 |
|---|---:|---|
| `controller_frequency` | `10.0` | 局部控制器运行频率。 |
| `min_x_velocity_threshold` | `0.001` | x 方向速度低于该值视为 0。 |
| `min_y_velocity_threshold` | `0.5` | y 方向速度低于该值视为 0；当前机器人不使用横移。 |
| `min_theta_velocity_threshold` | `0.001` | 角速度低于该值视为 0。 |
| `failure_tolerance` | `0.3` | 控制失败容忍时间。 |
| `progress_checker_plugin` | `progress_checker` | 使用进度检查插件。 |
| `goal_checker_plugins` | `[general_goal_checker]` | 使用目标到达检查插件。 |
| `controller_plugins` | `[FollowPath]` | 使用 DWB 局部路径跟踪器。 |

## Progress / Goal Checker

| 参数 | 当前值 | 作用 |
|---|---:|---|
| `progress_checker.plugin` | `nav2_controller::SimpleProgressChecker` | 检查机器人是否持续取得进展。 |
| `required_movement_radius` | `0.2` | 在允许时间内至少移动 0.2m。 |
| `movement_time_allowance` | `15.0` | 15s 内未达到移动要求则认为卡住。 |
| `general_goal_checker.plugin` | `nav2_controller::SimpleGoalChecker` | 检查是否到达目标。 |
| `stateful` | `true` | 目标检查保持状态。 |
| `xy_goal_tolerance` | `0.25` | 目标位置容差。 |
| `yaw_goal_tolerance` | `0.25` | 目标朝向容差，单位弧度。 |

## DWB Local Planner

| 参数 | 当前值 | 作用 |
|---|---:|---|
| `plugin` | `dwb_core::DWBLocalPlanner` | 使用 DWB 局部规划器。 |
| `debug_trajectory_details` | `false` | 不输出轨迹评分细节。 |
| `min_vel_x` | `0.0` | 最小前进速度。 |
| `max_vel_x` | `0.2` | 最大前进速度。 |
| `min_vel_y` | `0.0` | 最小横向速度。 |
| `max_vel_y` | `0.0` | 最大横向速度；当前禁用横移。 |
| `max_vel_theta` | `0.2` | 最大角速度。 |
| `max_speed_xy` | `0.2` | 最大平面线速度。 |
| `acc_lim_x` | `0.3` | x 方向最大加速度。 |
| `acc_lim_y` | `0.0` | y 方向最大加速度。 |
| `acc_lim_theta` | `0.4` | 最大角加速度。 |
| `decel_lim_x` | `-0.3` | x 方向最大减速度。 |
| `decel_lim_theta` | `-0.4` | 最大角减速度。 |
| `vx_samples` | `12` | x 速度采样数。 |
| `vy_samples` | `1` | y 速度采样数。 |
| `vtheta_samples` | `12` | 角速度采样数。 |
| `sim_time` | `2.0` | 轨迹前向仿真时间。 |
| `linear_granularity` | `0.05` | 线性轨迹检查分辨率。 |
| `angular_granularity` | `0.025` | 角度轨迹检查分辨率。 |
| `transform_tolerance` | `0.3` | TF 容忍时间。 |
| `trans_stopped_velocity` | `0.02` | 低于该线速度认为停止。 |
| `short_circuit_trajectory_evaluation` | `true` | 轨迹评分可提前终止，减少计算量。 |
| `critics` | `RotateToGoal, Oscillation, BaseObstacle, GoalAlign, PathAlign, PathDist, GoalDist` | DWB 轨迹评分插件列表。 |
| `BaseObstacle.scale` | `0.02` | 障碍物代价权重。 |
| `PathAlign.scale` | `24.0` | 路径朝向对齐权重。 |
| `GoalAlign.scale` | `16.0` | 目标朝向对齐权重。 |
| `PathDist.scale` | `24.0` | 距离全局路径的权重。 |
| `GoalDist.scale` | `16.0` | 距离目标点的权重。 |
| `RotateToGoal.scale` | `24.0` | 接近目标时旋转到目标朝向的权重。 |

## Local Costmap

| 参数 | 当前值 | 作用 |
|---|---:|---|
| `global_frame` | `odom` | 局部代价地图坐标系。 |
| `robot_base_frame` | `base_link` | 机器人基坐标系。 |
| `update_frequency` | `5.0` | 局部地图更新频率。 |
| `publish_frequency` | `2.0` | 局部地图发布频率。 |
| `transform_tolerance` | `1.0` | TF 容忍时间。 |
| `rolling_window` | `true` | 局部地图随机器人移动。 |
| `width` | `4` | 局部地图宽度，单位米。 |
| `height` | `4` | 局部地图高度，单位米。 |
| `resolution` | `0.05` | 局部地图分辨率。 |
| `plugins` | `[voxel_layer, inflation_layer]` | 使用 3D 体素障碍层和膨胀层。 |
| `always_send_full_costmap` | `true` | 每次发布完整 costmap。 |

## Local Voxel Layer

| 参数 | 当前值 | 作用 |
|---|---:|---|
| `plugin` | `nav2_costmap_2d::VoxelLayer` | 使用 3D 点云体素障碍层。 |
| `enabled` | `true` | 启用该层。 |
| `publish_voxel_map` | `true` | 发布体素调试地图。 |
| `origin_z` | `0.0` | 体素层 Z 轴起点。 |
| `z_resolution` | `0.1` | 每个 Z 体素高度。 |
| `z_voxels` | `16` | Z 方向体素层数。 |
| `max_obstacle_height` | `1.6` | VoxelLayer 总体最高障碍高度。 |
| `mark_threshold` | `5` | 一个栅格列至少 5 个体素命中才标记障碍。 |
| `unknown_threshold` | `15` | 未知体素阈值。 |
| `combination_method` | `1` | 与已有 costmap 取最大代价值合并。 |
| `observation_sources` | `velodyne` | 障碍观测源名称。 |
| `velodyne.topic` | `/velodyne_points/deskewed` | 局部规划使用的去畸变点云。 |
| `velodyne.data_type` | `PointCloud2` | 输入数据类型。 |
| `velodyne.sensor_frame` | 空 | 使用点云消息自身 frame。 |
| `expected_update_rate` | `0.0` | 不强制检查点云频率。 |
| `observation_persistence` | `0.0` | 不保留历史观测。 |
| `marking` | `true` | 点云可标记障碍。 |
| `clearing` | `true` | 点云可通过射线清除自由空间。 |
| `obstacle_min_range` | `0.6` | 0.6m 内点不标记障碍。 |
| `obstacle_max_range` | `3.0` | 3m 外点不标记障碍。 |
| `raytrace_min_range` | `0.0` | 清障射线最小距离。 |
| `raytrace_max_range` | `4.0` | 清障射线最大距离。 |
| `min_obstacle_height` | `0.08` | 8cm 以下点不作为障碍。 |
| `max_obstacle_height` | `1.3` | 1.3m 以上点不作为障碍。 |

## Inflation Layers

| 所属地图 | 参数 | 当前值 | 作用 |
|---|---|---:|---|
| local costmap | `inflation_radius` | `0.55` | 局部障碍膨胀半径。 |
| local costmap | `cost_scaling_factor` | `3.0` | 膨胀代价衰减速度。 |
| global costmap | `inflation_radius` | `0.6` | 全局地图障碍膨胀半径。 |
| global costmap | `cost_scaling_factor` | `3.0` | 膨胀代价衰减速度。 |

## Global Costmap

| 参数 | 当前值 | 作用 |
|---|---:|---|
| `global_frame` | `map` | 全局代价地图坐标系。 |
| `robot_base_frame` | `base_link` | 机器人基坐标系。 |
| `update_frequency` | `1.0` | 全局地图更新频率。 |
| `publish_frequency` | `1.0` | 全局地图发布频率。 |
| `transform_tolerance` | `1.0` | TF 容忍时间。 |
| `resolution` | `0.1` | 全局 costmap 分辨率。 |
| `track_unknown_space` | `true` | 保留未知区域。 |
| `plugins` | `[static_layer, inflation_layer]` | 使用静态地图层和膨胀层。 |
| `static_layer.plugin` | `nav2_costmap_2d::StaticLayer` | 使用 map_server 发布的静态地图。 |
| `static_layer.map_topic` | `/map` | AMCL 模式下静态地图话题。 |
| `map_subscribe_transient_local` | `true` | 以 transient local QoS 订阅静态地图。 |

## Map Server / Saver

| 参数 | 当前值 | 作用 |
|---|---:|---|
| `map_server.yaml_filename` | 空，launch 注入 `map` 参数 | 静态地图 YAML 路径。 |
| `map_server.topic_name` | `/map` | map_server 发布地图的话题。 |
| `map_saver.save_map_timeout` | `5.0` | 保存地图等待超时。 |
| `map_saver.free_thresh_default` | `0.25` | 默认自由阈值。 |
| `map_saver.occupied_thresh_default` | `0.65` | 默认占据阈值。 |
| `map_subscribe_transient_local` | `true` | 保存地图时使用 transient local 订阅。 |

## Planner Server

| 参数 | 当前值 | 作用 |
|---|---:|---|
| `expected_planner_frequency` | `5.0` | 期望全局规划频率。 |
| `planner_plugins` | `[GridBased]` | 使用 GridBased 规划器。 |
| `GridBased.plugin` | `nav2_navfn_planner/NavfnPlanner` | 使用 NavFn 全局规划器。 |
| `tolerance` | `0.5` | 目标不可达时允许靠近目标的距离。 |
| `use_astar` | `false` | 使用 Dijkstra，而不是 A*。 |
| `allow_unknown` | `true` | 允许规划经过未知区域。 |

## Smoother Server

| 参数 | 当前值 | 作用 |
|---|---:|---|
| `smoother_plugins` | `[simple_smoother]` | 使用简单路径平滑器。 |
| `simple_smoother.plugin` | `nav2_smoother::SimpleSmoother` | 平滑全局路径。 |
| `tolerance` | `1.0e-10` | 平滑收敛容差。 |
| `max_its` | `1000` | 最大迭代次数。 |
| `do_refinement` | `true` | 对路径进行额外细化。 |

## Behavior Server

| 参数 | 当前值 | 作用 |
|---|---:|---|
| `costmap_topic` | `local_costmap/costmap_raw` | 行为服务器使用的局部 costmap。 |
| `footprint_topic` | `local_costmap/published_footprint` | 行为服务器使用的机器人 footprint。 |
| `cycle_frequency` | `10.0` | 行为服务器循环频率。 |
| `global_frame` | `odom` | 行为执行坐标系。 |
| `robot_base_frame` | `base_link` | 机器人基坐标系。 |
| `simulate_ahead_time` | `2.0` | 行为前向仿真时间。 |
| `max_rotational_vel` | `0.2` | 最大旋转速度。 |
| `min_rotational_vel` | `0.05` | 最小旋转速度。 |
| `rotational_acc_lim` | `0.4` | 旋转加速度限制。 |
| `behavior_plugins` | `[spin, backup, drive_on_heading, wait]` | 启用旋转、后退、直行、等待行为。 |
| `backup.acceleration_limit` | `0.3` | 后退行为加速度限制。 |
| `backup.deceleration_limit` | `-0.3` | 后退行为减速度限制。 |
| `drive_on_heading.acceleration_limit` | `0.3` | 指定方向行驶加速度限制。 |
| `drive_on_heading.deceleration_limit` | `-0.3` | 指定方向行驶减速度限制。 |

## Waypoint Follower

| 参数 | 当前值 | 作用 |
|---|---:|---|
| `loop_rate` | `20` | 路点跟随循环频率。 |
| `stop_on_failure` | `false` | 单个路点失败时不停止整个任务。 |
| `waypoint_task_executor_plugin` | `wait_at_waypoint` | 到达路点后执行等待插件。 |
| `wait_at_waypoint.enabled` | `true` | 启用路点等待。 |
| `waypoint_pause_duration` | `200` | 路点停留时间，单位毫秒。 |

## Velocity Smoother

| 参数 | 当前值 | 作用 |
|---|---:|---|
| `smoothing_frequency` | `20.0` | 速度平滑频率。 |
| `scale_velocities` | `false` | 不按比例缩放速度向量。 |
| `feedback` | `OPEN_LOOP` | 开环速度平滑，不依赖实时反馈修正。 |
| `max_velocity` | `[0.2, 0.0, 0.2]` | x/y/theta 最大速度。 |
| `min_velocity` | `[-0.2, 0.0, -0.2]` | x/y/theta 最小速度。 |
| `max_accel` | `[0.3, 0.0, 0.4]` | x/y/theta 最大加速度。 |
| `max_decel` | `[-0.3, 0.0, -0.4]` | x/y/theta 最大减速度。 |
| `odom_topic` | `/odom` | 里程计话题。 |
| `odom_duration` | `0.3` | 里程计速度估计时间窗口。 |
| `deadband_velocity` | `[0.0, 0.0, 0.0]` | 速度死区。 |
| `velocity_timeout` | `1.0` | 超时未收到速度命令则输出停止。 |

## AMCL 参数

| 参数 | 当前值 | 作用 |
|---|---:|---|
| `scan_topic` | `scan` | AMCL 使用的 2D 激光话题。 |
| `base_frame_id` | `base_link` | 机器人基坐标系。 |
| `global_frame_id` | `map` | 全局地图坐标系。 |
| `odom_frame_id` | `odom` | 里程计坐标系。 |
| `robot_model_type` | `nav2_amcl::DifferentialMotionModel` | AMCL 运动模型。 |
| `laser_model_type` | `likelihood_field` | 激光匹配模型。 |
| `laser_min_range` | `0.3` | 激光最小有效距离。 |
| `laser_max_range` | `40.0` | 激光最大有效距离。 |
| `laser_likelihood_max_dist` | `2.0` | 似然场最大距离。 |
| `max_beams` | `60` | 每帧参与匹配的激光束数量。 |
| `min_particles` | `300` | 最小粒子数。 |
| `max_particles` | `1500` | 最大粒子数。 |
| `pf_err` | `0.05` | 粒子滤波误差界。 |
| `pf_z` | `0.99` | 粒子滤波置信度。 |
| `alpha1` - `alpha5` | `0.2` | 运动模型噪声参数。 |
| `update_min_d` | `0.25` | 移动超过 0.25m 才更新粒子。 |
| `update_min_a` | `0.2` | 转动超过 0.2rad 才更新粒子。 |
| `resample_interval` | `1` | 每次更新都重采样。 |
| `transform_tolerance` | `1.0` | 发布 TF 的时间容忍。 |
| `tf_broadcast` | `true` | AMCL 发布 `map -> odom`。 |
| `save_pose_rate` | `0.5` | 保存 AMCL 位姿频率。 |
| `do_beamskip` | `false` | 不启用 beam skipping。 |
| `beam_skip_distance` | `0.5` | beam skipping 距离阈值。 |
| `beam_skip_threshold` | `0.3` | beam skipping 比例阈值。 |
| `beam_skip_error_threshold` | `0.9` | beam skipping 错误阈值。 |
| `z_hit` | `0.5` | 命中障碍的激光模型权重。 |
| `z_rand` | `0.5` | 随机测量权重。 |
| `z_short` | `0.05` | 短读数权重。 |
| `z_max` | `0.05` | 最大量程读数权重。 |
| `sigma_hit` | `0.2` | 命中模型标准差。 |
| `lambda_short` | `0.1` | 短读数指数模型参数。 |
| `recovery_alpha_fast` | `0.0` | 快速恢复参数，当前关闭。 |
| `recovery_alpha_slow` | `0.0` | 慢速恢复参数，当前关闭。 |

## 话题与命令输出

| 项目 | 当前值 | 作用 |
|---|---|---|
| controller/behavior 输入 | `cmd_vel_nav` | 控制器和行为服务器输出内部速度命令。 |
| velocity_smoother 输出 | `/cmd_vel` | 平滑后发送到底盘的速度命令。 |
| map_server 输出 | `/map` 或 `/static_map` | AMCL 模式发布 `/map`；RTAB-Map 定位 Nav2-only 模式中 remap 到 `/static_map`。 |
| local costmap 点云输入 | `/velodyne_points/deskewed` | 局部避障使用的实时去畸变点云。 |
| AMCL scan 输入 | `/scan` | AMCL 使用的 2D 激光投影。 |
