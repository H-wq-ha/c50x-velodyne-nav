# 项目问题与解决方案 QA

本文汇总本项目开发、调试 RTAB-Map 建图定位、Nav2 导航、VLP-16 点云、底盘串口与日志分析时遇到的问题和处理办法。

## 依据来源

- Codex/Claude 项目记忆：当前仓库下 `.codex/`、`.agents/` 未发现可读项目记忆文件；`.claude/` 仅有本地设置文件。
- 项目文档：`docs/node_data_flows.md`、`docs/nav2_parameters.md`、`docs/rtabmap_mapping_parameters.md`。
- 本次对话日志：`docs/conversation_log_2026-08-02_nav2_rtabmap.md`。
- ROS 日志：`~/.ros/log/2026-08-02-19-49-31-546044-y-10550`、`~/.ros/log/2026-08-02-20-08-04-388120-y-13978`、`~/.ros/log/2026-08-02-20-13-18-450495-y-18533`。

## QA

### Q1: 如何保存当前 RTAB-Map 构造的地图？

**现象/需求**：建图完成后，需要保存给 Nav2 使用的 2D 地图，同时保留 RTAB-Map 定位所需数据库。

**解决方案**：

```bash
cd $PROJECT_ROOT
source /opt/ros/humble/setup.bash
source install/setup.bash
./scripts/save_rtabmap_map.sh
```

脚本会从 `/map`、`/proj_map`、`/grid_prob_map` 中选择可用的 `nav_msgs/OccupancyGrid`，并保存：

```text
maps/rtabmap_nav2_latest.yaml
maps/rtabmap_nav2_latest.pgm
maps/rtabmap_nav2_latest.db
```

若自动选择失败，可手动指定：

```bash
./scripts/save_rtabmap_map.sh /map
./scripts/save_rtabmap_map.sh /grid_prob_map
```

### Q2: RTAB-Map 定位 + Nav2 导航的完整层次是什么？

**答案**：当前流程分为 5 层。

1. 传感器/底盘层：VLP-16、IMU、`chassis_node`。
2. 里程计层：`lidar_deskewing` 去畸变，`icp_odometry` 发布 `/odom` 和 `odom -> base_link`。
3. 全局定位层：RTAB-Map 加载 `maps/rtabmap_nav2_latest.db`，发布 `map -> odom`。
4. 地图/代价地图层：`map_server` 加载 `maps/rtabmap_nav2_latest.yaml`，Nav2 维护 global/local costmap。
5. Nav2 任务层：`bt_navigator`、`planner_server`、`controller_server`、`behavior_server`、`velocity_smoother`。

速度链路为：

```text
Nav2 Goal
  -> bt_navigator
  -> planner_server
  -> controller_server / behavior_server
  -> cmd_vel_nav
  -> velocity_smoother
  -> /cmd_vel
  -> chassis_node
```

### Q3: 什么情况下才会发布 `/cmd_vel`？

**原因机制**：当前工程中 `controller_server` 和 `behavior_server` 不直接发布最终 `/cmd_vel`，它们发布 `cmd_vel_nav`，再由 `velocity_smoother` 输出 `/cmd_vel`。

**会发布的情况**：

- Nav2 生命周期节点已经 active。
- 存在正在执行的导航目标。
- `planner_server` 有路径，`controller_server` 能算出合法局部速度。
- TF 链 `map -> odom -> base_link` 连通。
- local/global costmap 可用。
- Nav2 进入恢复行为时，`behavior_server` 也会通过同一速度链路发布旋转、后退、等待后的停止速度等命令。

**不会继续正常发布运动速度的情况**：目标真正成功、被取消、导航失败结束后，通常只会短暂发布零速度或平滑停止速度。

### Q4: 为什么第二次到达 goal 后仍然发布 `/cmd_vel`？

**日志依据**：`~/.ros/log/2026-08-02-19-49-31-546044-y-10550`。

**结论**：第二个 goal 没有被 Nav2 判定为成功，因此 Nav2 仍处于执行状态，速度链路继续工作。

**关键日志模式**：

```text
第一次 goal:
controller_server: Reached the goal!
bt_navigator: Goal succeeded

第二次 goal:
没有出现第二次 Reached the goal / Goal succeeded
controller_server: Passing new path to controller
No valid trajectories out of 155!
Transform data too old when converting from map to odom
Extrapolation Error ... base_link to map
```

用户 Ctrl-C 时还出现：

```text
[navigate_to_pose] Requested to deactivate server but goal is still executing
Goal canceled
```

**解决方案**：

- 先修复雷达数据流和 TF 新鲜度，而不是只盯 `/cmd_vel`。
- 同时观察：

```bash
ros2 topic echo /cmd_vel_nav
ros2 topic echo /cmd_vel
ros2 topic hz /velodyne_points
ros2 topic hz /velodyne_points/deskewed
ros2 run tf2_ros tf2_echo map base_link
```

### Q5: 为什么到达地点后还会原地转圈？

**日志依据**：`~/.ros/log/2026-08-02-20-08-04-388120-y-13978`。

**结论**：这次原地转圈是 Nav2 的恢复行为 `spin`，不是已经成功到达后仍在正常导航。

**关键日志**：

```text
controller_server: Failed to make progress
[follow_path] Aborting handle
behavior_server: Running spin
behavior_server: Turning 1.57 for spin behavior.
behavior_server: spin completed successfully
```

**解决方案**：

- 放宽 goal checker，减少已经接近目标但不判成功的情况。
- 放宽 progress checker，避免低速、窄空间或 TF 抖动时过早判定无进展。
- 排查 local costmap 是否因点云/TF 时间戳问题丢数据。

### Q6: 如何让 Nav2 判断到达 goal 的条件更宽松？

**配置文件**：`src/velodyne_bringup/config/nav2_params.yaml`。

**建议初始值**：

```yaml
general_goal_checker:
  stateful: true
  plugin: nav2_controller::SimpleGoalChecker
  xy_goal_tolerance: 0.45
  yaw_goal_tolerance: 0.70
```

含义：

- `xy_goal_tolerance: 0.45`：进入目标 45 cm 范围内算到达。
- `yaw_goal_tolerance: 0.70`：最终朝向误差约 40 度内算到达。

如果完全不关心最终朝向：

```yaml
yaw_goal_tolerance: 6.28
```

同时可考虑：

```yaml
progress_checker:
  required_movement_radius: 0.10
  movement_time_allowance: 25.0
```

### Q7: 为什么日志里有 `Failed to make progress`？

**含义**：`controller_server` 的 progress checker 认为机器人在规定时间内没有移动足够距离。

**常见原因**：

- 机器人实际被障碍、虚假障碍或局部规划约束挡住。
- 目标点附近速度很小，实际位移达不到 `required_movement_radius`。
- TF 或定位跳变，Nav2 认为机器人没有朝目标推进。
- 点云数据中断，local costmap 无法稳定更新。

**解决方案**：

- 将 `required_movement_radius` 从 `0.2` 降到 `0.10`。
- 将 `movement_time_allowance` 从 `15.0` 提高到 `25.0`。
- 同时检查局部 costmap 和 TF，不要只改参数掩盖传感器问题。

### Q8: 为什么日志里反复出现 `Velodyne poll() timeout`？

**含义**：`velodyne_driver_node` 没有收到 VLP-16 UDP 数据包。

**可能原因**：

- 雷达未供电或未旋转。
- 电脑网卡 IP 与雷达不在同一网段。
- 雷达目标 IP/端口设置不对。
- UDP 2368 被网卡、防火墙或网络配置挡住。
- 网线、交换机或接口异常。

**验证命令**：

```bash
ip addr
sudo tcpdump -ni <网卡名> udp port 2368
ros2 topic hz /velodyne_packets
ros2 topic hz /velodyne_points
```

**解决方向**：先确保能抓到 UDP 2368，再看 ROS 点云话题频率。

### Q9: 为什么 `lidar_deskewing` / `icp_odometry` 报 5 秒没有数据？

**含义**：去畸变或 ICP 的输入点云长时间没有到达。

**常见原因**：

- 上游 VLP-16 数据已经中断，通常伴随 `Velodyne poll() timeout`。
- `/velodyne_points` 有数据，但 TF 缺失或时间戳不匹配，导致消息过滤器丢弃。
- IMU 稳定帧 `base_link_stabilized` 没有及时发布，启动初期可短暂出现。

**验证命令**：

```bash
ros2 topic hz /velodyne_points
ros2 topic hz /velodyne_points/deskewed
ros2 topic hz /imu/data
ros2 run tf2_ros tf2_echo base_link base_link_stabilized
ros2 run tf2_ros tf2_echo odom base_link
```

### Q10: 为什么 local costmap 报 `Message Filter dropping message ... timestamp ... earlier than all the data in the transform cache`？

**含义**：点云消息时间戳早于 TF 缓存中最早可用的 TF，costmap 无法把点云转换到目标坐标系。

**解决方案**：

- 确认所有节点都使用真实时间：`use_sim_time: false`。
- 确认系统时间没有跳变。
- 确认 VLP-16 点云时间戳策略与当前 TF 兼容。
- 检查 TF 是否连续发布：

```bash
ros2 run tf2_ros tf2_echo odom base_link
ros2 run tf2_ros tf2_echo map base_link
ros2 topic hz /tf
```

**注意**：增大 `transform_tolerance` 只能缓解小幅延迟，不能解决点云时间戳整体过旧的问题。

### Q11: 为什么出现 `Transform data too old` 或 `Extrapolation Error`？

**含义**：Nav2 请求某一时刻的 TF，但 TF 缓存中没有覆盖该时间点的数据。

**常见原因**：

- 定位节点没有稳定发布 `map -> odom`。
- ICP 里程计没有稳定发布 `odom -> base_link`。
- 传感器输入中断导致定位/里程计停止刷新。
- goal、path、pose 或点云的时间戳和 TF 时间轴不一致。

**解决方案**：

- 修复 VLP-16 UDP 数据流。
- 确认 RTAB-Map 或 AMCL 正在发布 `map -> odom`。
- 确认 ICP 正在发布 `odom -> base_link`。
- 运行：

```bash
ros2 run tf2_ros tf2_echo map base_link
ros2 topic hz /odom
ros2 topic hz /localization_pose
```

### Q12: 为什么期待 RTAB-Map 定位，日志里却看到了 AMCL？

**原因**：启动文件不同。

- `rtabmap_nav_complete.launch.py`：RTAB-Map 定位 + Nav2。
- `nav2_bringup.launch.py`：AMCL + Nav2。
- `nav2_navigation.launch.py`：只启动 Nav2，需要外部已有定位。

**验证命令**：

```bash
ros2 node list | sort
```

如果是 RTAB-Map 定位，应看到 `rtabmap` 相关节点；如果看到 `amcl`，说明当前是 AMCL 流程或正在查看 AMCL 运行日志。

### Q13: RTAB-Map 定位导航里 `/map` 和 `/static_map` 为什么容易混淆？

**现状**：`nav2_navigation.launch.py` 中 `map_server` 可能把 `map` remap 到 `/static_map`，用于避免和 RTAB-Map 的动态 `/map` 冲突；但 `nav2_params.yaml` 里 global costmap 的 `static_layer.map_topic` 当前记录为 `/map`。

**风险**：

- global costmap 可能订阅到 RTAB-Map 的 `/map`，而不是 map_server 的静态地图。
- 或者在某些启动组合下 global costmap 等不到预期静态地图。

**解决方案**：二选一保持一致。

方案 A，RTAB-Map 定位 + Nav2 模式下使用 `/static_map`：

```yaml
global_costmap:
  global_costmap:
    ros__parameters:
      static_layer:
        map_topic: /static_map
```

方案 B，取消 map_server 的 `/static_map` remap，统一使用 `/map`，但要确保不和 RTAB-Map 发布的 `/map` 冲突。

### Q14: 为什么 AMCL 无法正常定位或需要手动初始位姿？

**原因**：AMCL 依赖 `/scan`、静态 `/map`、`/odom` 和初始位姿。

**解决方案**：

- 在 RViz 使用 `2D Pose Estimate`。
- 或发布 `/initialpose`。
- 检查：

```bash
ros2 topic hz /scan
ros2 topic hz /map
ros2 topic hz /odom
ros2 run tf2_ros tf2_echo map base_link
```

### Q15: 为什么 `icp_odometry` 同时提示收到 `/scan` 和点云？

**含义**：ICP 里程计可能同时收到了 2D scan 和 3D cloud 输入，通常是 launch/remap 或订阅参数不够明确。

**解决方案**：

- 对 3D ICP 流程，确保只订阅 `/velodyne_points/deskewed`。
- 显式设置或检查 `subscribe_scan=false`、`subscribe_scan_cloud=true`。
- 避免 AMCL 的 `/scan` 流误接入 ICP。

### Q16: 为什么地图里会有车体附近假障碍或弧形噪声？

**原因**：VLP-16 会扫到车体、地面反射或远处稀疏点；如果直接进 RTAB-Map 占据栅格，会形成假障碍、弧形墙或毛刺。

**当前处理**：

- `velodyne_transform min_range: 0.5`
- `Icp/RangeMin: 0.5`
- `Grid/RangeMin: 0.5`
- `Grid/RangeMax: 5.0`
- `Grid/NoiseFilteringRadius: 0.3`
- `Grid/NoiseFilteringMinNeighbors: 8`
- `Grid/MinClusterSize: 40`
- `Grid/RayTracing: true`
- `Grid/MaxObstacleHeight: 0.8`

**调参方向**：若假障碍仍多，优先调近距离过滤、Z 高度过滤、噪声邻域过滤和雷达安装 TF。

### Q17: `/cmd_vel` 有输出但底盘不动怎么办？

**速度链路**：

```text
cmd_vel_nav -> velocity_smoother -> /cmd_vel -> chassis_node -> 串口 -> 底盘
```

**排查顺序**：

```bash
ros2 topic echo /cmd_vel_nav
ros2 topic echo /cmd_vel
ros2 node list | grep chassis
ls -l /dev/wheeltec_chassis /dev/ttyACM0
python3 tools/check_chassis_serial.py --port /dev/ttyACM0 --baudrate 460800
```

**常见原因**：

- `velocity_smoother` 未 active。
- `chassis_node` 没连接到正确串口。
- 串口权限或 udev 规则问题。
- 底盘急停、供电或控制模式不允许速度命令。

### Q18: 底盘串口协议和检测方式是什么？

**项目实现**：

- `src/car_bringup/car_bringup/chassis_node.py` 订阅 `/cmd_vel`，发布 `/chassis_vel`、`/imu_raw`、`/battery`。
- `src/car_bringup/car_bringup/protocol.py` 实现 WHEELTEC 串口协议，默认 460800 baud。
- 控制帧头尾为 `0x7B` 和 `0x7D`，状态帧长度 24 字节，控制帧长度 11 字节，BCC XOR 校验。

**检测命令**：

```bash
python3 tools/check_chassis_serial.py --port /dev/ttyACM0 --baudrate 460800
```

### Q19: 为什么启动初期有 TF timeout，但后面可能正常？

**原因**：`imu_to_tf`、`icp_odometry`、RTAB-Map/AMCL 的 TF 链需要一点启动时间。刚启动时 `base_link_stabilized`、`odom -> base_link`、`map -> odom` 可能还没完整。

**判断标准**：

- 启动前几秒偶发 warning，可以观察。
- 如果 5 秒以上持续出现 timeout、extrapolation、message filter drop，就要排查传感器输入和 TF 发布。

### Q20: 为什么 RViz 可能崩溃或显示 OpenGL/GLSL 错误？

**现象**：日志中可能出现 `rviz2` 退出码 `-11` 或 GLSL 相关错误。

**影响**：RViz 崩溃不一定代表 Nav2、RTAB-Map 或底盘节点已停止；导航节点可能仍在运行。

**解决方案**：

- 单独重启 RViz。
- 降低显示负载，先关闭点云/voxel 等重显示项。
- 若频繁发生，检查显卡驱动、远程桌面/OpenGL 环境。

### Q21: 如何区分“正常到达前最后旋转”和“恢复行为 spin”？

**正常到达前最后旋转**：

- 多发生在接近目标点时。
- 与 `RotateToGoal` critic、`yaw_goal_tolerance` 有关。
- 最后应出现：

```text
controller_server: Reached the goal!
bt_navigator: Goal succeeded
```

**恢复行为 spin**：

- 通常先出现 `Failed to make progress` 或控制 action abort。
- 日志中明确出现：

```text
behavior_server: Running spin
```

如果是恢复行为，应优先处理 progress、costmap、TF 和传感器输入问题。

### Q22: 如何验证一次导航是否健康？

**启动后检查**：

```bash
ros2 node list | sort
ros2 topic hz /velodyne_points
ros2 topic hz /velodyne_points/deskewed
ros2 topic hz /odom
ros2 run tf2_ros tf2_echo map base_link
```

**发 goal 时检查**：

```bash
ros2 topic echo /cmd_vel_nav
ros2 topic echo /cmd_vel
ros2 topic echo /behavior_tree_log
```

**日志成功标志**：

```text
controller_server: Reached the goal!
bt_navigator: Goal succeeded
```

**异常优先级**：

1. `Velodyne poll() timeout`
2. 点云 5 秒无数据
3. TF extrapolation / data too old
4. local costmap message filter drop
5. controller failed to make progress
6. behavior_server recovery spin/backup

### Q23: 本项目目前最值得优先修的点是什么？

**优先级 1：雷达 UDP 数据稳定性**  
反复出现的 `Velodyne poll() timeout` 是根问题之一。没有稳定点云，ICP、RTAB-Map、local costmap 都会失效。

**优先级 2：TF 时间轴稳定性**  
`Transform data too old`、`Extrapolation Error`、message filter drop 会直接导致 Nav2 不能正确判断位姿、障碍和到达状态。

**优先级 3：goal/progress 参数适配真实底盘**  
当前底盘速度较低，目标附近容易低速调整。建议放宽 `xy_goal_tolerance`、`yaw_goal_tolerance`、`movement_time_allowance`。

**优先级 4：统一 RTAB-Map 导航模式下的地图话题**  
确认 global costmap 订阅的是预期的静态地图话题，避免 `/map` 与 `/static_map` 混用。

**优先级 5：用日志判定状态，不凭肉眼判断到达**  
只有出现 `Reached the goal!` 和 `Goal succeeded`，才代表 Nav2 任务层真正成功。
