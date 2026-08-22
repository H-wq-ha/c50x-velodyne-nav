# 2026-08-02 RTAB-Map / Nav2 对话日志

本文记录本次关于 RTAB-Map 建图保存、RTAB-Map 定位 + Nav2 导航、`/cmd_vel` 发布条件、到达目标后继续发速度/原地转圈问题的分析结论。

## 1. 如何保存当前构造的地图

项目中已有保存脚本：

```bash
cd $PROJECT_ROOT
source /opt/ros/humble/setup.bash
source install/setup.bash
./scripts/save_rtabmap_map.sh
```

前提是 RTAB-Map 建图或定位节点正在运行，例如：

```bash
ros2 launch velodyne_bringup rtabmap_slam.launch.py
```

脚本会自动从 `/map`、`/proj_map`、`/grid_prob_map` 中选择可用的 `nav_msgs/OccupancyGrid` 话题，并保存：

```text
maps/rtabmap_nav2_latest.yaml
maps/rtabmap_nav2_latest.pgm
maps/rtabmap_nav2_latest.db
```

如果自动选择失败，可以手动指定话题：

```bash
./scripts/save_rtabmap_map.sh /map
./scripts/save_rtabmap_map.sh /grid_prob_map
```

保存后可启动定位导航：

```bash
ros2 launch velodyne_bringup rtabmap_nav_complete.launch.py
```

## 2. RTAB-Map 定位 + Nav2 导航工作流层次

工作流可分为 5 层：

1. 传感器/底盘层：`velodyne_driver`、IMU、`chassis_node`。底盘节点订阅最终 `/cmd_vel`。
2. 里程计层：`lidar_deskewing` 去畸变，`icp_odometry` 发布 `/odom` 和 `odom -> base_link`。
3. 全局定位层：RTAB-Map 加载 `maps/rtabmap_nav2_latest.db`，发布 `map -> odom`。
4. 地图/代价地图层：`map_server` 加载 `maps/rtabmap_nav2_latest.yaml`，Nav2 使用 global/local costmap。
5. Nav2 任务层：`bt_navigator` 接收目标，`planner_server` 规划路径，`controller_server` 计算局部速度，`velocity_smoother` 平滑后发布 `/cmd_vel`。

速度链路：

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

## 3. 什么情况下才会发布 `/cmd_vel`

`/cmd_vel` 不是 `controller_server` 直接发布的最终话题。当前工程中：

- `controller_server` 和 `behavior_server` 发布内部速度 `cmd_vel_nav`。
- `velocity_smoother` 订阅 `cmd_vel_nav`。
- `velocity_smoother` 输出最终 `/cmd_vel` 给 `chassis_node`。

通常只有以下条件满足时才会发布运动速度：

- Nav2 生命周期节点已激活。
- 有正在执行的 `Nav2 Goal`。
- `planner_server` 成功产生路径。
- TF 连通：`map -> odom -> base_link`。
- local/global costmap 可用。
- `controller_server` 能算出合法局部速度。

到达目标、取消任务、失败或恢复行为期间，也可能发布零速度或短暂恢复动作速度。

## 4. 上一次第二个 goal 后仍发布 `/cmd_vel` 的日志分析

分析的日志目录：

```text
~/.ros/log/2026-08-02-19-49-31-546044-y-10550
```

关键结论：第二个 goal 并没有被 Nav2 判定为成功，所以 Nav2 仍处于执行状态，`controller_server -> cmd_vel_nav -> velocity_smoother -> /cmd_vel` 链路继续工作。

关键时间线：

```text
19:50:53  第一次 Nav2 Goal 发出
19:51:13  controller_server: Reached the goal!
19:51:13  bt_navigator: Goal succeeded

19:51:21  第二次 Nav2 Goal 发出
之后没有出现第二次:
  controller_server: Reached the goal!
  bt_navigator: Goal succeeded
```

第二次 goal 期间出现：

```text
controller_server: Passing new path to controller
No valid trajectories out of 155!
Transform data too old when converting from map to odom
Extrapolation Error ... base_link to map
```

到用户 Ctrl-C 时，Nav2 仍认为 goal 正在执行：

```text
[navigate_to_pose] Requested to deactivate server but goal is still executing
Goal canceled
```

底层问题包括：

- 雷达数据中断：`Velodyne poll() timeout`。
- `lidar_deskewing` / `icp_odometry` 报 5 秒没有输入。
- TF 过期：`Transform data too old`。

该组日志还显示实际启动的是 AMCL 节点，而不是 RTAB-Map 定位节点，因此当时更像 AMCL + Nav2 流程。

建议复现时同时观察：

```bash
ros2 topic echo /cmd_vel
ros2 topic echo /cmd_vel_nav
ros2 topic hz /velodyne_points
ros2 run tf2_ros tf2_echo map base_link
```

## 5. 如何让到达 goal 的判断更宽松

当前配置位置：

```text
src/velodyne_bringup/config/nav2_params.yaml
```

当前参数：

```yaml
general_goal_checker:
  stateful: true
  plugin: nav2_controller::SimpleGoalChecker
  xy_goal_tolerance: 0.25
  yaw_goal_tolerance: 0.25
```

建议先放宽为：

```yaml
general_goal_checker:
  stateful: true
  plugin: nav2_controller::SimpleGoalChecker
  xy_goal_tolerance: 0.45
  yaw_goal_tolerance: 0.70
```

含义：

- `xy_goal_tolerance: 0.45`：距离目标 45 cm 内算到达。
- `yaw_goal_tolerance: 0.70`：朝向误差约 40 度内算到达。

如果完全不关心最终朝向，可设为：

```yaml
yaw_goal_tolerance: 6.28
```

也可以临时在线测试：

```bash
ros2 param set /controller_server general_goal_checker.xy_goal_tolerance 0.45
ros2 param set /controller_server general_goal_checker.yaw_goal_tolerance 0.70
```

## 6. 当前日志中“到达地点后原地转圈”的原因

分析的最新日志：

```text
~/.ros/log/2026-08-02-20-08-04-388120-y-13978
```

关键结论：原地转圈是 Nav2 的恢复行为 `spin`，不是已成功到达后仍正常导航。

关键日志：

```text
20:08:33  第二个目标开始:
bt_navigator: Begin navigating from current location (3.27, 1.38) to (-1.11, 1.99)

20:09:47 起多次:
controller_server: Failed to make progress
[follow_path] Aborting handle

20:10:32:
behavior_server: Running spin
behavior_server: Turning 1.57 for spin behavior.

20:10:40:
behavior_server: spin completed successfully
```

因此，车看起来已经到达地点，但 Nav2 没有判定 `Goal succeeded`，而是认为局部控制没有进展，于是行为树进入恢复动作，执行原地旋转约 90 度。

同时日志里还出现点云/TF 时间不同步：

```text
local_costmap: Message Filter dropping message ...
timestamp on the message is earlier than all the data in the transform cache
```

这会影响 local costmap 和 Nav2 对当前状态的判断。

建议配置：

```yaml
general_goal_checker:
  xy_goal_tolerance: 0.45
  yaw_goal_tolerance: 0.70

progress_checker:
  required_movement_radius: 0.10
  movement_time_allowance: 25.0
```

核心判断：

```text
这次转圈是第二个 goal 没有成功结束，Failed to make progress 后触发 behavior_server 的 Spin 恢复动作。
```

放宽 goal tolerance 可减少“到点后继续对角度”的行为；放宽 progress checker 可减少过快进入恢复动作。但 TF/点云时间戳问题仍需继续排查。
