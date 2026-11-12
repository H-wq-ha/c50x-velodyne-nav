# piper_grasp

相机 + YOLO + 手眼标定 + MoveIt 的 PiPER 视觉抓取规划/执行节点。

## 1. 数据链

```text
D435i color + aligned depth
  -> YOLOv8n: bbox "cell phone"
  -> calibration.yaml: K/D, cv2.undistortPoints
  -> p_camera
  -> left_hand.yaml: p_base = T_cam2base * p_camera
  -> target_grasp_node:
       p_tcp    = p_base + tool_z * tcp_offset
       p_link6  = p_tcp  - tool_z * gripper_length
  -> MoveIt group "arm", eef "link6", top-down orientation
  -> /move_action (plan_only=True)
  -> /execute_trajectory (only if planning succeeded and execute:=true)
  -> fake controllers -> /joint_states
  -> piper driver (start_single_piper.launch.py) -> CAN -> real arm
```

夹爪长度：

```text
link6 -> gripper_base -> TCP = 0.1358 m (URDF joint7 origin z)
```

`target_grasp_node` 仅规划 `arm` 组（joint1~joint6）。夹爪开合由
`gripper` 组或 `/gripper_controller` 单独控制。

## 2. 分阶段启动

### Stage 1：只验证视觉和 base 坐标

```bash
cd tools/arm_new
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch piper_vision yolo_depth.launch.py
ros2 topic echo /vision/detections_json
```

确认 `cell phone` 的 `xyz_base_m`。

### Stage 2：只规划，不执行（不启动真机驱动）

```bash
ros2 launch piper_grasp grasp_pipeline.launch.py \
  start_piper_driver:=false \
  start_moveit:=true \
  start_vision:=true \
  execute:=false
```

日志中应看到：

```text
[STAGE 2] stable target ...
[STAGE 3] TCP target=... link6 target=...
[STAGE 5] PLANNING SUCCESS ...
[STAGE 6] plan_only=True: trajectory NOT executed
```

### Stage 3：在 MoveIt mock 上执行规划轨迹

```bash
ros2 launch piper_grasp grasp_pipeline.launch.py \
  start_piper_driver:=false \
  start_moveit:=true \
  start_vision:=true \
  execute:=true
```

日志中应看到：

```text
[STAGE 5] PLANNING SUCCESS
[STAGE 6] executing the planned trajectory...
[STAGE 7] EXECUTION SUCCESS
```

此阶段只运动 MoveIt 的 mock 机械臂，不会碰真机。

### Stage 4：真机执行

前提：

1. 机械臂已上电；
2. CAN 能收到反馈（`candump can0` 有报文）；
3. `can0` 已配置为 1 Mbps；
4. 已确认 `base_link` 初始位姿和真实机械臂一致；
5. 工作范围清空。

启动底层（只启动一次，规划节点手动启动，便于分阶段检查）：

```bash
ros2 launch piper_grasp grasp_pipeline.launch.py \
  start_piper_driver:=true \
  auto_enable:=false \
  start_moveit:=true \
  start_vision:=true \
  start_planner:=false
```

先只规划：

```bash
ros2 run piper_grasp target_grasp_node --ros-args \
  -p execute:=false \
  -p target_label:="cell phone" \
  -p gripper_length:=0.1358
```

确认规划成功、工作范围安全后，手动使能：

```bash
ros2 service call /enable_srv piper_msgs/srv/Enable "{enable_request: true}"
```

再执行（先停止上一次 plan-only 的 target_grasp_node）：

```bash
ros2 run piper_grasp target_grasp_node --ros-args \
  -p execute:=true \
  -p target_label:="cell phone" \
  -p gripper_length:=0.1358
```

> 注意：官方 piper 驱动是开环桥接。MoveIt 的 fake
> `joint_state_broadcaster` 发布 `/joint_states`，piper 驱动订阅后转发到
> CAN。真机反馈在 `/joint_states_single`，默认不参与 MoveIt 规划。
> 使能前必须确保 MoveIt 的初始 fake 状态与真机当前姿态接近。

## 3. 参数

| 参数 | 默认值 | 说明 |
|---|---|---|
| `target_label` | `cell phone` | 目标类别 |
| `min_score` | `0.25` | 最低置信度 |
| `stable_frames` | `6` | 稳定检测帧数 |
| `stability_tolerance` | `0.01` | 目标位置允许波动（m） |
| `gripper_length` | `0.1358` | link6 到 TCP 的长度（m） |
| `tcp_offset` | `0.0` | 沿工具 +Z 的额外偏移；负值表示在物体上方 |
| `grasp_yaw` | `0.0` | 绕 base Z 的抓取偏航角 |
| `position_tolerance` | `0.01` | MoveIt 位置容差（m） |
| `orientation_tolerance` | `0.35` | MoveIt 姿态容差（rad） |
| `planning_time` | `10.0` | 单次规划时间（s） |
| `num_planning_attempts` | `10` | 规划尝试次数 |
| `velocity_scaling` | `0.10` | 速度缩放 |
| `acceleration_scaling` | `0.10` | 加速度缩放 |
| `execute` | `false` | 规划成功后是否执行 |
| `require_piper` | 跟随 `start_piper_driver` | 为 true 时，若 `/joint_states_single` 超时则拒绝执行，避免误以为真机在动 |

## 4. 输出话题

| 话题 | 说明 |
|---|---|
| `/grasp/object_pose` | 物体在 base 坐标系的位姿 |
| `/grasp/link6_target` | 加夹爪偏置后的 link6 目标位姿 |
| `/grasp/target_markers` | RViz MarkerArray：物体、link6、TCP、夹爪长度箭头、坐标轴 |

RViz 中：

```text
Fixed Frame: base_link
Add -> MarkerArray -> /grasp/target_markers
```

## 5. 已知问题

- `piper_with_gripper_moveit/config/sensors_3d.yaml` 里是占位 Kinect
  配置，`sensors_3d.yaml` 的 octomap 插件未安装时会打印两条 ERROR，
  但不影响规划。当前已改为 `sensors: []`。
- 真机驱动在 CAN 无反馈时会打印 `can0 is loss` 并退出，这是官方驱动的
  行为；需要先确认机械臂上电和 CAN 连接。
- `target_grasp_node` 是单次任务；需要重新规划请重启节点。
- 当前未添加桌面碰撞物体，MoveIt 只做机器人自碰撞检查。真机执行前请
  清空工作范围或自行添加 planning scene。
- 物体抓取姿态固定为 top-down；`cell phone` 是扁平物体，实际夹取效果
  需要根据夹爪行程和物体厚度进一步调整 `tcp_offset`。
