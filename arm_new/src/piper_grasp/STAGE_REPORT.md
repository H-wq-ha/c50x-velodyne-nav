# 视觉抓取联调阶段报告

日期：2026-11-09

## 阶段结果

| 阶段 | 内容 | 结果 |
|---|---|---|
| 1 | D435i + YOLOv8n 检测 `cell phone` | 通过 |
| 2 | 内参反投影 + 手眼外参变换到 `base_link` | 通过 |
| 3 | 加夹爪长度偏置得到 `link6` 目标 | 通过 |
| 4 | MoveIt `arm` 组规划 | 通过 |
| 5 | `/execute_trajectory` 执行规划轨迹（MoveIt mock） | 通过 |
| 6 | 真机 piper 驱动 | 未通过：CAN 无反馈 |

## 阶段 1-2：视觉与 base 坐标

```text
target = cell phone
score  = 0.692
base_xyz = (+0.2361, +0.1823, -0.0306) m
frame_id = base_link
```

## 阶段 3：夹爪长度偏置

URDF：

```text
joint6_to_gripper_base: link6 -> gripper_base, identity
joint7 origin: gripper_base + (0, 0, 0.1358) m
```

计算：

```text
tool_z = (0, 0, -1)          # top-down
tcp_target  = object_base + tool_z * tcp_offset
link6_target = tcp_target - tool_z * gripper_length
```

结果：

```text
tcp_target   = (+0.2361, +0.1823, -0.0306) m
link6_target = (+0.2361, +0.1823, +0.1052) m
quaternion   = (+1.000, +0.000, +0.000, +0.000)
position_tolerance    = 0.01 m
orientation_tolerance = 0.35 rad
```

## 阶段 4：规划

```text
group = arm
eef   = link6
plan_only = true
trajectory points = 65
trajectory duration = 6.33 s
planning time = 0.06 s
result = SUCCESS
```

## 阶段 5：mock 执行

```text
/execute_trajectory accepted
EXECUTION SUCCESS
```

执行后 `base_link -> link6`：

```text
translation = (0.235, 0.186, 0.114) m
target      = (0.236, 0.182, 0.105) m
```

位置误差约 8 mm，姿态误差约 13°，均在设定容差内。

## 阶段 6：真机驱动

执行：

```bash
ros2 launch piper start_single_piper.launch.py \
  auto_enable:=false gripper_val_mutiple:=2
```

结果：

```text
SearchMotorMaxAngleSpdAccLimit send failed: SEND_MESSAGE_FAILED
can0 is loss
process died
```

同时 `candump can0` 没有收到任何报文。

判断：机械臂未上电、CAN 线未连接，或 CAN 总线没有 ACK。

因此**没有向真机发送任何运动轨迹**，真机执行阶段未进行。

## 当前运行

```text
ros2 launch piper_grasp grasp_pipeline.launch.py \
  start_piper_driver:=false \
  start_moveit:=true \
  start_vision:=true \
  execute:=true
```

只验证了视觉 + MoveIt mock 全链路，真实机械臂未动。
