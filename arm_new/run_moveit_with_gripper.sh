#!/usr/bin/env bash
# 启动 piper_with_gripper_moveit demo（MoveIt2 + RViz，含夹爪）
set -e
cd "$(dirname "$0")"
source /opt/ros/humble/setup.bash
source install/setup.bash
export LC_NUMERIC=en_US.UTF-8
export PATH=/usr/bin:$PATH
exec ros2 launch piper_with_gripper_moveit demo.launch.py
