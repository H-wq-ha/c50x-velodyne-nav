#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Integrated PiPER + camera + YOLO + MoveIt + grasp planner launch.

Start order:
  1. piper driver (start_single_piper.launch.py)
  2. camera + YOLO vision
  3. MoveIt with gripper (demo.launch.py) -- fake controllers + RViz
  4. target_grasp_node after MoveIt is ready

The piper driver subscribes /joint_states (remapped from
/joint_ctrl_single), so MoveIt's fake controller stream is forwarded to
the real arm.  Keep auto_enable:=false until the arm workspace and the
MoveIt start state have been checked.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    TimerAction,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    piper_share = get_package_share_directory('piper')
    moveit_share = get_package_share_directory('piper_with_gripper_moveit')
    vision_share = get_package_share_directory('piper_vision')

    start_piper = DeclareLaunchArgument(
        'start_piper_driver', default_value='true',
        description='Start ros2 launch piper start_single_piper.launch.py')
    auto_enable = DeclareLaunchArgument(
        'auto_enable', default_value='false',
        description='Auto-enable the real arm in the piper driver')
    start_moveit = DeclareLaunchArgument(
        'start_moveit', default_value='true',
        description='Start piper_with_gripper_moveit demo.launch.py')
    start_vision = DeclareLaunchArgument(
        'start_vision', default_value='true',
        description='Start camera + YOLO vision launch')
    start_planner = DeclareLaunchArgument(
        'start_planner', default_value='true',
        description='Start the target grasp planner node')
    use_gui = DeclareLaunchArgument(
        'use_gui', default_value='true',
        description='Show rqt_image_view and MoveIt RViz')
    use_plot = DeclareLaunchArgument(
        'use_plot', default_value='true',
        description='Show the matplotlib base/camera/object TF plot')
    execute = DeclareLaunchArgument(
        'execute', default_value='false',
        description='Execute the planned trajectory after successful planning')
    target_label = DeclareLaunchArgument(
        'target_label', default_value='cell phone')
    gripper_length = DeclareLaunchArgument(
        'gripper_length', default_value='0.1358',
        description='link6 -> gripper TCP length in metres')
    tcp_offset = DeclareLaunchArgument(
        'tcp_offset', default_value='0.0',
        description='TCP offset along tool +Z; negative = above object')
    orientation_tolerance = DeclareLaunchArgument(
        'orientation_tolerance', default_value='0.35',
        description='MoveIt orientation tolerance in radians')
    position_tolerance = DeclareLaunchArgument(
        'position_tolerance', default_value='0.01',
        description='MoveIt position tolerance in metres')
    planning_time = DeclareLaunchArgument(
        'planning_time', default_value='10.0')
    num_planning_attempts = DeclareLaunchArgument(
        'num_planning_attempts', default_value='10')
    velocity_scaling = DeclareLaunchArgument(
        'velocity_scaling', default_value='0.10')
    acceleration_scaling = DeclareLaunchArgument(
        'acceleration_scaling', default_value='0.10')
    moveit_delay = DeclareLaunchArgument(
        'moveit_delay', default_value='3.0')
    planner_delay = DeclareLaunchArgument(
        'planner_delay', default_value='8.0')

    piper_driver = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(piper_share, 'launch',
                         'start_single_piper.launch.py')),
        launch_arguments={
            'can_port': 'can0',
            'gripper_exist': 'true',
            'gripper_val_mutiple': '2',
            'auto_enable': LaunchConfiguration('auto_enable'),
        }.items(),
        condition=IfCondition(LaunchConfiguration('start_piper_driver')),
    )

    vision = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(vision_share, 'launch',
                         'yolo_depth.launch.py')),
        launch_arguments={
            'use_gui': LaunchConfiguration('use_gui'),
            'use_plot': LaunchConfiguration('use_plot'),
        }.items(),
        condition=IfCondition(LaunchConfiguration('start_vision')),
    )

    moveit = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(moveit_share, 'launch', 'demo.launch.py')),
        condition=IfCondition(LaunchConfiguration('start_moveit')),
    )

    planner = Node(
        package='piper_grasp',
        executable='target_grasp_node',
        name='target_grasp_node',
        output='screen',
        parameters=[{
            'target_label': LaunchConfiguration('target_label'),
            'gripper_length': ParameterValue(
                LaunchConfiguration('gripper_length'), value_type=float),
            'tcp_offset': ParameterValue(
                LaunchConfiguration('tcp_offset'), value_type=float),
            'orientation_tolerance': ParameterValue(
                LaunchConfiguration('orientation_tolerance'),
                value_type=float),
            'position_tolerance': ParameterValue(
                LaunchConfiguration('position_tolerance'),
                value_type=float),
            'planning_time': ParameterValue(
                LaunchConfiguration('planning_time'), value_type=float),
            'num_planning_attempts': ParameterValue(
                LaunchConfiguration('num_planning_attempts'),
                value_type=int),
            'velocity_scaling': ParameterValue(
                LaunchConfiguration('velocity_scaling'), value_type=float),
            'acceleration_scaling': ParameterValue(
                LaunchConfiguration('acceleration_scaling'),
                value_type=float),
            'execute': ParameterValue(
                LaunchConfiguration('execute'), value_type=bool),
            'require_piper': ParameterValue(
                LaunchConfiguration('start_piper_driver'), value_type=bool),
        }],
        condition=IfCondition(LaunchConfiguration('start_planner')),
    )

    return LaunchDescription([
        start_piper,
        auto_enable,
        start_moveit,
        start_vision,
        start_planner,
        use_gui,
        use_plot,
        execute,
        target_label,
        gripper_length,
        tcp_offset,
        orientation_tolerance,
        position_tolerance,
        planning_time,
        num_planning_attempts,
        velocity_scaling,
        acceleration_scaling,
        moveit_delay,
        planner_delay,
        piper_driver,
        vision,
        TimerAction(
            period=LaunchConfiguration('moveit_delay'),
            actions=[moveit]),
        TimerAction(
            period=LaunchConfiguration('planner_delay'),
            actions=[planner]),
    ])
