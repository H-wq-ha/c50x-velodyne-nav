"""Nav2 navigation for WHEELTEC C50X + VLP-16 using RTAB-Map localization.

RTAB-Map must provide map -> icp_odom, while icp_odometry provides
icp_odom -> base_link. This launch starts Nav2 planning/control only.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_dir = get_package_share_directory('velodyne_bringup')
    default_params = os.path.join(pkg_dir, 'config', 'nav2_params.yaml')
    default_rviz_config = os.path.join(pkg_dir, 'config', 'nav2_wheeltec.rviz')
    default_map = os.path.join(
        os.path.expanduser('~'),
        'arm_car',
        'lidar',
        'maps',
        'latest.yaml')

    params_file = LaunchConfiguration('params_file')
    map_file = LaunchConfiguration('map')
    autostart = LaunchConfiguration('autostart')
    use_rviz = LaunchConfiguration('use_rviz')
    rviz_config = LaunchConfiguration('rviz_config')

    lifecycle_nodes = [
        'map_server',
        'controller_server',
        'smoother_server',
        'planner_server',
        'behavior_server',
        'bt_navigator',
        'waypoint_follower',
        'velocity_smoother',
    ]

    return LaunchDescription([
        DeclareLaunchArgument(
            'params_file',
            default_value=default_params,
            description='Full path to the Nav2 parameters file.'),
        DeclareLaunchArgument(
            'map',
            default_value=default_map,
            description='Full path to the Nav2 occupancy-grid map YAML.'),
        DeclareLaunchArgument(
            'autostart',
            default_value='true',
            description='Automatically activate Nav2 lifecycle nodes.'),
        DeclareLaunchArgument(
            'use_rviz',
            default_value='true',
            description='Start RViz2 with the WHEELTEC Nav2 configuration.'),
        DeclareLaunchArgument(
            'rviz_config',
            default_value=default_rviz_config,
            description='Full path to the RViz2 configuration file.'),

        Node(
            package='nav2_map_server',
            executable='map_server',
            name='map_server',
            output='screen',
            parameters=[params_file, {'yaml_filename': map_file}],
            remappings=[('map', '/static_map')]),

        Node(
            package='nav2_controller',
            executable='controller_server',
            name='controller_server',
            output='screen',
            parameters=[params_file],
            remappings=[('cmd_vel', 'cmd_vel_nav')]),

        Node(
            package='nav2_smoother',
            executable='smoother_server',
            name='smoother_server',
            output='screen',
            parameters=[params_file]),

        Node(
            package='nav2_planner',
            executable='planner_server',
            name='planner_server',
            output='screen',
            parameters=[params_file]),

        Node(
            package='nav2_behaviors',
            executable='behavior_server',
            name='behavior_server',
            output='screen',
            parameters=[params_file],
            remappings=[('cmd_vel', 'cmd_vel_nav')]),

        Node(
            package='nav2_bt_navigator',
            executable='bt_navigator',
            name='bt_navigator',
            output='screen',
            parameters=[params_file]),

        Node(
            package='nav2_waypoint_follower',
            executable='waypoint_follower',
            name='waypoint_follower',
            output='screen',
            parameters=[params_file]),

        Node(
            package='nav2_velocity_smoother',
            executable='velocity_smoother',
            name='velocity_smoother',
            output='screen',
            parameters=[params_file],
            remappings=[
                ('cmd_vel', 'cmd_vel_nav'),
                ('cmd_vel_smoothed', '/cmd_vel'),
            ]),

        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_navigation',
            output='screen',
            parameters=[{
                'use_sim_time': False,
                'autostart': autostart,
                'node_names': lifecycle_nodes,
            }]),

        Node(
            condition=IfCondition(use_rviz),
            package='rviz2',
            executable='rviz2',
            name='rviz2_nav2',
            output='screen',
            arguments=['-d', rviz_config]),
    ])
