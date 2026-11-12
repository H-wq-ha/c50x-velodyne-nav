"""RTAB-Map 定位 + Nav2 导航 一键启动

工作流:
  1. 首先建图:  ros2 launch velodyne_bringup rtabmap_slam.launch.py
  2. 保存地图:  ./scripts/save_rtabmap_map.sh
  3. 定位导航:  ros2 launch velodyne_bringup rtabmap_nav_complete.launch.py

本 launch 文件:
  - 启动 VLP-16 驱动 + WHEELTEC 底盘 + IMU 滤波
  - 启动 RTAB-Map 定位模式 (使用 maps/latest/rtabmap.db)
  - 启动 Nav2 导航 (使用 maps/latest.yaml)
  - 启动 RViz2 (Nav2 面板 + 2D Pose Estimate / Nav2 Goal)
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    pkg_dir = get_package_share_directory('velodyne_bringup')

    # ========== 默认地图 ==========
    # 固定使用 maps/latest 导出的 Nav2 静态地图
    default_map = os.path.join(
        os.path.expanduser('~'),
        'arm_car', 'lidar', 'maps',
        'latest.yaml')

    # ========== DeclareLaunchArgument(s) ==========
    # 注意: 直接在顶层用 LaunchConfiguration 变量,
    #       不在这里声明, 以免与子 launch 的 DeclareLaunchArgument 冲突;
    #       我们通过 IncludeLaunchDescription 的 launch_arguments 传递.

    map_file = LaunchConfiguration('map', default=default_map)
    params_file = LaunchConfiguration(
        'params_file',
        default=os.path.join(pkg_dir, 'config', 'nav2_params.yaml'))
    autostart = LaunchConfiguration('autostart', default='true')
    use_rviz = LaunchConfiguration('use_rviz', default='true')

    # ========== (1) RTAB-Map 定位模式 ==========
    # 包含: velodyne_driver + transform + TF + chassis + IMU + rtabmap_localization
    rtabmap_localization = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_dir, 'launch', 'rtabmap_localization.launch.py')),
    )

    # ========== (2) Nav2 导航 ==========
    nav2_navigation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_dir, 'launch', 'nav2_navigation.launch.py')),
        launch_arguments={
            'map': map_file,
            'params_file': params_file,
            'autostart': autostart,
            'use_rviz': use_rviz,
        }.items(),
    )

    return LaunchDescription([
        rtabmap_localization,
        nav2_navigation,
    ])
