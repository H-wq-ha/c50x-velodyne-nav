"""One-click launch: VLP-16 driver + RTAB-Map 3D LiDAR SLAM (本地 lidar3d)."""

import os

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pkg_velodyne_driver = FindPackageShare('velodyne_driver').find(
        'velodyne_driver')
    pkg_velodyne_pointcloud = FindPackageShare('velodyne_pointcloud').find(
        'velodyne_pointcloud')
    pkg_dir = FindPackageShare('velodyne_bringup').find('velodyne_bringup')

    # ========== VLP-16 雷达驱动 ==========

    velodyne_driver = Node(
        package='velodyne_driver',
        executable='velodyne_driver_node',
        name='velodyne_driver_node',
        output='both',
        parameters=[os.path.join(pkg_velodyne_driver, 'config',
                                 'VLP16-velodyne_driver_node-params.yaml'),
                    {'timestamp_first_packet': True}],
    )

    velodyne_transform = Node(
        package='velodyne_pointcloud',
        executable='velodyne_transform_node',
        name='velodyne_transform_node',
        output='both',
        parameters=[os.path.join(pkg_velodyne_pointcloud, 'config',
                                 'VLP16-velodyne_transform_node-params.yaml'),
                    {'calibration': os.path.join(
                        pkg_velodyne_pointcloud, 'params', 'VLP16db.yaml'),
                     'min_range': 0.5}],
    )

    # ========== 静态 TF：base_link → velodyne ==========

    static_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_tf_base_to_velodyne',
        arguments=['0', '0', '0.3', '0', '0.0873', '0', 'base_link', 'velodyne'],
    )

    # ========== WHEELTEC 底盘驱动 ==========

    chassis_node = Node(
        package='car_bringup',
        executable='chassis_node',
        name='chassis_node',
        output='screen',
        parameters=[{'port': '/dev/wheeltec_chassis', 'baudrate': 460800}],
    )

    # ========== IMU filter: /imu_raw -> /imu/data ==========

    imu_filter = Node(
        package='imu_filter_madgwick',
        executable='imu_filter_madgwick_node',
        name='imu_filter_madgwick',
        output='screen',
        parameters=[{
            'use_mag': False,
            'publish_tf': False,
            'world_frame': 'enu',
            # 严格滤波参数 — 降低 gain 使姿态平滑，适合雷达去畸变
            'gain': 0.01,           # 默认 0.1 → 0.01, 10x 平滑, 代价 ~0.1s 滞后
            'zeta': 0.001,          # 陀螺零偏跟踪, 减少积分漂移
            'remove_gravity_vector': False,  # 保留重力在加速度中（默认）
        }],
        remappings=[
            ('imu/data_raw', '/imu_raw'),
            ('imu/data', '/imu/data'),
        ],
    )

    # ========== 静态 TF：base_link → imu_link ==========
    # IMU 位于底盘中心，与 base_link 无偏移

    static_tf_imu = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_tf_base_to_imu',
        arguments=['0', '0', '0', '0', '0', '0', 'base_link', 'imu_link'],
    )

    # ========== RTAB-Map 3D LiDAR SLAM（本地 lidar3d，集成 IMU）==========

    rtabmap_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_dir, 'launch', 'lidar3d.launch.py')),
        launch_arguments={
            'lidar_topic': '/velodyne_points',
            'frame_id': 'base_link',
            'deskewing': 'true',
            'expected_update_rate': '15.0',
            'imu_topic': '/imu/data',
            'qos': '2',
        }.items(),
    )

    return LaunchDescription([
        velodyne_driver,
        velodyne_transform,
        static_tf,
        static_tf_imu,
        chassis_node,
        imu_filter,
        rtabmap_launch,
    ])
