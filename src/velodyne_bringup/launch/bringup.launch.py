import os

from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pkg_dir = FindPackageShare('velodyne_bringup').find('velodyne_bringup')
    pkg_velodyne_driver = FindPackageShare('velodyne_driver').find(
        'velodyne_driver')
    pkg_velodyne_pointcloud = FindPackageShare('velodyne_pointcloud').find(
        'velodyne_pointcloud')
    calibration_path = os.path.join(
        pkg_velodyne_pointcloud, 'params', 'VLP16db.yaml')

    return LaunchDescription([

        Node(
            package='velodyne_driver',
            executable='velodyne_driver_node',
            name='velodyne_driver_node',
            output='screen',
            parameters=[os.path.join(pkg_velodyne_driver, 'config',
                                     'VLP16-velodyne_driver_node-params.yaml'),
                        os.path.join(pkg_dir, 'config',
                                     'velodyne_hardware_time.yaml')]
        ),

        Node(
            package='velodyne_pointcloud',
            executable='velodyne_transform_node',
            name='velodyne_transform_node',
            output='screen',
            parameters=[os.path.join(pkg_velodyne_pointcloud, 'config',
                                     'VLP16-velodyne_transform_node-params.yaml'),
                        {'calibration': calibration_path}]
        ),

        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            arguments=['--x', '0', '--y', '0', '--z', '1.45',
                       '--roll', '0', '--pitch', '0', '--yaw', '0',
                       '--frame-id', 'base_link', '--child-frame-id', 'velodyne']
        )
    ])
