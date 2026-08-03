"""Nav2 标准定位 + 导航一键启动（使用 AMCL 而非 RTAB-Map 定位）

架构:
  VLP-16 → velodyne_driver → velodyne_transform_node → /velodyne_points
    ├── velodyne_laserscan → /scan ──→ AMCL ──→ map→odom TF
    └── lidar_deskewing → deskewed ──→ icp_odometry ──→ /odom + odom→base_link TF
  chassis_node → /imu_raw → imu_filter_madgwick → /imu/data (去畸变用)
  Nav2: map_server + planner + controller + BT + velocity_smoother

用法:
  ros2 launch velodyne_bringup nav2_bringup.launch.py
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
    pkg_velodyne_driver = get_package_share_directory('velodyne_driver')
    pkg_velodyne_pointcloud = get_package_share_directory('velodyne_pointcloud')

    # ========== 参数默认值 ==========
    default_map = os.path.join(
        os.path.expanduser('~'),
        'arm_car', 'lidar', 'maps',
        'rtabmap_nav2_latest.yaml')
    default_params = os.path.join(pkg_dir, 'config', 'nav2_params.yaml')
    default_rviz_config = os.path.join(pkg_dir, 'config', 'nav2_wheeltec.rviz')

    # ========== 声明启动参数 ==========
    declared_args = [
        DeclareLaunchArgument('map', default_value=default_map,
                              description='Nav2 静态地图 YAML 路径'),
        DeclareLaunchArgument('params_file', default_value=default_params,
                              description='Nav2 参数文件路径'),
        DeclareLaunchArgument('rviz_config', default_value=default_rviz_config,
                              description='RViz2 配置文件路径'),
        DeclareLaunchArgument('use_rviz', default_value='true',
                              description='启动 RViz2'),
        DeclareLaunchArgument('autostart', default_value='true',
                              description='自动激活 Nav2 生命周期节点'),
    ]

    default_amcl_params = os.path.join(pkg_dir, 'config', 'amcl_params.yaml')

    # ========== 运行时配置 ==========
    use_rviz = LaunchConfiguration('use_rviz')
    autostart = LaunchConfiguration('autostart')
    map_file = LaunchConfiguration('map')
    params_file = LaunchConfiguration('params_file')
    rviz_config = LaunchConfiguration('rviz_config')

    # ========== 1. VLP-16 雷达驱动 ==========

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

    # ========== 2. 静态 TF ==========

    static_tf_velodyne = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_tf_base_to_velodyne',
        arguments=['0', '0', '0.3', '0', '0.0873', '0', 'base_link', 'velodyne'],
    )

    static_tf_imu = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_tf_base_to_imu',
        arguments=['0', '0', '0', '0', '0', '0', 'base_link', 'imu_link'],
    )

    # ========== 3. WHEELTEC 底盘 + IMU 滤波 ==========

    chassis_node = Node(
        package='car_bringup',
        executable='chassis_node',
        name='chassis_node',
        output='screen',
        parameters=[{'port': '/dev/wheeltec_chassis', 'baudrate': 460800}],
    )

    imu_filter = Node(
        package='imu_filter_madgwick',
        executable='imu_filter_madgwick_node',
        name='imu_filter_madgwick',
        output='screen',
        parameters=[{
            'use_mag': False,
            'publish_tf': False,
            'world_frame': 'enu',
            'gain': 0.01,
            'zeta': 0.001,
            'remove_gravity_vector': False,
        }],
        remappings=[
            ('imu/data_raw', '/imu_raw'),
            ('imu/data', '/imu/data'),
        ],
    )

    # ========== 4. 3D → 2D 激光投影 (for AMCL) ==========

    velodyne_laserscan = Node(
        package='velodyne_laserscan',
        executable='velodyne_laserscan_node',
        name='velodyne_laserscan',
        output='screen',
        parameters=[{'ring': -1, 'resolution': 0.007}],
    )

    # ========== 5. ICP 里程计 (odom → base_link) ==========

    # 引入 lidar_deskewing 为 ICP 提供去畸变点云
    deskewing = Node(
        package='rtabmap_util',
        executable='lidar_deskewing',
        name='lidar_deskewing',
        output='screen',
        parameters=[{
            'use_sim_time': False,
            'fixed_frame_id': 'base_link_stabilized',
            'wait_for_transform': 0.2,
            'slerp': True,
        }],
        remappings=[('input_cloud', '/velodyne_points')],
    )

    # IMU → 稳定帧 TF (为去畸变提供旋转先验)
    imu_to_tf = Node(
        package='rtabmap_util',
        executable='imu_to_tf',
        name='imu_to_tf',
        output='screen',
        parameters=[{
            'use_sim_time': False,
            'fixed_frame_id': 'base_link_stabilized',
            'base_frame_id': 'base_link',
            'wait_for_transform_duration': 0.2,
        }],
        remappings=[('imu/data', '/imu/data')],
    )

    icp_odometry = Node(
        package='rtabmap_odom',
        executable='icp_odometry',
        name='icp_odometry',
        output='screen',
        parameters=[{
            'frame_id': 'base_link',
            'odom_frame_id': 'odom',
            'publish_tf': True,
            'expected_update_rate': 15.0,
            'wait_imu_to_init': False,
            'deskewing': True,
            'deskewing_slerp': True,
            'guess_frame_id': 'base_link_stabilized',
            'Icp/PointToPlane': 'true',
            'Icp/Iterations': '10',
            'Icp/VoxelSize': '0.10',
            'Icp/Epsilon': '0.001',
            'Icp/PointToPlaneK': '20',
            'Icp/PointToPlaneGroundNormalsUp': '0.5',
            'Icp/MaxTranslation': '3',
            'Icp/MaxCorrespondenceDistance': '1.0',
            'Icp/Strategy': '1',
            'Icp/OutlierRatio': '0.7',
            'Icp/RangeMin': '0.5',
            'Icp/RangeMax': '40.0',
            'Icp/CorrespondenceRatio': '0.01',
            'Odom/ScanKeyFrameThr': '0.4',
            'OdomF2M/ScanSubtractRadius': '0.1',
            'OdomF2M/ScanMaxSize': '30000',
            'OdomF2M/BundleAdjustment': 'false',
        }],
        remappings=[('scan_cloud', '/velodyne_points/deskewed')],
    )

    # ========== 6. AMCL 定位 (map → odom) ==========

    amcl = Node(
        package='nav2_amcl',
        executable='amcl',
        name='amcl',
        output='screen',
        parameters=[default_amcl_params],
    )

    # ========== 7. Nav2 导航堆栈 ==========

    lifecycle_nodes = [
        'map_server',
        'amcl',
        'controller_server',
        'smoother_server',
        'planner_server',
        'behavior_server',
        'bt_navigator',
        'waypoint_follower',
        'velocity_smoother',
    ]

    map_server = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        output='screen',
        parameters=[params_file, {'yaml_filename': map_file}],
        # map_server 发布到 /map（被 AMCL 和 global_costmap static_layer 消费）
    )

    controller_server = Node(
        package='nav2_controller',
        executable='controller_server',
        name='controller_server',
        output='screen',
        parameters=[params_file],
        remappings=[('cmd_vel', 'cmd_vel_nav')],
    )

    smoother_server = Node(
        package='nav2_smoother',
        executable='smoother_server',
        name='smoother_server',
        output='screen',
        parameters=[params_file],
    )

    planner_server = Node(
        package='nav2_planner',
        executable='planner_server',
        name='planner_server',
        output='screen',
        parameters=[params_file],
    )

    behavior_server = Node(
        package='nav2_behaviors',
        executable='behavior_server',
        name='behavior_server',
        output='screen',
        parameters=[params_file],
        remappings=[('cmd_vel', 'cmd_vel_nav')],
    )

    bt_navigator = Node(
        package='nav2_bt_navigator',
        executable='bt_navigator',
        name='bt_navigator',
        output='screen',
        parameters=[params_file],
    )

    waypoint_follower = Node(
        package='nav2_waypoint_follower',
        executable='waypoint_follower',
        name='waypoint_follower',
        output='screen',
        parameters=[params_file],
    )

    velocity_smoother = Node(
        package='nav2_velocity_smoother',
        executable='velocity_smoother',
        name='velocity_smoother',
        output='screen',
        parameters=[params_file],
        remappings=[
            ('cmd_vel', 'cmd_vel_nav'),
            ('cmd_vel_smoothed', '/cmd_vel'),
        ],
    )

    lifecycle_manager = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_navigation',
        output='screen',
        parameters=[{
            'use_sim_time': False,
            'autostart': autostart,
            'node_names': lifecycle_nodes,
        }],
    )

    # ========== 8. RViz2 ==========

    rviz = Node(
        condition=IfCondition(use_rviz),
        package='rviz2',
        executable='rviz2',
        name='rviz2_nav2',
        output='screen',
        arguments=['-d', rviz_config],
    )

    return LaunchDescription([
        *declared_args,
        velodyne_driver,
        velodyne_transform,
        static_tf_velodyne,
        static_tf_imu,
        chassis_node,
        imu_filter,
        velodyne_laserscan,
        imu_to_tf,
        deskewing,
        icp_odometry,
        amcl,
        map_server,
        controller_server,
        smoother_server,
        planner_server,
        behavior_server,
        bt_navigator,
        waypoint_follower,
        velocity_smoother,
        lifecycle_manager,
        rviz,
    ])
