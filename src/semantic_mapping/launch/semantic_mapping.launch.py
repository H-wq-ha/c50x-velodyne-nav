"""Multi-class semantic mapping and optional, safe RTAB-Map integration."""

import os
import re

from launch import LaunchDescription
from launch.actions import (
    AppendEnvironmentVariable,
    DeclareLaunchArgument,
    EmitEvent,
    IncludeLaunchDescription,
    OpaqueFunction,
    RegisterEventHandler,
)
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from semantic_mapping.database_lifecycle import (
    DatabaseLifecycleError,
    prepare_databases,
)


def _resolved_semantic_db_path(context):
    map_id = LaunchConfiguration('map_id').perform(context).strip()
    requested_path = LaunchConfiguration('semantic_db_path').perform(context).strip()
    if requested_path:
        db_path = os.path.abspath(os.path.expanduser(requested_path))
    else:
        db_path = os.path.join(os.path.expanduser('~'), '.ros',
                               'semantic_mapping', 'runs', map_id,
                               'semantic.sqlite3')
    return map_id, db_path


def _preflight(context, *args, **kwargs):
    """Validate identity and database paths before any process is launched."""
    map_id = LaunchConfiguration('map_id').perform(context).strip()
    if not re.fullmatch(r'[A-Za-z0-9_-][A-Za-z0-9_.-]*', map_id) or map_id in ('.', '..'):
        raise RuntimeError(
            'map_id must be a single identifier using letters, digits, underscore, dot, or dash')
    _, semantic_db_path = _resolved_semantic_db_path(context)
    params_file = os.path.expanduser(
        LaunchConfiguration('params_file').perform(context).strip())
    if not params_file or not os.path.isfile(params_file):
        raise RuntimeError(f'params_file does not exist: {params_file}')

    required_packages = []
    if LaunchConfiguration('start_camera').perform(context).lower() == 'true':
        required_packages.append('realsense2_camera')
    if LaunchConfiguration('start_rviz').perform(context).lower() == 'true':
        required_packages.append('rviz2')
    for package_name in required_packages:
        FindPackageShare(package_name).find(package_name)

    if LaunchConfiguration('start_rtabmap').perform(context).lower() != 'true':
        return []
    mode = LaunchConfiguration('rtabmap_mode').perform(context).lower()
    requested_rtabmap_path = LaunchConfiguration(
        'rtabmap_database_path').perform(context).strip()
    if not requested_rtabmap_path:
        raise RuntimeError('Set rtabmap_database_path when start_rtabmap:=true')
    rtabmap_db_path = os.path.abspath(os.path.expanduser(requested_rtabmap_path))
    if mode not in ('mapping', 'localization'):
        raise RuntimeError("rtabmap_mode must be 'mapping' or 'localization'")
    for package_name in (
        'velodyne_bringup', 'velodyne_driver', 'velodyne_pointcloud',
        'tf2_ros', 'car_bringup', 'imu_filter_madgwick',
        'rtabmap_odom', 'rtabmap_slam', 'rtabmap_util',
    ):
        FindPackageShare(package_name).find(package_name)
    try:
        prepare_databases(
            rtabmap_db_path,
            semantic_db_path,
            start_rtabmap=True,
            mode=mode,
            overwrite_databases=(
                LaunchConfiguration('overwrite_databases').perform(context).lower() == 'true'),
        )
    except DatabaseLifecycleError as exc:
        raise RuntimeError(str(exc)) from exc
    return []


def _optional_rtabmap(context, *args, **kwargs):
    if LaunchConfiguration('start_rtabmap').perform(context).lower() != 'true':
        return []

    mode = LaunchConfiguration('rtabmap_mode').perform(context).lower()
    db_path = os.path.abspath(os.path.expanduser(
        LaunchConfiguration('rtabmap_database_path').perform(context).strip()))

    bringup_share = FindPackageShare('velodyne_bringup').find('velodyne_bringup')
    return [IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(bringup_share, 'launch', 'rtabmap_slam.launch.py')),
        launch_arguments={
            'localization': 'true' if mode == 'localization' else 'false',
            'delete_db': 'false',
            'start_rtabmap_viz': 'false',
            'database_path': db_path,
            'lidar_topic': '/velodyne_points',
            'frame_id': 'base_link',
            'deskewing': 'true',
            'expected_update_rate': '15.0',
            'imu_topic': '/imu/data',
            'qos': '2',
        }.items(),
    )]


def _semantic_node(context, *args, **kwargs):
    params_path = LaunchConfiguration('params_file').perform(context)
    map_id, db_path = _resolved_semantic_db_path(context)
    os.makedirs(os.path.dirname(db_path), exist_ok=True)
    semantic_node = Node(
        package='semantic_mapping', executable='semantic_mapping',
        name='semantic_mapping', output='screen',
        parameters=[params_path, {'map_id': map_id, 'db_path': db_path}],
    )

    def shutdown_launch_on_unexpected_exit(event, launch_context):
        if launch_context.is_shutdown:
            return []
        return [EmitEvent(event=Shutdown(
            reason=(
                'semantic_mapping exited unexpectedly '
                f'with return code {event.returncode}; stopping the full launch'
            ),
        ))]

    exit_handler = RegisterEventHandler(OnProcessExit(
        target_action=semantic_node,
        on_exit=shutdown_launch_on_unexpected_exit,
    ))
    return [exit_handler, semantic_node]


def generate_launch_description():
    package_share = FindPackageShare('semantic_mapping').find(
        'semantic_mapping')
    realsense_share = FindPackageShare('realsense2_camera').find(
        'realsense2_camera')
    params_path = os.path.join(package_share, 'config', 'params.yaml')
    rviz_path = os.path.join(package_share, 'config', 'semantic_mapping.rviz')
    ort_site = (
        '/home/y/arm_car/lidar/yolov8_d435i/.venv/'
        'lib/python3.10/site-packages'
    )

    start_camera = LaunchConfiguration('start_camera')
    start_rviz = LaunchConfiguration('start_rviz')
    camera = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(realsense_share, 'launch', 'rs_launch.py')),
        condition=IfCondition(start_camera),
        launch_arguments={
            'camera_name': 'camera',
            'camera_namespace': 'camera',
            'serial_no': LaunchConfiguration('serial_no'),
            'enable_color': 'true',
            'enable_depth': 'true',
            'enable_sync': 'true',
            'align_depth.enable': 'true',
            'rgb_camera.color_profile': LaunchConfiguration('color_profile'),
            'depth_module.depth_profile': LaunchConfiguration('depth_profile'),
        }.items(),
    )
    mount_tf = Node(
        package='tf2_ros', executable='static_transform_publisher',
        name='static_tf_velodyne_to_camera_link', output='screen',
        arguments=[
            '--x', '0.096769992', '--y', '0.017500000', '--z', '-0.064108820',
            '--qx', '0', '--qy', '0.4233835445', '--qz', '0', '--qw', '0.9059505363',
            '--frame-id', 'velodyne', '--child-frame-id', 'camera_link',
        ],
        condition=IfCondition(LaunchConfiguration('publish_camera_mount_tf')),
    )

    rviz = Node(
        package='rviz2', executable='rviz2', name='semantic_mapping_rviz',
        arguments=['-d', rviz_path], output='screen',
        condition=IfCondition(start_rviz),
    )

    return LaunchDescription([
        DeclareLaunchArgument('start_camera', default_value='true'),
        DeclareLaunchArgument('start_rviz', default_value='true'),
        DeclareLaunchArgument('start_rtabmap', default_value='false'),
        DeclareLaunchArgument(
            'overwrite_databases', default_value='true',
            description='Reset the specified mapping databases before a new mapping run.'),
        DeclareLaunchArgument('serial_no', default_value="'327122073765'"),
        DeclareLaunchArgument('color_profile', default_value='640,480,30'),
        DeclareLaunchArgument('depth_profile', default_value='640,480,30'),
        DeclareLaunchArgument('map_id', default_value=''),
        DeclareLaunchArgument('semantic_db_path', default_value=''),
        DeclareLaunchArgument('params_file', default_value=params_path),
        DeclareLaunchArgument('publish_camera_mount_tf', default_value='true'),
        DeclareLaunchArgument('rtabmap_mode', default_value='mapping'),
        DeclareLaunchArgument('rtabmap_database_path', default_value=''),
        OpaqueFunction(function=_preflight),
        AppendEnvironmentVariable(name='PYTHONPATH', value=ort_site),
        camera,
        mount_tf,
        OpaqueFunction(function=_semantic_node),
        rviz,
        OpaqueFunction(function=_optional_rtabmap),
    ])
