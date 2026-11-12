"""Safe one-click saved-map localization, Nav2 and semantic target navigation."""

import hashlib
import json
import os
import shutil
import sqlite3
import tempfile
import time

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, LogInfo, OpaqueFunction, SetEnvironmentVariable
from launch.actions import SetLaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import EnvironmentVariable, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare

from semantic_mapping.camera_config import camera_serial_yaml_literal, normalize_camera_serial
from semantic_mapping.nav2_config import configure_semantic_navigation_params
from car_bringup.protocol import FRAME_HEADER, STATUS_FRAME_SIZE, find_wheeltec_port, parse_status_frame
import serial


def _sha256(path):
    digest = hashlib.sha256()
    with open(path, 'rb') as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b''):
            digest.update(block)
    return digest.hexdigest()


def _probe_chassis_imu(port, baudrate):
    if port.lower() == 'auto':
        port = find_wheeltec_port() or ''
    port = os.path.realpath(port) if port else ''
    if not port or not os.path.exists(port):
        raise RuntimeError(f'chassis IMU port is unavailable: {port!r}')
    try:
        device = serial.Serial(port=port, baudrate=baudrate, timeout=0.05, exclusive=True)
    except Exception as exc:
        raise RuntimeError(f'cannot exclusively open chassis IMU port {port}: {exc}') from exc
    valid, buffer, deadline = 0, bytearray(), time.monotonic()+0.8
    try:
        while time.monotonic() < deadline and valid < 3:
            buffer.extend(device.read(512))
            while len(buffer) >= STATUS_FRAME_SIZE:
                index = buffer.find(bytes([FRAME_HEADER]))
                if index < 0:
                    buffer.clear()
                    break
                if index:
                    del buffer[:index]
                if len(buffer) < STATUS_FRAME_SIZE:
                    break
                frame = bytes(buffer[:STATUS_FRAME_SIZE])
                del buffer[:STATUS_FRAME_SIZE]
                if parse_status_frame(frame) is not None:
                    valid += 1
    finally:
        device.close()
    if valid < 3:
        raise RuntimeError(f'only {valid} valid IMU frames decoded from {port}')
    return port, valid


def _prepare(context, *args, **kwargs):
    control_arg = LaunchConfiguration('enable_chassis_control').perform(context).strip().lower()
    if control_arg not in ('true', 'false'):
        raise RuntimeError('enable_chassis_control must be true or false')
    control_enabled = control_arg == 'true'
    maps_root = os.path.abspath(os.path.expanduser(
        LaunchConfiguration('maps_root').perform(context)))
    bundle = os.path.join(maps_root, 'latest')
    required = ('map.yaml', 'map.pgm', 'rtabmap.db', 'semantic.sqlite3',
                'run_identity.json', 'map_snapshot.json')
    if not all(os.path.isfile(os.path.join(bundle, item)) for item in required):
        raise RuntimeError(f'incomplete saved semantic map bundle: {bundle}')
    with open(os.path.join(bundle, 'run_identity.json'), encoding='utf-8') as stream:
        identity = json.load(stream)
    with open(os.path.join(bundle, 'map_snapshot.json'), encoding='utf-8') as stream:
        snapshot = json.load(stream)
    if not identity.get('map_id') or identity['map_id'] != snapshot.get('map_id'):
        raise RuntimeError('map bundle identity and map snapshot disagree')
    if snapshot.get('frame_id') != 'map' or not snapshot.get('confirmed_semantic_objects'):
        raise RuntimeError('map snapshot has no confirmed semantic coordinates in map frame')
    with open(os.path.join(bundle, 'map.yaml'), encoding='utf-8') as stream:
        map_yaml = yaml.safe_load(stream)
    if map_yaml.get('image') != 'map.pgm' or map_yaml.get('resolution') is None:
        raise RuntimeError('saved occupancy map YAML is incomplete')
    if not os.path.isfile(os.path.join(bundle, map_yaml['image'])):
        raise RuntimeError('saved occupancy image is missing')

    serial = normalize_camera_serial(LaunchConfiguration('serial_no').perform(context))
    serial_literal = camera_serial_yaml_literal(serial)
    baudrate = int(LaunchConfiguration('imu_baudrate').perform(context))
    imu_port, valid_frames = _probe_chassis_imu(
        LaunchConfiguration('imu_port').perform(context), baudrate)

    run_root = os.path.abspath(os.path.expanduser(
        LaunchConfiguration('run_root').perform(context)))
    os.makedirs(run_root, exist_ok=True)
    run_dir = tempfile.mkdtemp(prefix='nav_' + identity['map_id'] + '_', dir=run_root)
    rtabmap_copy = os.path.join(run_dir, 'rtabmap.db')
    semantic_copy = os.path.join(run_dir, 'semantic.sqlite3')
    source_hash = _sha256(os.path.join(bundle, 'rtabmap.db'))
    try:
        with sqlite3.connect('file:' + os.path.join(bundle, 'rtabmap.db') + '?mode=ro', uri=True) as source:
            with sqlite3.connect(rtabmap_copy) as target:
                source.backup(target)
        # Preserve a validated read-only copy for semantic lookups.
        shutil.copy2(os.path.join(bundle, 'semantic.sqlite3'), semantic_copy)
        os.chmod(semantic_copy, 0o444)
        with sqlite3.connect('file:' + semantic_copy + '?mode=ro&immutable=1', uri=True) as db:
            meta = dict(db.execute('SELECT key,value FROM metadata'))
            count = db.execute('SELECT COUNT(*) FROM objects').fetchone()[0]
        if meta.get('map_id') != identity['map_id'] or not count:
            raise RuntimeError('semantic DB map identity or object table does not match the selected bundle')
        with open(os.path.join(run_dir, 'navrun_identity.json'), 'w', encoding='utf-8') as stream:
            json.dump({'map_id': identity['map_id'], 'source_bundle': bundle,
                       'source_rtabmap_sha256': source_hash, 'rtabmap_copy': rtabmap_copy,
                       'semantic_db': semantic_copy, 'semantic_db_mode': 'read-only',
                       'control_enabled': control_enabled, 'imu_preflight_valid_frames': valid_frames}, stream, indent=2)
            stream.write('\n')
        params = yaml.safe_load(open(os.path.join(
            FindPackageShare('velodyne_bringup').find('velodyne_bringup'),
            'config', 'nav2_params.yaml'), encoding='utf-8'))
        params['global_costmap']['global_costmap']['ros__parameters']['static_layer']['map_topic'] = '/static_map'
        planner_params = params['planner_server']['ros__parameters']['GridBased']
        planner_params['allow_unknown'] = False
        planner_params['tolerance'] = 0.05
        configure_semantic_navigation_params(params)
        nav_params = os.path.join(run_dir, 'nav2_params.yaml')
        with open(nav_params, 'w', encoding='utf-8') as stream:
            yaml.safe_dump(params, stream, sort_keys=False)
    except Exception:
        shutil.rmtree(run_dir, ignore_errors=True)
        raise
    return [
        LogInfo(msg=f'Semantic navigation map={identity["map_id"]} source={bundle}; source DB sha256={source_hash}; chassis control enabled={control_enabled}.'),
        SetEnvironmentVariable('ROS_DOMAIN_ID', LaunchConfiguration('ros_domain_id')),
        # Parameter substitutions set below are carried by launch configurations.
        SetLaunchConfiguration('selected_map_bundle', bundle),
        SetLaunchConfiguration('selected_map_id', identity['map_id']),
        SetLaunchConfiguration('selected_rtabmap_db', rtabmap_copy),
        SetLaunchConfiguration('selected_semantic_db', semantic_copy),
        SetLaunchConfiguration('selected_nav_params', nav_params),
        SetLaunchConfiguration('selected_run_dir', run_dir),
        SetLaunchConfiguration('camera_serial_literal', serial_literal),
        SetLaunchConfiguration('imu_port', imu_port),
    ]


def _setup(context, *args, **kwargs):
    bundle = LaunchConfiguration('selected_map_bundle').perform(context)
    rtabmap = LaunchConfiguration('selected_rtabmap_db').perform(context)
    semantic = LaunchConfiguration('selected_semantic_db').perform(context)
    nav_params = LaunchConfiguration('selected_nav_params').perform(context)
    map_id = LaunchConfiguration('selected_map_id').perform(context)
    serial_literal = LaunchConfiguration('camera_serial_literal')
    ld = get_package_share_directory('semantic_mapping')
    velodyne_driver_share = get_package_share_directory('velodyne_driver')
    velodyne_cloud_share = get_package_share_directory('velodyne_pointcloud')
    velodyne_share = get_package_share_directory('velodyne_bringup')
    rs_share = get_package_share_directory('realsense2_camera')
    rviz_cfg = os.path.join(ld, 'config', 'semantic_navigation.rviz')

    camera = IncludeLaunchDescription(PythonLaunchDescriptionSource(
        os.path.join(rs_share, 'launch', 'rs_launch.py')),
        launch_arguments={
            'camera_namespace': 'camera', 'camera_name': 'camera',
            'serial_no': serial_literal, 'enable_color': 'true', 'enable_depth': 'true',
            'enable_sync': 'true', 'initial_reset': 'false', 'enable_motion': 'false',
            'enable_gyro': 'false', 'enable_accel': 'false', 'enable_infra': 'false',
            'enable_infra1': 'false', 'enable_infra2': 'false', 'align_depth.enable': 'true',
            'rgb_camera.color_profile': '640,480,30', 'depth_module.depth_profile': '640,480,30',
            'output': 'screen'}.items())

    sensors = [
        Node(package='car_bringup', executable='chassis_node', name='chassis_control',
             output='screen', parameters=[{'port': LaunchConfiguration('imu_port'),
             'baudrate': ParameterValue(LaunchConfiguration('imu_baudrate'), value_type=int),
             'chassis_yaw_offset': ParameterValue(
                 LaunchConfiguration('chassis_yaw_offset'), value_type=float),
             'enable_control': ParameterValue(LaunchConfiguration('enable_chassis_control'), value_type=bool)}]),
        Node(package='imu_filter_madgwick', executable='imu_filter_madgwick_node',
             name='imu_filter_madgwick', output='screen', parameters=[{
             'use_mag': False, 'publish_tf': False, 'world_frame': 'enu', 'gain': 0.01,
             'zeta': 0.001, 'remove_gravity_vector': False}],
             remappings=[('imu/data_raw', '/imu_raw'), ('imu/data', '/imu/data')]),
        Node(package='tf2_ros', executable='static_transform_publisher', name='static_tf_base_imu',
             arguments=['--x','0','--y','0','--z','0','--roll','0','--pitch','0',
                        '--yaw',LaunchConfiguration('chassis_yaw_offset'),
                        '--frame-id','base_link','--child-frame-id','imu_link']),
        Node(package='tf2_ros', executable='static_transform_publisher', name='static_tf_imu_lidar',
             arguments=['--x','-0.0718','--y','0.006','--z','1.0345','--roll','0','--pitch','0',
                        '--yaw','3.141592653589793','--frame-id','imu_link','--child-frame-id','velodyne']),
        Node(package='tf2_ros', executable='static_transform_publisher', name='static_tf_lidar_camera',
             arguments=['--x','0.096769992','--y','0.017500000','--z','-0.064108820',
                        '--qx','0','--qy','0.4233835445','--qz','0','--qw','0.9059505363',
                        '--frame-id','velodyne','--child-frame-id','camera_link']),
        Node(package='velodyne_driver', executable='velodyne_driver_node', name='velodyne_driver_node',
             output='both', parameters=[os.path.join(velodyne_driver_share,'config','VLP16-velodyne_driver_node-params.yaml'),
             os.path.join(velodyne_share,'config','velodyne_hardware_time.yaml'), {'timestamp_first_packet':True}]),
        Node(package='velodyne_pointcloud', executable='velodyne_transform_node', name='velodyne_transform_node',
             output='both', parameters=[os.path.join(velodyne_cloud_share,'config','VLP16-velodyne_transform_node-params.yaml'),
             {'calibration':os.path.join(velodyne_cloud_share,'params','VLP16db.yaml'),'min_range':0.5}]),
    ]
    localization = IncludeLaunchDescription(PythonLaunchDescriptionSource(
        os.path.join(velodyne_share,'launch','lidar3d.launch.py')),
        launch_arguments={'localization':'true','database_path':rtabmap,'delete_db':'false',
                          'start_rtabmap_viz':'false','lidar_topic':'/velodyne_points',
                          'frame_id':'base_link','deskewing':'true','fixed_frame_id':'',
                          'expected_update_rate':'15.0','imu_topic':'/imu/data','qos':'2',
                          'use_sim_time':'false','rtabmap_map_topic':'/rtabmap_map'}.items())
    nav = IncludeLaunchDescription(PythonLaunchDescriptionSource(
        os.path.join(velodyne_share,'launch','nav2_navigation.launch.py')),
        launch_arguments={'map':os.path.join(bundle,'map.yaml'),'params_file':nav_params,
                          'autostart':'true','use_rviz':'true','rviz_config':rviz_cfg}.items())
    semantic = Node(package='semantic_mapping', executable='semantic_navigator',
        name='semantic_navigator', output='screen', parameters=[{
            'semantic_db':semantic, 'map_id':map_id, 'map_frame':'map','base_frame':'base_link',
            'camera_frame':'camera_color_optical_frame','static_map_topic':'/static_map',
            'standoff_min':LaunchConfiguration('standoff_min'),
            'standoff_max':LaunchConfiguration('standoff_max'),'camera_margin':LaunchConfiguration('camera_margin'),
            'enable_chassis_control':ParameterValue(LaunchConfiguration('enable_chassis_control'), value_type=bool),
            'navigation_timeout_s':ParameterValue(LaunchConfiguration('navigation_timeout_s'), value_type=float)}])
    return [camera, *sensors, localization, nav, semantic]


def generate_launch_description():
    defaults = os.path.join(os.path.expanduser('~'), 'arm_car', 'lidar', 'maps')
    return LaunchDescription([
        DeclareLaunchArgument('maps_root', default_value=defaults),
        DeclareLaunchArgument('run_root', default_value='~/.ros/semantic_navigation/runs'),
        DeclareLaunchArgument('serial_no', default_value='327122073765'),
        DeclareLaunchArgument('imu_port', default_value='/dev/serial/by-id/usb-WCH.CN_USB_Single_Serial_0002-if00'),
        DeclareLaunchArgument('imu_baudrate', default_value='460800'),
        DeclareLaunchArgument('chassis_yaw_offset', default_value='3.141592653589793',
                              description='Yaw from logical camera-facing base_link to chassis/IMU coordinates.'),
        DeclareLaunchArgument('ros_domain_id', default_value=EnvironmentVariable('ROS_DOMAIN_ID', default_value='0')),
        DeclareLaunchArgument('standoff_min', default_value='0.9'),
        DeclareLaunchArgument('standoff_max', default_value='2.5'),
        DeclareLaunchArgument('camera_margin', default_value='0.10'),
        DeclareLaunchArgument('enable_chassis_control', default_value='true',
                              description='Subscribe to /cmd_vel and execute navigation goals on the chassis.'),
        DeclareLaunchArgument('navigation_timeout_s', default_value='300.0'),
        OpaqueFunction(function=_prepare),
        OpaqueFunction(function=_setup),
    ])
