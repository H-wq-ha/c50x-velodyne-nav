"""Start a fresh VLP-16, D435i, chassis-IMU and three-class semantic SLAM run."""

import json
import os
import re
import sys
import time
import uuid

import serial
import yaml
from car_bringup.protocol import (
    FRAME_HEADER, STATUS_FRAME_SIZE, find_wheeltec_port, parse_status_frame,
)
from launch import LaunchDescription
from launch.actions import (
    AppendEnvironmentVariable,
    DeclareLaunchArgument,
    EmitEvent,
    IncludeLaunchDescription,
    LogInfo,
    OpaqueFunction,
    RegisterEventHandler,
    SetLaunchConfiguration,
)
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare
from semantic_mapping.database_lifecycle import (
    DatabaseLifecycleError,
    prepare_databases,
)
from semantic_mapping.camera_config import (
    camera_serial_yaml_literal, normalize_camera_serial,
)


def _strict_bool(context, name):
    value = LaunchConfiguration(name).perform(context).strip().lower()
    if value not in ('true', 'false'):
        raise RuntimeError(f'{name} must be true or false')
    return value == 'true'


def _probe_chassis_imu(port, baudrate, seconds=0.8):
    """Read and validate status frames without sending any serial data."""
    try:
        device = serial.Serial(
            port=port, baudrate=baudrate, timeout=0.05, exclusive=True)
    except Exception as exc:
        raise RuntimeError(
            f'cannot exclusively open the chassis IMU port {port}: {exc}') from exc
    buffer = bytearray()
    valid_frames = 0
    deadline = time.monotonic() + seconds
    try:
        while time.monotonic() < deadline and valid_frames < 3:
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
                    valid_frames += 1
    finally:
        device.close()
    if valid_frames < 3:
        raise RuntimeError(
            f'chassis IMU preflight decoded only {valid_frames} valid status frames from {port}')
    return valid_frames


def _preflight_and_prepare(context, *args, **kwargs):
    """Finish hardware/model checks and reserve fresh, non-overwriting DB paths."""
    share = FindPackageShare('semantic_mapping').find(
        'semantic_mapping')
    params_file = os.path.abspath(os.path.expanduser(
        LaunchConfiguration('params_file').perform(context).strip()))
    if not os.path.isfile(params_file):
        raise RuntimeError(f'semantic params file does not exist: {params_file}')
    with open(params_file, 'r', encoding='utf-8') as stream:
        document = yaml.safe_load(stream) or {}
    semantic_params = document.get('semantic_mapping', {}).get(
        'ros__parameters', {})
    configured_classes = semantic_params.get('target_class_ids', [])
    if set(configured_classes) != {39, 41, 63}:
        raise RuntimeError(
            'one-click launch requires target_class_ids [63, 39, 41]')

    python_site = os.path.abspath(os.path.expanduser(
        LaunchConfiguration('python_site_packages').perform(context).strip()))
    if not os.path.isdir(python_site):
        raise RuntimeError(f'AI Python packages directory does not exist: {python_site}')
    if python_site not in sys.path:
        sys.path.insert(0, python_site)
    model_arg = LaunchConfiguration('model_path').perform(context).strip()
    model_path = os.path.abspath(os.path.expanduser(
        model_arg or semantic_params.get('model_path', '')))
    if not os.path.isfile(model_path):
        raise RuntimeError(f'YOLO ONNX model does not exist: {model_path}')
    try:
        import onnxruntime as ort
        session_options = ort.SessionOptions()
        session_options.intra_op_num_threads = 2
        session_options.inter_op_num_threads = 1
        session = ort.InferenceSession(
            model_path, sess_options=session_options,
            providers=['CPUExecutionProvider'])
        input_shape = session.get_inputs()[0].shape
        output_shape = session.get_outputs()[0].shape
        if len(input_shape) != 4 or input_shape[1] != 3 or 84 not in output_shape[1:]:
            raise RuntimeError(
                f'ONNX model has incompatible YOLO input/output: {input_shape}, {output_shape}')
        del session
    except Exception as exc:
        raise RuntimeError(f'YOLO model preflight failed for {model_path}: {exc}') from exc

    try:
        serial_no = normalize_camera_serial(
            LaunchConfiguration('serial_no').perform(context))
        serial_literal = camera_serial_yaml_literal(serial_no)
    except ValueError as exc:
        raise RuntimeError(str(exc)) from exc
    try:
        import pyrealsense2 as rs
        devices = rs.context().query_devices()
        connected_serials = [
            device.get_info(rs.camera_info.serial_number) for device in devices]
    except Exception as exc:
        raise RuntimeError(f'RealSense device enumeration failed: {exc}') from exc
    if serial_no not in connected_serials:
        raise RuntimeError(
            f'RealSense serial {serial_no} is not connected; detected {connected_serials}')

    use_chassis_imu = _strict_bool(context, 'use_chassis_imu')
    imu_port = LaunchConfiguration('imu_port').perform(context).strip()
    imu_baudrate = int(LaunchConfiguration('imu_baudrate').perform(context))
    valid_imu_frames = 0
    if use_chassis_imu:
        if imu_port.lower() == 'auto':
            imu_port = find_wheeltec_port() or ''
        if not imu_port or not os.path.exists(imu_port):
            raise RuntimeError(
                f'chassis IMU port is not available: {imu_port!r}; set imu_port:=auto or a valid device path')
        imu_port = os.path.realpath(imu_port)
        valid_imu_frames = _probe_chassis_imu(imu_port, imu_baudrate)

    required_packages = [
        'car_bringup', 'velodyne_bringup', 'velodyne_driver',
        'velodyne_pointcloud', 'realsense2_camera', 'tf2_ros',
        'rtabmap_odom', 'rtabmap_slam', 'rtabmap_util',
    ]
    if use_chassis_imu:
        required_packages.append('imu_filter_madgwick')
    if _strict_bool(context, 'start_rviz'):
        required_packages.append('rviz2')
    for package_name in required_packages:
        FindPackageShare(package_name).find(package_name)

    overwrite = _strict_bool(context, 'overwrite_databases')
    run_root = os.path.abspath(os.path.expanduser(
        LaunchConfiguration('run_root').perform(context).strip()))
    os.makedirs(run_root, exist_ok=True)
    requested_id = LaunchConfiguration('map_id').perform(context).strip()
    if requested_id and (
            not re.fullmatch(r'[A-Za-z0-9_-][A-Za-z0-9_.-]*', requested_id)
            or requested_id in ('.', '..')):
        raise RuntimeError(
            'map_id must be a single identifier using letters, digits, underscore, dot, or dash')
    if requested_id:
        map_id = requested_id
        run_dir = os.path.join(run_root, map_id)
        if overwrite:
            os.makedirs(run_dir, exist_ok=True)
        else:
            try:
                os.mkdir(run_dir)
            except FileExistsError as exc:
                raise RuntimeError(
                    f'run directory already exists: {run_dir}; choose a new map_id or explicitly set overwrite_databases:=true') from exc
    else:
        stamp = time.strftime('%Y%m%d_%H%M%S', time.localtime())
        while True:
            map_id = f'semantic_{stamp}_{uuid.uuid4().hex[:8]}'
            run_dir = os.path.join(run_root, map_id)
            try:
                os.mkdir(run_dir)
                break
            except FileExistsError:
                continue

    rtabmap_db = os.path.join(run_dir, 'rtabmap.db')
    semantic_db = os.path.join(run_dir, 'semantic.sqlite3')
    try:
        prepare_databases(
            rtabmap_db, semantic_db, start_rtabmap=True,
            mode='mapping', overwrite_databases=overwrite)
    except DatabaseLifecycleError as exc:
        raise RuntimeError(str(exc)) from exc

    identity = {
        'map_id': map_id,
        'run_dir': run_dir,
        'rtabmap_db': rtabmap_db,
        'semantic_db': semantic_db,
        'created_unix': time.time(),
        'realsense_serial': serial_no,
        'target_class_ids': [63, 39, 41],
        'model_path': model_path,
        'chassis_imu_enabled': use_chassis_imu,
        'chassis_imu_port': imu_port if use_chassis_imu else None,
        'chassis_imu_baudrate': imu_baudrate if use_chassis_imu else None,
        'chassis_imu_preflight_valid_frames': valid_imu_frames,
        'chassis_control_enabled': False,
        'cmd_vel_subscription_enabled': False,
        'database_overwrite_enabled': overwrite,
        'lidar_deskewing_enabled': use_chassis_imu,
    }
    with open(os.path.join(run_dir, 'run_identity.json'), 'w', encoding='utf-8') as stream:
        json.dump(identity, stream, indent=2)
        stream.write('\n')

    return [
        SetLaunchConfiguration('map_id', map_id),
        SetLaunchConfiguration('run_dir', run_dir),
        SetLaunchConfiguration('rtabmap_database_path', rtabmap_db),
        SetLaunchConfiguration('semantic_db_path', semantic_db),
        SetLaunchConfiguration('model_path', model_path),
        SetLaunchConfiguration('serial_no', serial_no),
        SetLaunchConfiguration('camera_serial_literal', serial_literal),
        SetLaunchConfiguration('imu_port', imu_port),
        *([] if use_chassis_imu else [
            SetLaunchConfiguration('deskewing', 'false'),
            SetLaunchConfiguration('imu_topic', ''),
        ]),
        LogInfo(msg=(
            f'One-click semantic SLAM run={map_id}; RTAB-Map DB={rtabmap_db}; '
            f'semantic DB={semantic_db}; IMU frames preflighted={valid_imu_frames}; '
            f'camera serial={serial_no}; no chassis control subscription.'
        )),
    ]


def _stop_stack_on_process_exit(event, context):
    if context.is_shutdown:
        return []
    command = ' '.join(event.cmd or [])
    reason = (
        f'critical process {event.process_name!r} exited with code {event.returncode}; '
        'stopping the one-click sensor/SLAM stack')
    return [LogInfo(msg=f'{reason}; command={command}'), EmitEvent(event=Shutdown(reason=reason))]


def _share_launch(package, filename):
    package_share = FindPackageShare(package).find(package)
    return os.path.join(package_share, 'launch', filename)


def generate_launch_description():
    semantic_share = FindPackageShare('semantic_mapping').find(
        'semantic_mapping')
    velodyne_driver_share = FindPackageShare('velodyne_driver').find(
        'velodyne_driver')
    velodyne_pointcloud_share = FindPackageShare('velodyne_pointcloud').find(
        'velodyne_pointcloud')
    velodyne_bringup_share = FindPackageShare('velodyne_bringup').find(
        'velodyne_bringup')
    realsense_share = FindPackageShare('realsense2_camera').find(
        'realsense2_camera')
    rviz_share = os.path.join(semantic_share, 'config', 'semantic_mapping.rviz')

    params_default = os.path.join(semantic_share, 'config', 'params.yaml')
    model_default = '/home/y/arm_car/lidar/yolov8_d435i/models/yolov8n_256.onnx'
    python_site_default = (
        '/home/y/arm_car/lidar/yolov8_d435i/.venv/'
        'lib/python3.10/site-packages')
    run_root_default = '~/.ros/semantic_mapping/runs'

    camera = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            realsense_share, 'launch', 'rs_launch.py')),
        launch_arguments={
            'camera_namespace': 'camera',
            'camera_name': 'camera',
            'serial_no': LaunchConfiguration('camera_serial_literal'),
            'enable_color': 'true',
            'enable_depth': 'true',
            'enable_sync': 'true',
            'initial_reset': 'false',
            'enable_motion': 'false',
            'enable_gyro': 'false',
            'enable_accel': 'false',
            'enable_infra': 'false',
            'enable_infra1': 'false',
            'enable_infra2': 'false',
            'align_depth.enable': 'true',
            'rgb_camera.color_profile': LaunchConfiguration('color_profile'),
            'depth_module.depth_profile': LaunchConfiguration('depth_profile'),
            'output': 'screen',
        }.items(),
    )

    chassis_imu = Node(
        package='car_bringup', executable='chassis_node',
        name='chassis_imu_readonly', output='screen',
        parameters=[{
            'port': LaunchConfiguration('imu_port'),
            'baudrate': ParameterValue(
                LaunchConfiguration('imu_baudrate'), value_type=int),
            'enable_control': False,
        }],
        condition=IfCondition(LaunchConfiguration('use_chassis_imu')),
    )
    imu_filter = Node(
        package='imu_filter_madgwick', executable='imu_filter_madgwick_node',
        name='imu_filter_madgwick', output='screen',
        parameters=[{
            'use_mag': False,
            'publish_tf': False,
            'world_frame': 'enu',
            'gain': 0.01,
            'zeta': 0.001,
            'remove_gravity_vector': False,
        }],
        remappings=[('imu/data_raw', '/imu_raw'), ('imu/data', '/imu/data')],
        condition=IfCondition(LaunchConfiguration('use_chassis_imu')),
    )

    base_to_imu = Node(
        package='tf2_ros', executable='static_transform_publisher',
        name='static_tf_base_to_imu', output='screen',
        arguments=[
            '--x', '0', '--y', '0', '--z', '0',
            '--roll', '0', '--pitch', '0', '--yaw', '0',
            '--frame-id', 'base_link', '--child-frame-id', 'imu_link',
        ],
    )
    imu_to_lidar = Node(
        package='tf2_ros', executable='static_transform_publisher',
        name='static_tf_imu_to_velodyne', output='screen',
        arguments=[
            '--x', '-0.0718', '--y', '0.006', '--z', '1.0345',
            '--roll', '0', '--pitch', '0', '--yaw', '3.141592653589793',
            '--frame-id', 'imu_link', '--child-frame-id', 'velodyne',
        ],
    )
    lidar_to_camera = Node(
        package='tf2_ros', executable='static_transform_publisher',
        name='static_tf_velodyne_to_camera_link', output='screen',
        arguments=[
            '--x', '0.096769992', '--y', '0.017500000', '--z', '-0.064108820',
            '--qx', '0', '--qy', '0.4233835445', '--qz', '0', '--qw', '0.9059505363',
            '--frame-id', 'velodyne', '--child-frame-id', 'camera_link',
        ],
    )

    velodyne_driver = Node(
        package='velodyne_driver', executable='velodyne_driver_node',
        name='velodyne_driver_node', output='both',
        parameters=[
            os.path.join(velodyne_driver_share, 'config',
                         'VLP16-velodyne_driver_node-params.yaml'),
            os.path.join(velodyne_bringup_share, 'config',
                         'velodyne_hardware_time.yaml'),
            {'timestamp_first_packet': True},
        ],
    )
    velodyne_transform = Node(
        package='velodyne_pointcloud', executable='velodyne_transform_node',
        name='velodyne_transform_node', output='both',
        parameters=[
            os.path.join(velodyne_pointcloud_share, 'config',
                         'VLP16-velodyne_transform_node-params.yaml'),
            {'calibration': os.path.join(velodyne_pointcloud_share, 'params',
                                         'VLP16db.yaml'),
             'min_range': 0.5},
        ],
    )

    rtabmap = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            velodyne_bringup_share, 'launch', 'lidar3d.launch.py')),
        launch_arguments={
            'localization': 'false',
            'database_path': LaunchConfiguration('rtabmap_database_path'),
            'delete_db': 'false',
            'start_rtabmap_viz': 'false',
            'lidar_topic': '/velodyne_points',
            'frame_id': 'base_link',
            'deskewing': LaunchConfiguration('deskewing'),
            'fixed_frame_id': '',
            'expected_update_rate': '15.0',
            'imu_topic': LaunchConfiguration('imu_topic'),
            'qos': '2',
            'use_sim_time': 'false',
        }.items(),
    )

    semantic_node = Node(
        package='semantic_mapping', executable='semantic_mapping',
        name='semantic_mapping', output='screen',
        parameters=[
            LaunchConfiguration('params_file'),
            {
                'map_id': LaunchConfiguration('map_id'),
                'db_path': LaunchConfiguration('semantic_db_path'),
                'model_path': LaunchConfiguration('model_path'),
                'target_class_ids': [63, 39, 41],
            },
        ],
    )
    rviz = Node(
        package='rviz2', executable='rviz2', name='semantic_mapping_rviz',
        arguments=['-d', rviz_share], output='screen',
        condition=IfCondition(LaunchConfiguration('start_rviz')),
    )
    health_monitor = Node(
        package='semantic_mapping',
        executable='one_click_health_monitor',
        name='one_click_mapping_health', output='screen',
        parameters=[{
            'run_dir': LaunchConfiguration('run_dir'),
            'use_chassis_imu': ParameterValue(
                LaunchConfiguration('use_chassis_imu'), value_type=bool),
            'startup_timeout_s': 90.0,
            'stream_timeout_s': 10.0,
        }],
    )

    return LaunchDescription([
        DeclareLaunchArgument('start_rviz', default_value='false'),
        DeclareLaunchArgument('use_chassis_imu', default_value='true'),
        DeclareLaunchArgument('imu_port', default_value=(
            '/dev/serial/by-id/usb-WCH.CN_USB_Single_Serial_0002-if00')),
        DeclareLaunchArgument('imu_baudrate', default_value='460800'),
        DeclareLaunchArgument('serial_no', default_value='327122073765'),
        DeclareLaunchArgument('color_profile', default_value='640,480,30'),
        DeclareLaunchArgument('depth_profile', default_value='640,480,30'),
        DeclareLaunchArgument('deskewing', default_value='true'),
        DeclareLaunchArgument('imu_topic', default_value='/imu/data'),
        DeclareLaunchArgument('map_id', default_value=''),
        DeclareLaunchArgument('run_root', default_value=run_root_default),
        DeclareLaunchArgument('overwrite_databases', default_value='false'),
        DeclareLaunchArgument('params_file', default_value=params_default),
        DeclareLaunchArgument('model_path', default_value=''),
        DeclareLaunchArgument('python_site_packages', default_value=python_site_default),
        OpaqueFunction(function=_preflight_and_prepare),
        AppendEnvironmentVariable(name='PYTHONPATH', value=LaunchConfiguration(
            'python_site_packages')),
        RegisterEventHandler(OnProcessExit(on_exit=_stop_stack_on_process_exit)),
        camera,
        chassis_imu,
        imu_filter,
        base_to_imu,
        imu_to_lidar,
        lidar_to_camera,
        velodyne_driver,
        velodyne_transform,
        rtabmap,
        semantic_node,
        health_monitor,
        rviz,
    ])
