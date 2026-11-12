"""Capture hardware-timed calibration inputs for offline OA-LICalib."""
import json
import math
from datetime import datetime
from pathlib import Path
from zoneinfo import ZoneInfo

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, EmitEvent, ExecuteProcess,
                            LogInfo, OpaqueFunction, RegisterEventHandler, TimerAction)
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[1]


def capture(context):
    out = Path(LaunchConfiguration('output').perform(context)).expanduser().resolve()
    duration = float(LaunchConfiguration('duration').perform(context))
    if not math.isfinite(duration) or duration < 0:
        raise ValueError('duration 必须是非负秒数；0 表示手动停止')
    out.mkdir(parents=True, exist_ok=False)
    port = LaunchConfiguration('port').perform(context)
    start = LaunchConfiguration('start_lidar').perform(context)
    (out/'capture_config.json').write_text(json.dumps(dict(
        output=str(out), port=port, start_lidar=start, duration_s=duration,
        backend='OA-LICalib offline'), indent=2))
    qos = out/'record_qos.yaml'
    qos.write_text(''.join(
        f'{topic}:\n  history: keep_last\n  depth: 2000\n'
        f'  reliability: {reliability}\n  durability: volatile\n'
        for topic, reliability in [('/calibration/imu', 'reliable'),
            ('/calibration/velodyne_packets', 'reliable'),
            ('/calibration/velodyne_points', 'reliable')]))
    vendor = Path(get_package_share_directory('velodyne_pointcloud'))
    frontend = ExecuteProcess(cmd=['/usr/bin/python3',
        str(ROOT/'tool/calibration/hardware_frontend.py'), '--port', port,
        '--output', str(out/'frontend'), '--tolerant', '--quality-guidance',
        '--points-input', '/calibration/velodyne_points_unchecked',
        '--min-valid-seconds', LaunchConfiguration('min_valid_seconds'),
        '--min-clouds', LaunchConfiguration('min_clouds')], output='screen')
    driver = Node(package='velodyne_driver', executable='velodyne_driver_node',
        name='oa_calibration_velodyne_driver', output='screen',
        condition=IfCondition(LaunchConfiguration('start_lidar')),
        parameters=[dict(model='VLP16', device_ip='192.168.1.201', port=2368,
            rpm=600.0, frame_id='velodyne', gps_time=True,
            time_offset=0.0, timestamp_first_packet=True)])
    decoder = Node(package='velodyne_pointcloud', executable='velodyne_transform_node',
        name='oa_calibration_velodyne_transform', output='screen',
        parameters=[dict(calibration=str(vendor/'params/VLP16db.yaml'),
            model='VLP16', min_range=0.5, max_range=50.0,
            fixed_frame='', target_frame='', organize_cloud=False)],
        remappings=[('velodyne_packets', '/calibration/velodyne_packets'),
                    ('velodyne_points', '/calibration/velodyne_points_unchecked')])
    recorder = ExecuteProcess(cmd=['ros2', 'bag', 'record', '-o', str(out/'bag'),
        '--qos-profile-overrides-path', str(qos), '--max-cache-size', '200000000',
        '/calibration/imu', '/calibration/velodyne_points', '/calibration/velodyne_packets'],
        output='screen', sigterm_timeout='20', sigkill_timeout='10')
    actions = [LogInfo(msg=f'独立 OA-LICalib 采集目录：{out}'),
        LogInfo(msg='前 10 秒完全静止；检查通过后再静止 5 秒，然后整体做三轴往复转动和多方向平移。'),
        LogInfo(msg=f'停止后执行：{HERE / "calibrate.sh"} {out}'),
        LogInfo(msg='录包已自动启动。Ctrl+C 停止采集，等待所有进程退出后再运行离线标定。')]
    actions.append(LogInfo(msg='已启用坏帧过滤和采集建议。等待【可以结束采集】提示；严重时基故障或长缺口仍会停止。'))
    def stopped(event, context):
        if context.is_shutdown:
            return []
        (out/'capture_failure.json').write_text(json.dumps(dict(
            reason='采集进程提前退出', exit_code=event.returncode), indent=2))
        return [EmitEvent(event=Shutdown(reason='OA 采集进程已停止'))]

    for process in (frontend, driver, decoder, recorder):
        actions.append(RegisterEventHandler(OnProcessExit(target_action=process,
            on_exit=stopped)))
    actions.extend([recorder, driver, decoder, frontend])
    if duration:
        actions.append(TimerAction(period=duration, actions=[EmitEvent(
            event=Shutdown(reason='到达指定采集时长，保存录包'))]))
    return actions


def generate_launch_description():
    stamp = datetime.now(ZoneInfo('Asia/Shanghai')).strftime('%Y%m%d_%H%M%S_%f')
    return LaunchDescription([
        DeclareLaunchArgument('port', default_value='/dev/serial/by-id/usb-FIREPHX_USB_SER_FX2348N-if00'),
        DeclareLaunchArgument('start_lidar', default_value='true',
            description='true 启动雷达驱动；已有驱动时用 false'),
        DeclareLaunchArgument('duration', default_value='0',
            description='采集总秒数，包含最初静止时间；0 用 Ctrl+C 手动结束'),
        DeclareLaunchArgument('min_valid_seconds', default_value='90',
            description='采集建议所需的最少有效秒数（仍需三轴运动和输入质量达标）'),
        DeclareLaunchArgument('min_clouds', default_value='600',
            description='采集建议所需的最少有效点云帧数'),
        DeclareLaunchArgument('output', default_value=str(ROOT/'tool/runs'/f'oa_capture_{stamp}'),
            description='本轮独立采集目录，必须尚不存在'),
        OpaqueFunction(function=capture)])
