#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Start RealSense D435i + YOLOv8n ONNX + depth 3D detection."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    pkg_share = get_package_share_directory('piper_vision')
    realsense_share = get_package_share_directory('realsense2_camera')
    default_model = os.path.join(pkg_share, 'models', 'yolov8n.onnx')

    workspace_root = os.path.abspath(
        os.path.join(pkg_share, '..', '..', '..', '..'))
    default_config_dir = os.environ.get(
        'PIPER_VISION_CONFIG_DIR',
        os.path.join(workspace_root, 'src', 'config'))
    if not os.path.isdir(default_config_dir):
        default_config_dir = os.path.join(
            os.getcwd(), 'src', 'config')

    default_intrinsics = os.path.join(
        default_config_dir, 'calibration.yaml')
    default_extrinsics = os.path.join(
        default_config_dir, 'left_hand.yaml')

    start_camera = DeclareLaunchArgument(
        'start_camera', default_value='true',
        description='Start the realsense2_camera driver')
    use_gui = DeclareLaunchArgument(
        'use_gui', default_value='true',
        description='Open rqt_image_view for the annotated image')
    use_plot = DeclareLaunchArgument(
        'use_plot', default_value='true',
        description='Open the matplotlib base/camera/object TF plot')
    intrinsics_file = DeclareLaunchArgument(
        'intrinsics_file', default_value=default_intrinsics,
        description='Camera intrinsics YAML (calibration.yaml)')
    extrinsics_file = DeclareLaunchArgument(
        'extrinsics_file', default_value=default_extrinsics,
        description='Eye-to-hand camera-to-base YAML')
    base_frame = DeclareLaunchArgument(
        'base_frame', default_value='base_link',
        description='Robot base frame used for transformed 3D coordinates')
    plot_rate = DeclareLaunchArgument(
        'plot_rate', default_value='2.0',
        description='Matplotlib TF plot update rate in Hz')
    model_path = DeclareLaunchArgument(
        'model_path', default_value=default_model,
        description='Path to the YOLOv8 ONNX model')
    color_topic = DeclareLaunchArgument(
        'color_topic', default_value='/camera/camera/color/image_raw')
    depth_topic = DeclareLaunchArgument(
        'depth_topic',
        default_value='/camera/camera/aligned_depth_to_color/image_raw')
    camera_info_topic = DeclareLaunchArgument(
        'camera_info_topic',
        default_value='/camera/camera/color/camera_info')
    conf_threshold = DeclareLaunchArgument(
        'conf_threshold', default_value='0.25')
    iou_threshold = DeclareLaunchArgument(
        'iou_threshold', default_value='0.45')
    input_size = DeclareLaunchArgument(
        'input_size', default_value='640')
    infer_every_n_frames = DeclareLaunchArgument(
        'infer_every_n_frames', default_value='3',
        description='Run YOLO once every N frames '
                    '(3 = infer once, skip two frames)')
    depth_scale = DeclareLaunchArgument(
        'depth_scale', default_value='0.001',
        description='16UC1 depth unit in metres (D435i default 0.001)')

    camera = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(realsense_share, 'launch', 'rs_launch.py')),
        launch_arguments={
            'align_depth.enable': 'true',
            'rgb_camera.color_profile': '640,480,30',
            'depth_module.depth_profile': '640,480,30',
            'pointcloud.enable': 'false',
            'enable_infra1': 'false',
            'enable_infra2': 'false',
        }.items(),
        condition=IfCondition(LaunchConfiguration('start_camera')),
    )

    yolo_node = Node(
        package='piper_vision',
        executable='yolo_depth_node',
        name='yolo_depth_node',
        output='screen',
        parameters=[{
            'model_path': LaunchConfiguration('model_path'),
            'color_topic': LaunchConfiguration('color_topic'),
            'depth_topic': LaunchConfiguration('depth_topic'),
            'camera_info_topic': LaunchConfiguration('camera_info_topic'),
            'intrinsics_file': LaunchConfiguration('intrinsics_file'),
            'extrinsics_file': LaunchConfiguration('extrinsics_file'),
            'base_frame': LaunchConfiguration('base_frame'),
            'conf_threshold': ParameterValue(
                LaunchConfiguration('conf_threshold'), value_type=float),
            'iou_threshold': ParameterValue(
                LaunchConfiguration('iou_threshold'), value_type=float),
            'input_size': ParameterValue(
                LaunchConfiguration('input_size'), value_type=int),
            'infer_every_n_frames': ParameterValue(
                LaunchConfiguration('infer_every_n_frames'), value_type=int),
            'depth_scale': ParameterValue(
                LaunchConfiguration('depth_scale'), value_type=float),
        }],
    )

    image_view = Node(
        package='rqt_image_view',
        executable='rqt_image_view',
        name='yolo_image_view',
        arguments=['/vision/annotated_image'],
        condition=IfCondition(LaunchConfiguration('use_gui')),
    )

    tf_plot = Node(
        package='piper_vision',
        executable='vision_tf_plot',
        name='vision_tf_plot',
        output='screen',
        parameters=[{
            'extrinsics_file': LaunchConfiguration('extrinsics_file'),
            'base_frame': LaunchConfiguration('base_frame'),
            'plot_rate': ParameterValue(
                LaunchConfiguration('plot_rate'), value_type=float),
        }],
        additional_env={'PYTHONNOUSERSITE': '1'},
        condition=IfCondition(LaunchConfiguration('use_plot')),
    )

    return LaunchDescription([
        start_camera,
        use_gui,
        use_plot,
        intrinsics_file,
        extrinsics_file,
        base_frame,
        plot_rate,
        model_path,
        color_topic,
        depth_topic,
        camera_info_topic,
        conf_threshold,
        iou_threshold,
        input_size,
        infer_every_n_frames,
        depth_scale,
        camera,
        yolo_node,
        image_view,
        tf_plot,
    ])
