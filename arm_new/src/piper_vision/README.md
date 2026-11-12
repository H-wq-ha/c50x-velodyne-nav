# piper_vision

基于 **Intel RealSense D435i + YOLOv8n ONNX** 的实时 2D/3D 物体检测。

- 相机：`realsense2_camera` 官方 ROS 2 驱动
- 模型：`yolov8n.onnx`（Ultralytics 官方 `yolov8n.pt` 导出，opset 12，输入 640×640）
- 推理：`onnxruntime` CPU
- 深度：D435i 的 `aligned_depth_to_color`，与彩色图像逐像素对齐
- 3D 坐标：彩色相机光学坐标系，单位米

## 1. 依赖

```bash
sudo apt install ros-humble-realsense2-camera ros-humble-vision-msgs \
                 ros-humble-cv-bridge ros-humble-message-filters \
                 python3-opencv python3-numpy
/usr/bin/python3 -m pip install --user onnxruntime
```

## 2. 编译

```bash
cd tools/arm_new
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-select piper_vision
source install/setup.bash
```

## 3. 启动

```bash
ros2 launch piper_vision yolo_depth.launch.py
```

默认会：

1. 启动 D435i 驱动，彩色 640×480@30，深度 640×480@30，开启深度对齐彩色；
2. 启动 `yolo_depth_node`；
3. 打开 `rqt_image_view` 显示带检测框的画面。

只启动检测节点（相机已在别处启动）：

```bash
ros2 launch piper_vision yolo_depth.launch.py start_camera:=false
```

无图形界面：

```bash
ros2 launch piper_vision yolo_depth.launch.py use_gui:=false
```

## 4. 输出话题

| 话题 | 类型 | 说明 |
|---|---|---|
| `/vision/annotated_image` | `sensor_msgs/Image` | 带检测框、类别、深度、XYZ 的画面 |
| `/vision/detections_3d` | `vision_msgs/Detection3DArray` | 每个目标的 3D 位姿、3D bbox、类别、分数 |
| `/vision/detections_json` | `std_msgs/String` | JSON 格式，方便脚本/大模型读取 |
| `/vision/markers` | `visualization_msgs/MarkerArray` | RViz 中的球体 + 文字标签 |

查看 JSON：

```bash
ros2 topic echo /vision/detections_json
```

查看 3D 检测：

```bash
ros2 topic echo /vision/detections_3d
```

查看带框图：

```bash
ros2 run rqt_image_view rqt_image_view /vision/annotated_image
```

RViz：

```bash
rviz2
```

- Fixed Frame 设为 `camera_color_optical_frame`
- 添加 `MarkerArray`，话题 `/vision/markers`
- 添加 `Image`，话题 `/vision/annotated_image`

## 5. 3D 坐标定义

检测框中心像素 `(u, v)` 处的深度为 `Z`（米），彩色相机内参为：

```text
fx, fy, cx, cy
```

则物体相对相机的 3D 坐标：

```text
X = (u - cx) * Z / fx
Y = (v - cy) * Z / fy
Z = depth
```

坐标系为 **彩色相机光学坐标系**：

- `+X`：图像右方
- `+Y`：图像下方
- `+Z`：相机正前方（光轴）

单位为米。

### 隔帧推理

默认 `infer_every_n_frames=3`：

```text
第 1 帧：YOLO 推理
第 2 帧：跳过推理，复用上一帧检测框，用当前深度重新算 3D
第 3 帧：跳过推理，复用上一帧检测框，用当前深度重新算 3D
第 4 帧：YOLO 推理
...
```

这样带框画面仍按相机帧率发布，YOLO 的 CPU 占用降到约 1/3。
JSON 中 `inferred` 表示当前帧是否真正跑了推理，`inference_ms` 是最近一次推理耗时。

深度采样策略：

1. 在检测框中心取一个约 20% 框大小的 patch；
2. 去掉 0、NaN、超过 `max_depth` 的无效值；
3. 取有效深度的中位数作为 `Z`；
4. 如果中心 patch 无效，回退到整个检测框内的有效深度中位数；
5. 仍然无效则不输出 3D 点（JSON 中为 `null`）。

## 6. 主要参数

| 参数 | 默认值 | 说明 |
|---|---|---|
| `model_path` | 包内 `models/yolov8n.onnx` | ONNX 模型路径 |
| `color_topic` | `/camera/camera/color/image_raw` | 彩色图 |
| `depth_topic` | `/camera/camera/aligned_depth_to_color/image_raw` | 对齐深度 |
| `camera_info_topic` | `/camera/camera/color/camera_info` | 彩色内参 |
| `conf_threshold` | `0.25` | 置信度阈值 |
| `iou_threshold` | `0.45` | NMS IoU 阈值 |
| `input_size` | `640` | 模型输入尺寸 |
| `infer_every_n_frames` | `3` | 每隔 N 帧推理一次；`3` 表示推理 1 帧、跳过 2 帧 |
| `depth_scale` | `0.001` | 16UC1 深度单位（米/单位） |
| `max_depth` | `8.0` | 超过此深度视为无效 |
| `depth_patch_ratio` | `0.20` | 中心深度采样 patch 比例 |
| `publish_annotated` | `true` | 发布带框图 |
| `publish_markers` | `true` | 发布 RViz Marker |
| `num_threads` | `4` | onnxruntime CPU 线程数 |

示例：

```bash
ros2 run piper_vision yolo_depth_node --ros-args \
  -p conf_threshold:=0.5 \
  -p model_path:=src/piper_vision/models/yolov8n.onnx
```

## 7. 模型说明

`models/yolov8n.onnx` 由官方 `yolov8n.pt` 导出：

```bash
python3 - <<'PY'
from ultralytics import YOLO
YOLO('yolov8n.pt').export(
    format='onnx', imgsz=640, opset=12,
    simplify=False, dynamic=False)
PY
```

模型信息：

```text
input : images  [1, 3, 640, 640] float32
output: output0 [1, 84, 8400] float32
```

## 8. 注意事项

- 必须使用 **对齐到彩色** 的深度图，否则彩色像素和深度像素不对应，3D 坐标会偏。
- D435i 的 `16UC1` 深度默认单位是毫米，`depth_scale=0.001`。
- 透明、镜面、黑色吸光物体会导致深度无效，此时只输出 2D 框，`xyz_m` 为 `null`。
- 检测框中心不一定在物体表面（例如人形框中心可能是背景），当前用中心 patch 中值，必要时可改成框内多点聚类。
- 模型是 COCO 80 类，只能识别训练过的类别。
- 3D 坐标是相对 `camera_color_optical_frame` 的，若要转到机械臂基座，需要再做 TF 变换。
