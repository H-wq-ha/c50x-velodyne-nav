# 雷达与相机的当前 TF 关系

更新日期：2026-10-30。按用户指定采用以下安装关系，替代2026-10-27的初步参数。语义建图启动文件 `src/semantic_mapping/launch/semantic_mapping.launch.py` 和预览工具 `tools/preview_lidar_camera_tf.py` 已同步更新。

## 坐标约定

- 父坐标系：`velodyne`。
- 子坐标系：`camera_link`，按 ROS 相机机体约定 +X 镜头前向、+Y 左、+Z 上。
- 平移 `[0.096769992, 0.017500000, -0.064108820] m` 表示 camera_link 原点在 velodyne 中的位置，即雷达 +X 方向96.769992 mm、+Y 方向17.5 mm、下方64.108820 mm。
- 四元数 xyzw：`[0, 0.4233835445, 0, 0.9059505363]`。
- 等价 RPY：`[0, 0.8743537510167253, 0] rad`，即 `[0°, 50.09677973469077°, 0°]`。
- 镜头前向的水平投影与雷达 +X 同向，向下俯视约50.09678°。
- 此配置已按用户要求采用；模型几何关系不替代实物外参标定验收。

## 点坐标转换

```text
p_velodyne = R_velodyne_from_camera_link × p_camera_link + t

R = [ 0.641492748   0   0.767129098 ]
    [ 0             1   0           ]
    [-0.767129098   0   0.641492748 ]

t = [0.096769992, 0.017500000, -0.064108820] m
```

相机三根轴在雷达坐标系中的方向分别是 R 的三列：

| 相机方向 | 雷达坐标中的单位方向 |
|---|---|
| +X，镜头前向 | `[0.641492748, 0, -0.767129098]` |
| +Y，相机左方 | `[0, 1, 0]` |
| +Z，相机上方 | `[0.767129098, 0, 0.641492748]` |

camera_link 的 +X 与光学坐标系的 +Z 同为镜头前向。深度、彩色及光学坐标系之间的内部 TF 继续由 RealSense 驱动发布。不要将这组 camera_link 四元数直接用于 camera_depth_optical_frame；按标准相机光学轴换算，后者的四元数约为 `[0.6646670404, -0.6646670404, 0.2412834959, -0.2412834959]`。

## 与现有底盘 TF 的衔接

建图和此页整车预览使用 `base_link → imu_link` 单位变换；`imu_link → velodyne` 的物理平移为 `[-0.0718, 0.006, 1.0345] m`、yaw=180°：

```text
base_link → camera_link
translation = [-0.168569992, -0.011500000, 0.970391180] m
RPY = [0°, 50.09677973469077°, 180°]
镜头前向在 base_link 中 = [-0.641492748, 0, -0.767129098]
```

在这个建图/预览坐标约定中，相机水平前向与雷达 +X 同向，但在 base_link 中朝 -X 方向。

语义导航默认 `chassis_yaw_offset=π`，会把 `base_link → imu_link` 旋转 π。与同一物理外参组合后：

```text
base_link → velodyne
translation = [+0.0718, -0.006, 1.0345] m
yaw = 0°

base_link → camera_link
translation = [+0.168569992, +0.011500000, 0.970391180] m
RPY = [0°, 50.09677973469077°, 0°]
镜头水平前向在 base_link 中 = [+0.641492748, 0, -0.767129098]
```

`--whole-vehicle` 预览按建图/预览的 yaw=0 基准显示；语义导航的组合关系如上，不能将两种 base_link 约定混用。

## 更新并加载启动配置

```bash
cd /home/y/arm_car/lidar
source /opt/ros/humble/setup.bash
colcon build --packages-select semantic_mapping velodyne_bringup
source install/setup.bash
```

下次启动语义建图时，默认 `publish_camera_mount_tf:=true` 会发布本页的新配置。已运行的发布节点需要重启才会加载更新。

## 单独发布静态 TF

仅在没有其他节点发布 camera_link 的安装 TF 时单独运行：

```bash
source /opt/ros/humble/setup.bash
ros2 run tf2_ros static_transform_publisher \
  --x 0.096769992 --y 0.017500000 --z -0.064108820 \
  --qx 0 --qy 0.4233835445 --qz 0 --qw 0.9059505363 \
  --frame-id velodyne --child-frame-id camera_link
```

## RViz 预览

```bash
source /opt/ros/humble/setup.bash
cd /home/y/arm_car/lidar
/usr/bin/python3 tools/preview_lidar_camera_tf.py --rviz
```

整车模式按建图/预览的 base_link→imu_link yaw=0 包含底盘、IMU和雷达关系：

```bash
/usr/bin/python3 tools/preview_lidar_camera_tf.py --whole-vehicle --rviz
```

```text
base_link
└── imu_link         t=[0, 0, 0], RPY=[0, 0, 0]
    └── velodyne     t=[-0.0718, 0.006, 1.0345] m, yaw=180°
        └── camera_link  t=[0.096769992, 0.017500000, -0.064108820] m,
                         pitch≈50.096779735°
```

- 预览自身发布静态 TF；它与实车安装 TF 发布者二选一，避免重复发布相同子坐标系。关闭由工具打开的 RViz 窗口时，预览也会退出。
- 红 X、绿 Y、蓝 Z；青色表示镜头前向，黄色表示其水平投影与俯角。
- 预览只显示几何关系，不启动雷达、相机、SLAM或底盘驱动；传感器图标和车体平面为示意。
- 新参数的俯角由给定四元数计算，预览的 RPY 表达与启动配置的四元数表达是同一旋转。

2026-11-01：根据用户实物确认，IMU与雷达沿车体前后方向相距7.18 cm；旧值71.8 cm存在十倍小数点误差，启动配置和整车预览默认值已同步修正。旧导航包显示 odom 横向速度与角速度的关系和旧雷达杆臂一致；基于录包作的反事实重参考仅属离线证据，未实车验证。

历史：2026-10-27曾验证旧版50°预览的软件显示。该记录不作为本次新参数的实时硬件验证结果。
