# IMU–LiDAR 标定工具

当前保留两套联合标定算法：

| 算法 | 说明 | 运行入口 |
|---|---|---|
| OA-LICalib | [采集与离线标定教程](oa_calibration/README.md)，需要三轴运动激励 | `oa_calibration/run.sh`、`oa_calibration/calibrate.sh` |
| GRIL-Calib | [算法说明](GRIL-Calib/README.md)，利用地面约束进行平面运动标定 | `gril_calibration/run.sh` |

LI-Init 的 ROS1/ROS2 源码、编译产物及专用运行入口已移除。
历史采集数据、运行记录、标定结果和所有 rosbag（含仿真示例录包）已清理。

`calibration/` 保留 OA/GRIL 共用的硬件时间采集程序、质量检查和正在使用的加速度校正配置 `accel_calibration03.json`。
新采集数据保存在 `runs/`，OA 离线结果保存在项目根目录的 `output/`。

IMU 单独内参标定见 [使用教程](../ImuCalibration/README.md)。
