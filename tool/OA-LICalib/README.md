# OA-LICalib: Observability-Aware Intrinsic and Extrinsic Calibration of LiDAR-IMU Systems

**OA-LICalib** is a versatile and highly repeatable calibration method for the LiDAR-inertial system within a continuous-time batch-optimization framework, where the intrinsics of both sensors and the spatial-temporal extrinsics between sensors are calibrated comprehensively without explicit hand-crafted targets. To improve efficiency and cope with challenges from degenerate motions, we introduce two dedicated modules to enable observability-aware calibration. Firstly, a data selection policy based on the information-theoretic metric selects informative segments for calibration in unconscious data collection process. Secondly, an observability-aware state update mechanism in the back-end optimization is introduced to update only the identifiable directions of the calibrated parameters by leveraging truncated singular value decomposition. In this way, the proposed method can get accurate calibration results even under degenerate cases where informative enough data segments do not exist. Extensive evaluations by both simulated and real-world experiments are carried out. The results demonstrate the high accuracy and repeatability of the proposed method in common human-made scenarios and various robot platforms.

## Prerequisites

- To use this tool, install Docker in your computer with [this](https://docs.docker.com/engine/install/) link. Make sure you follow the [post-installation](https://docs.docker.com/engine/install/linux-postinstall/) steps.


## Install & Run

```shell
# clone the project repo.
git clone https://github.com/leo-drive/OA-LICalib

# Build docker image
cd OA-LICalib/docker
docker image build -t calib:v1 .

# Create container from docker image
# define env. var. with your local repo. path
export REPO_PATH="/home/bzeren/projects/OA-LICalib/"
docker run -it --env="DISPLAY" --volume="$HOME/.Xauthority:/root/.Xauthority:rw" --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" --volume="$REPO_PATH:/root/catkin_oa_calib/src/OA-LICalib" calib:v1 bash

cd catkin_oa_calib/
catkin_make -DCATKIN_WHITELIST_PACKAGES=""

# install your environment and launch calibration tool
source ./devel/setup.bash
roslaunch oa_licalib li_calib.launch
```

## Intrinsic and Extrinsic Calibration

![](./data/lidar_intrinsic.png)

The intrinsics of an individual laser comprising a multi-beam 3D LiDAR.

A example to calibrate extrinsics between LiDAR and IMU while simultaneously calibrating intrinsics of both LiDAR and IMU in simulation. You can find simulated data at [`./data/bag/simu_bag.bag`]. 
The ground truth of intrinsics are at `[./data/bag]` and of extrinsics are as follows:

```yaml
P_LinI [0.30, 0.15, 0.05] meter
euler_LtoI [1.0, 2.0, 5.0] degree
```
Check the  parameter `path_bag` in the `config/simu.yaml`, **change it to your absolute path**. Then run it!

```shell
roslaunch oa_licalib li_calib.launch
```

## Data Collection

OA-LICalib works with rosbag files which contains `sensor_msgs/PointCloud2` and `sensor_msgs/Imu` topics. Also you can collect `nav_msgs/NavSatFix` or `nav_msgs/Odometry` to visualize your calibration results on Rviz.

The calibration accuracy is affected by the data collection environment. You should collect your data in a place that contains a lot of flat surfaces, and indoor spaces are the best locations under these conditions. However, you can also achieve good results outdoors. When collecting data, make sure to draw figures of eights and grids, capturing data from every angle

## Parameter Tuning

To achieve the best calibration results, you should tune the parameters in the `config/simu.yaml` file. The parameters are as follows:

| Parameter                           | Value                                                     |
|-------------------------------------|-----------------------------------------------------------|
| ndtResolution | Resolution of NDT grid structure (VoxelGridCovariance)<br/>0,5 for indoor case and 1.0 for outdoor case | 
| ndt_key_frame_downsample    | Resolutation parameter for voxel grid downsample function |
| map_downsample_size       | Resolutation parameter for voxel grid downsample function |
| knot_distance            | time interval |
| plane_motion       | set true if you collect data from vehicle |
| gyro_weight       | gyrometer sensor output’s weight for trajectory estimation |
| accel_weight       | accelerometer sensor output’s weight for trajectory estimation |
| lidar_weight       | lidar sensor output’s weight for trajectory estimation |

## Credits

This code was developed by the [APRIL Lab](https://april.zju.edu.cn/) in Zhejiang University. For researchers that leveraged this work, please cite the
following:

```txt
@Conference{lv2020targetless,
  title={Targetless calibration of lidar-imu system based on continuous-time batch estimation},
  author={Lv, Jiajun and Xu, Jinhong and Hu, Kewei and Liu, Yong and Zuo, Xingxing},
  booktitle={2020 IEEE/RSJ International Conference on Intelligent Robots and Systems (IROS)},
  pages={9968--9975},
  year={2020},
  organization={IEEE}
}
@Journal{lv2022,
  title={{OA-LICalib}: Observability-Aware Intrinsic and Extrinsic Calibration of LiDAR-IMU Systems},
  author={Jiajun Lv, Xingxing Zuo, Kewei Hu,  Jinhong Xu, Guoquan Huang, and Yong Liu},
  journal={IEEE Transactions on Robotics},
  year={2022},
  publisher={IEEE}
}
```

## License

The code is provided under the [GNU General Public License v3 (GPL-3)](https://www.gnu.org/licenses/gpl-3.0.txt).
