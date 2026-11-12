# lidar 项目记忆

副本文档与文件状态核对日期：2026-11-12（Asia/Shanghai）。本目录是上传准备副本；下文保留的实车试验记录来自原工作区，不能据此推断本副本的运行状态、地图或硬件可用性。

## 项目与源码

- 当前副本：`/home/y/arm_car/lidar（复件）`；来源工作区为 `/home/y/arm_car/lidar`。系统面向 ROS 2 Humble、实车 WHEELTEC C50X、VLP-16、底盘 IMU、RealSense D435i；使用副本时显式指定模型、推理环境、设备与地图路径。
- `src/velodyne_bringup/`：雷达、ICP、RTAB-Map、Nav2 的启动文件和参数。
- `src/car_bringup/`：底盘串口、IMU、速度反馈与键盘控制。`src/car/serial_read.py` 为独立脚本，不等同于主底盘包。
- `src/semantic_mapping/`：YOLO 三维观测、SQLite 物体库、目标规划、Nav2 执行与健康监控。包名与 Python 模块统一为 `semantic_mapping`，当前参数支持 laptop / bottle / cup（类别 63 / 39 / 41）；`/semantic/laptop/*` 话题保留以兼容现有接口。
- `yolov8_d435i/`：独立检测实验与 ONNX 模型；ROS 主链使用 `src/semantic_mapping/`。
- `../new_one/`：模型与仿真；`../ROSA/`：自然语言控制入口，现已接入语义导航结构化协议。
- `../arm/`：独立 Piper ChArUco / MoveIt 工作区，标定快照固定在 `config/calibration/`；TCP 为 link6 局部 +Z 0.1358 m，位置约束、姿态自由。2026-11-09 默认入口切换为“SDK V2 真机归零 → 硬件与物理模型门控 → MoveIt 真机执行端点”，回零可能移动；显式仿真入口为 `scripts/start_sim_demo.sh`。当日只读 CAN 六轴为 0°，SDK 回零后20个连续样本仍为0°，`err_code=0`、状态正常、硬件限位标志未置位；MoveGroup 零位 state validity 为 valid、无接触。11-06 越界值是历史模型起点拒绝，不是当次硬件报警。真机 RViz 使用新建官方 Humble 简化配置，在 stock 11.2.27 MoveGroup 集成35 s通过；原视觉布局 GDB 定位为 OGRE GL resize/expose drawable SIGSEGV。随后 SDK 从非零反馈 `[0.301,-2.082,3.106,1.391,16.790,0.749]°` 实际回零并连续20帧确认，状态/硬件限位正常。SDK 回零与实现记录见 [arm README](../arm/README.md)、[SDK 回零与 MoveIt 启动记录](../arm/docs/REAL_MOVEIT_STARTUP_PLAN.md)；未验证非零姿态回零、目标轨迹执行或 TCP 精度；不加载该工作区环境到 lidar 或 new_one。

## 已有能力与缺口

- 主链：雷达几何建图 + RGB-D 物体定位 + SQLite 记忆 + Nav2 物体接近；建图和导航分阶段运行，以配套地图包交接。
- 依据 2026-10-31 架构核对：P1–P3 已有实现和数据，P4 已有候选位姿、路径预检、执行与取消；不能据此宣称完成实车验收。
- 待补齐：语义地图包正式保存入口（2026-11-01 已从正在运行的建图会话手工保存并切换 latest）、到达后视觉复核、回环后的历史语义点重投影、物体时效与地图/模型/外参版本绑定。
- 上次架构核对发现相机发布缺失和内参缓存时效缺口；这是历史观察，修复前需重新核查。详见 [当前架构](docs/语义导航数据流架构.md)，旧计划的“尚待实施”描述已落后。
- 2026-11-02 回查2026-11-01会话 `18-40-38-653571-y-45669` 完整节点日志：18:42:25起出现MPPI求解失败；18:44:38、18:47:32、18:50:59共启动三次恢复Spin，各次紧邻前置错误均为 `Failed to make progress`，首次Spin因前方碰撞预测退出，后两次超时。证据：`~/.ros/log/controller_server_45696_1793529639354.log` 与 `~/.ros/log/behavior_server_45730_1793529639375.log`；替正此前“单次MPPI失败后Spin”的简化记录。随后录制的 `output/nav_diag_20261101_185833` 显示路径未收敛但未执行Spin（Spin动作状态/反馈及BT Spin事件均为0）。用户确认将雷达X安装距从误写的71.8 cm修为7.18 cm后，转圈问题已解决；这项用户实车确认与本轮离线集成验证分开记录。旧bag的角速度响应比例约1.73，仍待新TF下复测。2026-11-02起步段离线复核：第二次目标的首条非零MPPI指令已含正角速度，系统初始车头与前方15 cm路径方向相差约+11.64°；约3 s时 `/cmd_vel` 为vx=0.080 m/s、wz=0.200 rad/s，附近IMU读数为0.260 rad/s。仅证明起步转向已存在于指令及估计初始方向支持左转，不能据此认定指令全程合理；接收时刻近邻值未补偿传感器延迟。证据 `output/nav_diag_20261101_185833/initial_motion_summary.json`。离线分析JSON见 `output/nav_diag_20261101_185833/nav_tracking_summary.json`；此前59项离线测试及两包编译通过。
- 2026-11-01 ROSA语义导航接口更新：注册 `semantic_map_context`、`semantic_preview_approach`、`semantic_navigate_to_object`、`semantic_task_status`、`semantic_cancel_task` 五个工具；查询/预检不启动导航，明确行动才调用start。客户端不再用意图词表、`--enable-navigation` 或自然语言正则作第二权限判断，真正启动权限仅由服务端 `enable_chassis_control` 控制（launch默认true、节点默认false，保持现有默认）。上下文包含地图/会话、物体观测时间与fresh TF下的相对位置、地图/相机/Nav2可用性；颜色、别名、房间未知，预检只表示当时的ComputePath结果，start仍重新规划。源码/接口见 [ROSA 使用指南](../ROSA/ROSA_源码与使用指南.md)。本次 lidar 离线测试81项、ROSA chassis+semantic 38项通过；`semantic_mapping` 定向构建后源码与installed比对一致。真实DeepSeek + fakeNav2隔离E2E共32项断言全部通过：nearest行动选择synthetic `bottle:2`并达成 `succeeded/nav2_succeeded`，三类只读问题零start/新导航目标，preview实际调用且选中ID 2，全程0个 `/cmd_vel` 发布者。证据 `output/e2e_semantic_rosa_20261101_210058/`；最终离线协议E2E在完成TF新鲜度修复和installed同步后全过，证据 `output/e2e_semantic_rosa_20261101_210412/`。未驱动实车。
- 2026-11-01 早期 ROSA DeepSeek + fakeNav2 E2E 证据保存在 `output/e2e_semantic_rosa_20261101_202829/`；该运行与后续接口更新分开记录。早期修复 `../ROSA/rosa-ros2/src/rosa_semantic/client.py` transport 关闭时的 spin 线程 join（CLI原先SIGABRT，修复后rc=0）。当前接口版本的完整集成验收见上一条。

## 坐标与时间约定

- `imu_link → velodyne` 当前物理外参为平移 `[-0.0718, 0.006, 1.0345] m`，yaw=π。2026-11-01 用户确认IMU与雷达前后间距为7.18 cm，替正旧参数71.8 cm。建图入口 `base_link→imu_link` yaw=0，因此合成雷达 +X 与 base_link +X 相反；语义导航默认 yaw=π，因此合成 `base_link→velodyne` 为 `[+0.0718, -0.006, 1.0345] m`、yaw=0，+X同向。旧导航包 `/odom.twist.linear.y` 与角速度拟合斜率约−0.77 m，和旧杆臂误差吻合；离线重参考显著减小侧向残差，仍待实车验证。详见 [TF 文档](docs/雷达相机TF初步关系.md)。
- 相机采用 2026-10-30 指定安装关系；数值和光学轴换算只在 [TF 文档](docs/雷达相机TF初步关系.md) 维护。修改时同步语义建图 launch 和 `tools/preview_lidar_camera_tf.py`，避免重复发布同一安装 TF。
- 日常建图/导航：雷达使用主机接收时间（`gps_time=false`）；OA/GRIL 标定：STM32 共同硬件时基。两种模式不混用。

## 常用入口与验证

- 项目概览与功能分类：[README.md](README.md)；环境及分场景启动：[启动指南](docs/启动指南.md)；文档索引：[docs/README.md](docs/README.md)；当前源码架构：[数据流架构](docs/语义导航数据流架构.md)。2026-11-12 已按副本源码核对启动参数、默认控制权限、地图包要求和外部 ROSA 接口。
- 2026-11-12 从原件补入两篇知乎技术教程及七张配图，恢复其相对图片链接；原有架构快照也在本副本。当前架构文档仍以本副本源码核对结果为准，历史教程仅作面向读者的原理说明。
- 几何建图：`rtabmap_slam.launch.py`；定位导航：`rtabmap_nav_complete.launch.py` / `nav2_bringup.launch.py`（均属 velodyne_bringup）。
- 语义入口：`ros2 launch semantic_mapping one_click.launch.py`、`semantic_mapping.launch.py`、`semantic_navigation.launch.py`；源码位于 `src/semantic_mapping/launch/`。新建图默认目录为 `~/.ros/semantic_mapping/runs/`，历史数据保留原位置。执行前读取实际参数，不凭记忆假定是否启用底盘控制。
- ROSA语义客户端：`../ROSA/run_semantic_agent.sh --objects` 查询对象与运行上下文；`--enable-navigation` 为兼容保留参数，不控制权限。是否接受导航任务由实际 `semantic_navigator` 的 `enable_chassis_control` 参数决定（launch默认true、节点默认false）。五个任务工具及状态语义见 [ROSA 使用指南](../ROSA/ROSA_源码与使用指南.md)。离线 fakeNav2 验收入口：`source /opt/ros/humble/setup.bash && source install/setup.bash && /usr/bin/python3 src/semantic_mapping/test/e2e_semantic_rosa_offline.py --domain 80 --skip-llm`；不接真实模型或底盘。
- 2026-11-01 语义导航 RViz 修正：全局路径订阅 `/plan`（Volatile），局部最优轨迹订阅 MPPI `/trajectories`（MarkerArray），开启 MPPI 可视化并降采样候选标记；补齐 Navigation 2 面板，配合 Nav2 Goal 工具发送位姿目标。`/semantic_nav/planned_path` 仅为一次预检路径。证据：新包编译、49 项离线测试通过；安装文件与源码一致，独立 ROS 域离屏 RViz 实测上述订阅与 `/navigate_to_pose` 客户端加载成功。未下发实车目标，实际行驶显示待验证。
- 加载 `/opt/ros/humble/setup.bash`，按需 `colcon build --packages-select <相关包>`，再加载本工作区 `install/setup.bash`。ROS Python 使用 `/usr/bin/python3`，避免 Conda 解释器不匹配。
- 离线测试位置：`src/semantic_mapping/test/test_*.py`、`src/car_bringup/test/test_chassis_readonly.py`、`tools/tests/test_bluetooth_chassis_control.py`；按修改范围选择，使用已加载环境的 `/usr/bin/python3 -m pytest <路径>`。`integration_smoke.py` 单独评估运行条件。底盘关机端到端验证：加载 Humble 与本工作区后 `/usr/bin/python3 src/semantic_mapping/test/e2e_semantic_rosa_offline.py --domain 78`（假 Nav2/TF/地图 + 真 navigator + 真实LLM，不接触串口）。
- 本副本 `maps/` 当前只保留 `.gitkeep`，没有可用的几何地图或 `maps/latest/` 语义地图包。导航前须生成或恢复自己的配套地图；语义导航要求六件套与一致 `map_id`。`scripts/save_rtabmap_map.sh` 不会生成完整语义地图包，且其数据库备份源固定为 `~/.ros/rtabmap.db`。证据：2026-11-12 文件盘点、保存脚本与导航 launch 源码。

## 上传副本与文档

- 2026-11-12 状态：本副本已清理构建产物、虚拟环境、地图、output、日志和嵌套 Git 元数据；源码与模型仍保留，现有工作区文件内容不因移除 Git 元数据而丢失。部署前按启动指南重建环境。
- 2026-11-12 文档入口：README 汇总功能与总数据流；架构文档维护阶段、话题与 ROSA 五工具；启动指南维护依赖、路径覆盖、控制默认值和地图要求；索引链接教程与配图。依据为 `src/` launch、节点、参数与 `scripts/save_rtabmap_map.sh`，本次只检查文档内容和链接，未启动硬件或执行导航。
- 2026-11-13 按用户要求将本副本覆盖同步到公开仓库 `H-wq-ha/c50x-velodyne-nav` 的 `master` 分支；提交 `3acb8cdb` 以本副本文件树为准，移除了远端存在但副本没有的旧文件，保留既有提交历史。根 `.gitignore` 排除了本机工作区、设备标定结果 JSON 和历史运行快照；`maps/` 只上传 `.gitkeep`。提交已推送并核验。
