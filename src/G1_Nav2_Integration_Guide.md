# G1 / Lightning-LM / Nav2 Jazzy 接入包

你已完成的外参补偿和额外 PointCloud2 输出作为现有输入。本包读取你选择的官方 G1 URDF，不修改驱动数据，不创建第二个点云转换节点。

当前交付为可审阅的接入实现：7 项离线几何/配置测试通过，补丁在固定上游源码上通过 `git apply --check`。此环境没有 ROS 2、colcon 或机器人连接，尚未完成 Jazzy 编译、ROS 图联调和 G1 实机验证。

## 1. 内容与坐标设计

| 内容 | 功能 |
|---|---|
| `g1_nav_bridge/` | 可用 colcon 构建的 ament_python 包；桥接节点使用 rclpy |
| `lightning_integration.patch` | 小范围 C++ 补丁：地图分辨率、位姿输出、初始化接口和 ROS 参数兼容 |
| `vendor/reference_g1_*.urdf` | 官方 23/29 DOF URDF 参考快照；使用时选择与你本体一致的文件 |
| `tools/apply_lightning_patch.py` | 先检查上下文，明确添加 `--apply` 后应用；不覆盖整个源文件 |
| `tests/test_geometry_and_config.py` | 脱离 ROS 的几何和配置测试 |

TF 主链：`map → pelvis → 官方腰部关节链 → torso_link → mid360_link`。

- `map → pelvis` 由 pose_bridge 发布，保留真实三维位置与姿态。
- 骨盆至雷达的关节链由 robot_state_publisher 根据官方 URDF 与真实 joint_states 发布。
- 23 DOF 与 29 DOF 的腰部结构不同；launch 直接读取 `urdf` 参数指定的文件，不默认假定你的型号。
- 历史记录找回的 MID360 固定关节是 `xyz=[0.0002835,0.00003,0.40618]`、`rpy=[0,0.04014257279586953,0]`，与两份参考模型一致；完整型号没有从历史检索中确定。
- 官方 `imu_in_torso` 和 `imu_in_pelvis` 是本体 IMU 的坐标，不能直接充当 MID360 内部 IMU。

原版 LIO 的状态是 IMU 状态。包读取你已经补偿好的 YAML 中 `fasterlio.extrinsic_R/T`，以 `T_I_L` 的逆建立 `mid360_link → lightning_tracking` 固定关系。它描述算法原点与雷达的关系，不对点云或 IMU 做第二次数值补偿。核心转换为 `T_map_pelvis = T_map_tracking × T_tracking_pelvis(t)`，使用位姿时间戳对应的关节 TF。

**适用接口约定：**你的补偿后的 LiDAR 数据轴与官方 `mid360_link` 一致，`extrinsic_R/T` 仍表示该 LiDAR 坐标到算法 IMU 坐标的变换；算法输出保留原版的状态参考点语义。如果本地修改还改变了 `pose_` 的参考点，需要调整这一输出约定，不能重复套用 IMU→LiDAR 关系。桥接不猜测未知的点云坐标，不把本体 IMU 当雷达 IMU。

Nav2 的 global/local costmap、behavior server 都使用 `map` 和 `pelvis`。`/lightning/odom` 只提供 LIO 局部状态对应的骨盆速度反馈，其 header.frame_id 为 `lightning_odom`，本包不发布该 frame 的 TF；当前启动的消费者从中读取 twist。不要把这份配置直接与其他要求 `odom` TF 的启动文件混用。

## 2. 在现有工作区接入

先保留现有驱动启动方式以及你的 PointCloud2 发布代码。以下路径按你已出现的 `/opt/G1/lighting_ws` 举例。

在解压目录中检查补丁：

```bash
python3 tools/apply_lightning_patch.py /opt/G1/lighting_ws/src/lightning-lm
```

检查通过后应用并构建：

```bash
python3 tools/apply_lightning_patch.py /opt/G1/lighting_ws/src/lightning-lm --apply
cp -a g1_nav_bridge /opt/G1/lighting_ws/src/
cd /opt/G1/lighting_ws
source /opt/ros/jazzy/setup.bash
colcon build --packages-select lightning g1_nav_bridge --symlink-install
source install/setup.bash
```

需要已有的 `rclpy`、`tf2_ros`、`robot_state_publisher`、Nav2 Jazzy、NumPy/PyYAML。使用内置 G1 关节/速度适配时还需要官方 `unitree_hg` 和 `unitree_api` ROS 消息包，并 source 它们的安装环境；这是运动/状态接口依赖。本包未复制 SDK 的低层运动示例。

若补丁检查失败，工具会停止。查看失败文件对应的改动，将新增接口合并进你的已修改文件；不要用上游文件覆盖你的外参补偿和点云发布代码。

## 3. 启动参数

| 参数 | 填写内容 |
|---|---|
| `urdf` | 你本机实际使用的官方 G1 URDF 绝对路径；也可选择本包中匹配型号的参考文件 |
| `lightning_config` | 你已经调整好的完整 Lightning YAML 绝对路径 |
| `lightning_root` | Lightning 工作目录；原版相对 data 路径以此为基准 |
| `cloud_topic` | 你已经新增的 PointCloud2 话题；导航模式必须指定 |
| `cloud_alias` | 可选，只有点云确实在已校正 LiDAR 坐标且 header 使用另一个未发布 TF 的名称时才填写该名称；map 点云留空 |
| `map_yaml` | 默认取 `system.map_path/map.yaml`，可指定其他导航栅格地图 |
| `floor_z` | 导航地图坐标下的地面 z；默认读取现有 g2p5.floor_height。启用动态地面估计时必须明确指定 |
| `publish_joint_states` | 默认 true，读取 LowState；已有准确 /joint_states 时设 false |
| `lowstate_topic` | 默认 /lowstate，可改为实际话题 |
| `start_drive` | 默认 false，只启动建图/定位/导航链；true 启动 G1 高层速度适配 |

为避免官方 mesh 路径的安装差异，运行时生成仅含运动学的 robot_description：保留官方关节 origin、axis、limit，去除 visual/collision/inertial。它用于准确发布 TF，不用于显示完整机器人外观或自动推导运动包络。

### 建图

启动你现有的 Livox 驱动后，带入本机路径：

```bash
ros2 launch g1_nav_bridge g1_navigation.launch.py \
  mode:=mapping \
  urdf:=/你的官方URDF绝对路径 \
  lightning_config:=/你的Lightning配置绝对路径 \
  lightning_root:=/opt/G1/lighting_ws/src/lightning-lm
```

以未使用过的地图名保存，启用现有 YAML 的 `system.with_g2p5` 才会输出栅格文件：

```bash
ros2 service call /lightning/save_map lightning/srv/SaveMap "{map_id: 'g1_nav_test01'}"
```

原版保存逻辑会重建同名地图目录，测试时使用新名字。修正分辨率后应重新保存地图；仅修改未来保存代码不会自动修复旧 map.yaml。将定位 YAML 的 `system.map_path` 指向新目录，例如 `./data/g1_nav_test01/`。

### 定位与导航

停止建图程序，保留现有驱动：

```bash
ros2 launch g1_nav_bridge g1_navigation.launch.py \
  mode:=navigation \
  urdf:=/你的官方URDF绝对路径 \
  lightning_config:=/你的Lightning配置绝对路径 \
  lightning_root:=/opt/G1/lighting_ws/src/lightning-lm \
  cloud_topic:=/你已输出的PointCloud2话题
```

RViz Fixed Frame 使用 `map`。`2D Pose Estimate` 指定骨盆的平面位置和朝向；桥接会结合当前关节关系换算成算法参考点的初始位姿，保留当前参考高度与倾斜。`2D Nav Goal` 发送导航目标。

需要把最终速度交给本体时，在相同命令中加入 `start_drive:=true`。适配使用官方 G1 高层速度 API 7105、`/api/sport/request`，发送 `velocity=[vx,vy,wz]` 和有限 duration。它不负责让机器人从任意状态自动站立或切换运动模式；本体需要已经能接受高层 Move 命令。收到 ROS 消息并不代表本体确认执行，实机验证需观察运动与接口反馈。

不要同时运行另一套发布 `map→pelvis` 或同一根运动链的 TF 节点。此 launch 已组合 map_server 与 Lightning 定位，不启动 AMCL。

## 4. 数据约定与初始参数

- 新增的 `/lightning/map_pose`：原版全局 IMU 状态位姿，父坐标 `map`；定位有效时发布。建图模式使用该阶段的 LIO 地图坐标。
- `/lightning/lio_pose`：局部 IMU 递推位姿，父坐标 `lightning_odom`，定位模式发布；速度由约 80 ms 窗口的骨盆局部位姿差分得到。
- `/lightning/odom` 的协方差未标定，仅供本配置中的速度消费者；不应直接作为已标定测量输入融合滤波器。
- `/g1_nav/pose_valid` 表示位姿/TF 时效性，不是额外的定位置信度评估。
- LowState 没有 ROS Header。内置关节桥使用接收时刻，并提供 `joint_time_offset` 参数。跨机/高延迟部署应使用已经同步到传感器时钟的 /joint_states；不会把 tick 直接当作 Unix 时间。
- 内置关节映射只接受 PR 模式；遇到 A/B 数据不将其误作腰部/踝部关节角，不自动改变本体控制模式。特殊带手模型有未映射关节时，使用外部完整 /joint_states。
- 现有障碍点云需要有正确 frame_id、时间戳及对应时刻 TF。若是 world/map 点云，costmap 的 raytrace 原点仍使用 `mid360_link`；若是 sensor 点云，必须确认该帧的轴与现有补偿一致。
- pose_bridge 不拿最新 TF 冒充测量时刻 TF，数据超时或时间戳回退会丢弃并记录日志。

本包初始调试参数：MPPI Omni，前后速度上限 0.20/0.15 m/s，侧移 0.10 m/s，转动 0.40 rad/s。选择 Omni 是接入方案，非声称已在你的本体调优。

0.40 m 圆形导航半径和 0.60 m 膨胀半径是初始测试值，不能视为官方认证运动包络；手臂姿态、携带物和步态需要实机校准。costmap 高度在 map 中按地面 z + [0.10,1.8] m 过滤；collision monitor 高度在 pelvis 中按 [-0.45,1.2] m 过滤，两者参考系不同。初次调参重点检查地面、自身点与低障碍。

## 5. 已验证与实机核查入口

本包离线验证：

```bash
python3 tests/test_geometry_and_config.py
```

覆盖官方 23/29 DOF 的关节链、外参逆变换、腰部运动时静止骨盆恢复、初始位姿的平面调整、旋转跨 ±π、身体坐标速度及 Nav2 帧/点云配置。补丁已针对审阅的上游版本检查，尚未证明可以干净套用到你的所有本地改动。

实机首轮只需要核对这些入口：

```bash
ros2 topic list -t
ros2 run tf2_ros tf2_echo map pelvis
ros2 run tf2_ros tf2_echo pelvis mid360_link
ros2 topic hz /lightning/odom
ros2 topic echo /cmd_vel --once
```

需要达到：站立/身体小幅运动时固定环境保持稳定；保存后的栅格与点云地图尺度一致；重启定位和 RViz 初始化正确；Nav2 生成速度且本体处于可执行高层 Move 的模式。上述运行条件尚未在本环境验证。

## 6. 来源

- [官方 G1 URDF](https://github.com/unitreerobotics/unitree_ros/tree/master/robots/g1_description)，本次访问仓库引用 `7d6075f7f58588b189b940130e3edab3c839b2df`。
- [官方 G1 电机关节索引](https://github.com/unitreerobotics/unitree_ros2/blob/master/example/src/src/g1/lowlevel/g1_low_level_example.cpp)。本包只参考读状态映射，没有运行该文件里的低层驱动动作。
- [官方 G1 ROS 高层速度 API](https://github.com/unitreerobotics/unitree_ros2/blob/master/example/src/include/g1/g1_loco_client.hpp)，API 7105、sport request、velocity/duration 数据结构。
- Lightning 补丁基线：[1325fed8fa97a2506878360fc23427ea00a767da](https://github.com/gaoxiang12/lightning-lm/tree/1325fed8fa97a2506878360fc23427ea00a767da)。
- Nav2 参数基线：[jazzy f4108e5b1c2bce804a1aa0c7be6673a8eb4a1501](https://github.com/ros-navigation/navigation2/tree/f4108e5b1c2bce804a1aa0c7be6673a8eb4a1501)。

官方 URDF 的许可保留在 vendor/LICENSE.Unitree。Nav2 参数与派生配置遵循 Apache-2.0；原仓库链接与版本保留用于追溯。本包中的自编桥接代码按 Apache-2.0 提供。
