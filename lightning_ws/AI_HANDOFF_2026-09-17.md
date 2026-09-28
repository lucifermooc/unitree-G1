# G1 Lightning-LM / Nav2 项目交接说明

更新时间：2026-09-17  
本地工作区：`/home/lmw/project/unitree/G1/lightning_ws`  
Thor 工作区：`/opt/G1/lighting_ws`  
Thor 最近使用 IP：`192.168.128.146`，用户：`unitree`

## 1. 当前总体目标

在 Unitree G1 + Thor 上完成以下链路：

```text
MID360 + IMU
  -> Lightning-LM 建图/定位
  -> map/base_link TF 与二维地图
  -> Nav2 规划和局部避障
  -> /cmd_vel
  -> Unitree 官方 LocoClient SetVelocity
  -> G1 高层运控
```

另外接入：

- D435 RealSense 点云，仅进入 local costmap 的 STVL。
- Unitree `/lf/bmsstate` 转标准 `/battery_state`。
- `/odommodestate` 转 `/odom`，但不发布 odom TF。
- `map -> base_link` 查询后发布 `/current_pose`。

## 2. 本地源码中已经完成的主要工作

### 2.1 Lightning-LM 与 TF

代码位置：`lightning_ws/src/lightning-lm`

- 使用 `lightning-lm/urdf/g1.urdf`。
- `base_link` 定义为机器人旋转中心在地面的投影。
- 当前静态外参：

```text
base_link -> mid360_link
xyz = [0.0002835, 0.00003, 1.23618]
rpy = [0, 0.0401425728, 0]  # pitch约2.3度
```

- `LocSystem` 只从 `/tf_static` 缓存一次雷达到 base 的静态外参。
- Lightning 算法输出本质为 `T_map_lidar`，输出 TF 时计算：

```text
T_map_base = T_map_lidar * T_lidar_base
```

- 外参未就绪时不发布伪造的 `map -> base_link`。
- 不在每次 `ToGeoMsg()` 中阻塞查询 TF。
- 建图模式已接回 G2P5 地图更新回调并发布 `/map`。
- OpenCV 2D UI 行为与 ROS `/map` 发布合并在同一个更新回调中。
- 当前 G2P5 和 FasterLIO 主要配置在：
  `lightning-lm/config/default_livox.yaml`。

当前 LIO 关键参数：

```yaml
point_filter_num: 2
filter_size_scan: 0.1
filter_size_map: 0.2
extrinsic_est_en: false
extrinsic_T: [-0.011, 0.02329, -0.04412]
```

Lightning 直接订阅 `/livox/lidar` 的 `CustomMsg`，使用
`points[i].offset_time` 做逐点去畸变，这部分必须保留。

### 2.2 导航桥接包

代码位置：`lightning_ws/src/g1_nav_bridge`

现在作为统一桥接包，包含：

- `sport_to_odom.py`
  - `/odommodestate` -> `/odom`
  - 不发布 TF。
- `tf_to_current_pose.py`
  - 只查询 `map -> base_link`
  - 发布 `geometry_msgs/msg/PoseStamped` 到 `/current_pose`
  - 不生成 TF。
- `cmdvel_to_sport.cpp`
  - 订阅 Twist。
  - 调用移植到包内的 Unitree 官方 `LocoClient::SetVelocity()`。
  - 不再依赖部署时链接外部 `unitree_ros2` 源码目录。
  - 已在真机验证机器人能够移动。
- `battery_state_bridge.cpp`
  - `/lf/bmsstate` (`unitree_hg/msg/BmsState`)
  - -> `/battery_state` (`sensor_msgs/msg/BatteryState`)

Unitree 官方运动接口代码已移植到：

```text
g1_nav_bridge/include/g1_nav_bridge/unitree/g1_loco_client.hpp
g1_nav_bridge/include/g1_nav_bridge/unitree/common/
```

运动桥独立启动命令：

```bash
ros2 run g1_nav_bridge cmdvel_to_sport --ros-args \
  -p cmd_vel_topic:=/cmd_vel \
  -p duration:=0.5
```

不要在 `robot.launch.py` 已运行时再启动第二个运动桥。

### 2.3 BatteryState

新增工作区本地接口包：`lightning_ws/src/unitree_hg`

只包含当前需要的官方 `BmsState.msg`，使工作空间部署不依赖外部
`unitree_ros2` 源码目录。

转换已在 Thor 真数据验证：

```text
/lf/bmsstate -> /battery_state
电压 mV -> V
电流 mA -> A
SOC 0..100 -> percentage 0..1
单体电压 mV -> V
```

实测输出示例：

```text
voltage: 46.428 V
current: -3.588 A
percentage: 0.33
present: true
13节 cell_voltage
```

独立启动：

```bash
ros2 launch g1_nav_bridge battery_bridge.launch.py
```

### 2.4 Nav2

代码位置：`lightning_ws/src/aid_navigation2`

- `map_server` 加载 `/opt/G1/lighting_ws/data/new_map/map.yaml`。
- 默认不用 AMCL；定位来自 Lightning-LM TF。
- collision monitor 默认关闭。
- keepout 默认关闭，没有 mask 发布者时不应启用。
- MID360 障碍层保留。
- D435 点云只加入 local costmap 的 STVL，不进入 global costmap。
- 机器人不允许后退，velocity smoother 的线速度最小值为 0。
- 前进速度有效范围设计为 `0.3..0.5 m/s`。

### 2.5 RealSense D435

- URDF 静态关系：`base_link -> d435_link -> camera_link`。
- 相机通过序列号绑定，不依赖 USB 端口或 `/dev/video*` 编号。
- 驱动点云话题：

```text
/camera/camera/depth/color/points
```

- RealSense 驱动不改源码。
- STVL 做 local costmap 标记和清除。
- 配置：`robot_bringup/param/realsense_g1.yaml`。

### 2.6 统一启动入口

入口：`lightning_ws/src/robot_bringup/launch/robot.launch.py`

目标是只编排原子 launch，不重复创建已有节点。

默认：

```bash
ros2 launch robot_bringup robot.launch.py
```

支持 `base/mapping/localization/navigation` 模式。

已修复顶层 `mode:=navigation` 被 Lightning 子 launch 的
`mode:=localization` 污染的问题。现在先保存为 `requested_mode`，因此默认导航启动时
Lightning、map_server、Nav2、运动桥、电池桥能同时出现。

停止入口：`robot_bringup/launch/stop_all.launch.py`。

## 3. 已经部署到 Thor 并验证的部分

以下内容在 2026-09-16 已同步到 `.146` 并编译：

```text
unitree_hg
g1_nav_bridge
robot_bringup
```

编译结果：3 packages finished。

真机完整启动时确认存在：

```text
/lightning_slam
/map_server
/planner_server
/controller_server
/g1_cmdvel_to_sport
/g1_battery_state_bridge
```

并确认 `/battery_state` 有真实数据。

顶层 `requested_mode` 隔离修复也已经部署并重新编译过
`robot_bringup`。

## 4. 仅在本地完成、明确尚未部署的修改

接手者必须先获得用户允许，不能擅自部署。

### 4.1 Python入口 shebang 修复

本地已给以下文件补了 `#!/usr/bin/env python3`：

```text
g1_nav_bridge/g1_nav_bridge/sport_to_odom.py
g1_nav_bridge/g1_nav_bridge/tf_to_current_pose.py
```

原因：Thor 完整启动曾报：

```text
Exec format error: .../tf_to_current_pose
Exec format error: .../sport_to_odom
```

Thor 当时关机，修复未同步。之后网络恢复也没有再部署。

### 4.2 最新角速度修改

用户明确要求“不要部署到 Thor”。本地已修改：

```text
最小有效转向速度：0.8 rad/s
最大转向速度：0.9 rad/s
velocity_smoother angular deadband：0.8
velocity_smoother angular range：[-0.9, 0.9]
```

涉及文件：

```text
aid_navigation2/launch/g1_config.py
aid_navigation2/param/nav2_params.yaml
aid_navigation2/test/test_g1_direct_config.py
```

本地测试：`11 passed`。

## 5. 完整启动仍待解决的问题

### 5.1 RealSense launch 参数泄漏

完整 `robot.launch.py` 启动时，官方 `rs_launch.py` 会扫描整个 launch context，
因此产生约 52 条：

```text
Warning: Parameter 'mode' is not supported
Warning: Parameter 'start_livox' is not supported
...
```

相机节点仍能启动，但警告很多。现有 `ResetLaunchConfigurations` 没有真正隔离
Include 产生的后续 OpaqueFunction。建议在 `sensor_driver.launch.py` 中直接创建
`realsense2_camera_node`，或使用真正独立的 launch context；不要改 RealSense
驱动源码。

### 5.2 keepout/forbidden 节点重复

日志出现：

```text
Publisher already registered for node name: forbidden_map_create_node
get_forbidden_client return false
```

即使 `use_keepout:=false`，后台仍启动了 forbidden map 相关节点，并且导航 launch
中可能还有一份。需要统一所有权：keepout=false 时不要启动/调用禁行区服务。

### 5.3 Nav2 inflation 警告

SmacPlanner 报 inflation radius 对非圆形碰撞检查不足。当前机器人按圆形半径配置，
但需要核对最终动态生成 YAML 中的 `robot_radius`、footprint、inflation radius 是否一致。

### 5.4 RealSense偶发硬件警告

曾出现：

```text
Depth stream start failure / Hardware Error
```

可在单独启动时用 `initial_reset:=true` 验证，但不应每次默认 reset。

### 5.5 测试超时退出噪声

完整启动验证用 `timeout --signal=INT` 停止后，部分 Python 节点会二次 shutdown，
Livox 驱动退出时偶发 `-11`。这些发生在测试主动 SIGINT 之后，不能与启动期错误混淆。

## 6. Unitree 官方 SLAM 黑盒检查结果

2026-09-17 在 Thor `.146` 只读检查官方 SLAM，未改任何代码。

### 6.1 官方不发布 ROS TF

```text
/tf        Publisher count: 0
/tf_static Publisher count: 0
```

官方通过消息表达坐标关系：

```text
/unitree/slam_mapping/odom
  header.frame_id = map
  child_frame_id = base_link

/unitree/slam_mapping/points
  frame_id = map
```

即：位姿放在 Odometry 中，配准点云已经直接烘焙到 map，不依赖 ROS TF。

### 6.2 原始点云

```text
/utlidar/cloud_livox_mid360
frame_id = livox_frame
约20064点/帧
point_step = 22
字段 = x y z intensity ring time
```

### 6.3 官方SLAM点云

```text
/unitree/slam_mapping/points
frame_id = map
样本约979点/帧
point_step = 48
字段 = x y z normal_x normal_y normal_z intensity curvature
stamp = 0
```

说明官方对原始点云进行了去畸变、重力对齐、降采样/特征提取和 map 变换。
单帧展示点数约为原始点数的 5%，但不代表内部地图也只有这些点。

### 6.4 官方处理链推断

```text
原始MID360点云（ring + time）
 -> IMU逐点去畸变
 -> LiDAR/IMU内部外参
 -> 重力初始化，map Z轴对齐重力
 -> LIO匹配
 -> 体素/法向/曲率特征筛选
 -> Odometry(map, base_link)
 -> 稀疏 registered cloud(frame=map)
```

官方服务是 Bare DDS App，实际进程不在 Thor ROS 节点列表中，Thor 上也找不到对应
配置/二进制，因此内部 `base_link <-> livox_frame` 精确外参暂时无法读取。

官方还存在多条用途分离的点云：

```text
/collision_clouds
/safe_clouds
/warning_clouds
/grid_clouds
/ele_clouds
```

重要启示：SLAM特征点云、导航障碍点云和原始点云应该分开处理。

## 7. 下一步建议

### 优先级1：录制两种 bag

#### Lightning 调参包

使用现有脚本：

```bash
cd /opt/G1/lighting_ws
./record_g1_slam_bag.sh lightning_tune_01 120
```

必须确认 `/livox/lidar` 和 `/livox/imu` 消息数非零。

#### 官方对照包

录制至少：

```text
/utlidar/cloud_livox_mid360
/utlidar/imu_livox_mid360
/unitree/slam_mapping/odom
/unitree/slam_mapping/points
/slam_info
/slam_key_info
/api/slam_operate/request
/api/slam_operate/response
/collision_clouds
/safe_clouds
/warning_clouds
/grid_clouds
/tf
/tf_static
```

录制顺序：启动录包 -> 静止 -> 启动官方SLAM -> 静止15秒 -> 原地分段旋转一圈
-> 矩形闭环 -> 回到起点静止。

### 优先级2：发布 Lightning registered cloud

建议新增：

```text
/lightning/registered_points
frame_id = map
stamp = 当前扫描结束时间
```

内容应来自 LIO 去畸变并转换到 map 后的点，做可配置体素降采样。它用于判断：

- 重力对齐是否正确。
- 单帧墙面是否厚。
- 轨迹累计导致的重影还是 G2P5 投影噪声。

不要用高度稀疏的SLAM特征点云直接替代安全避障点云。

### 优先级3：用同一 bag 对比参数

```yaml
# 当前
point_filter_num: 2
filter_size_scan: 0.10
filter_size_map: 0.20

# 方案A
point_filter_num: 3
filter_size_scan: 0.15
filter_size_map: 0.20

# 方案B
point_filter_num: 4
filter_size_scan: 0.20
filter_size_map: 0.20
```

比较有效点数、地面/墙面厚度、ICP有效点、ESKF拒绝次数、轨迹漂移和G2P5噪点。

### 优先级4：再部署尚未同步的本地修改

只有用户明确同意后才能部署：

1. 两个 Python 文件的 shebang 修复。
2. 角速度 `0.8..0.9` 修改。
3. 重新编译 `g1_nav_bridge aid_navigation2 robot_bringup`。
4. 再执行完整启动验证，不发送导航目标和速度。

## 8. 常用环境与命令

Thor：

```bash
cd /opt/G1/lighting_ws
source /opt/ros/jazzy/setup.bash
source /home/unitree/unitree_ros2/cyclonedds_ws/install/setup.bash
source install/setup.bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
```

编译：

```bash
colcon build --packages-select \
  unitree_hg g1_nav_bridge aid_navigation2 robot_bringup \
  --executor sequential --symlink-install
```

完整启动：

```bash
ros2 launch robot_bringup robot.launch.py
```

单独运动桥：

```bash
ros2 run g1_nav_bridge cmdvel_to_sport --ros-args \
  -p cmd_vel_topic:=/cmd_vel -p duration:=0.5
```

单独电池桥：

```bash
ros2 launch g1_nav_bridge battery_bridge.launch.py
```

## 9. 会话与附件位置

当前 Codex 会话原始 JSONL：

```text
/home/lmw/.codex/sessions/2026/09/17/rollout-2026-09-17T09-35-47-01a06a15-e3a3-7ce0-b8fa-e7723bfbd122_01a0ad01-6e48-79a1-a183-802aaea7a906.jsonl
```

本轮用户上传/粘贴附件根目录：

```text
/home/lmw/.codex/attachments
```

本项目最相关附件包括：

```text
/home/lmw/.codex/attachments/13ad22a9-90f4-413f-adf0-387653a6bbae/pasted-text.txt
/home/lmw/.codex/attachments/5d56b5c6-f74b-4939-97c8-09dcd11b0ab0/pasted-text.txt
/home/lmw/.codex/attachments/413e5d84-e5dc-4a60-949c-1a3481b2ccfd/pasted-text.txt
/home/lmw/.codex/attachments/8eccc549-db9f-401a-bdd4-0423b633ee38/pasted-text.txt
/home/lmw/.codex/attachments/05200083-fa8e-4334-b32b-c365b2c77ec2/pasted-text.txt
/home/lmw/.codex/attachments/e08c6c46-3aa4-4931-b15a-b308dfb8c44b/pasted-text.txt
/home/lmw/.codex/attachments/a0faa4d9-76e6-43c5-96de-626f005031b9/pasted-text.txt
/home/lmw/.codex/attachments/b573b090-c3cb-4aca-bfbe-754cb4bf9bf2/pasted-text.txt
/home/lmw/.codex/attachments/bc16465f-5f5e-4099-a66e-f5a25f449b91/pasted-text.txt
/home/lmw/.codex/attachments/9b003152-e7b7-4e95-be93-7e37da6e9c86/pasted-text.txt
/home/lmw/.codex/attachments/a067ef80-0873-40ed-91f3-6477b9377be6/pasted-text.txt
/home/lmw/.codex/attachments/5e71c95c-44f7-42c7-9a57-c4f2f353d30f/pasted-text.txt
/home/lmw/.codex/attachments/4e3d55b4-84e0-480e-88d9-dec6911a75bb/pasted-text.txt
```

接手 AI 应优先阅读本文件，再查看会话 JSONL 和上述源码文件。不要假设本地修改已经全部部署到 Thor。

---

## 10. 2026-09-17 第二轮交接（接手 AI 的工作记录）

本轮 Thor 离线/关机，所有真机相关工作（录包、参数对比、部署）均未推进。
以下只涉及本地源码。

### 10.1 已修复：5.2 keepout/forbidden 节点重复

根因：`robot_bringup/launch/robot.launch.py` 无条件启动
`forbidden_map_create_node`，完全没有读 `use_keepout`（默认 false）；而
`aid_navigation2/launch/g1_navigation_direct.launch.py` 那边是正确响应该参数的，
所有权不统一。

`get_forbidden_client return false` 噪声同源：
`robot_bringup/src/forbidden_map_create.cpp` 的 `MapCallback` 每收到一帧 `/map`
就调 `GetForbiddenAndDraw()`，去请求不存在的 `get_current_forbidden` 服务并报错。

改动：该 Node 加上 `condition=IfCondition(LaunchConfiguration('use_keepout'))`。
离线验证：条件求值 `use_keepout=false -> False`、`true -> True`。

**未部署到 Thor。**

### 10.2 结论修正：5.1 RealSense 参数泄漏其实已经修好了

原文档建议"改写成直接创建 realsense2_camera_node"，**这是不必要的返工**。

- `ResetLaunchConfigurations.execute()` 确实会 `context.launch_configurations.clear()`。
- `rs_launch.py` 的警告来自遍历 `context.launch_configurations` 与 supported 比对，
  清空后自然无残留。
- `sensor_driver.launch.py` 现在已把 Reset 放在 scoped GroupAction 内、Include 之前。

离线模拟（注入 mode/start_livox/requested_mode 等 12 个顶层参数）：
残留顶层参数 0 个，unsupported 警告 0 条。且 Reset 的 14 个 key 在
`/opt/ros/humble/...`（79 项）与 `src/realsense-ros/...`（80 项）两个版本中全部受支持。

**结论：不动代码**，Thor 上线后跑完整启动确认 52 条 warning 消失即可。

### 10.3 结论修正：5.3 inflation 告警定性错了

- 数值是自洽的：`g1_config.py` 对 local/global 用同一循环写 `robot_radius` 并显式
  `pop('footprint')`；`inflation_radius = max(0.60, robot_radius+0.20)`，
  完整启动为 0.60 vs 半径 0.40。三条启动路径都满足 `inflation ≥ robot_radius`。
- 规划器是 **SmacPlanner2D**，走纯半径分支，**根本不做非圆形碰撞检查**，
  `planner_server` 块内没有任何 footprint / motion_model 项。
- 同一句告警文案也存在于 MPPI 的 `obstacles_critic.cpp`。而
  `nav2_params.yaml` 的 `critics` 列表里**包含 ObstaclesCritic**（尽管
  `enabled: false`），MPPI 对列表中所有 critic 都会 `initialize()`。
- 真实的不一致在这里：ObstaclesCritic 持有 costmap 膨胀层的镜像参数
  `inflation_radius: 0.4` / `cost_scaling_factor: 10.0`，与 costmap 生成值
  `0.60 / 3.0` 对不上，而 `g1_config.py` 从不同步这两个值。

**建议**：把 ObstaclesCritic 从 critics 列表移除（它本来就 disabled）。
**但必须先在 Thor 上确认告警行的 logger 名是 `ObstaclesCritic` 还是 `SmacPlanner2D`，
未验证前不要盲改。**

附带发现：`robot_radius` 有四套并存默认值（0.40 / 0.40 / 0.250 / 0.250 / 0.20），
走完整 `robot.launch.py` 会被逐层覆盖成 0.40，链路是通的；但单独跑
`g1_navigation_direct.launch.py` 得到 0.250，单独跑 `navigation2.launch.py`
会绕过 g1_config 得到 0.20。这是长期审计风险，建议后续统一。

### 10.4 新增：`/lightning/registered_points` 诊断桥（优先级2的替代实现）

**重要约束（用户 2026-09-17 明确要求）：不得修改 lightning-lm 核心代码，
新功能只能做在桥接层。** 因此没有去接 `localization.h` 里那个被注释掉的
`SetPointcloudWorldCallback` 脚手架。

新增文件：
```text
g1_nav_bridge/g1_nav_bridge/registered_cloud_bridge.py
g1_nav_bridge/launch/registered_cloud_bridge.launch.py
```
并在 `CMakeLists.txt` 增加 install 规则、`package.xml` 增加
`sensor_msgs_py` / `python3-numpy` 依赖。

做法：订阅 `/livox/points`（由 `livox_custom_to_pointcloud2` 提供，frame
`mid360_link`），按该帧自己的时间戳查 `map -> mid360_link` TF，刚体变换到 map 系，
体素降采样后发布 `/lightning/registered_points`。只读 TF，不发布任何 TF。

启动：
```bash
ros2 launch g1_nav_bridge registered_cloud_bridge.launch.py
```

**已知局限，用前必读**：
- 这里拿到的是驱动直出点云，**没有逐点去畸变**。Lightning 内部基于 CustomMsg
  `offset_time` 的 per-point deskew 在桥接层拿不到。静止/慢速时结论可信，
  快速旋转时单帧会被拉花，不能据此判定建图质量。
- 整帧用单个位姿做刚体变换。
- **仅供诊断，不可作为安全避障点云使用。**

离线验证（本机 ROS humble）：
- 纯函数测试 11/11 通过（四元数、重力 pitch、体素降采样含负坐标与保序、刚体变换）。
- 端到端测试通过：假 TF + 假点云 → 输出 frame_id 改写为 map、
  绕 Z 转 90° 加平移的坐标正确、同体素重复点由 3 合并为 2。
- launch 参数类型求值正确（数值经 `ParameterValue(value_type=float)` 转为 double，
  否则会与节点里 `declare_parameter` 的 float 类型冲突抛异常）。
- 参数校验路径验证通过（`tf_lag_sec` 为负、`global_frame` 为空均正确抛 ValueError）。

**未部署到 Thor，也未在真机数据上验证过。**

### 10.5 本轮未动的部分

- 4.1 shebang 修复、4.2 角速度 0.8~0.9：仍只在本地，未部署。
- 优先级1（录两种 bag）、优先级3（同一 bag 跑三组 LIO 参数对比）：需要 Thor 在线。
- 5.4 RealSense 偶发硬件警告、5.5 测试超时退出噪声：未处理（5.5 本就是
  测试主动 SIGINT 之后的现象，不是启动期错误）。

### 10.6 Thor 上线后的建议顺序

1. 部署 shebang 修复（纯 bug 修复，风险最低）。
2. 部署 10.1 的 keepout 条件化修复（同属 robot_bringup）。
3. 完整启动，核对三件事：keepout 噪声是否消失、RealSense 52 条 warning 是否消失、
   inflation 告警的 logger 名到底是谁。
4. 按 3 的结果决定是否执行 10.3 的 ObstaclesCritic 改动。
5. 角速度修改需**重新征得用户同意**后才能部署。
6. 录包与 LIO 参数对比。

---

## 11. 2026-09-17 第三轮：Thor 部署与文档勘误

### 11.1 【重要勘误】第 4 节"尚未部署"已过时

Thor 上线后直接核对，发现第 4 节两项**其实早已部署**：

```text
Thor: src/aid_navigation2/launch/g1_config.py:127
      max_rotational_vel=0.9, min_rotational_vel=0.8   ← 已在 Thor 上
Thor: src/g1_nav_bridge/g1_nav_bridge/sport_to_odom.py
      #!/usr/bin/env python3                            ← 已在 Thor 上
```

即 §4.1 shebang 修复和 §4.2 角速度修改都已生效。**接手者不要再把这两项当作待部署项。**

### 11.2 【重要勘误】第 10.2 节关于 RealSense 的结论不完整

第 10.2 节称"5.1 已修好、不用动代码"，**只对一半成立**。
`rs_launch.py` 有【两处独立】的 unsupported 校验：

```text
rs_launch.py:124-126   检查 launch 参数        ← 已修好，Thor 实测 0 条
rs_launch.py:129-132   检查 config 文件参数    ← 当时完全没验证
```

`realsense_g1.yaml` 里的参数走的是第二条路径。核对 RealSense 配置时
必须同时看这两条路径。

**但要特别注意**：第二条路径打印的警告【是误报，不代表参数无效】。
`rs_launch.py` 只拿它自己的 launch 参数清单做校验，而 config 文件是经
`parameters=[params, params_from_file]` 直接透传给节点的，节点真实的参数名
与 rs_launch 的清单并不一致。详见 11.3。

**不要根据这条警告删改 config 文件里的参数。** 判断参数是否真的生效，
唯一可靠的方法是在节点起来后执行：

```bash
ros2 param list /camera/camera | grep -i <关键字>
ros2 param get  /camera/camera <完整参数名>
```

### 11.3 相机点云 0 帧的根因（含一次错误结论的更正）

**【务必先读这条】参数前缀不是 `pointcloud.`，是 `pointcloud__neon_.`**

节点上点云滤波器的参数前缀取自 librealsense 给该滤波器的名字。Jetson/ARM64 启用
NEON 后名字是 "Pointcloud (NEON)"，被规范化成 `pointcloud__neon_`。Thor 实测：

```bash
ros2 param list /camera/camera | grep -i point
  pointcloud__neon_.enable
  pointcloud__neon_.stream_filter
  pointcloud__neon_.allow_no_texture_points
  pointcloud__neon_.ordered_pc
# 注意：pointcloud.enable 等名字在节点上【根本不存在】
```

**原始 `realsense_g1.yaml` 里的 `pointcloud__neon_` 前缀是正确的，原作者没写错。**

**曾犯过的错误**：`rs_launch.py` 会打印
`Parameter 'pointcloud__neon_.xxx' in config file is not supported`，
据此把前缀改成 `pointcloud.`，结果点云被彻底关掉、话题都不存在。
**那条警告是误报**——它只拿 rs_launch 自己的 launch 参数清单做校验，
不反映节点的真实参数名；本文件是经 `parameters=[params, params_from_file]`
直接透传给节点的，带 `__neon_` 前缀的参数确实生效。

**真正的根因：`stream_filter`**

```cpp
// realsense2_camera/src/pointcloud_filter.cpp:105-115
if (use_texture) {                       // stream_filter != 0 时为真
    texture_frame_itr = std::find_if(frameset.begin(), frameset.end(), ...);
    if (texture_frame_itr == frameset.end()) { ...; return; }   // 一帧都不发
}
```

depth 与 color 未同步进同一 frameset 时，发布函数每次都在这里提前 `return`，
所以是【0 帧】而不是低帧率；逐点纹理坐标计算则拖垮整个节点。
`stream_filter = 0`（`RS2_STREAM_ANY`）走无纹理分支
（`pointcloud_filter.cpp:218-239`），只判 `vertex->z > 0`，无提前 return，
输出纯 XYZ。STVL 标记障碍只要 XYZ，纹理是纯浪费。

**A/B 对照实验（Thor 实测，决定性证据）**：

```text
stream_filter = 0  ->  /camera/camera/depth/color/points  23.3~23.7 Hz
运行时 param set 改成 2  ->  立刻 0 帧
（改回 0 需重启相机才恢复，运行时改不足以恢复流）
```

**最终配置**（`robot_bringup/param/realsense_g1.yaml`，同时写两套前缀以便移植，
节点未声明的参数会被静默忽略）：

```yaml
pointcloud__neon_.enable: true          # ARM/NEON，Thor 实际生效的一套
pointcloud__neon_.stream_filter: 0
pointcloud__neon_.allow_no_texture_points: true
pointcloud__neon_.ordered_pc: false
pointcloud.enable: true                 # 非 NEON 构建的等价配置
pointcloud.stream_filter: 0
pointcloud.allow_no_texture_points: true
pointcloud.ordered_pc: false
```

**Thor 实测结果（2026-09-17）**：

```text
color  /camera/camera/color/image_raw        26.7~28.0 Hz
depth  /camera/camera/depth/image_rect_raw   22.3~27.8 Hz
points /camera/camera/depth/color/points     23.3~23.7 Hz   ← 修复前为 0 帧
启动时 unsupported 警告（launch 参数路径）   0 条
```

因为点云不再依赖 color，新增 launch 参数 `realsense_enable_color`
（默认 true，不改变现有行为）。纯避障可以：

```bash
ros2 launch robot_bringup robot.launch.py realsense_enable_color:=false
```

**两类无害噪声（不要误判为故障）**：

```text
1. Warning: Parameter 'pointcloud__neon_.xxx' in config file is not supported
   -> rs_launch.py 的误报，见上文，参数其实生效
2. No matching stream for texture 'Process - Any'. Set 'pointcloud.stream_profile'...
   -> stream_filter=0 时无纹理分支的 throttled 提示，正常
```

**排查经验（踩过的坑）**：

```text
1. pgrep -c realsense2_camera_node 恒返回 0
   -> 进程名超过 15 字符被截断，必须用 pgrep -f
   -> 曾因此误判“相机崩溃”，实际节点一直活着
2. pkill -f "start_cam" 会杀掉调用它的 ssh 会话
   -> 因为 ssh 的命令行里也含该字符串。用 pgrep -f "[s]tart_cam" 的方括号写法，
      或把清理逻辑写进 Thor 上的独立脚本再调用（见 /tmp/camctl.sh）
3. 两个相机节点并存 -> 第二个报 xioctl(VIDIOC_S_FMT) errno=16
   Device or resource busy，且会把 UVC 状态搞乱导致 depth 也停发
   -> 启动前务必确认只有一个实例
4. ros2 topic hz 用 14s 窗口测点云会读到 0
   -> 点云发布受订阅者数量门控（pointcloud_filter.cpp:93-95），
      需要预热。测点云至少给 30~40s 窗口
```

### 11.4 registered_cloud_bridge 已删除

第 10.4 节新增的桥接节点已按用户要求删除，改为在 lightning-lm 内部直接发布
（见 11.5）。用户同意为此放宽"不改核心代码"的约束。

### 11.5 新增 `/lightning/registered_scan`

发布配准后的 map 系点云（`sensor_msgs/PointCloud2`），**默认关闭**，
参数 `pub_registered_scan:=true` 开启。

```text
建图模式  core/system/slam.cc       PublishRegisteredScan()
          用 lio_->GetScanUndist() + lio_->GetState().GetPose()
          在 lio_->Run() 之后、关键帧判定之前调用（每帧都发）
定位模式  core/system/loc_system.cc + core/localization/localization.cpp
          复活作者原本注释掉的 SetPointcloudWorldCallback 脚手架
          发布的 (点云, 位姿) 与 ui_->UpdateScan() 完全同源
```

内容是 IMU 逐点去畸变后、送进配准的那份点云，按定位位姿摆到 map 系。
点类型 PointXYZIT，保留 intensity 与 time。

**刻意保留的既有偏差**：Lightning 投影关键帧和拼全局地图时用的都是
`state.GetPose() * 点`，**没有乘 lidar→IMU 外参**
（`extrinsic_T = [-0.011, 0.02329, -0.04412]`，约 5cm，见 `common/keyframe.h:22`
与 `laser_mapping.cc` 的 `ProjectKFs`）。发布时沿用同一约定，
保证点云与 Lightning 内部地图同源可叠加。若"修正"反而对不齐。
这是 Lightning 原有行为，不是新引入的。

**定位模式是跳帧的**：`lidar_loc_skip_num_ = 4`，该话题约为雷达帧率的 1/4；
建图模式为全帧率。

### 11.6 Lightning 内部确实做了去畸变与配准

为回答"Lightning 有没有做配准/去畸变"，只读核对结果：

```text
逐点时间戳    pointcloud_preprocess.cc:64   offset_time/1e6
IMU逐点去畸变 imu_processing.hpp:174        UndistortPcl，反向传播
              :295-300 p_compensate 末项乘 R_lidar_imu_^T
              → scan_undistort_ 位于【雷达系】(扫描结束时刻)
重力初始化    imu_processing.hpp:159        grav_ = -mean_acc/‖·‖*G
LIO配准       laser_mapping.cc:613 ObsModel 点面ICP + iVox最近邻 + ESKF
对先验图重定位 lidar_loc.h:189              NDT_OMP 粗/精两级，可选ICP精修
位姿图        core/localization/pose_graph/ 融合 LO 与 LidarLoc
```

但 Lightning 对外【只发布 `/map` (OccupancyGrid) 与 TF】，
全仓库 `create_publisher` 仅 `slam.cc:97` 一处。所有点云此前都只存在于进程内存，
这正是新增 11.5 话题的原因。


---

## 12. 2026-09-18：base_link 相对 map 倾斜（机器人站直但 TF 歪）

### 12.1 现象
机器人站直，`/base_link_pose`（= map->base_link）却为 roll≈-8.4°, pitch≈+3.3°；
`/lightning/registered_scan` 的地面在 map 系里倾斜约 6°。

### 12.2 排查结论（均为 Thor 实测）
| 检查 | 结果 | 说明 |
|---|---|---|
| IMU 重力（mid360_link） | (-0.047,-0.046,0.995) | 雷达相对重力 roll -2.66°, pitch +2.70° |
| 单帧点云地面法向（mid360_link） | (-0.052,-0.046,0.998) | 与 IMU 一致 → IMU、驱动旋转均正确 |
| 实时扫描：地面 vs 墙面推出的竖直 | 相差 0.42° | 扫描自身几何自洽 |
| 地图 new_map：地面 vs 墙面 | 相差 0.4~0.9°，整体倾斜 0.5° | 地图水平、未扭曲 |
| NDT 原始结果 | roll -7~-9.6°，静止时仍缓慢滑动 | **错在 NDT** |
| PGO / 高频输出 | 与 NDT 一致 | PGO 未引入误差 |
| 4DoF/6DoF ICP 多初值 | yaw 散布 8.6~14.2°，残差几乎不变 | 当前站位配准退化（周围 0.75~1 m 遮挡，墙面几乎单一朝向） |

另：`lidar_loc.cc` 的 `esti_balanced = guess * exp(0.1*delta)` 每周期只向 NDT 结果靠 10%，
所以 NDT 的偏置表现为"静止时位姿慢慢漂"。

### 12.3 修复
1. **URDF `base_to_mid360_joint`**：rpy 改为 `-0.046373 0.049515 0`（站立实测）。
   2026-09-17 按官方 -2.3° 改成负号是错的：官方值是倒装原始系下的，驱动绕 x 翻 180° 后绕 y 的角度变号。
2. **`lidar_loc.gravity_constrain: true`**（`lidar_loc.cc` `ApplyGravityConstraint`）：
   NDT 结果以最小旋转把 LIO 重力方向对到 map +z，保留航向与平移；init 与 track 两处都约束。
   单次修正 >15° 视为异常不修正。前提：地图 z 轴对齐重力。
3. **`fasterlio.gravity_align_init: true`**（`imu_processing.hpp` `IMUInit`）：
   LIO 世界系初始化即对齐重力。否则世界系=首帧 IMU 系，雷达倾斜多少新地图就倾斜多少，
   会破坏第 2 条的前提和 g2p5 按高度切 2D 栅格。定位只用 LIO 相对位姿，不受影响；
   **建图路径尚未实车跑过**，下次建图后请用地面/墙面法向核验地图倾斜 < 1°。
4. 诊断日志：NDT 每次结果与高频输出均打印 rpy（`lidar_loc.cc` confidence 行、`localization.cpp` "hf output"）。

### 12.4 修复后实测
| 项 | 修复前 | 修复后 |
|---|---|---|
| `/base_link_pose` rpy | (-8.43°, 3.25°) | (0.01°, -0.14°) |
| registered_scan 地面倾斜 | 6.17° | 0.36° |
| registered_scan 地面高度 | 0.126 m | -0.005 m |
| base_link z | 0.033 | -0.02 |

### 12.5 未解决 / 建议
- NDT 在当前站位仍偏好 roll≈-8.7°（被约束掉了），其 xy/yaw 也可能随之有偏；空旷、结构丰富处会好转。
- 雷达 0.35 m 内每帧约 600 个机器人头部自身点（`fasterlio.blind: 0.1` 滤不掉），建议评估 blind 提到 0.5。
- 若机器人站姿/头部角度改变，需按 12.2 前两行方法重测 URDF 的 roll/pitch。


---

## 13. 2026-09-18（下午）：前端"新建地图"无法进入建图

### 13.1 根因
前端建图流程：`mode_set mapping` → `mode_set remote_control` → 完成扫描 `map_save`(`/maps/<ts>`) → `saveMapDb`
→ 离开页面 `mode_set localization`。
`robot.launch.py` 给 robot_status_manager 写死 `manage_stack: False`（为避免两套定位/Nav2），
导致 `mode_set mapping` 直接被拒：
`Cannot switch process mode to 'mapping': SLAM/Nav2 are owned by robot.launch.py`。

### 13.2 修改（已部署编译，除 13.3 两项）
- `robot.launch.py`：start_backend:=true 时不再直接 include 定位/导航/建图，改由状态管理节点唯一拥有；
  `startup_mode`、`default_map_dir` 及 `localization/navigation/mapping_launch_args` 由 OpaqueFunction 透传
  （with_ui、start_rviz、pub_registered_scan、robot_radius、use_realsense_obstacles、use_collision_monitor、
  map_save_root、start_map_transform:=false 等），保证与顶层直接启动时一致。
- `robot_status_manager.cpp`：新增上述参数；按 startup_mode 启动（navigation/localization→定位+导航，
  mapping→建图，base→空闲）；定位地图优先数据库当前地图，否则 default_map_dir。
- `launch_manager.py`：启动前清理已退出的记录（否则永远 "already running"）；停止后 killpg 清孤儿。
- 实测（Thor）：启动后由状态管理节点在 test1（~/maps/1789468870702）上拉起定位+导航，进程各 1 份。

### 13.3 未部署（Thor 已关机）——下次开机先做
`mode_set mapping` 实测仍失败：`StopLaunch` 只等 5 s，而 launch_manager 要等 Nav2/Lightning 退出才回复。
已本地修改：`StopLaunch` 等待改 45 s；launch_manager 停止流程 SIGINT 10 s → 进程组 SIGTERM 3 s → SIGKILL。
部署：
```bash
rsync -a --relative src/robot_bringup/src/robot_status_manager.cpp src/aid_robot_py/aid_robot_py/launch_manager.py \
  unitree@192.168.128.146:/opt/G1/lighting_ws/
# Thor 上
colcon build --packages-select robot_bringup aid_robot_py
```
验证：启动后调用 `ros2 service call /mode_set aid_robot_msgs/srv/StatusChange "{action: mapping}"`，
应在约 30 s 内返回 ok，且 run_slam_online=1、run_loc_online=0、controller_server=0；
再调 `{action: localization}` 应切回。之后在前端走一遍新建地图。

### 13.4 同期其它改动（已部署并验证）
- **平面 base_link**：G1 两次站立头部俯仰相差约 3.5°（IMU 实测 +2.70° → -0.81°），静态 URDF 无法保证
  base_link 水平。URDF 根改为 `body_link`；LocSystem 发布 `map->base_link`（只含航向）与
  `base_link->body_link`（动态倾斜）、`base_link->base_footprint`（恒等）。实测 map->base_link rpy=(0,0,-5.8)，
  map->mid360_link 与定位输出一致。URDF 无 body_link 时自动退回 6DoF。mid360 rpy 恢复为官方 (0,+2.3°,0)，
  以保持与 d435 外参的相对关系（第 12 节写入的站姿实测值已撤销）。
- **地图重力方向**：旧地图世界系=首帧 IMU 系，实测 test1 倾斜 1.99°、new_map 0.19°、aaa≈6°。
  已为 test1、new_map 写 `gravity_up.txt`；其余点太少未写（不做重力约束）。建议用新配置重建 test1。
- 本次开机 D435 未在 USB 上枚举（lsusb 无 8086 设备，驱动报 No RealSense devices were found），需查线缆。
