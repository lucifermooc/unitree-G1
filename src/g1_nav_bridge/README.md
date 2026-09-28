# G1 导航桥接与统一启动

本包是当前工作区唯一的 G1 导航桥接包。保留 Lightning-LM 的
`map → base_link → 传感器` 链路，**本包不发布任何 TF，不修改 SLAM 核心**。
Nav2 参数仍由 `aid_navigation2` 管理，前端由原 `robot_bringup` 等包管理，
不将这些完整包重复复制进来。

## 功能和数据流

| 功能 | 输入 | 输出/接口 |
| --- | --- | --- |
| `sport_to_odom` 平面里程计转换 | `/odommodestate`，`unitree_go/msg/SportModeState` | `/odom`，`nav_msgs/msg/Odometry`，无 TF |
| `tf_to_current_pose` 定位话题转换 | TF `map → base_link` | `/current_pose`，`geometry_msgs/msg/PoseStamped`，无 TF |
| `cmdvel_to_sport` ROS 速度桥 | `/cmd_vel_safe`，`geometry_msgs/msg/Twist` | 调用包内 `SetVelocity(vx, vy, omega, duration)` |
| `battery_state_bridge` 电池转换 | `/lf/bmsstate`，`unitree_hg/msg/BmsState` | `/battery_state`，`sensor_msgs/msg/BatteryState` |
| `g1_navigation.launch.py` | 地图 YAML、实测地面高度 | 启动现有 Nav2、碰撞监测及可选 bridge |

命令链：`Nav2 → /cmd_vel_nav → velocity_smoother → /cmd_vel → collision_monitor
→ /cmd_vel_safe → cmdvel_to_sport → 包内 G1LocoClient → /api/sport/request → G1`。
G1 通过 `/api/sport/response` 返回调用结果。客户端代码已随本包保存，不再包含或搜索
外部 `unitree_ros2` 头文件。ROS 回调只覆盖缓存最新速度并唤醒条件变量；独立工作线程
调用 `SetVelocity()`。`BaseClient::Call()` 只发布请求并立即返回，不再每次创建订阅或等待
5 秒；`cmdvel_to_sport` 使用常驻 `/api/sport/response` 订阅异步记录 7105 的本体返回码。
处理期间到达多条命令时只发送最新值，不积压旧速度。
`duration` 默认 0.50 s，必须大于正常 `/cmd_vel` 周期；话题断流后由机器人端在该
有效期结束时停止。桥不再实现第二套使能、FSM、限速、状态或急停逻辑；这些由
Nav2、collision monitor、机器人内置运控及硬件急停负责。NaN/Inf 被转换为零速。

`/odom` 的 pose 是本体开机参考系中的平面 XY/yaw，**不是 Lightning 地图中的位姿**。
header 为 `odom`，child 为 `base_link`，保留源时间戳，Z/roll/pitch 置零。
不在 bridge 启动时重新归零，也不假定那时 base_link 与 odom 重合。
目前 Nav2 controller/BT 只取其速度反馈；velocity_smoother 为 OPEN_LOOP。
不为 odom 添加静态 TF，也不把它伪装成 map。详见 [ODOMETRY.md](ODOMETRY.md)。

## 合并结果

旧 `g1_nav_integration` 是带 COLCON_IGNORE 的参考工程，其中还嵌套了另一个
同名 `g1_nav_bridge`。已移出 src，备份在工作区外层
`archive/nav_merge_20260911/g1_nav_integration`，可恢复。

- 保留/整合：参数分层、点云障碍物接入、速度输入有效性检查、统一启动和配置测试。
- 不使用：第二个速度桥接、pelvis/全关节 URDF、pose_bridge、位姿差分生成里程计、
  `/lightning/map_pose`/`lio_pose` 新接口、SLAM 核心 patch 和自动打补丁工具。
- 原前端的禁行区、任务与地图接口保持独立，未删除。
- 2026-09-21 起取消参数生成器（原 `aid_navigation2/launch/g1_config.py`）：
  Nav2 与 collision_monitor 的全部参数直接写在
  `aid_navigation2/param/nav2_params.yaml` 与
  `robot_bringup/param/collision_monitor_params.yaml` 里，launch 只按
  `use_keepout` / `use_realsense_obstacles` 删层，不再改写任何数值。

## 配置审查结果

`aid_navigation2/param/nav2_params.yaml` 已清理 AMCL、旧 Aurora/STVL、未启动的
waypoint 插件及重复 collision 配置。保留 MPPI、SmacPlanner2D、原 BT、自定义禁行区。
统一真机时间、map/base_link、/odom、0.40 m 初始测试半径和低速限制，补齐目标/进度检查器。
目标发行版只有 jazzy：插件名写 `::` 形式，bt_navigator 不列 plugin_lib_names，
controller_server 用复数 `progress_checker_plugins`（humble 需自行改回单数）。
自定义 `aid_costmap_plugin/KeepoutLayer` 是普通 Layer，仍放在 plugins 中，不改成 filters。

**YAML 不是无需现场参数即可安全行驶的承诺。** 地图原点可能在雷达处，
costmap 的高度过滤使用 map 坐标；collision_monitor 的高度使用 base_link 坐标。
平面模式下 map->base_link 的 z 恒为 0（base_link 即地面投影），两边高度基准一致，
所以 yaml 里的高度就是 map 系高度，不再需要 floor_z / base_floor_z 参数。
保留地面上方 0.10～1.80 m 的障碍物。
低于 10 cm 的障碍物不在此检测范围；台阶、坑洞及点云盲区也不是本配置的完整防护对象。
机器人半径需覆盖实际手臂/身体姿态，雷达自体回波及障碍物检测需要现场验证。

## 启动

### 临时不使用碰撞监测、尚未设置禁行线

停止旧导航任务并停止旧导航 launch 后，再使用下面入口；不能与旧 bridge 并行运行：

```bash
ros2 launch robot_bringup robot.launch.py mode:=navigation \
  nav_map:=/opt/G1/lighting_ws/data/new_map/map.yaml \
  floor_z:=0.0 base_floor_z:=0.0 \
  use_collision_monitor:=false use_keepout:=false
```

此例地面高度为 0，仍须符合实际地图。`use_collision_monitor=false` 不启动碰撞监测，
本入口新启动的 bridge 直接订阅速度平滑器的 `/cmd_vel`；不是仍等待 `/cmd_vel_safe`。
Nav2 障碍层、膨胀层仍保留，但不再有独立碰撞监测停车。bridge 启动后直接转发输入，
测试前必须确保没有意外速度发布者，并检查障碍物和硬件急停。
该参数不能修改已运行的 bridge；此测试方式请让本入口独占 bridge，勿设 start_bridge=false。

`use_keepout` 默认 false：不加载 local/global 的 keepout_layer，未设置禁行线时不再等待 mask。
这不是向缺失禁行区数据伪造“全空地图”。以后接通 `/keepout_filter_map` 发布端后设为 true。
`use_collision_monitor` 在统一入口中默认 false；完成独立验收后设 true 并重启导航入口。
底层 navigation2.launch.py 不使用这些开关，请使用本统一入口。

统一入口已启动 Nav2 `map_server`，将 `map.yaml` 引用的 `map.pgm` 发布到 `/map`。
默认地图路径为 `/opt/G1/lighting_ws/data/new_map/map.yaml`，不需要 Lightning
定位发布 `/map`，也不要再单独启动第二个 map_server。PCD/index.txt 由定位算法使用，
不是 map_server 的输入。

三个参数各自的作用：

- `map`：二维静态地图 YAML 路径；默认是上面的 Thor 路径，换地图时覆盖。
- `floor_z`：实际地面点云在 **SLAM map 三维坐标** 中的 Z，只用于障碍物高度过滤。
  OccupancyGrid 绘制在 z=0 不意味着三维地面就是 z=0。仅做雷达→base 外参转换
  不会改变 SLAM map 原点；若没有把整个 SLAM 世界原点平移到地面，地面 Z 仍可能为负。
- `base_floor_z`：地面在 base_link 中的 Z；当前 URDF 为地面投影，默认 0，无需每次传入。

平地上可查看 `ros2 run tf2_ros tf2_echo map base_link` 的 translation.z 作为
floor_z 的检查依据（前提是 base_link 确实在地面），并与地面点云核对。
若实测为 0，则启动 `ros2 launch robot_bringup robot.launch.py mode:=navigation floor_z:=0.0`；
若不是 0，填写实际值。此处不自动平移地图，不修改 TF。

先启动当前已验证的 Lightning **定位**、对应 URDF、Livox 驱动与
`point_filter`（`/livox/lidar` → `/livox/points`，按 tag 剔除噪点）。确认 `/livox/points` 是 PointCloud2，
frame 为 `mid360_link`，`map → base_link` 连续有效。
不要同时启动建图与导航 map_server 争抢 `/map`。

客户端实现及许可证均已放入 `g1_nav_bridge`，编译不再访问相邻的
`unitree_ros2-master` 或 `/home/unitree/unitree_ros2/example/src/include`。机器人端要求
话题类型名保持为 `unitree_api/msg/Request`、`unitree_api/msg/Response` 和
`unitree_go/msg/SportModeState`，因此这两个接口包必须是工作空间 `src` 下的独立 ROS 包，
不能嵌套进 `g1_nav_bridge` 后改名。当前工作空间已经包含所需定义，无需 source 或链接
外部 `unitree_ros2`：

```bash
source /opt/ros/humble/setup.bash
cd /home/lmw/project/unitree/G1/lightning_ws
colcon build --base-paths src --packages-up-to g1_nav_bridge --executor sequential
source install/setup.bash
```

真机运行时使用 Unitree 官方环境，即 `rmw_cyclonedds_cpp` 和连接机器人的网卡配置；
不要再额外加载 `/usr/local/lib` 的裸 SDK2 DDS 库，也不要启动旧 SDK 守护进程。
Unitree 网口是 `enP2p1s0`。每个真机启动终端先执行：

```bash
source /opt/ros/jazzy/setup.bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export CYCLONEDDS_URI='<CycloneDDS><Domain><General><Interfaces><NetworkInterface name="enP2p1s0" priority="default" multicast="default" /></Interfaces></General></Domain></CycloneDDS>'
source /opt/G1/lighting_ws/install/setup.bash
```

以下 **-1.20 只是例子**，请换成你当前定位地图中实测地面 Z。
base_link 如果确实定义在地面投影处，base_floor_z 为 0：

```bash
ros2 launch robot_bringup robot.launch.py mode:=navigation \
  map_dir:=/绝对路径/地图目录 nav_map:=/绝对路径/map.yaml \
  floor_z:=-1.20 base_floor_z:=0.0
```

整机运行统一从 `robot_bringup/robot.launch.py` 进入。顶层会启动 Lightning 定位，
并只包含一次本包的导航 launch；运动桥和可选 collision monitor 不会被重复创建。
顶层默认还会启动一次电池转换节点；如需诊断可传 `start_battery_bridge:=false`。
本包的 `g1_navigation.launch.py` 保留为内部原子 launch 和独立诊断入口，不再与
`robot.launch.py` 并列作为日常启动方式。
不要再并行启动旧 `navigation2.launch.py` 或本包内部 launch。前端当前没有速度遥控话题。

只启动 bridge 和定位话题转换（不启动 Nav2）：

```bash
ros2 launch g1_nav_bridge nav_bridge.launch.py
```

只读取里程计：`ros2 run g1_nav_bridge sport_to_odom`。
同一种节点只启动一份。

## /battery_state 标准电量输出

`robot.launch.py` 默认启动电量转换，也可以单独运行：

```bash
ros2 launch g1_nav_bridge battery_bridge.launch.py
ros2 topic echo /battery_state --once
```

换算规则为：BMS 总电压和单体电压从 mV 转为 V，电流从 mA 转为 A，SOC 从
`0~100` 转为 `0.0~1.0`。无容量数据的字段按 `BatteryState` 规范填写 NaN；SOH 百分比
不冒充故障枚举。ROS 话题 `/lf/bmsstate` 在底层 DDS 中显示为 `rt/lf/bmsstate`，二者
是同一数据通道。`unitree_hg/msg/BmsState` 的官方字段定义已作为独立接口包放在当前
工作空间 `src/unitree_hg`，部署时不依赖外部 `unitree_ros2`。

## /current_pose 定位输出

`nav_bridge.launch.py` 自动启动此节点，也可以单独运行：

```bash
ros2 run g1_nav_bridge tf_to_current_pose
ros2 topic echo /current_pose --once
```

节点以最高 50 Hz 非阻塞查询最新 `map → base_link`，仅在 TF 时间戳更新时发布，
保留源时间戳与完整 XYZ/四元数，不重复做雷达外参转换。
缺失或过期超过 0.5 s 的 TF 不发布，避免前端误认定位仍在更新。
可通过 ROS 参数修改 `global_frame`、`base_frame`、`output_topic`、`rate_hz`、`max_tf_age`。
`PoseStamped` 只有 header 和 pose，没有 child_frame_id；header.frame_id=map，
话题语义约定 pose 是 base_link 在 map 中的位姿。Nav2 仍使用 TF 查询定位，
此话题供前端/其他节点消费，不是 Nav2 新增的必要输入。

## 行驶前检查

```bash
ros2 topic hz /odom
ros2 topic echo /odom --once
ros2 run tf2_ros tf2_echo map base_link
ros2 lifecycle get /controller_server
ros2 lifecycle get /collision_monitor
ros2 topic info /cmd_vel_safe -v
```

RViz Fixed Frame 设 map，查看地图、原始点云、局部/全局 costmap。将真实障碍物
放入行走区域，确认 costmap 标记与碰撞监测停车都生效，再低速测试。
确认本体处于 SDK 允许速度控制的内置运控模式，检查时间同步及 /odom 速度正负号。
节点不会调用 `Start()`、`SetFsmId()` 或进入用户开发模式，也没有软件使能服务。

### SDK 最小运动验证

关闭 `robot.launch.py`、Nav2 和其他速度发布端，只启动原子桥 launch。它会启动包内
G1 API 速度桥以及里程计/定位转换，不启动 SDK 守护进程。确认机器人处于内置走跑运控，
由一人持硬件急停：

```bash
ros2 launch g1_nav_bridge nav_bridge.launch.py cmd_vel_topic:=/g1_test_cmd_vel
```

随后一次只测试一个方向。停止发布后，本体应在 `duration=0.20` 秒到期时停止：

```bash
# 前进、后退
timeout 0.6s ros2 topic pub -r 20 /g1_test_cmd_vel geometry_msgs/msg/Twist \
  '{linear: {x: 0.10}}'
timeout 0.6s ros2 topic pub -r 20 /g1_test_cmd_vel geometry_msgs/msg/Twist \
  '{linear: {x: -0.10}}'

# 原地左转、右转；不是横移
timeout 0.6s ros2 topic pub -r 20 /g1_test_cmd_vel geometry_msgs/msg/Twist \
  '{angular: {z: 0.15}}'
timeout 0.6s ros2 topic pub -r 20 /g1_test_cmd_vel geometry_msgs/msg/Twist \
  '{angular: {z: -0.15}}'
```

速度范围由上游 Nav2 参数和 Unitree 接口裁剪。首次只发低速，并保持实体急停在手。
若接口返回 0 但低速不动，再单独验证本体速度死区，不能直接归因于通信失败。

odom 断流目前报警但不是独立运动联锁；不能以“收到 /odom”作为整机安全验收。
map 直接驱动局部 costmap 在定位跳变时可能不连续，当前按约定保留此方案。
本地静态测试/编译不等于 Thor Jazzy 实机导航已验收。

## 本次验证

- 包内 G1 客户端由 Unitree BSD-3-Clause 示例所需部分移植并精简；许可证随包安装。
- 仅 source `/opt/ros/humble` 的隔离构建中，工作空间内 `unitree_api` 与
  `g1_nav_bridge` 编译通过；没有使用相邻 `unitree_ros2-master` 的头文件或 install。
- 本地模拟服务确认单条非零命令只触发一次 API 7105，请求参数为
  `velocity=[0.5,0,0]`、`duration=0.2`；没有固定频率轮询或额外状态机。
- 既有里程计、定位和 G1 Nav2 配置测试保留；旧 Python API 运动桥测试已随实现移除。
- 本机隔离 DDS 域 187、ROS_LOCALHOST_ONLY=1、start_bridge=false：提供测试 TF 后，
  map_server、controller_server、planner_server、bt_navigator、velocity_smoother、
  collision_monitor 均进入 active；测试已结束，未连接/驱动机器人。
- 未验证：Thor Jazzy 二进制运行、真实障碍物停车、真实里程计反馈闭环、前端完整导航切换。

Jazzy 标准行为插件命名依据：
[Nav2 Behavior Server 官方参数说明](https://docs.nav2.org/jazzy/configuration_and_development/configuration_guide/core_servers/configuring_behavior_server/)。
自定义禁行区名称以本仓库 `aid_costmap_plugin/costmap_plugins.xml` 为准。
