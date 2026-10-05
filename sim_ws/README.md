# 导航仿真平台（ROS 2 Humble + Gazebo Harmonic）

一个场景、三个机器人，用来练 Nav2、2D 建图和跨楼层导航：

| 机器人 | `robot:=` | 底盘 | Nav2 规划 / 控制 | 能上楼梯 |
|---|---|---|---|---|
| Unitree Go2 机器狗 | `go2` | 外观是 Go2 站立姿态，隐藏的四轮滑移转向 | NavFn(A*) / DWB | ✅ |
| 差速小车 | `diff` | 两轮差速 + 前后万向球 | NavFn(A*) / DWB | ❌ |
| 阿克曼小车 | `ackermann` | 后驱、前轮转向，最小转弯半径约 0.8 m | Smac Hybrid-A* / Regulated Pure Pursuit | ❌ |

三台都装 **Livox Mid-360**（近似模型）和 IMU，话题、坐标系完全一样，换机器人只改一个参数。

## 一键安装

要求：Ubuntu 22.04，已装 ROS 2 Humble（没装就加 `--install-ros`）。

```bash
cd sim_ws
./install.sh
```

脚本会做这些事（在全新的 Ubuntu 22.04 容器里验证过）：

1. **修 ROS apt 源**：2025 年 ROS 更换了签名密钥，旧机器 `apt update` 会报 `NO_PUBKEY`。脚本改用官方的 `ros2-apt-source`，旧的源文件会备份成 `.bak`。
2. 添加 OSRF 源，安装 **Gazebo Harmonic** 和 Harmonic 版的 `ros_gz`（`ros-humble-ros-gzharmonic`）。
   Humble 官方搭配的是 Fortress，两套 `ros_gz` 互相冲突，**脚本会卸载 Fortress 版的 `ros-humble-ros-gz*`**。
3. 安装 Nav2（含 MPPI、STVL 插件）、slam_toolbox、pointcloud_to_laserscan、xacro、teleop 等。
4. 从 [unitree_ros](https://github.com/unitreerobotics/unitree_ros)（BSD-3）下载 Go2 网格，约 25 MB。下载失败时会改用简化外观，不影响使用。
5. `colcon build`。

装完后每个终端：

```bash
source /opt/ros/humble/setup.bash && source ~/你的路径/sim_ws/install/setup.bash
```

## 怎么用

```bash
# 1. 只开仿真 + 键盘遥控（另开一个终端运行 teleop）
ros2 launch robot_nav_sim sim.launch.py robot:=go2 world:=two_floor
ros2 run teleop_twist_keyboard teleop_twist_keyboard

# 2. 2D 建图（slam_toolbox），遥控转几圈后保存
ros2 launch robot_nav_sim mapping.launch.py robot:=diff world:=flat
ros2 run nav2_map_server map_saver_cli -f ~/my_map      # 偶尔第一次会超时，再运行一次即可

# 3. Nav2 导航：默认加载生成好的真值地图，出生点已设为 AMCL 初始位姿
#    在 RViz 里用 "2D Goal Pose" 点目标
ros2 launch robot_nav_sim navigation.launch.py robot:=ackermann world:=two_floor
ros2 launch robot_nav_sim navigation.launch.py robot:=go2 map:=$HOME/my_map.yaml   # 用自己建的图

# 4. 跨楼层导航（只有 Go2）
ros2 launch robot_nav_sim navigation.launch.py robot:=go2 world:=two_floor
ros2 run robot_nav_sim go_to --floor 2 --x 3.0 --y 2.0
ros2 run robot_nav_sim go_to --floor 1 --x 3.0 --y 4.2
```

常用参数：`gui:=false` 不开 Gazebo 窗口，`rviz:=false` 不开 RViz，`x:= y:= yaw:=` 改出生点，`sim:=false` 只启动建图或导航（仿真已经在运行时用）。

## 场景

建筑内部 16 m × 10 m，两层，层高 2.6 m。坐标就是 `map` 坐标，单位米。

```
 y
10 ┌──────────────────────────────────────────┐   一楼：西侧房间（出生点 3, 4.2）
   │            ║ ▓▓▓▓▓▓▓ 楼梯 ▓▓▓▓▓▓▓▓▓▓▓▓▓▓ │         中间大厅 + 两根柱子
 8 │            ║━━━━━━━━━━━━━━━━━━━━━━━━━━━━━│         东侧两个房间
   │  沙发      ║                         柜子│   楼梯：沿北墙由西向东上楼，
   │            ║     ■                       │         15 级台阶，坡度 24.9°
 5 │                          ━━━━━━  ━━━━━━━│         入口 (6.9, 9.15)，二楼出口 (14.4, 9.15)
   │            ║                             │
   │  箱子      ║     ■              桌子      │   二楼：另一套隔墙和家具，
 0 └──────────────────────────────────────────┘         楼梯开口有栏杆
   0            5                            16 x
```

- `worlds/two_floor.sdf`：两层楼，带楼梯
- `worlds/flat.sdf`：只有一层，没有楼梯，适合练建图
- `maps/floor1|floor2|flat.yaml`：从场景几何直接生成的**真值地图**（只画离地 0.1~1.0 m 的障碍，和 `/scan` 一致）
- `config/floors.yaml`：楼层高度、每层地图、楼梯入口和出口位姿

改场景只改 `src/robot_nav_sim/tools/gen_world.py`，然后运行 `python3 tools/gen_world.py`，世界文件、地图、楼层配置会一起更新。

## 接你自己的 Lightning-LM + Nav2（推荐）

上面的 AMCL / slam_toolbox 是本包自带的"独立模式"，不依赖任何外部代码，拿到就能跑。
如果你的 `lightning_ws`（Lightning-LM + aid_navigation2）已经调好，用 `lightning.launch.py`：
**仿真只顶替传感器驱动**，建图、定位、导航全部用你自己的代码和参数。

```
Gazebo ──/livox/points──> livox_bridge ──/livox/lidar (Livox CustomMsg)──┐
       ──/livox/imu ─────────────────────────────────────────────────────┤
       ──/odom（不发 TF）──> LocGuard / Nav2                              ├─> 你的 Lightning-LM（g1_online.launch.py）
       ──/clock                                                          │      map -> base_link -> body_link
                                                                         │      /lightning/registered_scan
你的 scan_range_filter <─────────────────────────────────────────────────┘
       ──/lightning/registered_scan_nav──> 你的 Nav2（navigation2.launch.py + nav2_params.yaml，MPPI）──/cmd_vel──> Gazebo
```

前提：本机 `lightning_ws` 已经编译好（含 `lightning`、`livox_ros_driver2`、`aid_navigation2`、`aid_costmap_plugin`、
`g1_nav_bridge`）。你的 Nav2 要用的 MPPI 和 STVL（`mid360_voxel_layer`）插件 `install.sh` 已经装好。

每个终端先 source 你的工作空间，再 source sim_ws（顺序不能反，`livox_bridge` 要用你的 `livox_ros_driver2` 消息）：

```bash
source /opt/ros/humble/setup.bash
source ~/lightning_ws/install/setup.bash
source ~/sim_ws/install/setup.bash
```

```bash
# 1. 建图：Lightning run_slam_online，遥控走一圈
ros2 launch robot_nav_sim lightning.launch.py mode:=mapping robot:=go2 world:=flat
ros2 run teleop_twist_keyboard teleop_twist_keyboard
ros2 service call /lightning/save_map lightning/srv/SaveMap "{map_id: sim_go2_flat}"   # 存到 ~/maps/sim_go2_flat

# 2. 定位 + 导航：Lightning run_loc_online + 你的 Nav2，RViz 里点 "2D Goal Pose"
ros2 launch robot_nav_sim lightning.launch.py mode:=navigation robot:=go2 world:=flat
#    地图目录默认 ~/maps/sim_<robot>_<world>，也可以 map_dir:=~/maps/xxx

# 只要定位不要导航
ros2 launch robot_nav_sim lightning.launch.py mode:=localization robot:=go2 world:=flat
```

**你的文件一个都没改**。仿真需要的差异都是 launch 时生成临时文件来覆盖：

| 项目 | 真机 | 仿真里怎么处理 |
|---|---|---|
| 雷达驱动 | `livox_ros_driver2` | 不启动（`start_livox:=false`），`livox_bridge` 把仿真点云转成同样的 CustomMsg 发到 `/livox/lidar` |
| URDF | G1 的 URDF | 仿真机器人 URDF，根改名为 `body_link`，雷达是 `mid360_link`，交给你的 `g1_online.launch.py` 启动 robot_state_publisher |
| Lightning 配置 | `default_livox.yaml` | 读你的 `default_livox.yaml`，只把 `extrinsic_T/R` 改成零/单位阵（真机驱动做过 Rx(π) 翻转，仿真没有）；`floor_height` 按雷达离地高度给 |
| 仿真时钟 | — | Lightning 可执行文件用 gflags 解析参数，不能带 `--ros-args`，所以节点起来后用 `ros2 param set /lightning_slam use_sim_time true` 打开 |
| 机器人外形 | G1：`robot_radius 0.50`（为覆盖 D435 盲区放大） | Go2 0.33 m、差速车 0.32 m，`inflation_radius` 按你的规则 = 半径 + 0.20，MPPI `ObstaclesCritic.inflation_radius` 同步；阿克曼车用多边形足迹。`robot_radius:=0.50` 可还原 G1 的值（1.2 m 门洞会过不去） |
| Nav2 参数 | `nav2_params.yaml`（Jazzy 写法） | 读你的文件，生成仿真版：删掉 D435 的 `stvl_voxel_layer` 和 `keepout_layer`（和你 launch 里 `use_realsense_obstacles:=false use_keepout:=false` 一样），全部 `use_sim_time: true`，插件名 `::` 改成 Humble 的 `/`，补上 Humble 的单数 `progress_checker_plugin` |
| 阿克曼车 | — | MPPI 的 `motion_model` 换成 `Ackermann`，最小转弯半径 0.85 m |

注意：

- 阿克曼车不能原地转，建图时用键盘遥控走大弧线；也可以像测试里那样直接用差速车的地图，把出生点挪到雷达和建图起点重合的位置。
- **出生点必须和建图时一样**。Lightning 定位从建图起点开始，换了出生点要在 RViz 里用 "2D Pose Estimate" 发 `/initialpose`（你的 LocSystem 支持）。
- Lightning 出的 2D 栅格（g2p5）是把所有高度压到一张图上的，**two_floor 场景两层会叠在一起**，所以 Lightning 模式下 2D 导航先在 `flat` 场景用；跨楼层还是用上面的独立模式。3D 点云地图本身是完整的两层。
- 你的 `nav2_params.yaml` 是按 G1 调的（`robot_radius 0.50`、MPPI 速度上限等）。Go2 / 小车比 G1 小，窄处可能过不去，这是参数问题，不是仿真问题。

## 话题和坐标系

| 话题 | 类型 | 说明 |
|---|---|---|
| `/cmd_vel` | Twist | 速度指令（阿克曼车按 v 和 ω 换算前轮转角） |
| `/odom` | Odometry | 轮式里程计（2D），同时发布 TF `odom → base_footprint` |
| `/livox/points` | PointCloud2 | Mid-360 点云，frame `mid360_link`，字段 x y z intensity ring |
| `/livox/imu` | Imu | 200 Hz，frame `mid360_link` |
| `/scan` | LaserScan | 从点云里切出离地 0.1~1.0 m 的一层，frame `base_footprint` |
| `/ground_truth` | Odometry | 仿真真值（3D），只用于楼层自动识别和评估定位误差 |
| `/current_floor`、`/floor_request` | Int32 | 楼层管理 |

TF：`map →(AMCL / slam_toolbox) odom →(轮式里程计) base_footprint → base_link → mid360_link`

## 设计取舍（为什么这么做）

- **Go2 不是真的迈腿走**：腿是固定的站立姿态，运动靠藏在身体下面的四个轮子。练 Nav2 和 SLAM 关心的是传感器、TF、里程计和 `cmd_vel`，腿部动力学会让仿真变脆弱，尤其是爬楼梯。CMU 的 [autonomy_stack_go2](https://github.com/jizhang-cmu/autonomy_stack_go2) 仿真也是同样的思路。
  隐藏底盘调过三轮，下面是踩过的坑：
  - **中间两轮加前后万向球**：在坡脚会被前球顶起，驱动轮悬空，然后打转翻车。
  - **轴距比轮距大的四轮**：横向摩擦把原地转向锁死。
  - **真实高度的质心加短轴距**：在 25° 楼梯上向后翻。
  最终方案是：轮距 0.36 m、轴距 0.24 m、摩擦系数 0.8，质心下移，等效轮距标定为 0.49 m。
- **楼梯是"看起来是台阶、踩上去是斜坡"**：外观是 15 级台阶，雷达看到的是台阶；碰撞体是一段隐藏的光滑斜坡，轮子能爬上去。
- **Mid-360 是近似模型**：用 `gpu_lidar` 做规则扫描（水平 360° 共 900 点，垂直 -7°~52° 共 32 线，10 Hz），真机是非重复扫描。需要更逼真的非重复扫描和 Livox CustomMsg 输出，可以换 [mid360_gazebo_harmonic](https://github.com/ashduwihch/mid360_gazebo_harmonic)。
- **跨楼层用"每层一张 2D 地图"**：Nav2 是 2D 的，所以每层一张地图，三层分工如下：
  - `go_to` 负责把机器人送到楼梯口，再用 IMU 俯仰角判断上下楼梯的过程；
  - `floor_manager` 负责切换地图，并在楼梯口给 AMCL 重设初始位姿；
  - 遥控上下楼时，`floor_manager` 按仿真真值高度自动换图（`auto_floor:=false` 可关闭）。这是仿真里的捷径，真机要换成楼梯口的已知位姿或 3D 定位。

## 测试结果（Ubuntu 22.04 容器，无界面，CPU 软件渲染）

| 项目 | 结果 |
|---|---|
| 传感器 | 点云 32×900，IMU 200 Hz，里程计 50 Hz（CPU 渲染时点云约 7 Hz，有显卡时是 10 Hz） |
| Go2 爬楼梯 | 上楼、下楼都通过，全程不侧偏，约 18 秒 |
| Go2 原地转向 | 指令转 180°，实际转 176°，里程计记 168° |
| Nav2 一楼导航 | Go2、差速车、阿克曼车都到达 (13, 7)，用时 21~24 秒；定位误差 0.12~0.23 m |
| 跨楼层 | 1 楼 → 2 楼 (3, 2) 成功，到点定位误差 0.12 m；2 楼 → 1 楼成功，误差 0.08 m |
| 建图 | slam_toolbox 在平面场景建图，`map_saver_cli` 保存成功 |
| Lightning 建图 | Go2 在 flat 场景按 9 个航点绕一圈（约 60 m），`run_slam_online` 全程正常，`/lightning/save_map` 保存成功，2D 栅格和场景一致；回到起点时和真值差 0.07~0.09 m |
| Lightning 定位 + 你的 Nav2 | `run_loc_online` + MPPI + SmacPlanner2D + 你的行为树。Go2、差速车各自建图后，从 (3, 4.2) 穿过 1.2 m 门洞到 (13, 7)，都是 28 秒到达；按时间戳对齐后定位误差 0.01~0.15 m（含建图本身的漂移） |
| 阿克曼车 | 复用差速车的地图（出生点 `x:=2.55`，让雷达和建图起点重合），MPPI 阿克曼模型到达 (10.5, 3.0)，用时 25.5 秒，误差 1~2 cm |

## 已知限制

- Go2 是运动学替身，不能用来练步态和运动控制。想要真正走路，可以接 CHAMP 或强化学习策略（参考 [unitree-go2-ros2](https://github.com/anujjain-dev/unitree-go2-ros2)，但它基于 Gazebo Classic），或者用 Isaac Sim 的 [go2_omniverse](https://github.com/abizovnuralem/go2_omniverse)（需要 NVIDIA RTX 显卡）。
- 两台小车不能上楼梯，只在一楼活动。
- 每次只能放一台机器人。

## 下一步可以做的

- Lightning 模式下按楼层切 2D 栅格（按 z 分段生成 g2p5），把跨楼层导航也换成 Lightning 定位。
- 在 `gen_world.py` 里加动态障碍、斜坡、窄通道，测试 Nav2 的参数。
