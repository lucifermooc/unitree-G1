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
3. 安装 Nav2、slam_toolbox、pointcloud_to_laserscan、xacro、teleop 等。
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

## 已知限制

- Go2 是运动学替身，不能用来练步态和运动控制。想要真正走路，可以接 CHAMP 或强化学习策略（参考 [unitree-go2-ros2](https://github.com/anujjain-dev/unitree-go2-ros2)，但它基于 Gazebo Classic），或者用 Isaac Sim 的 [go2_omniverse](https://github.com/abizovnuralem/go2_omniverse)（需要 NVIDIA RTX 显卡）。
- 两台小车不能上楼梯，只在一楼活动。
- 每次只能放一台机器人。

## 下一步可以做的

- 用 `/livox/points` 和 `/livox/imu` 跑 3D 激光 SLAM（Lightning-LM、FAST-LIO），做一张跨两层的 3D 地图。
- 用 3D 定位结果替代楼层管理里的仿真真值。
- 在 `gen_world.py` 里加动态障碍、斜坡、窄通道，测试 Nav2 的参数。
