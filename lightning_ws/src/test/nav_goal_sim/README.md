# Nav2 到点精度仿真（G1 运动学）

在 PC（ROS 2 humble + `ros-humble-navigation2`）上复现"导航报到达、人却停在偏处"，并比较改进方案。不是 colcon 包，直接运行脚本。

## 为什么会偏

`g1_nav_bridge/src/cmdvel_to_sport.cpp` 为了让 G1 能动，会改写 Nav2 的速度指令：

| 指令 | G1 实际执行 |
|---|---|
| 转向（\|wz\| ≥ 0.15） | wz 至少 0.6，**vx 至少 0.2**（不能原地转，走半径 0.22~0.5 m 的圆弧） |
| 前进 vx ≥ 0.05 | vx 至少 0.3（不能慢速蠕动） |
| vx < 0.05 且不转 | 完全停止 |

1. 原来的 SmacPlanner2D 只规划位置，终点朝向与来向不同时只能在终点"原地转"，G1 走成圆弧横移 0.4~0.6 m；
   到点判定 `stateful: true` 在位置第一次进入 0.2 m 时就锁定，所以照样报到达。
2. 到点停车时，速度平滑器按 0.6 rad/s² 慢慢减速，减速段被抬回 0.6 rad/s，停车后又多转 15~27°。

## 文件

| 文件 | 作用 |
|---|---|
| `g1_motion_sim.py` | 模拟 G1：订阅 `/cmd_vel`，套用 cmdvel_to_sport 的抬速规则 + 一阶速度响应，发 `/odom` 和 TF |
| `nav2_sim_params.yaml` | Nav2 参数（humble 写法）。控制器、到点判定、平滑器、半径与膨胀照抄 `aid_navigation2/param/nav2_params.yaml`；代价地图只有静态层 + 膨胀层 |
| `planner_2d.yaml` / `planner_hybrid.yaml` | 规划器：原来的 SmacPlanner2D / 改后的 SmacPlannerHybrid（DUBIN） |
| `decel_fast.yaml` | 速度平滑器快速减速（改后的值） |
| `goal_tight.yaml` | 到点位置容差 0.12 m（试验，未采用） |
| `sim_launch.py` | 起 map_server、planner/controller/smoother/behavior/bt_navigator（行为树用 Thor 的 xml）和 G1 仿真 |
| `run_goal_test.py` | 每个（变体, 场景, 重复）单独起一套仿真，发 NavigateToPose，记录结果和轨迹到 `results/<时间>/` |

## 运行

```bash
sudo apt install ros-humble-navigation2      # 只需一次
source /opt/ros/humble/setup.bash
cd lightning_ws/src/test/nav_goal_sim
python3 run_goal_test.py --reps 2 --jobs 4                       # 全部变体 × 全部场景
python3 run_goal_test.py --variants hybrid+fast --scenarios s1   # 只跑一组
```

地图默认 `/home/ap/G1_bags/maps/1790676615248/map.yaml`（`sim_launch.py` 的 `map` 参数可改）。

## 2026-10-03 结果（每格两次重复：位置误差 m / 朝向误差 °）

| 变体 | s1 事故复现 | s2 朝向与来向一致 | s3 回原点调头 | s4 朝向垂直 |
|---|---|---|---|---|
| 原配置（2d） | 0.36/26、0.35/27 | 0.18/4、0.16/5 | 0.47/23、0.47/23 | 0.39/14、0.35/15 |
| 只换规划器（hybrid） | 0.41/26、0.21/18 | 0.04/6、0.05/9 | 0.28/16、0.31/12 | 0.16/7、0.12/5 |
| 只加快减速（2d+fast） | 0.38/0、0.36/2 | 0.16/2、0.15/2 | 0.42/3、0.42/1 | 0.25/0、0.26/1 |
| **两项都改（hybrid+fast，已采用）** | **0.13/1、0.13/1** | 0.08/5、0.08/8 | 0.20/1、0.22/3 | **0.13/0、0.11/2** |
| hybrid+fast+容差 0.12 | 0.17/2、0.17/2 | 0.03/8、0.03/6 | 0.21/2、0.22/4 | 0.11/2、0.12/0 |

32+8 次全部 SUCCEEDED，到点用时各变体相近。剩下 0.1~0.2 m 是沿进场方向冲过头：
cmdvel_to_sport 把前进速度至少抬到 0.3 m/s，G1 没法慢速靠近终点，停车有距离。收紧到点容差基本没帮助，未采用。

## 局限

- 仿真里的 G1 是理想运动学 + 一阶响应（时间常数 0.25 s），没有步态、打滑和真实的停车距离；绝对误差以实机为准，变体之间的相对好坏可参考。
- 没有传感器障碍层、禁行区；只用 humble 版 Nav2（Thor 是 Jazzy），Jazzy 的参数写法以 `aid_navigation2/param/nav2_params.yaml` 为准。
