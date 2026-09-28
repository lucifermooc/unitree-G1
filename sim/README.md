# 全流程仿真：语义地图 + Cartographer + Nav2（TurtleBot3）

在仿真里把整条链路跑通：**建图 → 定位 → 一句话 → 语义地图查坐标 → Nav2 导航 → 到达对应点位**。
等真机（G1）准备好后，这套流程只需要把 TurtleBot3 换成 G1，并换成真实地图和真实点位。

```
                     ┌────────────── 仿真（Gazebo，TurtleBot3 World）──────────────┐
                     │   激光雷达 /scan、里程计 /odom          ←  速度指令 /cmd_vel   │
                     └───────────┬──────────────────────────────────────▲───────────┘
第 1 步 建图                     │                                      │
  auto_explore.py 自动开车 ──────┤                                      │
  Cartographer（建图模式）──→ maps/tb3_world.pbstream + .pgm/.yaml      │
                                 │                                      │
第 2 步 定位 + 导航               ▼                                      │
  Cartographer（纯定位，加载 .pbstream）── map→odom ──→ Nav2 ───────────┘
  map_server（加载 .yaml）─────── /map ──────────────→ Nav2
                                                          ▲ 目标点 PoseStamped
第 3 步 语义导航                                          │
  "我想喝水" → BGE-M3 → Qdrant → 茶水间 (x, y, yaw) ──────┘   semantic_map/semantic_nav.py
```

## 文件说明

| 文件 | 作用 |
|---|---|
| `1_build_map.sh` | 第 1 步：启动仿真，用 Cartographer 建图，机器人自动探索，保存地图 |
| `2_semantic_nav_test.sh` | 第 2 步：Cartographer 纯定位 + Nav2，跑语义导航全流程测试 |
| `semantic_map_tb3.json` | 仿真用的语义点位（地点名称、描述和正式数据一样，坐标来自仿真地图） |
| `maps/` | 建好的地图：`.pbstream` 给 Cartographer 定位，`.pgm/.yaml` 给 Nav2 和人看 |
| `config/tb3_mapping.lua`、`tb3_localization.lua` | Cartographer 建图 / 纯定位配置 |
| `config/nav2_params.yaml` | Nav2 参数 |
| `launch/tb3_world_headless.launch.py` | 无界面启动 TurtleBot3 World 仿真 |
| `launch/localization_nav.launch.py` | Cartographer 纯定位 + map_server + Nav2 |
| `scripts/auto_explore.py` | 建图时自动避障开车 |
| `scripts/semantic_nav_test.py` | 全流程测试：逐句导航并检查到达误差 |
| `env.sh`、`common.sh` | 环境变量和脚本公用函数 |
| `install_ros_cloud.sh` | 云端（没有 ROS apt 源）用 conda 安装 ROS2 的方法 |

语义导航的核心程序在 `semantic_map/semantic_nav.py`，它不只用于仿真：真机上只要跑着 Nav2，就能直接用。

## 在你自己的 Ubuntu 24.04 电脑上运行

1. 安装 ROS2 Jazzy（按官方文档 <https://docs.ros.org/en/jazzy/Installation/Ubuntu-Install-Debs.html>），然后：
   ```bash
   sudo apt install ros-jazzy-navigation2 ros-jazzy-nav2-bringup ros-jazzy-cartographer-ros \
     ros-jazzy-turtlebot3-gazebo ros-jazzy-turtlebot3-cartographer ros-jazzy-turtlebot3-navigation2
   ```
2. 语义地图的依赖装到 ROS 用的 Python 里（导航程序要同时 import rclpy 和语义地图）：
   ```bash
   pip install --break-system-packages torch --index-url https://download.pytorch.org/whl/cpu
   pip install --break-system-packages -r semantic_map/requirements.txt
   ```
3. 启动 Qdrant：`cd semantic_map && docker compose up -d`
4. 跑两步：
   ```bash
   bash sim/1_build_map.sh          # 建图，约 5 分钟
   bash sim/2_semantic_nav_test.sh  # 全流程测试
   ```

想看画面：把 `launch/tb3_world_headless.launch.py` 换成官方的 `ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py`（带 Gazebo 窗口），再开 `rviz2` 看地图和路径。

## 手动玩一玩

仿真和导航起来之后（`2_semantic_nav_test.sh` 里的第 2、3 步），另开一个终端：

```bash
source sim/env.sh
export SEMANTIC_MAP_DATA=$PWD/sim/semantic_map_tb3.json SEMANTIC_MAP_COLLECTION=semantic_map_tb3
cd semantic_map && python3 semantic_nav.py --sim      # 交互模式，输入"我想喝水"等
```

## 云端环境遇到的问题和处理（本地 apt 安装一般不会遇到）

- **ROS 官方 apt 源被网络策略拦截** → 用 RoboStack（conda 打包的 ROS2 Jazzy）安装，见 `install_ros_cloud.sh`。
- **没有显卡，Gazebo 激光雷达渲染崩溃** → 装 Mesa 软件渲染，`env.sh` 里自动设置 `__EGL_VENDOR_LIBRARY_FILENAMES`。
- **RoboStack 版 cartographer_node 启动报 "flag was defined more than once"** → 打包问题，`install_ros_cloud.sh` 里对库文件做了修补。
- **RoboStack 版 Cartographer 的 Lua `include` 读取出错** → 配置文件做成不带 include 的"展开版"（`config/*.lua`），apt 版用同样的文件也没问题。
- **Nav2 速度指令类型和 TurtleBot3 仿真不一致**（Twist vs TwistStamped）→ `nav2_params.yaml` 里设置 `enable_stamped_cmd_vel: true`。
