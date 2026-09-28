下面按 **Thor `/opt/G1/lighting_ws`** 给你整理。**建图和定位二选一，不要同时启动。** 命令按当前本地源码核对；新增导航入口需要先同步、编译到 Thor。

## 0. 每个终端先加载环境

```bash
source /home/unitree/unitree_ros2/setup.sh
source /opt/G1/lighting_ws/install/setup.bash
cd /opt/G1/lighting_ws
```

UI、RViz 请在 Thor 图形桌面的终端启动；普通 SSH 终端不一定能显示窗口。

### 新机器首次部署：RealSense SDK

RealSense SDK（librealsense 2.58.4）已随仓库放在 `src/librealsense`，colcon 会先编它再编 `realsense2_camera`，不依赖系统里另装的 SDK，也不需要联网下载第三方库。
这份 SDK 固定 `BUILD_WITH_NEON=OFF`，所以点云参数是标准的 `pointcloud.*`。新机器只需要做一次：

```bash
sudo apt install -y libusb-1.0-0-dev libudev-dev libssl-dev pkg-config
cd /opt/G1/lighting_ws/src/librealsense && sudo ./scripts/setup_udev_rules.sh   # 相机访问权限，插拔一次相机生效
```

检查相机驱动链接的是仓库里的 SDK（不是 `/usr/local/lib`）：

```bash
ldd install/realsense2_camera/lib/librealsense2_camera.so | grep librealsense2
# 应指向 /opt/G1/lighting_ws/install/librealsense2/lib/...
```

如果之前编过 `realsense2_camera`，加入 SDK 后第一次要带 `--cmake-clean-cache` 重编它，否则 CMake 会沿用缓存里的系统 SDK 路径。

### 新机器首次部署：DDS 接收缓冲（建图必需）

Ubuntu 默认 UDP 接收缓冲只有 208 KB，装不下一帧 Livox 点云；不改的话建图时 IMU 会被内核丢掉，快转后地图出现多层墙。只需做一次：

```bash
sudo cp /opt/G1/lighting_ws/src/robot_bringup/system/60-dds-buffers.conf /etc/sysctl.d/
sudo sysctl --system
sysctl net.core.rmem_max   # 应为 33554432
```

改完需重启整套栈（缓冲大小在进程创建 socket 时确定）。

## 1. 建立地图

终端①：

```bash
ros2 launch lightning g1_mapping.launch.py \
  start_livox:=true \
  with_ui:=true \
  with_2dui:=true \
  start_rviz:=true
```

这会启动：URDF、Livox 驱动、实时建图、算法 UI、二维地图窗口和 RViz。

终端②，启动点云转换，供 RViz 显示：

```bash
ros2 run lightning livox_custom_to_pointcloud2
```

RViz 添加：

- `Map`：`/map`
- `PointCloud2`：`/livox/points`
- 原始点云单独检查时，Fixed Frame 可设 `mid360_link`；在 map 中显示则需要有效的对应 TF。

### 保存地图

另开终端：

```bash
ros2 service call /lightning/save_map lightning/srv/SaveMap \
  "{map_id: 'new_map_02'}"
```

因为建图是在 `/opt/G1/lighting_ws` 下启动，保存目录是：

```text
/opt/G1/lighting_ws/data/new_map_02/
```

**同名目录会被代码删除后重建，所以这里用了新名字，避免覆盖已有 `new_map`。** 确认保存完成后，再 Ctrl+C 停止建图。

## 2. 使用现有地图定位

终端①，使用你已有的 `new_map`：

```bash
ros2 launch lightning g1_localization.launch.py \
  map_path:=/opt/G1/lighting_ws/data/new_map \
  start_livox:=true \
  with_ui:=true \
  start_rviz:=true
```

注意：`map_path` 是包含 `index.txt`、PCD 的**目录**，不是 `map.yaml`。

终端②，点云转换若未运行：

```bash
ros2 run lightning livox_custom_to_pointcloud2
```

如果 Livox 驱动已经单独运行，把 `start_livox:=true` 改成 `false`，不要重复启动。

## 3. 启动 Nav2

先确认定位 TF：

```bash
ros2 run tf2_ros tf2_echo map base_link
```

终端③启动导航。**下面 `0.0` 只适用于已经确认 map 中地面 Z 为零的情况，否则替换为实际值：**

```bash
ros2 launch g1_nav_bridge g1_navigation.launch.py \
  map:=/opt/G1/lighting_ws/data/new_map/map.yaml \
  floor_z:=0.0 \
  base_floor_z:=0.0

ros2 launch g1_nav_bridge g1_navigation.launch.py \
  map:=/opt/G1/lighting_ws/data/new_map/map.yaml \
  floor_z:=0.0 \
  base_floor_z:=0.0 \
  use_collision_monitor:=false \
  use_keepout:=false
```

该入口自动启动：

- `map_server`，加载静态地图并发布 `/map`
- Nav2、速度平滑、碰撞监测
- `/odommodestate → /odom`
- `map → base_link` TF 转 `/current_pose`
- G1 速度桥接（`cmdvel_to_sport`：订阅 `/cmd_vel` 直接调用 `LocoClient::SetVelocity`，**没有软件使能/急停服务**，Nav2 一出速度机器人就走）

不需要再单独启动 `nav_bridge.launch.py` 或 `tf_to_current_pose`。这套手动流程也不要与前端的“启动导航”重复运行。

## 4. 检查后再下发目标

```bash
ros2 topic echo /base_link_pose --once
ros2 topic hz /odom
ros2 lifecycle get /controller_server
```

确认定位正常、地图与点云对齐、真实障碍物进入 costmap，并有人拿着遥控器准备硬件急停后，
在 RViz 点击 **Nav2 Goal** 或前端下发目标。

速度桥没有 enable / stop 服务，也没有急停话题。停止方式：

- 取消导航目标（RViz / 前端取消，或 Ctrl-C 结束发目标的脚本）：Nav2 停止输出速度，
  速度桥每条指令只持续 `duration`（0.5 s），随后机器人停下；
- 硬件遥控器急停（必须有人手持）。

速度上限来自 Nav2（`aid_navigation2/launch/g1_config.py`）：vx ≤ 0.5 m/s、不倒退、不横移、wz ≤ 0.9 rad/s；
速度死区 vx 0.3 / wz 0.8（小于死区的指令置零）。

注意：软件停车依赖 Thor、网络、DDS 和程序正常，测试时必须有人拿着机器人遥控器的硬件急停。


### 1. 只启动 SDK 速度桥

先关闭 Nav2、`robot.launch.py` 和其他速度发布者。机器人通过手机/遥控器进入可以正常移动的走跑模式。

```bash
source /opt/G1/lighting_ws/install/setup.bash

ros2 run g1_nav_bridge cmdvel_to_sport --ros-args \
  --params-file /opt/G1/lighting_ws/src/g1_nav_bridge/config/nav_bridge.yaml \
  -p cmd_vel_topic:=/g1_test_cmd_vel \
  -p duration:=0.5
```

cd /opt/G1/lighting_ws/src/aidrobo_client-haier
NODE_OPTIONS=--openssl-legacy-provider npm run dev

cd /home/lmw/project/unitree/G1/lightning_ws/aidrobo_client-haier

python3 -m http.server 3333 \
  --bind 0.0.0.0 \
  --directory dist

http://127.0.0.1:3333/#/site