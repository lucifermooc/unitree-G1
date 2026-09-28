# G1 统一启动入口

`robot.launch.py` 是整机唯一顶层入口，只编排各包已经存在的原子 launch，避免重复
启动 Livox、Lightning、`g1_nav_bridge`、Nav2 和 `collision_monitor`。
不传 `mode` 时默认使用 `navigation`，启动完整定位与导航链路；默认地图目录为
`/opt/G1/lighting_ws/data/new_map`。

## 互斥模式

```bash
# 传感器、rosbridge 和业务后端，不启动 SLAM/Nav2/运动桥
ros2 launch robot_bringup robot.launch.py mode:=base

# 建图
ros2 launch robot_bringup robot.launch.py mode:=mapping with_ui:=true start_rviz:=true

# 仅定位
ros2 launch robot_bringup robot.launch.py mode:=localization \
  map_dir:=/opt/G1/lighting_ws/data/new_map with_ui:=true start_rviz:=true

# 定位 + 静态地图 + Nav2 + 唯一运动桥
ros2 launch robot_bringup robot.launch.py mode:=navigation \
  map_dir:=/opt/G1/lighting_ws/data/new_map \
  nav_map:=/opt/G1/lighting_ws/data/new_map/map.yaml \
  floor_z:=0.0 base_floor_z:=0.0 \
  use_collision_monitor:=false start_rviz:=true
```

`navigation` 模式中，Lightning 定位由 `g1_localization.launch.py` 创建；Nav2、运动桥和
可选的 collision monitor 由 `g1_navigation.launch.py` 创建。顶层不会再次创建这些节点。
`robot_status_manager_node` 在此入口下设置为 `manage_stack:=false`，只负责状态与业务服务，
不会再通过 `launch_manager_node` 启动第二套 Lightning/Nav2。需要切换建图、定位或导航
进程模式时，应停止当前顶层 launch，再用新的 `mode:=...` 启动。

`use_collision_monitor` 在完成点云高度、停车区和软件停车验收前默认关闭。前端当前没有
速度遥控话题，统一入口不会虚构或订阅一个前端遥控接口。

## RealSense D435 点云

`robot.launch.py` 默认启动 RealSense，显式开启 depth、color 和 pointcloud。
点云话题为：

```text
/camera/camera/depth/color/points
```

RealSense wrapper 只在该话题存在订阅者时组装并发布 PointCloud2，因此用下列
命令验证频率（命令本身会建立订阅）：

```bash
ros2 topic hz /camera/camera/depth/color/points
ros2 topic echo --once /camera/camera/depth/color/points --field header
```

若没有话题，先检查：

```bash
lsusb | grep -i -E 'Intel|RealSense|8086'
rs-enumerate-devices
ros2 node list | grep /camera/camera
ros2 param get /camera/camera pointcloud.enable
```

Nav2 的 RealSense 障碍物源默认关闭。开启前必须在机器人 URDF 中加入实测的
`base_link -> camera_link` 固定外参，并确认下列 TF 可查：

```bash
ros2 run tf2_ros tf2_echo base_link camera_depth_optical_frame
```

验证后使用：

```bash
ros2 launch robot_bringup robot.launch.py mode:=navigation \
  nav_map:=/opt/G1/lighting_ws/data/new_map/map.yaml \
  floor_z:=0.0 base_floor_z:=0.0 \
  use_realsense_obstacles:=true
```

不得为了消除 TF 报错把相机外参填成零；错误外参会直接把障碍物写到错误栅格。
