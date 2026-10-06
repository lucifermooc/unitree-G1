# G1 统一启动入口

`robot.launch.py` 是整机唯一顶层入口，只编排各包已经存在的原子 launch，避免重复
启动 Livox、Lightning、`g1_nav_bridge`、Nav2 和 `collision_monitor`。
不传 `mode` 时默认使用 `navigation`，启动完整定位与导航链路；默认地图目录为
`/opt/G1/lighting_ws/data/new_map`。

## 开机自启（Thor）

开机后自动启动整套栈和语义地图，不再手动执行 `ros2 launch robot_bringup robot.launch.py`。
由两个 systemd 服务负责，以 `unitree` 用户身份运行。unit 文件就是 `system/g1-robot.service`、`system/g1-semantic-map.service`，
`g1_service.sh install` 把它们拷到 `/etc/systemd/system/`；ExecStart 调 `g1_autostart.sh`（source ROS/install/DDS 环境、等网卡、写日志）：

| 服务 | 执行 | 等同于手动 |
|---|---|---|
| `g1-robot` | `script/g1_autostart.sh robot` | `robot.launch.py`（默认参数，navigation 模式） |
| `g1-semantic-map` | `script/g1_autostart.sh semantic` | `HF_HUB_OFFLINE=1 semantic_map.launch.py use_fp16:=true model_name:=~/models/bge-m3` |

两个服务启动时都会按顺序加载 `/opt/ros/jazzy`、工作区 `install/setup.bash` 和 `system/dds_env.sh`。其中 `dds_env.sh` 负责
DDS 设置：spdp 组播只用于发现，数据走单播，参与者上限为 100。启动前服务还会等待以下条件：

- DDS 网卡 `enP2p1s0` 拿到 IP；
- 语义地图服务等待 Qdrant 的 6333 端口就绪。

如果已经有一套栈在运行（例如手动用 nohup 起的），服务会拒绝启动，避免出现两套同名进程。

服务和下面的命令都只用 `install/` 里的文件：部署后 src 会删除，unit 里不能出现 src 路径。
旧版 install 写的 unit 指向 `src/robot_bringup/script/g1_autostart.sh`，删 src 后开机自启失败；
`restart`/`start`/`status` 遇到这种 unit 会告警，重新执行一次 `install` 即可。

```bash
S=/opt/G1/lighting_ws/install/robot_bringup/share/robot_bringup/script
$S/g1_service.sh install    # 只需一次（需要 sudo 密码）：安装两个服务并设为开机自启，同时安装 system/60-dds-buffers.conf
                            # 和 /etc/sudoers.d/g1-autostart（只放行启停这两个服务免密，之后 restart/stop/start 不再要密码）
$S/g1_service.sh restart    # 部署后重启：停服务并清理所有残留 ROS 进程（等同 stop_all），然后启动服务
$S/g1_service.sh status     # 查看服务状态和日志位置
$S/g1_service.sh log        # 跟踪 /opt/G1/logs/robot_latest.log；看语义地图用 log semantic
$S/g1_service.sh stop       # 停止并清理；下次开机仍会自启
$S/g1_service.sh uninstall  # 取消开机自启
```

- 启动参数写在 `script/g1_autostart.env`，改完执行 `restart` 生效。例如 `G1_ROBOT_ARGS="mode:=base"`，或给语义地图追加 `text_in_action:=search`（只搜索不导航）。
- 只对某台机器生效的设置写在该机器的 `/opt/G1/g1_autostart.local.env`，它不进仓库。比如 D435i 序列号和默认值
  347622073141 不同时，用 `rs-enumerate-devices -s` 查到序列号后写 `G1_ROBOT_ARGS="realsense_serial_no:='<序列号>'"`；
  否则相机节点会一直报 NOT found。
- 日志每次启动生成一份，保存在 `/opt/G1/logs/{robot,semantic}_<时间>.log`，各保留最近 10 份。
- 服务不会自动重启。手动执行 `stop_all.launch.py` 后，服务不会自己再拉起一套栈；此时仍可以按原方式手动 `robot.launch.py`。
- 如果不想开机自启语义地图：`sudo systemctl disable g1-semantic-map`。

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
