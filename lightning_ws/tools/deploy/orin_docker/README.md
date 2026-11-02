# G1 Orin 容器部署（无 Thor 的 G1）

G1 自带的 Orin NX（PC2，`192.168.123.164`）是 JetPack 5.1.1 / Ubuntu 20.04，只能装 foxy，跑不了我们的 jazzy 栈。
做法：Orin 主机不动，jazzy 栈跑在 Ubuntu 24.04 镜像 `g1-jazzy` 的**常驻容器 `g1`** 里。
这个容器只创建一次（`--restart unless-stopped`），shell、编译、开机服务都 `docker exec` 进同一个容器，在容器里装的东西会保留。
容器和 Thor 主机的路径完全一样，包括 `/opt/G1/lighting_ws`、`/home/unitree`、`/tmp`；网络、pid、ipc 都用 host，privileged 模式。
所以 install 里的 `g1_autostart.sh`、`dds_env.sh`、`g1_service.sh` 都照常用。

首台：阿里云转发 `ssh -p 30524 unitree@47.108.223.180`（本机 `~/.ssh/config` 别名 `g1-orin`），WiFi `192.168.1.109`。

## 文件

| 文件 | 装到 | 作用 |
|---|---|---|
| `Dockerfile`、`g1_env.sh` | `/opt/G1/docker/` | 镜像：ROS 包取 Thor 上手动安装的那批 `ros-jazzy-*`（不含 rviz / rqt 等界面包），开发库、语义地图 Python 依赖与 Thor 同版本（torch 为 CPU 版） |
| `g1_docker.sh` | `/opt/G1/docker/` | `build-image` / `up` / `recreate` / `shell` / `run <命令>` / `service`、`service-stop robot\|semantic` |
| `g1-robot.service`、`g1-semantic-map.service` | `/etc/systemd/system/` | 容器版 unit：ExecStart 为 `g1_docker.sh service ...`（docker exec）；docker exec 不转发信号，所以 ExecStop 按 `/tmp/g1-<名>.pid` 发 SIGINT，之后再清本服务残留的 ROS 进程（见下） |
| `container_setup.sh` | `/opt/G1/docker/` | 新建容器时由 `g1_docker.sh` 自动执行（root）：装 Pangolin 0.9.3（lightning thirdparty 里的 zip，lightning 编译硬依赖） |
| `install_orin.sh` | — | 安装上面这些、sudoers、DDS 缓冲、Qdrant 容器（`qdrant/qdrant:v1.19.1`，restart always） |

配置（2026-10-27 起全部在 src 的默认文件里，Thor 已停用，不再分机型、不再用 `/opt/G1/*.local.*` 覆盖）：
| 内容 | 文件（机器上改 install 里同路径的那份，仓库也要同步改） |
|---|---|
| Nav2：控制器 `FollowPath`（shim）/ 备用 `FollowPathRPP`、容差 0.3、BackUp 行为树、`base_link_nav`、smoother 后退下限 | `aid_navigation2/param/nav2_params.yaml` |
| 速度桥：`min_wz 0.32`、`walk_min_wz 0`、后退 0.3、蟹行补偿 `crab_deg 10` | `g1_nav_bridge/config/nav_bridge.yaml` |
| 定位：`loc_guard.init_yaw_search_deg 0` | `lightning-lm/config/default_livox.yaml` |
| DDS 网卡 `eth0` | `robot_bringup/system/cyclonedds_g1.xml` |
| MID360 `host_ip .164` | `livox_ros_driver2/config/G1_MID360s_config.json` |
| D435 关彩色、深度 848x480x15 | `robot_bringup/launch/robot.launch.py` 默认值 |
| 语义地图 `use_fp16:=false` | `robot_bringup/script/g1_autostart.env` |

**切控制器**：把 `nav2_params.yaml` 里 `FollowPath`（shim）和 `FollowPathRPP` 两个块名对调（shim 那块改叫 `FollowPathShim`），重启 g1-robot。

每台机器只剩 `/opt/G1/g1_autostart.local.env` 一行：`G1_ROBOT_ARGS="realsense_serial_no:='<该机 D435i 序列号>'"`
（30524 `250122078132`，30423 `254343062813`）。`*_params_overlay:=` 参数还在，只给临时试参数用。

从旧覆盖方式迁移（部署新版 robot_bringup / aid_navigation2 / g1_nav_bridge / lightning / livox_ros_driver2 时一起做）：
1. `local.env` 改成上面那一行（去掉 `livox_config:=`、`realsense_enable_color:=`、`realsense_depth_profile:=`、三个 `*_overlay:=`、`G1_SEMANTIC_ARGS`）。
2. `/opt/G1/` 下 `nav2_params.local.yaml`、`nav2_params.shim.local.yaml`、`nav_bridge.local.yaml`、`lightning.local.yaml`、
   `cyclonedds_g1.local.xml`、`G1_MID360s_config.local.json`、`nav_controller_switch.sh` 挪进备份目录（新版不再读它们）。
3. 重启后 `ros2 param get` 核对 `FollowPath.plugin`、`settle_time`、`min_wz`、`crab_deg`，看 glog `init yaw search +-0 deg`。

**不要在 Orin 上执行 `g1_service.sh install`**：它会装回主机版 unit，而主机上没有 jazzy。其余子命令（start/stop/restart/status/log）照常用。

**停服务后的残留进程**：ROS 进程在容器的 cgroup 里，不在 unit 的 cgroup 里，systemd 停服务时（`KillMode=control-group`）清不到它们。
launch_manager 拉起的定位 / 导航子 launch 若没被收掉，就会挂到容器下继续跑，下次启动后与新栈同名并存
（2026-10-24：10:33 `systemctl` 停服务后，09:18 的整套导航栈连同 `cmdvel_to_sport` 一直活着，负载 20 → 35）。
所以 `g1_docker.sh service-stop` 在 SIGINT 之后会调 `stop_ros_processes.py` 清本服务的残留（robot：`--exclude semantic_map`；
semantic：`--only semantic_map`）。`g1_service.sh restart/stop` 本身也会清理（纯 Python，主机上能跑）。
查有没有残留：`python3 /opt/G1/lighting_ws/install/robot_bringup/share/robot_bringup/script/stop_ros_processes.py --dry-run`。

## 首次部署

1. 主机：`usermod -aG docker unitree`；`/etc/docker/daemon.json` 加 `registry-mirrors`，daocloud 放第一位（docker.1ms.run 能取清单、拉不下层）；
   建 `/opt/G1`。
2. 从 Thor 局域网拷文件（`ssh -A` 转发本机密钥）：
   - `~/unitree_ros2`：Thor 上编好的，aarch64 + jazzy，容器里直接能用。
   - `/opt/G1/deps_backup/lightning_thirdparty`，拷到 `src/lightning-lm/thirdparty`。
   - `~/models/bge-m3`。
   - `/opt/G1/bags/replay/*.py`。
3. 同步源码到 `/opt/G1/lighting_ws/src`，`rsync` 排除规则和 `deploy_to_robot.sh` 相同。
4. `bash g1_docker.sh build-image`，然后在容器里全量编译（`run` 会先自动创建容器 `g1`）：
   `g1_docker.sh run bash -c 'cd /opt/G1/lighting_ws && export PYTHONNOUSERSITE=1 && colcon build --cmake-args -DROS_EDITION=ROS2 -DDISTRO_ROS=jazzy -DBUILD_WITH_MARCH_NATIVE=ON'`
5. `bash install_orin.sh`，写好 `/opt/G1/g1_autostart.local.env`（只有 D435 序列号一行），再执行 `g1_service.sh restart`。容器 `g1` 会在第一次 exec 时自动创建。

之后的增量部署照常：`tools/deploy/deploy_to_robot.sh g1-orin <包>`。脚本发现机器上有 `/opt/G1/docker/g1_docker.sh` 时，会在容器里编译。

## 手动用 ROS

```bash
bash /opt/G1/docker/g1_docker.sh shell              # 进常驻容器 g1，已 source jazzy + install + dds_env.sh
bash /opt/G1/docker/g1_docker.sh run ros2 topic list
```
- 镜像重建后要执行 `g1_docker.sh recreate`，用新镜像重建容器。容器里手动装的东西会丢，长期要用的请写进 Dockerfile。
不要在主机终端里直接用 `ros2`：主机的 `~/.bashrc` 是 foxy 环境。
