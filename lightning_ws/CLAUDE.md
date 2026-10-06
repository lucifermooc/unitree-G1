# 工作方式要求

调试以**速度优先**。

- 已确认的信息不要重复检查。
- 不要重复遍历整个仓库。
- 修改代码后**只编译受影响的 package**，不要每次全量 `colcon build`。
- 编译失败后只分析当前新增的关键 `error`，忽略无关 warning。
- 能并行执行的检查**并行执行**。
- 已经确定的问题**直接修改**，不需要先长篇解释。
- 持续修改、编译、验证，直到通过。
## Subagent policy

Do not spawn subagents by default.

For normal debugging, code search, compilation, testing, SSH operations,
and fixing build/runtime errors, perform the work directly in the main agent.

Do not spawn Explore or general-purpose subagents for tasks that can be
completed with grep, rg, find, reading files, or running commands directly.

Only use a subagent when:
- the task is genuinely independent and complex, or
- parallel investigation provides a clear benefit.

Never spawn multiple subagents for routine debugging.


## 本项目的固定事实（不要重新验证）

- Thor：`unitree@192.168.128.146`，`/opt/G1/lighting_ws`，aarch64 / Ubuntu 24.04 / ROS jazzy。
- 本地是 x86 humble，**不能**把 `src/lightning-lm/bin/` 同步到 Thor（那是 x86 二进制，会覆盖 ARM 版）。
  rsync 时必须 `--exclude='bin/'`（注意相对路径要匹配同步起点）。2026-09-24 起 lightning 可执行文件输出到
  `build/lightning/bin`，源码树不再生成 `bin/`；但 Thor 上旧的 `src/lightning-lm/bin` 还在，exclude 继续保留。
- **仓库卫生（2026-09-24 瘦身，不准再生成）**：git 仓库是 `src/`，`src/.gitignore` 挡住 build/install/log/bin、
  `__pycache__`、`.pytest_cache`、日志/录包；`src/pytest.ini`（`-p no:cacheprovider`）+ `src/conftest.py`
  （`sys.dont_write_bytecode`）让跑测试不留缓存。直接跑 Python 用 `python3 -B` 或 `PYTHONDONTWRITEBYTECODE=1`；
  临时脚本/分析输出放 scratchpad 或 Thor 的 `/opt/G1/bags/replay/`，不进 `src/`、`tools/`。
- **不在本地编译**（用户明确要求）：本地只改代码，编译、跑 gtest/pytest 一律 rsync 到 Thor 上做；Thor 离线就如实说"未编译"。
  本地 install 曾是 `--symlink-install`，删 build/ 让 install 里 1439 个链接悬空（2026-09-24 踩过），本地 install 已不完整、不可用。
- 编译不要加 `--symlink-install`（与 `build.sh jazzy` 的非 symlink 模式冲突，会报
  "failed to create symbolic link ... Is a directory"）。
- 部署真机前需用户同意；验证时不发导航目标、不发速度。
- **部署后重启的标准流程（唯一认可的方式）**：先 `ros2 launch robot_bringup stop_all.launch.py`
  停掉所有内容，再 `ros2 launch robot_bringup robot.launch.py` 启动。
  **开机自启**：`install/robot_bringup/share/robot_bringup/script/g1_service.sh install` 装 systemd 服务 `g1-robot` 和 `g1-semantic-map`
  （入口 `g1_autostart.sh`，自带 DDS 环境；参数写在 `g1_autostart.env`，本机专属参数写在 `/opt/G1/g1_autostart.local.env`）。
  **部署后 Thor 上的 src 会被删除（用户的部署方式）**：运行时（systemd unit、自启脚本、要 source 的文件、文档里的命令）
  只能引用 `install/`，不能写 `$G1_WS/src/...`。2026-10-03 前的 unit 指向 src，128.146 删 src 后开机自启失败；
  已改为 unit 固定指向 install，且 `system/` 随包安装。旧 unit 要用新的 install 重新执行一次 `g1_service.sh install`。
  2026-09-30 已装在 `unitree@192.168.112.70`（tegra-ubuntu，wlP1p1s0，sudo 免密）：用 systemctl start 验证过，整栈、语义地图、
  DDS、D435 都正常；还没做重启开机验证。该机 D435i 序列号是 317622075180（默认 347622073141），已写进它的 local.env。
  128.146 那台 Thor 还没装。
  装好后，重启改用 `g1_service.sh restart`（= 停服务 + stop_all 清理 + 启动服务）。install 会写入
  `/etc/sudoers.d/g1-autostart`：只有启停这两个服务的 systemctl 命令免密，所以 ssh 里 restart 不要密码；其它 sudo 仍要密码。
  stop_all → 手动 nohup robot.launch.py 也仍然可用：服务是 Restart=no，不会跟手动栈抢着起。日志在 `/opt/G1/logs/*_latest.log`。
  **128.146 Thor**：2026-09-30 14:30 已装（sudo 要密码，由用户执行 install），D435 序列号与默认值一致，不需要 local.env。
  不要用 `mode_set idle/patrol` 之类的局部重启代替——那样只重启 nav2，
  其它进程仍是旧状态，容易出现"参数半新半旧"而误判实验结果。
  定位输出直接看 `/base_link_pose`（= map->base_link）。
- TF 结构：`map->base_link`（平面，只含位置+航向）与 `base_link->body_link`（躯干实时倾斜）都由
  LocSystem 动态发布；URDF 根是 `body_link`，mid360/d435 挂在其下（官方外参，mid360 pitch=+2.3°，
  驱动已翻转）。G1 不同站姿头部相对重力差 3~4°，不要再把某次站姿实测倾斜写进 URDF。
- 高度基准：平面模式下 map->base_link 的 z 恒为 0（body_link 原点=脚下地面）；固定的 `g2p5.floor_height`
  偏移每张图不同，不能用于导航高度。Nav2 雷达障碍订阅 `/lightning/registered_scan`（依赖 pub_registered_scan:=true），
  它是 LIO 去畸变单帧，frame=mid360_link、stamp=该帧 lidar_end_time，由下游按扫描时刻查 TF；
  不要用高频外推位姿把扫描摆到 map 系（与扫描时刻差 0.1~0.5s，转向时墙点被甩成满屏假障碍）。
- 定位 tilt 由 `lidar_loc.gravity_constrain` 用 LIO 重力锁定，目标方向读 `<map>/gravity_up.txt`，
  无该文件则不约束；建图 `fasterlio.gravity_align_init` 让新地图 z 轴对齐重力并自动写该文件。
- `robot.launch.py` 默认（start_backend:=true）由 robot_status_manager 经 launch_manager 启停
  定位/导航/建图（前端才能切换模式），地图取数据库当前地图（~/maps/db.sqlite），
  取不到才用 map_dir；`start_backend:=false` 时才由顶层直接启动。

## 远程排查的坑（已踩过，别再踩）

- `pgrep -c <name>` 对超过 15 字符的进程名恒返回 0 → 必须用 `pgrep -f`。
- `pgrep -f` / `pkill -f` 会匹配到自己的 ssh 命令行（含 heredoc 内容），把会话杀掉（退出码 255）。
  用 `pgrep -f "[x]xx"` 方括号写法，或把逻辑写成脚本 `scp` 过去再调用。
- **kill launch 父进程不会杀掉子节点**，它们会变成 PPID=1 的孤儿继续运行，
  与新启动的实例同名冲突，导致各种诡异现象（TF 乱跳、设备 busy、帧率崩塌）。
  清理列表必须包含 `cmdvel_to_sport`、`point_filter` 等所有子节点（`stop_all` 按 `install/` 路径匹配，已覆盖）。
- `/livox/points` 由 C++ 包 `point_filter` 产生（2026-09-24 取代 `livox_custom_to_pointcloud2.py`）：按 MID360 tag
  剔除低置信度点（Mid-360 协议：bit[1:0] 粘连、bit[3:2] 雨雾尘、bit[5:4] 其他，值 0 高/1 中/2 低置信度），
  并做与 lightning 预处理相同的抽点/盲区/雷达系高度裁剪。**LIO 仍直接订阅 `/livox/lidar`，不吃它**；
  lightning 自己的 tag 过滤（`pointcloud_preprocess.cc:53`）是注释掉的，且去重/盲区判断有 `||`/`&&` 优先级 bug。
- 反复起临时 ROS 节点会耗尽 CycloneDDS 参与者槽位
  （"Failed to find a free participant index for domain 0"），需等待释放。
- `ros2 topic hz` 测点云需要 30~40s 窗口，14s 会读到 0。
- 判断参数是否生效必须 `ros2 param list/get` 查运行中节点，
  不能信 launch 脚本打印的 "not supported" 警告。
- 查 param / 调 service 前必须先 `source /home/unitree/unitree_ros2/setup.sh`：整套栈跑在 rmw_cyclonedds 上，
  只 source `/opt/ros/jazzy` 时是默认 RMW，topic 能互通但 param/service 调用一律超时无响应（不是节点卡死）。
- **启动整套栈的 shell 必须 `source install/robot_bringup/share/robot_bringup/system/dds_env.sh`（spdp：组播只做发现、数据单播）**，
  不能只 source `unitree_ros2/setup.sh`——它把 `CYCLONEDDS_URI` 盖回组播，同机进程间的点云也走网线，被交换机 PAUSE
  限在 ~10 MB/s（D435 掉到 12 Hz、scan 7 Hz，定位输入滞后 = 9-19 定位跳变的根因）。robot.launch.py **不会**自动设置
  DDS（`cyclonedds_g1.xml` 注释里的 dds_config 不存在），全看启动它的 shell。核对：`/proc/<pid>/environ` 里的
  CYCLONEDDS_URI；spdp 下 enP2p1s0 发送 ~72 KB/s，组播下 ~7 MB/s。
- 建图/定位 glog 出现 `get abnormal dt`（IMU 断档）= IMU 没进到 LIO（源头 200 Hz 完好，录包进程收全）。
  快转时一次断 0.1~0.3 s 就让航向跳几十度 → 地图多层墙。2026-09-19 逐项排除后的修复（缺一不可，别回退）：
  1. 不给任何节点加 `taskset` 绑核（原 3-7 让建图/RViz/Nav2 挤 5 核）；2. 建图/定位用 `with_ui:=false`
  （Pangolin 软件渲染吃 1 核且随地图变慢）；3. IMU 订阅队列 1000（本进程发布 ~1 MB 的 /map 时 DDS 接收被拖
  0.3~1 s，积压 IMU 一次灌入，深度 100 会覆盖）；4. `robot_bringup/system/60-dds-buffers.conf` 装到 /etc/sysctl.d。
  验证：`/opt/G1/bags/map500d_0919` 8 分钟建图 0 断档。查进程 socket 丢包看 `/proc/net/udp` drops 列。
- 远程脚本里 `pkill -f xxx` 与 `nohup ... xxx` 写在同一条 ssh 命令会杀掉自己的会话：先 scp 成脚本再执行。
- `ros2 bag record -d N` 是**分片时长**（每 N 秒换一个文件），不是"录 N 秒就停"。
  `record_nav_bag.sh` 的 `-t` 以前直接传给 `-d`，结果录制永不停止，ssh 一断就变成孤儿进程继续写盘
  （几分钟 5 GB）。已改成 `timeout -s INT "$DUR"` 包住（SIGINT 才会正常写 metadata.yaml）。
- **Thor 上编译 lightning 必须 `export PYTHONNOUSERSITE=1`**：2026-09-29 11:03 部署语义地图时 pip 往 `~/.local` 装了
  setuptools 79，thirdparty livox 的 python egg 构建报 `InvalidVersion: ''` 导致 colcon 失败。不要去改 `~/.local`。
- **栈在运行时不要 `colcon build` lightning**：install 会原地覆盖正在被 run_loc_online 使用的 .so。只需离线工具时用
  `cmake --build build/lightning --target run_loc_offline test_loc_guard`（不 install），或先 stop_all。
- **`run_loc_offline` 退出时会往地图目录写动态图层**（`maps.save_dyn_when_quit: true`）：离线回放一律用地图拷贝。
  临时启动配置 `/tmp/g1_lightning_*.yaml` 在停栈时被清理，离线要用 `install/.../config/default_livox.yaml` 自己改话题和 map_path。
- 重启栈的现成脚本：Thor `/opt/G1/bags/replay/restart_stack.sh`（stop_all → nohup robot.launch.py，日志 `robot_launch.log`）。
  自启服务装好后改用 `g1_service.sh restart`，否则手动栈和服务的日志分在两处。
- RealSense SDK 已 vendored 在 `src/librealsense`（包名 librealsense2，默认 BUILD_WITH_NEON=OFF，第三方依赖离线），
  点云参数是 `pointcloud.*`。前提是 `realsense2_camera` 链接 workspace 里的 SDK：
  若 ldd 显示链接的是 `/usr/local/lib`（别人装的系统 NEON 版），参数会变成 `pointcloud__neon_.*`，点云停发。


## Costmap 障碍物：动态障碍残影 / D435 点云（2026-09-21~23，local+global 已部署并实机验证）

### 现象
人从机器人侧面走到正前方再离开，local 和 global costmap 都留下清不掉的致命格（线上最长挂 174 s），挡住规划。
当时怀疑是 D435 点云问题（地面不平、空洞、打穿、噪点）。

### 本质原因
1. **传感器几何**：MID360 倒装，仰角 −47°~+5°，但**正前方 ±30° 被 G1 头部挡住**（只有 −11°~−1.5° 一条窄带偶有回波）；
   D435 ±43°、高 1.24 m、光轴下俯 47.6°。→ 前进方向近场 100% 靠 D435，30°~43° 两条侧带两者重叠。
2. **残影在 MID360 层（obstacle_layer），不在 D435（STVL 全程 0 残影）**：人经过侧带时 MID360 标了格，人走后那片落进
   MID360 前方盲区，**再没有 MID360 射线经过，永远清不掉**。D435 看得见那里已空，但只接在 STVL；各层取 Max，
   STVL 的"空"清不掉 obstacle_layer 的格。global 同理，且 global 不滚动，残影永久留在全局图上。
3. nav2 ObservationBuffer 先按 per-source 的 min/max_obstacle_height 裁点再清除，地面点（z<0.10）永远成不了射线端点
   → 只调 `raytrace_max_range` 没用。
4. D435 本身没坏：STVL 清除逻辑正常，只是盲区记忆太久（旧 `voxel_decay 3.0`）。
   - 地面成对椭圆空洞 = 天花板灯管在抛光地砖上的倒影，不制造障碍，调激光/曝光/预设都去不掉，**别再追**。
   - 深度图顶部几行（掠射远地面）双目误匹配 → 右前方 ~1.9 m 有一簇稳定错深度（深度被砍半，浮在 0.37~0.59 m，
     每帧都在，decay/邻域/帧间过滤都去不掉），靠 `realsense_mark.obstacle_range 1.5` 挡在外面，**别放宽**。

### 解决方案（`aid_navigation2/param/nav2_params.yaml`，已部署）
- **核心：D435 做 obstacle_layer 的只清除源**（标记/清除分源，借鉴 Haier）。local 和 global 的 obstacle_layer 都加
  `realsense_clear`：D435 原始点云，`marking false / clearing true`，`min_obstacle_height -0.30`（地面点做射线端点），
  local raytrace 3.5 m、global 2.5 m。
- **补标高物体**：2D 射线会从人腿间/桌下穿过，把 MID360 标的人、桌面误擦掉。所以 D435 加只标 0.7~1.8 m、2.5 m 内的
  `realsense_mark_tall`（local 放在 STVL；global 没有 STVL，放在 obstacle_layer——ObstacleLayer 每次更新**先清后标**，
  误擦的格同一帧补回）。0.7 m 下限高于错深度簇。**清除距离 ≤ 能补标的源的标记距离**（livox、tall 都是 2.5 m）。
- STVL：`realsense_mark` 1.5 m / ≥0.15 m / `voxel_min_points 4`；`voxel_decay 1.5` + `realsense_clear.decay_acceleration 15`。
  （2026-09-24 实机试过 ≥0.05 m、2.0 m 都已退回，原因见"Costmap 噪点障碍"一节。）
- `stvl_voxel_layer.update_footprint_enabled: false`：自清半径 = robot_radius，会把半径内的真障碍当自身点抹掉；
  该参数不在动态回调里，必须改 YAML 并重启整套栈。
- launch：`use_realsense_obstacles:=false` 时，同时摘掉两张图里的 D435 源（`drop_d435_clear`）；`test_g1_direct_config.py` 守着这些约束。
- **前提：栈跑在 spdp DDS 下**（见"远程排查的坑"），否则 planner_server 多订 D435 会把点云挤上网线。

### 效果（实机 walkby：人从侧面走到正前 1.5 m，站 3 s 后从另一侧离开，×3）
| | 修复前 | 修复后 |
|---|---|---|
| local 前方残影 | 89 格/帧，最长 174 s | 0，清除延迟 p90 0.3 s |
| global 前方残影 | 12 格/帧，挂满整个录制 | 0，约 2 s 内清掉（global ~1 Hz） |
| 高物体召回（0.5~1.5 / 1.5~2.5 m） | 回放：local 52% / 27%，global 9% / 0% | 实机 local 83% / 71%；回放 global 46% / 61% |
CPU：planner_server 7% → 14%，controller_server 约 26%。

### 已知代价与未做（用户决定暂不做）
- global：D435 视场内 1.5~2.5 m、低于 0.7 m、MID360 从侧面标过的矮障碍，会被从上方掠过的 2D 射线擦掉
  （demo2 回放：视场内丢失率比噪声底高约 40 个百分点，多为 1~3 s 短暂缺失，最长 11 s）。
  根治要把 global 障碍改成 3D（STVL，MID360 也放进去）。
- 1.5 m 外的前方矮障碍 global 看不到（MID360 盲区 + D435 低障碍只标 1.5 m），只能靠 MPPI 局部躲。
- 2026-09-24 的噪点排查（飞点、前摆的手、机身回波、地面回波、错深度）见下一节"Costmap 噪点障碍"。
- 曲率去地面在 D435 上不可用（地面 σ 2~3 cm 时和矮障碍不可分，单帧约 35 ms）。

### MID360 从 2D obstacle_layer 迁到 3D STVL（2026-09-24 已部署，实机运行中）
- 原因：MID360 配置是照 2D 激光搬的（obstacle_layer + 2D raytrace）。改成 Haier 3D 雷达的做法：两张图都去掉
  obstacle_layer，STVL 里加 `livox_mark`（只标，参数同原来）+ `livox_clear`（`model_type 1` 3D 雷达视锥，水平 360°）。
  global 同样改成 STVL（`voxel_decay 5`）。2026-09-24 global 改为 update/publish 5 Hz，MID360 标记源
  `observation_persistence` 随之从 1.0 改为 0.2（只攒一个周期；更长会让走开的人多挂这么久）。
  注意 publish == update 时 nav2 实际约隔一个周期才发一次（实测 ~3.2 Hz），规划在进程内读图，不受影响。
- STVL 没有 `obstacle_min_range`：新增 `g1_nav_bridge/scan_range_filter`（launch 始终启动），剔除离雷达 0.25 m 内的
  机身回波后发 `/lightning/registered_scan_nav`，两张图的 MID360 源都吃它。
- 数据依据（`tools/nav2_debug/mid360_stvl_probe.py`）：雷达系实测仰角 −50.5°~+6.4°（`vertical_fov_angle 1.80`）；
  同一格重访间隔 p95 1.0 s、p99 2.4~2.8 s，所以 `livox_clear.decay_acceleration` 不能大（≤2，测试守着）。

### 几何与参数常识
- 近处盲区随障碍高度变：`d_min(h) = 0.14 + (1.24−h)/tan(75.2°)`（h=0 → 0.47 m，h=0.57 → 0.32 m）；随躯干倾斜变，要用实时 TF 算。
- `robot_radius` 同时决定碰撞阈值、253 内切带和自清半径；最近 lethal ≈ max(传感器极限约 0.36 m, 自清边界)。
- 代价 `252·exp(-k(d-r))` 只在 inflation_radius 内成立。254 = lethal，253 = 内切带，255 = 未知。
- nav2 没有"被困"检测；清图只清得掉看不见的残影；`collision_margin_distance` 是软惩罚，不是硬安全圈。
- MID360 外参已用人体交叉验证；别用"点落在地图墙上的比例"判外参（噪声大到会误判成偏航 180°）。

### 调试方法（以后查 costmap 残影 / 噪点照这个来）
- **分层看**：`*/obstacle_layer_raw` 和 `*/stvl_voxel_layer_raw` 分开数 254，只看合成图会把两层混在一起；只数 254。
  再按"最近 N s 有没有该传感器的点支撑"分类，按传感器盲区扇区切片。
- 支撑判定用点云自己的 stamp 查 TF，给 ±1 格容差；rolling window 的帧间差分要在两帧窗口的交集里做；
  线上 D435 采集到 costmap 发布有 0.3~0.5 s 延迟（`costmap_recall.py --cam-lag 0.4`）。
- **离线回放台**（改参数前先 A/B）：`tools/nav2_debug/replay_costmap.sh <bag> <名> [--section global_costmap] [--start S --dur D] [--set 层.参数=值]`。
  跑在 domain 77 + spdp；`/tf_static` 由 `replay_tf_static.py` 补发；global 必须 `--start 0`（`/map` 只在开头发一次）；
  回放期间不要 stop_all（会连回放的录制一起杀）。有效性检查：录下的 D435 帧数应约等于原始包。
- 工具（`tools/nav2_debug/`，本地跑用 `python3 -s`）：`obstacle_residual_ab.py`（MID360 层残影 + 清除延迟）、
  `costmap_recall.py`（高物体召回、前方残影、`--overclear` 按 D435 视场切的误清）、`stvl_clear_latency.py`、
  `d435_false_obstacle_probe.py`（错深度反投影）、`d435_ground_quality.py`（地面质量）。
- 参考包（`/opt/G1/bags`）：walkby_0923（修复前）、walkby_s2_0923（修复后）、demo2_0922_1350（100~280 s 行走）、
  d435_ghost_0922（错深度）。

## Costmap 噪点障碍：MID360 / D435（2026-09-24，已部署并实机验证）⚠ 重要

### 现象（用户原话要点）
导航时"随时都是障碍物"：机器人走过的位置、base_link 旁边总有致命格，global 里激光和 D435 层都有"噪点"，清得慢。

### 结论：分来源处理，每一类都有实测证据（不要凭猜测改参数）
| # | 来源 | 证据 | 修法（文件/参数） | 实机结果 |
|---|---|---|---|---|
| 1 | **D435 孤立飞点**：镜头前 0.17~0.3 m、高 ~1.0 m、base_link 旁 0.12~0.2 m | 原始点云每个 5 cm 体素 ≥4 点就被标；这些像素 4 邻域一致邻居 0~2 个；落在清除视锥外，global 挂满 voxel_decay 5 s | `g1_nav_bridge/d435_mark_filter`（默认开，两张图的 `realsense_mark` 和 `realsense_mark_tall` 都吃它的输出 `/camera/camera/depth/mark_points`）：4 邻域一致性 + `min_depth 0.40` | base_link 0.5 m 内 D435 致命格：global 29 帧 → **0**，local 12 → **0**（各 2 min） |
| 2 | **走路时前摆的手** | 5 min 41 次，**100% 发生在走路时**；换到 base_link 系位置固定：前 0.25~0.43 m、左右 0.08~0.30 m、高 0.57~0.73 m；深度图里是深度一致的小块（8~51 像素），4 邻域删不掉 | `d435_mark_filter` 按扫描时刻 TF 删 base_link 水平 <0.5 m 且高 0.45~0.85 m 的点（`self_crop_radius/z_min/z_max`） | 同样走 5 min：41 次 → **0 次**（唯一 1 次是站着时右前方 0.5 m 的真实大表面，应该标） |
| 3 | **MID360 足迹内的机身/吊带回波** | 剔除 <0.25 m 后，base_link 0.5 m 内每帧仍 ~9 点（离雷达 0.25~0.26 m 的机身/吊带 + 贴身物体）；STVL 的 footprint 自清只把当前足迹在 2D 图上抹成空闲、**不删体素**，走开后沿路留下一串 | `g1_nav_bridge/scan_range_filter` 在进 STVL 前删 base_link 水平 <0.5 m 的点（`MID360_CROP_RADIUS = robot_radius`，按扫描时刻查 TF） | 离线：每帧删 ~8.6 点、TF 失败 0；走路拖尾未单独复测 |
| 4 | **MID360 地面回波** | global 里 1~3 格的单格噪点，31 帧只被 1~14 帧打到，支撑点全在 0.10~0.13 m | `livox_mark.min_obstacle_height` 0.10 → **0.15**（两张图） | 这类噪点 15 格 → 4 格 |
| 5 | D435 驱动官方滤波（用户要求用官方的） | — | `robot_bringup`：`decimation_filter.filter_magnitude 6`（144x80，点数 ~44%），`spatial_filter.enable true`（magnitude 5 / alpha 0.25 / delta 20 / holes_fill 0），temporal 关 | 已生效；2~2.5 m 高物体召回未单独测 |

**不是噪点、不要去滤的**：MID360 层里 1.4~2.4 m 的大团（20~400 格）每帧雷达都打到（30/30）、高 0.15~1.5 m、不在静态地图上
= 周围的真实物体（吊架、设备、人）。判断前先看"自己传感器的支撑点命中率 + 高度"，命中率 ~100% 且有高度的就是实物。

### 试过、实测效果不好、已退回的（别再重试，除非有新证据）
- **D435 `realsense_mark.min_obstacle_height` 0.15 → 0.05**（想标 7~8 cm 矮障碍）：D435 点云地面实测偏低 ~4.5 cm 且随站姿变，
  0.05 实际 ≈ 离真实地面 9.5 cm，靠不住 → 退回 0.15。
- **`realsense_mark.obstacle_range` 1.5 → 2.0**：1.5 m 外矮障碍能标了，但深度图顶部 0~25 行（掠射地面）的错深度在 1.6~2.1 m 带
  反复出现（悬空 0.35~0.65 m，左右前方都有，机器人不动就一直在）；开 `floating_filter` 后大幅减少但仍有残留，单帧耗时
  ~10 → ~26 ms → 退回 1.5、`floating_filter: false`。注意 obstacle_range 是**离相机的 3D 距离**（相机高 1.24 m），
  1.5 时地上矮障碍水平只标到 base_link 前 ~1.1 m。**STVL 的 obstacle_range 不能热改**（只在初始化读一次）。
- **剔除深度图光轴上方 >20° 的行**：能去掉顶部错深度，但 1.5~2.5 m 真实物体 2D 召回掉到 0 → 没上。
- **帧间一致性（temporal_filter）**：额外删 20~30% 真实物体 2D 格 → 关。
- **D435 整圈删 base_link 0.5 m**：正前方 ±50° 是 MID360 盲区，只有 D435，整圈删会看不见闯进足迹的人 → 只删手的高度带。
- **MID360 tag 过滤（`point_filter` 包）**：只作用于 `/livox/points`（RViz/诊断），LIO 吃 `/livox/lidar`、Nav2 吃
  `/lightning/registered_scan_nav`，对 costmap **没有影响**；实测 tag 只标出 ~0.2% 的点。

### 排查方法（这次踩过的弯路 + 最后管用的做法）
- **先分层、再找来源**：订阅 `/global_costmap/*_raw` 各层，主图每个 254 归到"静态墙 / D435 层 / MID360 层 / 不明"；
  对每团查它自己传感器最近 3 s 的支撑点（命中帧数、高度、位置）。RViz 里 OccupancyGrid 图层看到的格子要用 `*_raw` 复核。
- **近身事件要"触发式抓取"**：D435 层在 base_link 0.5 m 内出现新 254 就回溯最近 6 s 的 D435 帧，找出凑够 ≥4 点/体素的那几帧，
  看离镜头深度、高度、像素位置、深度图 5x5 邻域（孤立小岛 vs 连到图像边缘 vs 大表面）、同时刻 MID360 有没有。
  再把点换到 base_link 系、和机器人速度对齐 → "位置跟着机器人、只在走路时出现" = 自己的手。
- **判断错深度**：看它在机器人系里是否固定（机器人转身/走动时跟着相机走 = 错深度；map 系固定 = 实物）。
  "深度 / 同射线打到地面的深度"这个比值等价于高度（= 1 − z/相机高），**不是**独立证据。
- 用户偏好**实机直接测**（改完重启、实时盯），不要先跑长时间 bag 回放。
- 实时脚本在 Thor `/opt/G1/bags/replay/`（不进仓库）：`live_global_layers.py`（各层 254 来源 + 每团支撑）、
  `d435_near_watch.py`（D435 近身事件触发抓取，存 `d435_near/event_*.npz`）、`ghost_watch.py`（前方每团的高度/深度图行，
  标疑似错深度）、`live_d435_mon.py`、`low_obs.py`（矮障碍高度/离相机距离/每体素点数）。
- 参考包：`/opt/G1/bags/d435near_0924`（修手之前，41 次）、`d435near2_0924`（修手之后），`hang_0924`（吊着，机身回波）、
  `move_0924`（错深度 + 飞点）。

### 当前生效的相关配置（改之前先读这里）
- D435：`realsense_mark` 1.5 m / ≥0.15 m / voxel_min_points 4；`realsense_mark_tall` 0.7~1.8 m / 2.5 m；两者都吃 `mark_points`；
  `realsense_clear` 吃原始点云；`stvl_voxel_layer.update_footprint_enabled: false`（别开）。
- `d435_mark_filter.yaml`：min_depth 0.40、neighbor 开（0.08 / 3）、temporal 关、self_crop 0.5 m × 0.45~0.85 m、floating 关。
- MID360：`scan_range_filter` min_range 0.25 + base_link 水平 0.5 m 剔除；`livox_mark` ≥0.15 m / 2.5 m；
  global `observation_persistence 0.2`（global 5 Hz，只攒一个周期）。
- 以上都有 `aid_navigation2/test/test_g1_direct_config.py` 断言守着；改参数要同步改测试，并写明实测理由。

## 绝对禁止后退（最高优先级）

**G1 收到负的前进速度会直接摔倒。** 三道防线都不得放宽，`test_never_reverse` 守着：
`vx_min: 0.0` + `min_velocity[0]: 0.0`；BT 里的 `BackUp` 恢复节点**已删除**（`backup`/
`drive_on_heading` 同理不得进 BT）；`g1_cmdvel_to_sport` 对 `vx<0` 硬钳位为 0。
被困时只能靠 清图/Spin/Wait。

## 到点精度：G1 不能原地转（2026-10-03 仿真验证；10-06 已部署 128.146，未实机跑点）
- **⚠ 2026-10-06 推翻："G1 不能原地转"是错的**，只是 `cmdvel_to_sport` 的 `turn_min_vx: 0.2` 强加的（09-28 整合时写的注释，无实测）。
  128.146 遥控原地转 8 次，转前/转后激光对地图核对（Thor `spin_watch.py` + `fit_once.py`，数据 `/opt/G1/bags/replay/spin_1006/`）：
  31° 6.8 cm、58° 7.8、94° 4.2、100° 10.1、158° 16.8、330° 6.6 cm（每次约 5~7 cm 固定量，大角度更多）。腿式里程计在原地踏步时估不准位移。
  已改 `turn_min_vx: 0.0` + 规划换回 SmacPlanner2D + RPP `use_rotate_to_heading: true`（起步/到点原地转）。下面几条是此前的推理，前提已不成立。
- `g1_cmdvel_to_sport` 改写指令：|wz|≥0.15 → wz≥0.6 且 vx≥turn_min_vx（原 0.2，走 0.22~0.5 m 半径圆弧；现 0）；vx≥0.05 → vx≥0.3；
  其余 → 停。"慢速靠近"做不到（min_vx 0.3 同样无实测依据，未验证）。
- 实机 16:40 去 (-4.21,0.16,0°)：SmacPlanner2D 的路径没朝向，终点调头 180° 走成圆弧，横移 ~0.5 m；`goal_checker stateful: true`
  位置一进 0.2 m 就锁定，照样报到达。用户实测 `stateful: false` 会到不了点（不能慢速微调，来回绕）——**别再提**。
- 停车后多转 15~27°：速度平滑器 wz 减速 0.6 rad/s² 的减速段被抬回 0.6 rad/s。
- 改法（`aid_navigation2/param/nav2_params.yaml`）：规划器换 **SmacPlannerHybrid（DUBIN，最小转弯半径 0.40）**，
  平滑器 `max_decel` 改 **[-2.0, -0.35, -5.0]**。仿真到点误差 0.16~0.47 m / 3~27° → 0.08~0.22 m / 0~8°。
  剩下的是 0.3 m/s 最小速度造成的冲过头。仿真在 `src/test/nav_goal_sim/`（humble Nav2 + 照搬抬速规则的 G1 运动学），见其 README。
- 10-06 部署 128.146 时发现其 install 里 nav2_params 被就地改成 `stateful: false`（10-03 16:46 试验残留），已随部署恢复为 true。
- **10-06 实机 10 次（128.146，前台↔厕所交替）**：站稳后位置误差 厕所 0.14~0.31 m（均 0.24）、前台 0.34~0.42 m（均 0.38），航向都 ≤6°。
  根因不在规划器：Hybrid-A* 的 /plan 正确（开过目标再绕 0.4 m 小圈对准朝向进点），但 **MPPI 在离目标 1.4 m 内抄近路直奔终点**
  （`GoalCritic`/`PathFollowCritic` 的 `threshold_to_consider: 1.4`，终点圈整个在 1.4 m 内）→ 以错误朝向到点（差 150~230°）→
  stateful 锁位置 → MPPI 发 vx=0、wz=0.9 原地转 3 s → cmdvel_to_sport 抬成圆弧 → 甩出 0.3~0.4 m。仿真同参数，只是低估了横移。
  待选：降 GoalCritic/PathFollow 阈值到 ~0.3 m，或换 Regulated Pure Pursuit（use_rotate_to_heading: false）。数据：Thor `/opt/G1/bags/replay/goal_err_1006/`
  （runs.jsonl 每轮轨迹、plans.jsonl 路径、cmds.jsonl 速度；ana.py <轮次> 打印轨迹对路径）。
  **注意：这 10 次的定位本身也偏了**：测完后（机器人未动、腿式里程计重启前后完全一致）重启定位，新旧位姿差 0.25 m；激光核对新位姿吻合 82%、
  旧位姿 48%（剖面单峰在新位姿）。输出−LIO 的地图修正量在 10:56 第一次到前台时跳到 ~0.19 m，之后在前台一侧保持 0.15~0.25 m（厕所一侧 0.08~0.16），
  守护没报（每 60 s 窗口增量 <0.2 m）。所以上面的误差是"相对一个本身偏了 ~0.2 m 的定位"量的，物理到点误差未知。
  第二轮起 Thor `fit_watch.py` 在每次到点站稳后用激光核对定位（FIT 行，~5 s），区分导航误差和定位误差。
- **RPP + Hybrid（10-06 11:36 部署）**：厕所 0.13 m、前台 0.18 m（定位误差 1~2 cm），但前台调头圈空间不够：4 次 Hybrid 重规划
  "exceeded maximum iterations"、RPP "collision ahead"、BT Spin 恢复，46 s。之后改为原地转方案（见上），14:07 已部署 128.146（turn_min_vx 0、2D、RPP 原地转）。
- **原地转方案实测（10-06 14:11~14:18，128.146，10 次，数据 Thor `/opt/G1/bags/replay/goal_err_1006c/`）**：`turn_min_vx 0` + SmacPlanner2D
  + RPP `use_rotate_to_heading`。行为正确：起步原地转（vx 0 / wz 0.6）→ 直线 → 进 0.2 m 后原地转到朝向，无调头圈、无重规划，19~28 s。
  站稳误差（定位误差每次 1~4 cm，已激光核对）：厕所 0.129~0.146（均 0.136 m，航向 -6~-8°），前台 0.137~0.324（均 0.229 m，航向 0~6°）。
  剩余误差两项：① `xy_goal_tolerance 0.20` → 进 0.2 m 就停下转（前台全偏在 x 向，提前 0.14~0.2 m）；② 原地转 90°（顺时针）时身体后退
  5~17 cm（第 10 次 0.19 → 0.35 m）。航向 -6~-8° 是 `yaw_goal_tolerance 0.20`（11.5°）内就停转。下一步：两个容差收紧到 0.10。
- **容差 0.10 实测（10-06 14:23~14:31，10 次，数据 `goal_err_1006d/`）**：xy/yaw_goal_tolerance 0.20 → 0.10。按激光真实位置：
  厕所 9.5/12.6/14.9/19.8/15.9 cm（均 14.5），前台 7.6/12.7/7.3/11.1/18.0 cm（均 11.3），全体均 12.9 cm、最大 19.8，航向全部 ≤2.8°。
  第 1 次去厕所时 NDT 又沿走廊把输出拉偏 ~0.2 m，守护 14:24:08 冻结 + 回退 0.19 m，此后一直 NDT_SUSPECT / NEED RELOC（纯 LIO），
  激光核对定位偏 5~7 cm、方向固定 (-0.05, -0.04)：厕所侧与导航偏差同向叠加，所以厕所侧实际误差偏大。
  剩余误差主要是到点后原地转时身体位移（前台转 90° 常从 0.06 被甩到 0.2~0.3，站稳时又收回一部分）。
- 风险（Hybrid 时期）：调头要 ~1.8 m 宽的空地（更窄处规划器会去附近调头或规划失败）；实机停车距离、Hybrid 在 STVL 障碍下的规划耗时要实测。

## 定位链路（读源码确认，别再算错）

- `lidar_loc` 的 `balance_factor: 0.1`：每次只把 **NDT 残差的 10%** 注入输出
  （`lidar_loc.cc:675`）。glog 里 `loc using lo guess` 与 `confidence: ..., t:` 之差是
  **原始残差**，实际注入要再乘 0.1，否则幅度夸大 10 倍。
  推论：持续运动时输出对 NDT 解有 τ≈10 s 的系统性滞后 →「走路 p25 15 cm、停下回 0」。
- `track_min_score` 默认 **0.0**，`loc_monitor.py` 里的 "confidence<1.3" 是经验值，**不是定位器门限**。
- `sync package failed` 只是 IMU 时间还没覆盖雷达帧尾，下帧会补跑，**不等于丢包或过载**。
- 判断航向跳变要用**实际时间戳差**算角速度，不能假定固定 0.1 s/帧。
- 写点云分析脚本必须按**传感器距离**过滤（mid360 每帧约 40% 的点打在自己头上，
  costmap 靠 `obstacle_min_range: 0.25` 滤掉）。现成脚本：`tools/field_test/loc_monitor.py`
  （定位+导航常驻监控，注意它的 costmap 与位姿未做时间对齐）、`tools/nav2_debug/`。
- **出现问题先录包**：`tools/field_test/record_nav_bag.sh [名字] [-t 秒] [--light|--full]`，
  话题清单按"每个分析实际需要什么"分组写在脚本注释里，只录当前在线的。
  默认不压缩（zstd 吃 CPU，而 CPU 竞争本身是嫌疑之一）；`--light` 去掉原始传感器；
  **定位问题用 `--loc`**（livox 原始云 + IMU + /odom + TF，sqlite3，可直接给 run_loc_offline 回放）。
  2026-09-21 那次排查全程没录包，只能靠 glog 反推，很多结论无法复验——别再犯。

## 定位漂移 / 地图双解 / 一致性守护（2026-09-29）⚠ 重要

### 现象与结论
机器人回到原点，`/base_link_pose` 偏 1.9 m / 19°，glog 无任何告警，NDT 分值仍有 1.96。**根因在地图不在 LIO**：
- LIO 85 分钟回起点只差 9 cm；漂移全部来自 lidar_loc 的地图修正。
- 旧图 `1789975513067` 走廊尽头**双解**：横向 / 航向偏 0.1 m 左右，纵向最优解就在 x≈-4.5（对）和 x≈-5.0（错）之间跳。
  同一个包：实机落在 -4.5、离线回放落在 -4.97（=上午出事时的值），激光吻合度 49% vs 24%。
  该图行走区有大量现场已不存在的点团，只有一面沿走廊的长直墙约束纵向。
- **离线回放对这张图不具代表性**：同一包在线 NDT 残差 p90 0.064 m，离线 p90 0.42 m。别拿离线结论直接套实机。

### 排查方法（下次先做这几步）
- **|loc−起点| 与 |LIO−起点| 对比**（与旋转无关）：两者一致说明地图修正正常，差值变大 = 地图匹配在拉偏。
  glog 里 `current lo pose`（LIO）和 `confidence ... t:`（NDT 结果）成对解析即可。
- **激光-地图吻合度核对**：Thor `/opt/G1/bags/replay/loc_check.py <map_dir>`（抓当前 scan，体素查表，
  在当前位姿/原点附近搜 x-y-yaw，并打 x/y 剖面看是否单峰）。点云地图地面在建图雷达高度下方，z 偏移约 -1.1~-1.25。
  `bag_fit.py`：从包里取某时刻的 scan 比较候选位姿。`walk_monitor.py`：行走中每 3 s 核对（第 46 行订阅回调要写成
  `lambda m: self.q.append(m)`，当前 Thor 上是坏的版本）。
- NDT 分值（confidence）**不能**当作位置正确的依据。

### lightning 上游的相关事实（已对照 github 上游 1325fed 确认，不是我们改坏的）
- `Localize()` 永远返回 true（上游 issue #127，作者：原本靠 RTK 重置，开源版"定位丢了没办法处理"）→ 普通初始化不看分数，
  开机朝向偏 ~20° 会静默初始化到错误位置。
- `YawSearch` 上游注释掉（#123：会选错角度）；现在只在 `/initialpose` 重定位时用，±15°/3°，粗匹配前 3 名做精匹配后按精分判定
  （粗分辨率 5 m 的分值不能和 min_init_confidence 比）。
- 跟踪 NDT 在 `UpdateGlobalMap` 里是 4 次迭代、步长 0.1 → 单帧最多挪 ~0.6 m；残差卡在 0.55~0.6 m = 饱和，不是收敛。
- 退化检测 / DR 多初值比较上游都注释掉了；lightning 的 "DR" 就是 LIO 自身 IMU 状态（`localization.cpp` "没有 odom 用 lio 替代"），不独立。
- 相关 issue：#65（倒装 MID360 走廊漂移）、#5（退化场景，作者：引入里程计就好解决）、#122（NDT 高分误匹配，ICP 二次校验有效）。

### 本轮加的代码（已部署 Thor，未提交）
- `lidar_loc/loc_guard.{h,cc}`：窗口（20 s 或 LIO 行程 5 m）内比较"输出相对运动"与"LIO 相对运动"，超 0.30 m / 5° 报警；
  腿式里程计 `/odom` 投票区分 NDT 错 / LIO 错。yaml 独立 `loc_guard:` 段（不改 `lidar_loc` 段）。
  **当前 `action: warn` 只告警**：离线注入 0.55 m 偏差 A/B，freeze 只把 16 m 压到 5 m——里程计阈值太紧（正常行走 18% 窗口
  判不一致）导致冻结 / 解冻来回切约 100 次，冻结后残差 >0.15 m 也无法自动恢复。要再开 freeze 必须先改这两点。
- 输出 `/lightning/loc_status`（JSON）；glog 每帧 `loc guard frame:`（lo / out / odom 平面位姿、drift、odom_err），离线分析直接用。
- `/initialpose` 订阅（map 系 base_link 平面位姿 → `BaseTFPublisher::BasePoseToLidar`），成功后 `pgo_->Reset()`；
  重定位失败时不退回功能点初始化。**未实机验证。**
- `run_loc_offline`：`--odom_topic/--odom_time_offset/--odom_lidar_offset`，测试专用 `--inject_ndt_bias "bx by t0 t1"`。
- 测试：`test/test_loc_guard.cc`（gtest，闭环模拟走廊漂移）、`test/test_g1_loc_guard_config.py`。
  `test_g1_g2p5_config.py` 有 3 个**早已存在**的失败（URDF 关节已改名 `body_to_mid360_joint`），与本轮无关。

### 腿式里程计 `/odom`（sport_to_odom）实测（locguard_0929 包，20 min）
- 本体时钟与 Thor 差 <30 ms（已按前 50 条估计补偿）。只在 Nav2 运行时有（随 g1_navigation.launch 启动）。
- 与 LIO 比（20 s / 5 m 窗口）：平移误差 p50 6.7 cm、p95 28 cm；误差/行程 p50 4.9%、p95 10.9%；航向 p50 1.3°、p95 6°。
  全程路径比 LIO 短 5.5%；长期航向很稳（20 min 差 2°）。守护阈值应放宽到约 0.15 m + 15%×行程、10°，并加连续帧滞回。

### 新地图 1790676615248（2026-09-29 18:07 建，~3 min）
- 3D 点云好：实时激光吻合 97%+（近处到 20 m），单层墙；建图无 IMU 断档，回环修正 ≤0.14 m。
- **2D 栅格 `map.pgm` 相对 3D 转了 ~1.25°**（远处墙偏 0.3~0.7 m，用户看到的"双层墙"），影响网页和 Nav2 静态层，不影响定位。
  原因未查明：g2p5 用的也是 `kf->GetOptPose()`，回环后会 `RedrawGlobalMap`，看上去同源。下一步从 3D 图重生成栅格对比。
- 地图原点 = 建图进程启动时雷达位姿；切模式时 G1 会挪步，放回"原点"差 30 cm / 2° 正常。坐标系与旧图差 ~20°，点位需重标。
- 走廊是否还有双解**还没在新图上验证**（下次：新图走走廊 + walk_monitor；locguard_0929 包新图离线回放）。
- 包：`/opt/G1/bags/locguard_0929`（旧图 20 min 行走，完整）、`/opt/G1/bags/mapping_corridor_0929`（覆盖建图，09-30 已 reindex 补 metadata）。

### 新图走廊实测（2026-09-30）——新图没解决，**导航时必现**
- **导航控制**（~0.3 m/s，两端快速转身）下两次复现：输出沿走廊被往原点方向拉 1.46 m（15:03）、2.2 m（15:37，停下后还在拉），
  全程 LIO 与腿式里程计互差 ≈0.1 m → 错在地图匹配。手动遥控慢走、原地快速转圈都只偏 0.1~0.2 m。
- 走行中 NDT 残差常卡 0.55~0.6（饱和）；静止时也出现过一次：14:51:08 残差突跳 0.28 m，40 s 内把输出拉走 0.19 m，守护没报（每 20 s 窗口都 <0.30）。
- **不是 DDS**：进程 URI=cyclonedds_g1.xml（spdp）、enP2p1s0 发送 15 KB/s、`abnormal dt` 0 次、UDP drops 0；且 LIO 全程准。
  未排除：导航时 CPU 竞争让 lidar_loc 跟不上 → 离线回放可区分（回放无 Nav2 抢 CPU）。
- 守护每次都判对（NDT_SUSPECT + odom_agree=1），但转身时腿式里程计误差大（导航段 y 向达 0.5 m），NDT_SUSPECT/LIO_SUSPECT 来回切。
- 包（--loc，sqlite3，含 /odom）：`/opt/G1/bags/corridor_newmap_0930`（14:45~15:15，第 1 次复现）、`corridor_newmap_0930b`（15:25~15:42，手动+导航第 2 次复现）。
  分析脚本 Thor `/opt/G1/bags/replay/walk_attrib.py <glog>`（输出 / 纯 LIO / 纯里程计三路对比）、`live_attrib.py`（实时版）。
- **NDT 输入不是纯当前帧**：`localization.cpp` 用 `lio_->GetProjCloud()` = 当前帧 + 最多 5 个历史关键帧各取最早 1001 点
  （按 LIO 相对位姿摆过来）；`fasterlio.proj_kfs` 管不到（上游判断被注释）。原实现原地追加到 `scan_undistort_`，而关键帧与它共用
  CloudPtr → 当前帧是新关键帧时遍历自身同时 push_back（未定义行为）。09-30 改为副本上投影并跳过自身，新增 `loc_input.proj_kfs`
  开关（默认 true = 上游行为）做实机 A/B（开—关—开、每轮原点重启，统计用 Thor `ab_stats.py <glog> t0 t1`）。对漂移的贡献**尚未确定**。
  另：`laser_mapping.cc` MakeKF 的 `20 / 180 * M_PI` 是整数除法（=0），投影关键帧只按 3 m 更新；修它会改关键帧策略，未动。
- **离线 A/B（0930b 包，16:52，结果在 Thor `/opt/G1/bags/replay/ab0930/`）**：回放无 Nav2 也复现拉偏（峰值 1.32 m，实机 2.2 m）→ CPU 不是主因。
  末帧对原点激光真值：只告警+投影 0.36 m；只告警+无投影 **3.14 m**（纯当前帧更差，投影不是元凶）；冻结+回退+投影 **0.06 m**（导航段最大 0.23 m）。
  0930 包的四组回放 16:52 后在跑，结果未分析。16:54 已部署到 128.146（freeze + rollback、proj 开），**尚未实机导航验证**。
- lightning glog 现在由自启脚本设到 `/opt/G1/logs/glog`（原来在 /tmp，重启即丢，09-30 丢过一次）。

### 112.70 "漂移"（2026-10-06）——不是漂移，是重启定位从原点初始化
- 09:50:30 第一次进定位：机器人在原点，初始化正确（conf 2.6）；走到约 (-5.5,-2.0,21°)。
- 09:52:05、09:52:22 前端两次 `mode_set localization`（各接着一次 patrol）：`ModeSet("localization")` 在已定位时会
  **停掉再重启** g1_localization，而 `run_loc_online.cc:72` 永远 `SetInitPose(SE3())` = 地图原点。机器人离原点 5 m，
  NDT 收敛到错误局部最优（(-0.29,0.03,27°) conf 2.08、(-1.45,-0.25,9°) conf 1.78；Localize() 不看分数照单全收）。
- 激光核对（loc_check.py）：输出位姿吻合 30%/49%，真值 (-3.92,-2.05,30°) 吻合 74%/97.5% → 偏 ~4 m / 12°。
- 守护 09:53:25 冻结后一直 NEED RELOC（正确报警，但前端没显示）；PGO 刷 Cholesky failure（错误初始化后的副作用）。
- 用户确认：是有人在非原点位置手动重启了定位，属操作原因，不需要处理（10-06）。若以后要防呆：重启定位用上次位姿初始化 / 前端提示 need_reloc。
