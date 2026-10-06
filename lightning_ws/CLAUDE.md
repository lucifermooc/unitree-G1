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

---

# G1 导航栈工作手册 —— v1.0.0（2026-10-06）

本文件是接手者（Claude 会话或工程师）的经验库。**先看下面的总目录**：每一点一句话说清结论，要细节按编号去对应章节。
维护规则：新问题解决后按"现象 / 根因 / 解决 / 验证"补到第 3 章对应小节，并在总目录加一行；没解决的写进第 4 章。
结论要附实测数据和数据位置；过时的说法直接改掉，不要在后面追加"推翻"。

## 总目录（缩小版）

**1. 系统与机器现状**
- 1.1 数据链路：MID360+IMU → lightning（LIO + NDT + 守护）→ Nav2（2D 规划 + RPP，STVL costmap）→ 速度桥 → G1；网页控制台跑在电脑上。
- 1.2 机器：128.146 是主测机、v1.0.0 已全部部署；112.70 还是 09-30 的代码；开发机不编译。
- 1.3 v1.0.0 指标：到点平均 12.9 cm、最大 19.8 cm、航向 ≤2.8°；定位正常时 1~4 cm；costmap 残影和近身噪点清零。

**2. 硬性规则（动手前必读）**
- 2.1 绝对禁止后退：G1 收到负 vx 会摔倒，四道防线不得放宽。
- 2.2 编译与部署：只在 Thor 编译；部署要用户同意；用 `deploy_to_robot.sh`；运行时只能引用 install（src 会被删）。
- 2.3 重启：只用 `g1_service.sh restart`；**重启前机器人必须在原点、朝向约 0°，重启后用激光核对定位**。
- 2.4 DDS：启动栈和手动跑 `ros2` 前都要 source `dds_env.sh`。
- 2.5 坐标系与传感器：TF 结构、高度基准、MID360 倒装且正前方被头挡住、D435 下俯 47.6°。
- 2.6 G1 运动特性：**能原地转**（转一次偏 4~17 cm）；`min_vx 0.3` 没验证；`stateful` 必须为 true。

**3. 问题与解决记录**
- 3.1 建图与 LIO
  - 3.1.1 静止时位姿漂走 → 雷达系 Z 下限填成正数，点被裁光；改 −2.0。
  - 3.1.2 IMU 外参只对 x 取了负号 → 三个分量都取负。
  - 3.1.3 快转后地图多层墙 → IMU 断档；去掉绑核、关 UI、IMU 队列 1000、装 DDS 缓冲配置。
  - 3.1.4 回环修正跳动 → 用 20 帧子图做回环源，只用 x/y/航向修正。
  - 3.1.5 Thor 编译 lightning 失败 → `export PYTHONNOUSERSITE=1`。
  - 3.1.6 新地图 1790676615248 → 3D 好，2D 栅格转了 1.25°（未解决）。
- 3.2 定位
  - 3.2.1 静止时 roll 被拉歪 → 用 LIO 重力约束。
  - 3.2.2 定位跳变 → DDS 组播把点云挤上网线；改 spdp。
  - 3.2.3 每帧定位反而漂移 → 保持只在关键帧定位，启动用快速收敛。
  - 3.2.4 LIO 恒滞后 0.35 s → 补跑积压帧。
  - 3.2.5 段错误日志丢失 → glog 逐条落盘，放到 `/opt/G1/logs/glog`。
  - 3.2.6 走廊地图双解 / 导航时被拉偏 → 加一致性守护（冻结 + 回退）和单帧跳变检查；**部分解决**。
  - 3.2.7 112.70 偏 4 m → 不是漂移，是在非原点重启了定位（操作原因）。
  - 3.2.8 10-06 导航时又被拉偏两次 → 一次守护没发现，一次冻结后 NEED RELOC；**未解决**。
- 3.3 Costmap 障碍物
  - 3.3.1 动态障碍残影清不掉 → MID360 盲区无法清除；D435 做只清除源，补标高物体。
  - 3.3.2 MID360 迁到 3D STVL → 加 `scan_range_filter` 去机身回波。
  - 3.3.3 "随时都是障碍物" → 飞点、前摆的手、机身回波、地面回波分别处理，近身噪点清零。
  - 3.3.4 试过并否定的方案 → 降高度阈值、放宽 1.5 m、删顶部行、帧间滤波等，别再试。
  - 3.3.5 当前 costmap 配置 → 改之前先读，测试守着。
- 3.4 导航与到点精度
  - 3.4.1 现象 → Nav2 报到达，人却偏 0.3~0.5 m。
  - 3.4.2 演进过程 → MPPI 0.31 m → RPP+Hybrid 0.13~0.18 m → 原地转 0.18 m → 容差 0.1 后 **0.13 m**。
  - 3.4.3 根因 → "不能原地转"是桥接自己加的；MPPI 近目标抄近路；容差就是误差下限。
  - 3.4.4 v1.0.0 配置 → SmacPlanner2D + RPP 原地转 + 容差 0.10；MPPI 参数保留可回退。
  - 3.4.5 别再试 → `stateful: false`、MPPI 配调头圈、Hybrid 调头圈。
  - 3.4.6 仿真 → `src/test/nav_goal_sim`，会低估横移，新方案没重跑。
- 3.5 桥接与系统
  - 3.5.1 `/odom` 没有发布者 → Python 节点缺 `__main__`，启动即退出。
  - 3.5.2 开机自启失败 → unit 指向被删掉的 src；改为指向 install。
  - 3.5.3 planner_server 段错误 → 禁行区层加锁。
  - 3.5.4 录包停不下来 → `-d` 是分片时长；用 `timeout -s INT`。
  - 3.5.5 子节点变孤儿 → 一律用 `stop_all` / `g1_service.sh`。
  - 3.5.6 D435 点云停发 → 链接到了系统 NEON 版 SDK。
  - 3.5.7 `/livox/points` 由 `point_filter` 产生 → LIO 和 Nav2 都不吃它。
  - 3.5.8 install 里的配置被手改过 → 部署前用 md5 核对。
  - 3.5.9 语义地图部署 → 模型从本地目录加载，共用已有的 Qdrant 容器。
- 3.6 09-17 交接里的遗留项 → RealSense 参数警告、禁行区节点重复等，状态未核实。

**4. v1.0.0 已知问题（按优先级）**
- 4.1 导航时 NDT 沿走廊拉偏定位（最重要）；守护抓不到慢拉、冻结后不会自动恢复、前端不显示。
- 4.2 定位总从地图原点初始化；**128.146 当前偏 0.33 m，用前先重定位**。
- 4.3 到点原地转时身体位移，是剩余到点误差的主要来源。
- 4.4 RPP 不会绕障，没测过有人挡路。
- 4.5 新导航方案没有仿真。
- 4.6 `min_vx 0.3` 没验证，造成进点冲过头。
- 4.7 2D 栅格相对 3D 转了 1.25°。
- 4.8 global costmap 对矮障碍看不全。
- 4.9 112.70 没更新。
- 4.10 Thor 终端的 DDS 参与者编号不够。
- 4.11 其他小问题。

**5. 排查方法与工具**
- 5.1 出问题先录包（定位问题用 `--loc`）。
- 5.2 定位：**激光对地图核对是唯一可信的真值**；对比"输出 − LIO"区分地图匹配和 LIO 的问题。
- 5.3 到点精度测量：`goal_err.py` + `plan_rec.py` + `fit_watch.py`，误差按激光真实位置算。
- 5.4 Costmap：先分层、再找来源；离线回放台。
- 5.5 远程操作的坑：`pgrep/pkill -f` 会杀掉自己的 ssh 会话等。

**6. 参考**
- 6.1 录包和测量数据目录；6.2 lightning 上游事实；6.3 其他文档。

---

## 1. 系统与机器现状（v1.0.0）

### 1.1 数据链路
```text
MID360 /livox/lidar（CustomMsg，逐点 offset_time）+ IMU
  → lightning run_loc_online：LIO + NDT 地图匹配 + 一致性守护 loc_guard
  → TF map->base_link（平面）、base_link->body_link（躯干倾斜）；/base_link_pose
  → Nav2：SmacPlanner2D 全局规划 + Regulated Pure Pursuit 控制（起步/到点原地转向）
           costmap 用 STVL（MID360 + D435），global 另有静态层和禁行区
  → /cmd_vel → velocity_smoother → collision_monitor → /cmd_vel_safe
  → g1_cmdvel_to_sport（最小有效速度改写）→ Unitree LocoClient::SetVelocity
```
- 障碍点云：MID360 的 `/lightning/registered_scan` → `scan_range_filter` → `/lightning/registered_scan_nav`；
  D435 原始点云 → `d435_mark_filter` → `/camera/camera/depth/mark_points`（标记），原始点云另做清除源。
- 本体：`/odommodestate` → `sport_to_odom` → `/odom`（不发 TF，只在导航模式运行）；`/lf/bmsstate` → `/battery_state`。
- 模式管理：`robot.launch.py`（`start_backend:=true`）里由 robot_status_manager 经 launch_manager 启停定位/导航/建图，
  前端切模式走 `/mode_set`；地图取数据库当前地图（`~/maps/db.sqlite`），取不到才用 `map_dir`。
  `start_backend:=false` 时才由顶层 launch 直接启动定位和导航（前端就不能切模式了）。
- 语义地图 `semantic_map_ros`（BGE-M3 + Qdrant）跑在 Thor 上，开机自启服务 `g1-semantic-map`。
- 网页控制台 `src/g1_web` 跑在电脑上：`python3 serve.py 8080`，页面里连 `ws://<机器人IP>:9090`（rosbridge）。

### 1.2 机器
| 机器 | 状态 |
|---|---|
| **Thor 128.146**（`unitree@192.168.128.146`，`/opt/G1/lighting_ws`，aarch64 / Ubuntu 24.04 / jazzy） | 主测机。v1.0.0 全部已部署（10-06）。sudo 要密码（服务启停免密）；开机自启 unit 指向 install；D435 序列号与默认值一致 |
| **Thor 112.70**（`unitree@192.168.112.70`，tegra-ubuntu，wlP1p1s0） | **还是 09-30 的代码**，开机自启 unit 仍指向 src（删 src 后会起不来）。sudo 免密。D435i 序列号 317622075180，写在 `/opt/G1/g1_autostart.local.env` |
| 开发机（本地） | x86 / Ubuntu 22.04 / humble。**不在本地编译**，本地 install 不完整、不可用。Meteor Lake 核显在 6.8 内核下会 GPU hang 导致整机卡死（`journalctl -b -1 -k | grep -E 'i915|DMAR|GPU HANG'`） |

### 1.3 v1.0.0 实测指标（128.146，2026-10-06）
| 项目 | 结果 | 数据 |
|---|---|---|
| 到点精度（前台↔厕所 10 次，按激光真实位置） | 平均 12.9 cm、最大 19.8 cm，航向 ≤2.8°，10/10 成功，单程 21~30 s | Thor `/opt/G1/bags/replay/goal_err_1006d/` |
| 定位（正常时，到点后激光核对） | 1~4 cm | `goal_err_1006c/fit.txt` |
| costmap 动态障碍残影 | local 0（清除延迟 p90 0.3 s），global 约 2 s 清掉 | 见 3.3 |
| costmap 近身噪点 | base_link 0.5 m 内 D435 致命格 0；走路时手的误标 41 次 → 0 | 见 3.3 |

## 2. 硬性规则和固定事实（不要重新验证）

### 2.1 安全：绝对禁止后退（最高优先级）
**G1 收到负的前进速度会直接摔倒。** 以下防线都不得放宽，`test_never_reverse` 等测试守着：
- RPP `allow_reversing: false`；保留的 MPPI 参数 `vx_min: 0.0`；velocity_smoother `min_velocity[0]: 0.0`。
- BT 里的 `BackUp` 恢复节点已删除，`backup` / `drive_on_heading` 同样不得进 BT。被困时只能靠清图 / Spin / Wait。
- `g1_cmdvel_to_sport` 对 `vx<0` 硬钳位为 0。
- 规划器不能用 REEDS_SHEPP 之类会规划倒车的模型。

### 2.2 编译与部署
- **不在本地编译**（用户明确要求）：本地只改代码，编译、跑 gtest/pytest 一律在 Thor 上做；Thor 离线就如实说"未编译"。
- **部署前必须征得用户同意**。验证时不主动发导航目标、不发速度；到点测试由用户在前端发点，我们只订阅测量。
- 部署脚本：`tools/deploy/deploy_to_robot.sh <IP> [包名 ...]`（默认 lightning semantic_map_ros robot_bringup）：
  rsync 按内容同步 → `g1_service.sh stop` → 只编译指定包 → `start` → 打印守护状态。
  排除 `bin/`、`thirdparty/`、`script/g1_humble_env.sh`。重启会让定位重新初始化，见 2.3。
- **部署后 Thor 上的 src 会被删除**（用户的部署方式）：systemd unit、自启脚本、要 source 的文件、文档里的命令
  只能引用 `install/`，不能写 `$G1_WS/src/...`。
- 不能把本地 x86 二进制同步到 Thor：rsync 一律 `--exclude='bin/'`（lightning 可执行文件已输出到 `build/lightning/bin`，
  但 Thor 上旧的 `src/lightning-lm/bin` 可能还在）。
- 编译不要加 `--symlink-install`（与 `build.sh jazzy` 冲突，报 "failed to create symbolic link ... Is a directory"）。
- **Thor 上编译 lightning 必须 `export PYTHONNOUSERSITE=1`**（`~/.local` 里有 setuptools 79，不要去改它）。
- **栈在运行时不要 `colcon build` lightning**：install 会原地覆盖正在被 run_loc_online 使用的 .so。
  只要离线工具时用 `cmake --build build/lightning --target run_loc_offline test_loc_guard`（不 install），或先停栈。
- **部署前核对 install 与 src**：10-03 有人在 128.146 的 install 里就地改了 `nav2_params.yaml`（`stateful: false`），
  部署会覆盖掉。用 md5 对比 `install/<包>/share/<包>/...` 与本地文件。
- Thor 上跑 pytest：把需要的源码 rsync 到 `/opt/G1/bags/replay/pytest_src`（aid_navigation2、g1_nav_bridge、
  point_filter/config、robot_bringup/launch|param、lightning-lm/src/core/system/slam.cc、conftest.py、pytest.ini），
  先 `source /opt/ros/jazzy/setup.bash; source /opt/G1/lighting_ws/install/setup.bash`，再
  `python3 -m pytest -q aid_navigation2/test/test_g1_direct_config.py`。
- 仓库卫生：`src/.gitignore` 挡住 build/install/log/bin、`__pycache__`、`.pytest_cache`、日志、录包；
  `src/pytest.ini`（`-p no:cacheprovider`）+ `src/conftest.py`（`sys.dont_write_bytecode`）让测试不留缓存；直接跑 Python 用 `python3 -B`。临时脚本和分析输出放 scratchpad 或 Thor 的 `/opt/G1/bags/replay/`，不进 `src/`、`tools/`。
  测试类功能（VLN 等）单独成包放在 `src/test/` 或独立工作区，不改现有代码。

### 2.3 启动、重启与定位初始化
- **重启的唯一认可方式**：`bash /opt/G1/lighting_ws/install/robot_bringup/share/robot_bringup/script/g1_service.sh restart`
  （= 停服务 + stop_all 清理残留进程 + 启服务），或部署脚本。手动方式 `stop_all.launch.py` → `robot.launch.py` 也可用
  （服务是 `Restart=no`，不会抢着起），但日志会分在两处。
- **不要用 `mode_set idle/patrol` 之类的局部重启代替**：只重启 Nav2，其它进程仍是旧状态，参数半新半旧会误判实验结果。
- 开机自启：`g1_service.sh install` 装 `g1-robot`、`g1-semantic-map` 两个服务和 `/etc/sudoers.d/g1-autostart`
  （只放行这两个服务的启停）。参数在 `g1_autostart.env`，本机专属参数在 `/opt/G1/g1_autostart.local.env`。
  日志：`/opt/G1/logs/*_latest.log`；lightning glog：`/opt/G1/logs/glog/`。
- **重启前机器人必须在地图原点附近、朝向约 0°**：`run_loc_online.cc:72` 永远 `SetInitPose(SE3())`，从地图原点初始化，
  而上游 `Localize()` 不看分数、错了也照单全收。偏 ~5 m 或朝向偏 ~20° 都会静默初始化到错误位置（3.2.7）。
  **前端"定位"按钮（`ModeSet("localization")`）在已经定位时也会停掉重启定位**，同样从原点初始化。
- **重启后一定要用激光核对定位**（`/opt/G1/bags/replay/loc_check.py <map_dir>`，5.2）：在原点重启也出现过偏 0.33 m（4.2）。
- 地图原点 = 建图进程启动时的雷达位姿。切模式时 G1 会挪步，放回"原点"差 30 cm / 2° 属正常。
- 定位输出直接看 `/base_link_pose`（= map->base_link）。

### 2.4 DDS（CycloneDDS）
- **启动整套栈的 shell 必须 `source install/robot_bringup/share/robot_bringup/system/dds_env.sh`**
  （`cyclonedds_g1.xml`：spdp，组播只做发现、数据单播；参与者上限 100）。不能只 source `unitree_ros2/setup.sh`：
  它把 `CYCLONEDDS_URI` 盖回组播，同机进程间的点云也走网线，被交换机限在 ~10 MB/s（3.2.2 定位跳变的根因）。
  `robot.launch.py` 不会自己设 DDS，全看启动它的 shell。核对：`/proc/<pid>/environ` 里的 `CYCLONEDDS_URI`。
- 在 Thor 上手动跑 `ros2` 命令前同样先 source `dds_env.sh`：
  - 只 source `/opt/ros/jazzy` 时是默认 RMW，topic 能通但 param/service 调用全部超时（不是节点卡死）。
  - `~/.bashrc` 里的 `CYCLONEDDS_URI` 没设参与者上限，栈运行时（约 38 个参与者）新开的 `ros2` 命令会报
    "Failed to find a free participant index for domain 0"。
- 反复起临时 ROS 节点也会耗尽参与者编号，需要等释放。
- `60-dds-buffers.conf` 必须装进 `/etc/sysctl.d`（默认 UDP 接收缓冲装不下一帧 Livox 点云）。

### 2.5 坐标系、TF 与传感器几何
- `map->base_link`（平面，只含位置+航向）与 `base_link->body_link`（躯干实时倾斜）都由 LocSystem 动态发布；
  URDF 根是 `body_link`，mid360 / d435 挂在其下（官方外参，mid360 pitch=+2.3°，驱动已翻转）。
  G1 不同站姿头部相对重力差 3~4°，不要把某次站姿实测的倾斜写进 URDF。
- 高度基准：平面模式下 map->base_link 的 z 恒为 0（body_link 原点 = 脚下地面）。`g2p5.floor_height` 每张图不同，不能用于导航高度。
- `/lightning/registered_scan` 是 LIO 去畸变单帧，frame=mid360_link，stamp=该帧 lidar_end_time，下游按扫描时刻查 TF。
  不要用高频外推位姿把扫描摆到 map 系（差 0.1~0.5 s，转向时墙点被甩成满屏假障碍）。
- 定位 tilt 由 `lidar_loc.gravity_constrain` 用 LIO 重力锁定，目标方向读 `<map>/gravity_up.txt`；
  建图 `fasterlio.gravity_align_init` 让新地图 z 轴对齐重力并自动写该文件。
- **MID360 倒装**，仰角实测 −50.5°~+6.4°；**正前方 ±30° 被 G1 头部挡住**；每帧约 40% 的点打在机身上，分析脚本必须按传感器距离过滤。
- **D435**：±43°、高 1.24 m、光轴下俯 47.6°，画面上沿在水平线下 26.6°，2.5 m 外只拍得到地面（不能用于 VLN）。
- 前进方向近场 100% 靠 D435，30°~43° 两条侧带两者重叠。近处盲区 `d_min(h) = 0.14 + (1.24−h)/tan(75.2°)`（h=0 → 0.47 m）。
- 机身：卷尺实测 base_link 到机身最前方 0.10 m，前后 0.20 m；`robot_radius` 同时决定碰撞阈值、253 内切带和自清半径。

### 2.6 G1 运动特性（实测）
- **G1 能原地转向**（10-06 推翻了"不能原地转"的旧规则，那是 09-28 写进桥接的假设，没有实测）：
  遥控原地转 8 次、转前转后激光核对位移：31° 6.8 cm、58° 7.8、94° 4.2、100° 10.1、158° 16.8、330° 6.6 cm。
  每次有约 5~7 cm 的固定量，角度大时更多；导航中到点原地转 90° 时常被甩到 0.2~0.3 m，站稳后收回一部分。
- `g1_cmdvel_to_sport` 改写规则（`nav_bridge.yaml`）：`|wz|≥0.15` → `wz` 至少 0.6，`vx` 至少 `turn_min_vx`（现为 **0**，允许原地转）；
  直行 `vx≥0.05` → 至少 0.3；其余 → 停。**`min_vx 0.3`（不能慢速靠近）同样没有实测依据**，未验证。
- 腿式里程计 `/odom`：本体时钟与 Thor 差 <30 ms；和 LIO 比（20 s / 5 m 窗口）平移误差 p50 6.7 cm、p95 28 cm，
  误差/行程 p50 4.9%、p95 10.9%，航向 p50 1.3°、p95 6°；长期航向稳（20 min 差 2°）。**原地踏步时估不准位移**。
- `goal_checker stateful: true` 必须保留：用户实测 `false` 会到不了点（不能慢速微调，来回绕）——**别再提**。

## 3. 问题与解决记录（v1.0.0 之前）

每条格式：**现象 → 根因 → 解决 → 验证**。标"别再试"的是实测否定过的方案。

### 3.1 建图与 LIO

#### 3.1.1 静止时位姿匀速漂走（09-17）
- 现象：静止时 LIO 位姿以恒定速度漂走，曾漂到 z≈1158 m。
- 根因：`default_livox.yaml` 雷达系 Z 下限填了 1.1（`pointcloud_preprocess.cc:66` 直接按 `pt.z` 裁剪），而地面在雷达系 z≈−1.24，
  每帧 20000 点被裁到 20 点，LIO 只剩 IMU 推算。
- 解决：改为 −2.0（必须为负）。验证：每帧恢复约 5000 点，ESKF 拒绝次数 17 → 0。

#### 3.1.2 IMU 外参符号不自洽（09-17）
`extrinsic_T` 原来只对 x 取了负号。FastLIO 的 `extrinsic_T` 是雷达原点在 IMU 系的坐标，
  = 官方 IMU 相对雷达平移 (0.011, 0.02329, −0.04412) 取负 → `[-0.011, -0.02329, 0.04412]`。

#### 3.1.3 快转时地图出现多层墙（09-19）
- 现象：建图 glog 出现 `get abnormal dt`，快转时航向跳几十度，地图多层墙。
- 根因：IMU 没进到 LIO（源头 200 Hz 完好，录包进程收全），一次断 0.1~0.3 s。
- 解决（缺一不可，别回退）：① 不给任何节点加 `taskset` 绑核；② 建图/定位 `with_ui:=false`（Pangolin 软件渲染吃 1 核且随地图变慢）；
  ③ IMU 订阅队列 1000（本进程发布 ~1 MB 的 /map 时 DDS 接收被拖 0.3~1 s，积压 IMU 一次灌入，深度 100 会覆盖）；
  ④ `60-dds-buffers.conf` 装到 `/etc/sysctl.d`。
- 验证：`/opt/G1/bags/map500d_0919` 8 分钟建图 0 断档。查进程 socket 丢包看 `/proc/net/udp` 的 drops 列。

#### 3.1.4 回环修正跳动、墙体分层（09-19）
回环源点云用候选帧前后各 20 个关键帧拼的子图（`src_submap_range: 20`），
  回环只采用 NDT 的 x/y/航向修正（横滚/俯仰/高度保持 LIO）。`map500d_0919` 离线：回环 43 → 75 个，离群 2 → 0，平均修正 0.07 m / 0.7°，墙体单线。

#### 3.1.5 Thor 上 colcon 编译 lightning 失败（09-29）
部署语义地图时 pip 往 `~/.local` 装了 setuptools 79，thirdparty livox 的
  python egg 构建报 `InvalidVersion: ''`。解决：编译前 `export PYTHONNOUSERSITE=1`，不改 `~/.local`。

#### 3.1.6 新地图 1790676615248（09-29 建，当前在用）
3D 点云好（实时激光吻合 97%+，单层墙，无 IMU 断档，回环修正 ≤0.14 m）。
  坐标系与旧图 `1789975513067` 差 ~20°，点位要重标。2D 栅格有 1.25° 旋转（未解决，见 4.7）。

### 3.2 定位

#### 3.2.1 静止时 roll 被拉到 −7~−9°（09-18）
G1 站立处遮挡多，6DoF NDT 在 tilt 上退化。
  解决：`lidar_loc.gravity_constrain: true`，用 LIO 重力约束 NDT 的 roll/pitch，保留航向和平移。

#### 3.2.2 定位跳变（09-19）
- 根因：栈跑在组播 DDS 下，同机进程间的点云也走网线，被交换机限在 ~10 MB/s，D435 掉到 12 Hz、scan 7 Hz，定位输入滞后。
- 解决：启动栈的 shell 必须 source `dds_env.sh`（2.4）。核对：spdp 下 enP2p1s0 发送 ~72 KB/s，组播下 ~7 MB/s。

#### 3.2.3 改成每帧定位反而漂移（09-19）
`loc_on_kf: false`（每帧都跑 NDT）时，NDT 结果沿行进方向系统性落后 0.13 m，
  高频融合后累积成 7.6 m / 62° 漂移；`true`（只在 LIO 关键帧定位，原版行为）同数据 1.3 m / 7°。
  启动慢收敛另用 `fast_converge_*` 解决（NDT 分值 ≥1.5 时按 0.5 融合，连续 3 次残差够小后恢复 0.1）。

#### 3.2.4 LIO 整个运行期恒滞后 ~0.35 s（09-19）
IMU 晚到时 `SyncPackages` 失败，点云留在缓冲里，之后每来一帧仍只处理一帧，积压永远消不掉。
  解决：`localization.cpp` 把已就绪的积压帧补跑完（激光定位只用最新一帧）。

#### 3.2.5 段错误后 glog 只剩一行（09-19）
glog 默认缓冲 INFO 最长 30 s，进程被信号杀死时丢失。解决：`FLAGS_logbuflevel = -1`
  逐条落盘；glog 目录由自启脚本设到 `/opt/G1/logs/glog`（原来在 /tmp，重启即丢，09-30 丢过一次）。

#### 3.2.6 定位漂移 / 地图双解（09-29~30）——部分解决，仍有问题（4.1）
- 现象：机器人回到原点，`/base_link_pose` 偏 1.9 m / 19°，glog 无告警，NDT 分值仍有 1.96。
- 根因在地图匹配，不在 LIO：LIO 85 分钟回起点只差 9 cm，漂移全部来自 lidar_loc 的地图修正。
  - 旧图走廊尽头**双解**：纵向最优解在 x≈−4.5（对）和 x≈−5.0（错）之间跳；该图行走区有大量现场已不存在的点团，
    只有一面沿走廊的长直墙约束纵向。同一个包实机落在 −4.5、离线回放落在 −4.97，激光吻合度 49% vs 24%。
  - 新图走廊**导航时必现**（09-30）：~0.3 m/s 导航、两端快速转身时，输出沿走廊被往原点方向拉 1.46 m、2.2 m；
    同时 LIO 与腿式里程计互差 ≈0.1 m。手动遥控慢走、原地快转只偏 0.1~0.2 m。不是 DDS（spdp、0 断档、0 丢包），
    离线回放无 Nav2 也复现（峰值 1.32 m）→ CPU 竞争不是主因。
  - 走行中 NDT 残差常卡 0.55~0.6 m = 饱和（跟踪 NDT 4 次迭代、步长 0.1，单帧最多挪 ~0.6 m），不是收敛。
- 解决（已部署）：
  - **一致性守护 `lidar_loc/loc_guard.{h,cc}`**：窗口（60 s 或 LIO 行程 5 m）内比较"输出相对运动"与"LIO 相对运动"，
    超 0.2 m / 5°（静止时 0.1 m）判 NDT 可疑；腿式里程计投票区分 NDT 错 / LIO 错；`action: freeze` + `rollback`：
    冻结地图修正只跟 LIO，并退回窗口内累积的修正；连续 10 帧 NDT 残差 <0.15 m / 2° 才恢复；冻结超过 30 帧提示 NEED RELOC。
    输出 `/lightning/loc_status`（JSON）和 glog 每帧 `loc guard frame:` 行（lo / out / odom 位姿、drift、res、state）。
  - **NDT 单帧跳变检查**（`max_ndt_jump_m 0.2` / `_deg 3`）：NDT 结果离 LIO 预测超过该值本帧不采信。
  - NDT 输入投影历史关键帧的实现修正：原实现在 `scan_undistort_` 上原地追加，而关键帧与它共用 CloudPtr，
    当前帧是新关键帧时遍历自身同时 push_back（未定义行为）；改为副本上投影并跳过自身，`loc_input.proj_kfs` 开关（默认 true）。
  - `/initialpose` 重定位（map 系 base_link 平面位姿经 `BaseTFPublisher::BasePoseToLidar` 换成雷达位姿，±15° / 3° 航向搜索，
    成功后 `pgo_->Reset()`，失败时不退回功能点初始化）：**未实机验证**。
  - `run_loc_offline` 新增 `--odom_topic/--odom_time_offset/--odom_lidar_offset`，测试专用 `--inject_ndt_bias "bx by t0 t1"`（注入 NDT 偏差做 A/B）。
  - 测试：`test/test_loc_guard.cc`（gtest，闭环模拟走廊漂移）、`test/test_g1_loc_guard_config.py`。
- 验证（离线 A/B，0930b 包，末帧对原点激光真值）：只告警 + 投影 0.36 m；只告警 + 无投影 **3.14 m**（纯当前帧更差，投影不是元凶）；
  冻结 + 回退 + 投影 **0.06 m**（导航段最大 0.23 m）。10-06 实机：守护两次判对并冻结（3.2.8）。
- 别再试：守护 `freeze` 早期版本里程计阈值太紧（正常行走 18% 窗口判不一致），冻结 / 解冻来回切约 100 次；
  现在的阈值（0.15 m + 15%×行程、10°、连续 5 帧）是按 09-29 腿式里程计实测定的。

#### 3.2.7 112.70 "定位漂移"：其实是在非原点重启了定位（10-06）
- 现象：输出偏 ~4 m / 12°，守护一直 NEED RELOC。
- 根因：机器人走到离原点 ~5 m 处后，前端两次点了"定位"（`ModeSet("localization")` 会停掉重启定位），重启从地图原点初始化，
  NDT 收敛到错误局部最优（conf 2.08，`Localize()` 不看分数照单全收）。PGO 随后刷屏 `Cholesky failure` 是副作用。
- 处理：用户确认是操作原因（有人在非原点重启）。防呆未做（4.2）。
- 验证方法：激光核对（`loc_check.py`）输出位姿吻合 30% / 49%，真值吻合 74% / 97.5%。

#### 3.2.8 导航时 NDT 又被沿走廊拉偏（10-06，两次）——未解决（4.1）
- 10:56 第一次导航到前台后，输出 − LIO 的修正量跳到 ~0.19 m，之后在前台一侧保持 0.15~0.25 m。
  守护**没报**：每个 60 s 窗口的增量都低于 0.2 m。直到重启定位后才发现，新旧位姿差 0.25 m（腿式里程计证明机器人没动），
  激光吻合度新位姿 82%、旧位姿 48%。
- 14:23 去厕所途中被拉偏 ~0.2 m，守护 14:24:08 冻结 + 回退 0.19 m，之后一直 NEED RELOC，纯 LIO 下定位偏 5~7 cm。

### 3.3 Costmap 障碍物（09-21~24，已部署并实机验证）

#### 3.3.1 动态障碍残影清不掉（09-21~23）
- 现象：人从侧面走到正前方再离开，local 和 global costmap 都留下清不掉的致命格（最长挂 174 s），挡住规划。当时怀疑是 D435 点云问题。
- 根因：
  1. 残影在 MID360 层（obstacle_layer），不在 D435（STVL 全程 0 残影）：人经过侧带时 MID360 标了格，人走后那片落进
     MID360 前方盲区，再没有 MID360 射线经过，永远清不掉。D435 看得见那里已空，但只接在 STVL；各层取最大值，STVL 的"空"清不掉 obstacle_layer。
  2. nav2 ObservationBuffer 先按 per-source 的 min/max_obstacle_height 裁点再清除，地面点永远成不了射线端点 → 只调 `raytrace_max_range` 没用。
  3. D435 本身没坏，只是 STVL 盲区记忆太久（旧 `voxel_decay 3.0`）。
- 解决：
  - **D435 做 obstacle_layer 的只清除源**（标记 / 清除分源）：`realsense_clear` 吃 D435 原始点云，`marking false / clearing true`，
    `min_obstacle_height -0.30`（地面点做射线端点），local raytrace 3.5 m、global 2.5 m。
  - **补标高物体** `realsense_mark_tall`（0.7~1.8 m、2.5 m 内）：2D 射线会从人腿间 / 桌下穿过，把真实物体误擦掉；
    ObstacleLayer 每次更新先清后标，误擦的格同一帧补回。原则：**清除距离 ≤ 能补标的源的标记距离**。
  - `stvl_voxel_layer.update_footprint_enabled: false`：自清半径 = robot_radius，会把半径内的真障碍抹掉；不在动态回调里，必须改 YAML 重启。
- 验证（实机 walkby ×3）：local 前方残影 89 格/帧、最长 174 s → 0（清除延迟 p90 0.3 s）；global 12 格/帧 → 0（约 2 s）；
  高物体召回 local 52% / 27% → 83% / 71%。CPU：planner_server 7% → 14%。
- 不是问题、别再追：D435 地面成对椭圆空洞 = 天花板灯管在抛光地砖上的倒影，不制造障碍。

#### 3.3.2 MID360 从 2D obstacle_layer 迁到 3D STVL（09-24）
- 原因：MID360 配置是照 2D 激光搬的。两张图都改成 STVL：`livox_mark`（只标）+ `livox_clear`（`model_type 1` 3D 雷达视锥，水平 360°）。
- STVL 没有 `obstacle_min_range`，所以新增 `g1_nav_bridge/scan_range_filter`：剔除离雷达 0.25 m 内和 base_link 水平 0.5 m 内的点，
  发 `/lightning/registered_scan_nav`，两张图的 MID360 源都吃它。
- global 同样用 STVL（`voxel_decay 5`），update/publish 5 Hz，MID360 标记源 `observation_persistence` 1.0 → 0.2（只攒一个周期）。
  publish == update 时 nav2 实际约隔一个周期才发一次（~3.2 Hz），规划在进程内读图不受影响。
- 依据（`tools/nav2_debug/mid360_stvl_probe.py`）：雷达系仰角 −50.5°~+6.4°（`vertical_fov_angle 1.80`）；同一格重访间隔 p95 1.0 s、p99 2.4~2.8 s，所以 `livox_clear.decay_acceleration` ≤2（测试守着）。

#### 3.3.3 导航时"随时都是障碍物"（09-24）——分来源处理，每一类都有实测证据
| 来源 | 证据 | 解决 | 结果 |
|---|---|---|---|
| D435 孤立飞点（镜头前 0.17~0.3 m） | 每 5 cm 体素 ≥4 点就被标；这些像素 4 邻域一致邻居 0~2 个 | `d435_mark_filter`：4 邻域一致性 + `min_depth 0.40` | base_link 0.5 m 内 D435 致命格 global 29 帧 → 0，local 12 → 0 |
| **走路时前摆的手** | 5 min 41 次，100% 在走路时；base_link 系位置固定：前 0.25~0.43 m、高 0.57~0.73 m | `d435_mark_filter` 删 base_link 水平 <0.5 m 且高 0.45~0.85 m 的点 | 41 次 → 0 |
| MID360 机身 / 吊带回波 | 剔除 <0.25 m 后 base_link 0.5 m 内每帧仍 ~9 点；STVL 足迹自清不删体素，走开留一串 | `scan_range_filter` 删 base_link 水平 <0.5 m 的点 | 每帧删 ~8.6 点 |
| MID360 地面回波 | 单格噪点，支撑点全在 0.10~0.13 m | `livox_mark.min_obstacle_height` 0.10 → 0.15 | 15 格 → 4 格 |
| D435 驱动官方滤波 | — | decimation 6、spatial 开、temporal 关 | 已生效 |

不是噪点、不要去滤：MID360 层 1.4~2.4 m 的大团，每帧都打到、有高度、不在静态地图上 = 周围的真实物体（吊架、设备、人）。

#### 3.3.4 试过、实测效果不好、已退回（别再试，除非有新证据）
- D435 `realsense_mark.min_obstacle_height` 0.15 → 0.05：D435 地面实测偏低 ~4.5 cm 且随站姿变，靠不住。
- `realsense_mark.obstacle_range` 1.5 → 2.0：深度图顶部掠射地面的错深度在 1.6~2.1 m 反复出现（悬空 0.35~0.65 m）；
  `floating_filter` 能减少但单帧 ~10 → ~26 ms。注意 obstacle_range 是离相机的 3D 距离，1.5 时地上矮障碍只标到 base_link 前 ~1.1 m；
  STVL 的 obstacle_range 不能热改。另：右前方 ~1.9 m 有一簇稳定错深度（浮在 0.37~0.59 m），就是靠 1.5 m 挡在外面的。
- 剔除深度图光轴上方 >20° 的行：1.5~2.5 m 真实物体召回掉到 0。
- temporal_filter（帧间一致性）：额外删 20~30% 真实物体。
- D435 整圈删 base_link 0.5 m：正前方 ±50° 只有 D435，会看不见闯进足迹的人 → 只删手的高度带。
- 曲率去地面：地面 σ 2~3 cm 时和矮障碍不可分，单帧约 35 ms。
- MID360 tag 过滤（`point_filter`）：只作用于 `/livox/points`（RViz / 诊断），对 costmap 没影响，实测只标出 ~0.2% 的点。
- 别用"点落在地图墙上的比例"判 MID360 外参（噪声大到会误判成偏航 180°）；外参已用人体交叉验证。

#### 3.3.5 当前生效的 costmap 配置（改之前先读）
都有 `test_g1_direct_config.py` 断言守着，改参数要同步改测试并写明实测理由。
- D435：`realsense_mark` 1.5 m / ≥0.15 m / voxel_min_points 4；`realsense_mark_tall` 0.7~1.8 m / 2.5 m；两者吃 `mark_points`；
  `realsense_clear` 吃原始点云；`voxel_decay 1.5` + `realsense_clear.decay_acceleration 15`。
- `d435_mark_filter.yaml`：min_depth 0.40、neighbor 开（0.08 / 3）、temporal 关、self_crop 0.5 m × 0.45~0.85 m、floating 关。
- MID360：`scan_range_filter` 0.25 m + base_link 水平 0.5 m；`livox_mark` ≥0.15 m / 2.5 m；global `observation_persistence 0.2`。
- launch `use_realsense_obstacles:=false` 时两张图里的 D435 源都摘掉（含 `drop_d435_clear`）。前提：栈跑在 spdp DDS 下，否则多订 D435 会把点云挤上网线。
- 代价常识：`252·exp(-k(d-r))` 只在 inflation_radius 内成立；254 lethal、253 内切带、255 未知。nav2 没有"被困"检测；
  `collision_margin_distance` 是软惩罚，不是硬安全圈。

### 3.4 导航与到点精度（10-03~10-06）

#### 3.4.1 现象
Nav2 报"到达"，人却停在偏处。10-03 实机去 (−4.21, 0.16, 0°) 停在偏 0.55 m 处，停车后还多转 15~27°。

#### 3.4.2 演进过程
（128.146，前台 (0, 0, −90°) ↔ 厕所 (−4.21, 0.16, 0°) 各 5 次；10-06 起每次到点后用激光核对定位）：
| 阶段 | 配置 | 站稳误差 | 发现的问题 |
|---|---|---|---|
| 10-03 前 | SmacPlanner2D + MPPI，转向强制 vx≥0.2 | ~0.5 m | 2D 路径无朝向，终点调头被改成走圆弧横移；减速段被抬速，停车多转 15~27° |
| 10-03 | Hybrid-A*（DUBIN，半径 0.40）+ `max_decel [-2.0,-0.35,-5.0]` | 仿真 0.08~0.22 m；**实机均 0.31 m、最大 0.42 m** | Hybrid 路径是对的，但 **MPPI 离目标 1.4 m 内抄近路直奔终点**（`GoalCritic`/`PathFollowCritic` `threshold_to_consider 1.4`，调头圈整个在 1.4 m 内），以错误朝向到点（差 150~230°），原地转被改成圆弧，甩出 0.3~0.4 m。当时定位本身也偏 ~0.2 m（3.2.8） |
| 10-06 | MPPI 阈值 1.4 → 0.3 | 厕所 0.21 m，前台 0.37 m | 前台的调头圈去程就贴着目标（0.45 m），仍被抄近路 |
| 10-06 | 换 RPP（不原地转）+ Hybrid | 厕所 0.13 m，前台 0.18 m | 前台调头空间不够：Hybrid 连续 4 次 "exceeded maximum iterations"、RPP "collision ahead"、BT Spin 恢复，单程 46 s |
| 10-06 | **实测 G1 能原地转** → `turn_min_vx 0` + SmacPlanner2D + RPP 起步/到点原地转 | 均 0.18 m、最大 0.32 m，航向 6~8° | 进 `xy_goal_tolerance 0.20` 就停下转（前台提前 0.14~0.2 m）；`yaw_goal_tolerance 0.20`（11.5°）内就停转 |
| **v1.0.0** | 上一行 + 两个容差收紧到 **0.10** | **均 12.9 cm、最大 19.8 cm，航向 ≤2.8°** | 剩余误差主要是到点原地转时的身体位移（4.3） |

#### 3.4.3 根因汇总
1. "G1 不能原地转"是桥接自己强加的规则（`turn_min_vx: 0.2`），把所有原地转都变成了半径 0.22~0.5 m 的圆弧。
2. MPPI 离目标近时由 GoalCritic 直接拉向终点、PathFollow 停用，路径绕回目标附近的部分一定会被抄近路。
3. RPP 进 xy 容差就开始原地转、转进 yaw 容差就停，所以容差就是误差下限。
4. 停车减速慢时，减速段被桥接抬回最小有效速度，多走多转（`max_decel` 已加大）。

#### 3.4.4 v1.0.0 的导航配置
（`aid_navigation2/param/nav2_params.yaml`、`g1_nav_bridge/config/nav_bridge.yaml`）
- 规划：SmacPlanner2D（tolerance 0.125，`use_final_approach_orientation: false`）。
- 控制：RPP，`desired_linear_vel 0.40`，前视 0.6（0.4~0.9），`max_robot_pose_search_dist 1.0`（路径绕回目标附近时不跳到末段），
  `use_rotate_to_heading: true`（起步偏差 >45° 先原地转，到点后原地转到目标朝向），`rotate_to_heading_angular_vel 0.6`（= 桥接 min_wz），
  `allow_reversing: false`，弯道按曲率降速、最低 0.25，碰撞预测 1.0 s 内会撞就停车。
- 原 MPPI 参数整段保留为 `FollowPathMPPI`（不在 `controller_plugins` 里，不会加载）；回退时把名字改回 `FollowPath`。
- `goal_checker`：`xy_goal_tolerance 0.10`、`yaw_goal_tolerance 0.10`、`stateful: true`。
- velocity_smoother：`max_velocity [0.45, 0, 0.9]`，`max_decel [-2.0, -0.35, -5.0]`，OPEN_LOOP，死区 0。
- 桥接：`turn_min_vx 0.0`、`min_wz 0.6`、`min_vx 0.3`。

#### 3.4.5 别再试
- `goal_checker stateful: false`（用户实测到不了点）。
- MPPI + 带调头圈的路径（阈值调多少都会在目标附近抄近路；再降其他 0.5 的阈值会让 MPPI 在终点附近几乎只剩避障约束）。
- Hybrid-A* 调头圈（能原地转之后没必要，而且窄处规划失败）。
- 收紧容差到 0.12 m 而不改原地转（10-03 仿真：基本没帮助）。

#### 3.4.6 仿真
`src/test/nav_goal_sim/`（humble Nav2 + 照搬抬速规则的 G1 运动学，见其 README）。它低估了横移（理想圆弧，没有步态），
  只能比较方案之间的相对好坏；v1.0.0 的方案没有在仿真里重跑。

### 3.5 桥接与系统

#### 3.5.1 `/odom` 一直没有发布者（09-22）
`sport_to_odom.py`、`tf_to_current_pose.py` 被 `install(PROGRAMS ...)` 直接装成可执行文件，
  setup.py 的 console_scripts 入口不生效，文件没有 `__main__` 守卫，launch 起它时从头跑完就以 0 退出（"process has finished cleanly"，无日志）。
  解决：加 `if __name__ == "__main__": main()`。教训：节点"启动了"不等于在运行，要查话题有没有发布者。

#### 3.5.2 开机自启失败（10-03，128.146）
部署删掉 src 后，systemd unit 指向的 `src/.../g1_autostart.sh` 不存在。
  解决：unit 改为随包安装的 `system/g1-robot.service`、`g1-semantic-map.service`，ExecStart 固定指向 install；
  `g1_autostart.sh` 从同级 `system/` 读 DDS 配置；CMakeLists 把 `system/` 装进 install；`g1_service.sh start/restart/status`
  遇到旧 unit 会告警。旧 unit 要用新 install 重新执行一次 `g1_service.sh install`（112.70 还没做）。

#### 3.5.3 切巡逻模式时 planner_server 段错误（10-03）
禁行区节点重发掩码，`KeepoutLayer::maskCallback()` 释放 `mask_costmap_` 时
  全局代价地图正在遍历它（exit −11）。解决：`updateCosts()` 与 `maskCallback()` 持同一把锁。

#### 3.5.4 `ros2 bag record` 录制永不停止（09 月）
`-d N` 是分片时长，不是录 N 秒。`record_nav_bag.sh` 的 `-t` 原来直接传给 `-d`，
  ssh 一断变成孤儿进程继续写盘（几分钟 5 GB）。解决：用 `timeout -s INT "$DUR"` 包住（SIGINT 才会写 metadata.yaml）。

#### 3.5.5 kill launch 父进程后子节点变孤儿
子节点 PPID=1 继续运行，与新实例同名冲突（TF 乱跳、设备 busy、帧率崩塌）。
  解决：一律用 `stop_all` / `g1_service.sh`（按 `install/` 路径匹配，覆盖 `cmdvel_to_sport`、`point_filter` 等所有子节点）。

#### 3.5.6 D435 点云停发
RealSense SDK vendored 在 `src/librealsense`（包名 librealsense2，`BUILD_WITH_NEON=OFF`）。
  若 `realsense2_camera` 链接到 `/usr/local/lib` 下别人装的 NEON 版，点云参数会变成 `pointcloud__neon_.*`，点云停发。用 `ldd` 核对。

#### 3.5.7 `point_filter` 取代 Python 转换（09-24）
`/livox/points` 由 C++ 包 `point_filter` 产生（按 MID360 tag 剔除低置信度点，
  做与 lightning 预处理相同的抽点 / 盲区 / 高度裁剪）。LIO 仍直接订阅 `/livox/lidar`，Nav2 吃 `registered_scan_nav`，都不吃它。

#### 3.5.8 运行中的配置被就地改过（10-06 发现）
128.146 install 里的 `nav2_params.yaml` 在 10-03 被手工改成 `stateful: false`（试验残留），
  部署会覆盖。部署前用 md5 对比 install 和本地（2.2）。

#### 3.5.9 语义地图在 Thor 上部署（09-29）
BGE-M3 从本地目录 `/home/unitree/models/bge-m3` 加载（Thor 不方便联网）；Qdrant 共用 Thor 上
  已有的 `qdrant` 容器（6333，开机自启），不要再 `docker compose up`。launch 参数和 yaml 合并后再传给节点（Foxy 与 Humble/Jazzy 的参数优先级不同）。
  细节见 `src/semantic_map_ros/README.md`。

### 3.6 09-17 交接文档里的遗留项（状态未核实）
`AI_HANDOFF_2026-09-17.md` 记录的以下问题，之后没有专门核实是否已解决：
- 完整启动时官方 `rs_launch.py` 扫描整个 launch context，打印约 52 条 "Parameter 'xxx' is not supported"。相机能启动；
  判断参数是否生效必须 `ros2 param get` 查运行中的节点，不能信这些警告。
- `use_keepout:=false` 时仍可能启动 `forbidden_map_create_node`，日志出现 "Publisher already registered"。
- RealSense 偶发 "Depth stream start failure / Hardware Error"（可单独启动时用 `initial_reset:=true` 验证，不要默认每次 reset）。
- 用 `timeout --signal=INT` 停止时部分 Python 节点二次 shutdown、Livox 驱动退出偶发 −11，是测试主动停止后的噪声，不要和启动期错误混淆。

## 4. v1.0.0 已知问题（按优先级）

### 4.1 导航时 NDT 沿走廊把定位拉偏（最重要）
- 10-06 又出现两次（3.2.8）。慢拉（每个 60 s 窗口 <0.2 m）守护发现不了；守护冻结后 NDT 残差一直 >0.15 m，**不会自动恢复**，
  只能停在 NEED RELOC、靠 LIO 推算（偏差 5~7 cm，暂时不增长）。
- 前端不显示 `/lightning/loc_status` 里的 need_reloc，操作员不知道定位已冻结。
- 线索：走廊只有一面长直墙约束纵向；NDT 残差饱和（单帧最多挪 ~0.6 m）；上游 issue #65（倒装 MID360 走廊漂移）、#5（退化场景，引入里程计）、
  #122（NDT 高分误匹配，ICP 二次校验有效）。数据：`/opt/G1/bags/corridor_newmap_0930`、`corridor_newmap_0930b`、`goal_err_1006*/`。

### 4.2 定位初始化总是从地图原点开始，没有防呆
- `run_loc_online` 写死 `SetInitPose(SE3())`，`Localize()` 不看分数；前端"定位"按钮在已定位时也会重启定位。
- **10-06 部署 semantic_map_ros 后，机器人就在原点附近（朝向 −4°）重启，定位仍偏了 0.33 m**（激光最优 (0.05, 0.02) vs 输出 (0.37, −0.08)），
  守护显示 GOOD，没发现。当时 128.146 处于这个状态，下次用之前要先重定位。
- 待做：重启时用上次的位姿初始化，或多候选比分；`ModeSet("localization")` 已定位时不重启；前端显示 need_reloc；`/initialpose` 实机验证。

### 4.3 到点后原地转向的身体位移
v1.0.0 剩余到点误差的主要来源（转 90° 常从 6 cm 被甩到 20~30 cm）。可选：转完补一小段位置修正；
  或转向时加少量前进速度抵消后退（需要标定）。

### 4.4 RPP 不会局部绕障
只按碰撞预测停车，绕障靠全局重规划（BT 只在路径失效时重规划）。**还没测过有人 / 障碍物挡路的情况**。

### 4.5 新方案没有仿真
`nav_goal_sim` 是 humble 版，没有按 v1.0.0（2D + RPP + 原地转）重跑。

### 4.6 `min_vx 0.3` 没有实测依据
限制了慢速靠近，造成进点冲过头。应像原地转一样实测 G1 能否稳定低速行走。

### 4.7 2D 栅格相对 3D 地图转了 ~1.25°
（新图，远处墙偏 0.3~0.7 m，用户看到"双层墙"），影响网页和 Nav2 静态层，不影响定位。
  g2p5 用的也是 `kf->GetOptPose()`、回环后会 `RedrawGlobalMap`，原因未查明。下一步：从 3D 图重生成栅格对比。

### 4.8 global costmap 的矮障碍
（用户决定暂不做）：D435 视场内 1.5~2.5 m、低于 0.7 m、MID360 从侧面标过的矮障碍会被 2D 射线擦掉
  （回放视场内丢失率比噪声底高约 40 个百分点，多为 1~3 s，最长 11 s）；1.5 m 外的前方矮障碍 global 看不到。根治要把 global 的 D435 也放进 STVL。

### 4.9 112.70 没有更新
还是 09-30 的代码，开机自启 unit 指向 src（删 src 后会起不来）。

### 4.10 Thor `~/.bashrc` 的 DDS 配置
新开终端执行 `ros2` 会报参与者编号不够（2.4）。可在第 124 行加
  `<Discovery><MaxAutoParticipantIndex>120</MaxAutoParticipantIndex></Discovery>`，或改为 source `dds_env.sh`；需要用户同意。

### 4.11 小问题
- `walk_monitor.py`（Thor）第 46 行订阅回调要写成 `lambda m: self.q.append(m)`，Thor 上是坏的版本。
- `test_g1_g2p5_config.py` 有 3 个早已存在的失败（URDF 关节已改名 `body_to_mid360_joint`）。
- `laser_mapping.cc` MakeKF 的 `20 / 180 * M_PI` 是整数除法（=0），投影关键帧只按 3 m 更新；修它会改关键帧策略，未动。
- lightning 自己的 tag 过滤（`pointcloud_preprocess.cc:53`）是注释掉的，去重 / 盲区判断有 `||` / `&&` 优先级 bug。
- `loc_input.proj_kfs` 对漂移的贡献尚未确定（实机 A/B 没做完：开—关—开、每轮原点重启，统计用 Thor `ab_stats.py <glog> t0 t1`）。
- 3.6 的遗留项未核实。

## 5. 排查方法与工具

### 5.1 出现问题先录包
`tools/field_test/record_nav_bag.sh [名字] [-t 秒] [--light|--full|--loc]`：只录在线的话题，默认不压缩（zstd 吃 CPU）。
**定位问题用 `--loc`**（livox 原始云 + IMU + /odom + TF，sqlite3，可直接给 `run_loc_offline` 回放）。
09-21 那次排查全程没录包，只能靠 glog 反推，很多结论无法复验——别再犯。

### 5.2 定位
- **激光对地图核对是唯一可信的真值**（NDT 分值 confidence 不能当作位置正确的依据）：
  - `/opt/G1/bags/replay/loc_check.py <map_dir> [cx cy]`：抓当前 scan，体素查表，在当前位姿 / 原点 / 指定点附近搜 x-y-yaw，打 x/y 剖面看是否单峰。
    点云地图地面在建图雷达高度下方，z 偏移约 −1.1~−1.25。正常时 10 cm 内吻合 0.8 左右、25 cm 内 0.97。
  - `fit_once.py <map_dir>`（单次，输出 JSON）、`fit_watch.py`（每次到点后自动核对）。
  - `bag_fit.py`：从包里取某时刻的 scan 比较候选位姿。
- **区分地图匹配和 LIO 的问题**：比较 |输出 − 起点| 与 |LIO − 起点|，或用 `out_vs_lio.py <glog>` 看"输出 − LIO 推算"的修正量随时间的变化；
  `walk_attrib.py <glog>`（输出 / 纯 LIO / 纯里程计三路对比）、`live_attrib.py`（实时版）。
- **判断机器人有没有动**：腿式里程计 `/odom` 不随我们的栈重启，重启前后读数一样 = 没动。
- 看守护：`grep "loc guard:" <glog>`（状态切换）、`grep "loc guard frame" <glog>`（每帧）。
- 读 glog 要知道的源码事实：
  - `balance_factor 0.1`：每次只把 NDT 残差的 10% 注入输出。glog 里 `loc using lo guess` 与 `confidence ... t:` 之差是原始残差，
    实际注入要再乘 0.1。持续运动时输出对 NDT 解有 τ≈10 s 的滞后（"走路时偏 15 cm、停下回 0"）。
  - `track_min_score` 默认 0.0；`loc_monitor.py` 的 "confidence<1.3" 是经验值，不是定位器门限。
  - `sync package failed` 只是 IMU 还没覆盖雷达帧尾，下帧会补跑，不等于丢包。
  - 判断航向跳变用实际时间戳差算角速度，不要假定 0.1 s/帧。
- **离线回放对有些地图不具代表性**（旧图在线 NDT 残差 p90 0.064 m，离线 0.42 m）。`run_loc_offline` 退出时会往地图目录写动态图层，
  回放一律用地图拷贝；临时启动配置 `/tmp/g1_lightning_*.yaml` 停栈时会被清理，离线要用 install 里的 `default_livox.yaml` 自己改。

### 5.3 到点精度测量（10-06 用的方法）
Thor `/opt/G1/bags/replay/` 下，用 `run_goal_err*.sh` / `run_plan_rec*.sh` / `run_fit_watch*.sh` nohup 启动（自带 ROS 和 DDS 环境），都只订阅：
- `goal_err.py <输出目录>`：每次 `/nav_to_pose` 开一轮，`/task_status` 变成功时记"判到达"，位姿稳定后记"站稳"，按目标朝向拆成纵向 / 横向误差；
  点位坐标写在脚本里（前台、厕所）。
- `plan_rec.py`：录 `/plan` 和速度指令（`/cmd_vel_safe` 要用 best-effort QoS 才收得到）。
- `fit_watch.py`：每轮站稳后用激光核对定位，**到点误差要按激光真实位置算**，否则会被定位误差污染（10-06 早上那一轮就是）。
- `ana.py <轮次>`、`runplans.py <轮次>`：打印轨迹对路径（最近路径点序号会暴露抄近路）和速度指令。
- 原地转测试：`spin_watch.py`（自动识别转向开始 / 结束，转前转后激光核对）。

### 5.4 Costmap
- **先分层、再找来源**：订阅 `/global_costmap/*_raw` 各层，主图每个 254 归到"静态墙 / D435 层 / MID360 层 / 不明"，
  再查它自己传感器最近 3 s 的支撑点（命中帧数、高度、位置）。RViz 里看到的格子要用 `*_raw` 复核。
- 支撑判定用点云自己的 stamp 查 TF，给 ±1 格容差；D435 采集到 costmap 发布有 0.3~0.5 s 延迟（`costmap_recall.py --cam-lag 0.4`）。
- **近身事件触发式抓取**（`d435_near_watch.py`，存 `d435_near/event_*.npz`）：D435 层在 base_link 0.5 m 内出现新 254 就回溯最近 6 s 的帧，看深度、高度、像素位置、5x5 邻域；
  换到 base_link 系后"位置跟着机器人、只在走路时出现" = 自己的手。
- **判断错深度**：机器人转身时跟着相机走 = 错深度，map 系固定 = 实物。"深度 / 同射线地面深度"等价于高度，不是独立证据。
- 离线回放台：`tools/nav2_debug/replay_costmap.sh <bag> <名> [--section global_costmap] [--start S --dur D] [--set 层.参数=值]`
  （domain 77 + spdp；global 必须 `--start 0`；回放期间不要 stop_all）。
- 工具：`tools/nav2_debug/`（`obstacle_residual_ab.py`、`costmap_recall.py`、`stvl_clear_latency.py`、`d435_false_obstacle_probe.py`、
  `d435_ground_quality.py`、`mid360_stvl_probe.py` 等，本地跑用 `python3 -s`）；Thor 实时脚本 `live_global_layers.py`、`d435_near_watch.py`、
  `ghost_watch.py`、`live_d435_mon.py`、`low_obs.py`；`tools/field_test/loc_monitor.py`（定位 + 导航常驻监控，costmap 与位姿未做时间对齐）。
- 用户偏好**实机直接测**（改完重启、实时盯），不要先跑长时间 bag 回放。

### 5.5 远程操作的坑
- `pgrep -c <name>` 对超过 15 字符的进程名恒返回 0，用 `pgrep -f`。
- `pgrep -f` / `pkill -f` 会匹配到自己的 ssh 命令行（含 heredoc 内容），把会话杀掉：用 `pgrep -f "[x]xx"`，或把逻辑写成脚本 scp 过去再执行。
  同理 `pkill` 和 `nohup` 不要写在同一条 ssh 命令里。
- ssh 里的 `python3 - <<EOF` 中 f-string 带转义引号会语法错误：复杂脚本先写成文件再 scp。
- `ros2 topic hz` 测点云需要 30~40 s 窗口，14 s 会读到 0。
- 节点"启动了"不等于在运行（3.5.1）；判断参数是否生效用 `ros2 param get` 查运行中的节点。
- 机器人会突然掉线（10-06 下午出现过一次，ping 不通、ARP 失败），重新上线后测量进程要重启。

## 6. 参考

### 6.1 数据（Thor `/opt/G1/bags/`）
- 建图：`map500d_0919`（8 分钟 0 断档）、`mapping_corridor_0929`（覆盖建图，已 reindex）。
- 定位：`locguard_0929`（旧图 20 min 行走）、`corridor_newmap_0930`、`corridor_newmap_0930b`（导航时拉偏复现，`--loc`）；离线 A/B 结果 `replay/ab0930/`。
- costmap：`walkby_0923`（修复前）、`walkby_s2_0923`（修复后）、`demo2_0922_1350`、`d435_ghost_0922`（错深度）、`d435near_0924` / `d435near2_0924`（修手前 / 后）、
  `hang_0924`（机身回波）、`move_0924`（错深度 + 飞点）。
- 到点测量：`replay/goal_err_1006`（MPPI）、`goal_err_1006b`（MPPI 阈值 0.3 / RPP + Hybrid）、`goal_err_1006c`（原地转，容差 0.2）、
  `goal_err_1006d`（v1.0.0，容差 0.1）；原地转：`replay/spin_1006`。

### 6.2 lightning 上游事实（对照 github 上游 1325fed 确认，不是我们改坏的）
- `Localize()` 永远返回 true（issue #127：原本靠 RTK 重置，开源版"定位丢了没办法处理"）。
- `YawSearch` 上游注释掉（#123：会选错角度）；我们只在 `/initialpose` 重定位时用（粗匹配前 3 名做精匹配后按精分判定）。
- 跟踪 NDT 在 `UpdateGlobalMap` 里 4 次迭代、步长 0.1。
- 退化检测 / DR 多初值比较上游都注释掉了；lightning 的 "DR" 就是 LIO 自身 IMU 状态，不独立。
- NDT 输入不是纯当前帧：`GetProjCloud()` = 当前帧 + 最多 5 个历史关键帧各取最早 1001 点；`fasterlio.proj_kfs` 管不到（上游判断被注释）。

### 6.3 其他文档
- `launchreadmd.md`：手动启动 / 建图 / 定位 / 导航的命令，新机器首次部署（RealSense SDK、DDS 接收缓冲）。
- `AI_HANDOFF_2026-09-17.md`：早期交接（TF 设计、桥接包、电池、Unitree 官方 SLAM 黑盒检查）。
- `README_SLAM_NAV.md`、`G1_3D_NAV_MIGRATION_ANALYSIS.md`：工作区精简和迁移分析。
- `src/semantic_map_ros/README.md`、`DATA_FLOW.md`：语义地图部署和数据流；仓库根目录 `README.md`、`HANDOFF.md`：网页控制台和仿真。
- `src/test/nav_goal_sim/README.md`：到点精度仿真。
- VLN（LightNav-0）评估在独立工作区 `/home/ap/project/G1_VLN`，不在本仓库。
