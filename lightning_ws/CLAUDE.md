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

# G1 导航栈工作手册 —— v3.0.0（2026-10-08）

本文件是接手者（Claude 会话或工程师）的经验库。**先看下面的总目录**：每一点一句话说清结论，要细节按编号去对应章节。

**版本**：v3.0.0 = v1.0.0 的导航栈 + 10-08 的定位修复 + 网页导航叠加显示。导航配置、costmap、桥接、语义地图的代码都和 v1.0.0 相同，
这几部分的结论照用；下文没有标版本的内容都适用于 v3.0.0，标了 v1.0.0 的是当时测的数据，定位修好后没重测（如到点精度）。

| 版本 | 日期 / 提交 | 内容 |
|---|---|---|
| v1.0.0 | 10-06 `75c3c26` | 到点精度：原地转向 + SmacPlanner2D + RPP，容差 0.10（3.4）；开机自启只依赖 install；禁行区层加锁；部署脚本 |
| v1.0.1、v2.0.0 | 10-06 `67e37d8`、`1299ce4` | 只改文档（CLAUDE.md 重构、README / HANDOFF 同步），代码同 v1.0.0 |
| **v3.0.0** | 10-08 `0b1da34`、`c5b51fe` | **定位**：修好导航时 NDT 沿走廊拉偏（机身点 + pclomp 线搜索，3.2.9），128.146 实机验证；**网页**：导航模式叠加 local costmap 和三条规划路径（1.1） |
| Orin 调试（未定版本号） | 10-20 `ce00fa6` ~ 10-27 `db57b50`，含 10-27 的 shim 站稳 / 到点转身、loc_params_overlay、loc_global_init | 无 Thor 背包的 G1（Orin 30524）：docker 部署、运动特性实测（2.6）、蟹行补偿、shim 控制器、动作衔接（3.7）。**也改了仓库默认值**（会随部署带到 Thor）：JumpRelock、C++ `sport_to_odom`、`cost_travel_multiplier 10`、`inflation_radius 0.85`、`robot_radius 0.40`、定时重规划 BT、网页避障半径只改 robot_radius；只给 Orin 的值都在 `tools/deploy/orin_docker/*.local.yaml` |

维护规则：新问题解决后按"现象 / 根因 / 解决 / 验证"补到第 3 章对应小节，并在总目录加一行；没解决的写进第 4 章。
结论要附实测数据和数据位置；过时的说法直接改掉，不要在后面追加"推翻"。

## 总目录（缩小版）

**1. 系统与机器现状**
- 1.1 数据链路：MID360+IMU → lightning（LIO + NDT + 守护）→ Nav2（2D 规划 + RPP，STVL costmap）→ 速度桥 → G1；网页控制台跑在电脑上。
- 1.2 机器：128.146 是主测机，跑的是 v3.0.0；112.70 还是 09-30 的代码；Orin 30524（无背包）跑 docker 栈 + shim 控制器；Orin 30423 整包装好未启动；开发机默认不编译。
- 1.3 实测指标：定位 v3.0.0 导航 1 h 全部 0~4.2 cm；到点平均 12.9 cm、最大 19.8 cm、航向 ≤2.8°（v1.0.0 测，v3.0.0 未重测）；costmap 残影和近身噪点清零。

**2. 硬性规则（动手前必读）**
- 2.1 绝对禁止后退：带 Thor 背包的 G1 收到负 vx 会摔倒，四道防线不得放宽；仓库默认值一律不后退。唯一例外是无背包的 Orin 30524（实测能退），只经本机覆盖文件放开 BackUp 脱困。
- 2.2 编译与部署：只在 Thor 编译；部署要用户同意；用 `deploy_to_robot.sh`；运行时只能引用 install（src 会被删）。
- 2.3 重启：只用 `g1_service.sh restart`；**重启前机器人必须在原点、朝向约 0°，重启后用激光核对定位**。
- 2.4 DDS：启动栈和手动跑 `ros2` 前都要 source `dds_env.sh`。
- 2.5 坐标系与传感器：TF 结构、高度基准、MID360 倒装且正前方被头挡住、D435 下俯 47.6°。
- 2.6 G1 运动特性：**能原地转**（转一次偏 4~17 cm）；`min_vx 0.3` 没验证；`stateful` 必须为 true。
  以上是带 Thor 背包的 G1；**无背包的 Orin 30524**：原地转 |wz|<0.30 完全不转、绕右侧 ~0.28 m 的点画圈、转身位移 13~41 cm；
  走路转向没有死区、基本线性；**走路蟹行：只发 vx 时实际行进方向偏左 ~9.5°**；急停后身体还冲 0.27 m/s、原地转刹停后还转 0.6 rad/s（换动作要先站稳）；
  能后退（−0.1 死区）。本机值都在覆盖文件里（`min_wz 0.32`、原地转 0.9、容差 0.3 / 0.30）。

**3. 问题与解决记录**
- 3.1 建图与 LIO
  - 3.1.1 静止时位姿漂走 → 雷达系 Z 下限填成正数，点被裁光；改 −2.0。
  - 3.1.2 IMU 外参只对 x 取了负号 → 三个分量都取负。
  - 3.1.3 快转后地图多层墙 → IMU 断档；去掉绑核、关 UI、IMU 队列 1000、装 DDS 缓冲配置。
  - 3.1.4 回环修正跳动 → 用 20 帧子图做回环源，只用 x/y/航向修正。
  - 3.1.5 Thor 编译 lightning 失败 → `export PYTHONNOUSERSITE=1`。
  - 3.1.6 新地图 1790676615248 → 3D 好，2D 栅格多层墙、像转了 1.25°（根因 3.1.7）。
  - 3.1.7 **2D 栅格多层墙 / 相对 3D 转 1.25° 的根因（10-08）** → g2p5 扩容时旧内容按整块子网格（0.8 m）搬移、原点只对齐 0.05 m，每次扩容错位最多 0.4 m；已修 + 单测，10-08 已部署 128.146，在用图的 pgm 仍是旧的。
- 3.2 定位
  - 3.2.1 静止时 roll 被拉歪 → 用 LIO 重力约束。
  - 3.2.2 定位跳变 → DDS 组播把点云挤上网线；改 spdp。
  - 3.2.3 每帧定位反而漂移 → 保持只在关键帧定位，启动用快速收敛。
  - 3.2.4 LIO 恒滞后 0.35 s → 补跑积压帧。
  - 3.2.5 段错误日志丢失 → glog 逐条落盘，放到 `/opt/G1/logs/glog`。
  - 3.2.6 走廊地图双解 / 导航时被拉偏 → 加一致性守护（冻结 + 回退）和单帧跳变检查（根因见 3.2.9）。
  - 3.2.8 10-06 导航时又被拉偏两次 → 一次守护没发现，一次冻结后 NEED RELOC（根因见 3.2.9）。
  - 3.2.9 **拉偏的根因（10-08）** → 盲区过滤有优先级 bug，每帧 40% 机身点进了 LIO 和 NDT；pclomp 的线搜索也被关掉了。
    两处都已修，0930b 离线全程误差 ≤2.3 cm（原来 1.9 m）；**10-08 已部署 128.146 实机验证**：1 h / 483 m，
    有效激光核对 27 次全部 0~4.2 cm，守护全程 GOOD，跳变检查 0 次。
- 3.3 Costmap 障碍物
  - 3.3.1 动态障碍残影清不掉 → MID360 盲区无法清除；D435 做只清除源，补标高物体。
  - 3.3.2 MID360 迁到 3D STVL → 加 `scan_range_filter` 去机身回波。
  - 3.3.3 "随时都是障碍物" → 飞点、前摆的手、机身回波、地面回波分别处理，近身噪点清零。
  - 3.3.4 试过并否定的方案 → 降高度阈值、放宽 1.5 m、删顶部行、帧间滤波等，别再试。
  - 3.3.5 当前 costmap 配置 → 改之前先读，测试守着。
  - 3.3.6 **全局规划穿过灰色未知区（10-15）** → 禁行区掩码非禁行处全填 0，禁行区层拿 0 覆盖未知格；已改代码，10-20 随 Orin 30524 首次部署，未实机验证。
- 3.4 导航与到点精度
  - 3.4.1 现象 → Nav2 报到达，人却偏 0.3~0.5 m。
  - 3.4.2 演进过程 → MPPI 0.31 m → RPP+Hybrid 0.13~0.18 m → 原地转 0.18 m → 容差 0.1 后 **0.13 m**。
  - 3.4.3 根因 → "不能原地转"是桥接自己加的；MPPI 近目标抄近路；容差就是误差下限。
  - 3.4.4 当前配置（v1.0.0 定下）→ SmacPlanner2D + RPP 原地转 + 容差 0.10；MPPI 参数保留可回退。
  - 3.4.5 别再试 → `stateful: false`、MPPI 配调头圈、Hybrid 调头圈。
  - 3.4.6 仿真 → `src/test/nav_goal_sim`，会低估横移，新方案没重跑。
- 3.5 桥接与系统
  - 3.5.3 planner_server 段错误 → 禁行区层加锁。
  - 3.5.6 D435 点云停发 → 链接到了系统 NEON 版 SDK。
  - 3.5.7 `/livox/points` 由 `point_filter` 产生 → LIO 和 Nav2 都不吃它。
  - 3.5.8 install 里的配置被手改过 → 部署前用 md5 核对。
  - 3.5.9 语义地图部署 → 模型从本地目录加载，共用已有的 Qdrant 容器。
- 3.6 09-17 交接里的遗留项 → RealSense 参数警告、禁行区节点重复等，状态未核实。
- 3.7 **无背包 G1（Orin 30524）导航调试（10-24~10-27）**
  - 3.7.1 到点绕圈 / 原地转进不了容差 → 原地转硬死区 0.30 + 身体画圈；`min_wz 0.32`、容差 0.10 → 0.3。
  - 3.7.2 停服务留下整套孤儿导航栈 → ROS 进程在容器 cgroup 里，systemd 够不着；`service-stop` 自带清理。
  - 3.7.3 转弯切内角卡进障碍层 → `cost_travel_multiplier 2 → 10`、`inflation_radius 0.85`，原地转 0.35 → 0.9。
  - 3.7.4 只原地转、不走弧线（`turn_in_place_only`）→ 部署后用户要求撤回，别再提。
  - 3.7.5 **走路蟹行：一直偏路径一侧 ~9 cm** → 只发 vx 实际偏左 9.5°；A（走路不抬 wz）+ B（原地转 0.9）+ 航向补偿帧 `base_link_nav`，实测 xte p50 0.015 m。
  - 3.7.6 卡障：网页"避障半径"连带把膨胀改到 0.6 → 网页只改 robot_radius；仓库 robot_radius 0.50 → 0.40，膨胀固定 0.85。
  - 3.7.7 路径贴障、collision ahead 走走停停 → 规划后才看到的障碍不触发重规划；BT 加每 2 s 定时重规划。
  - 3.7.8 弧线步态不稳 → 主因是 collision ahead 急停；`walk_max_wz` 试过已撤回；velocity_smoother CLOSED_LOOP 起步死锁，别再试。
  - 3.7.9 **控制器 RPP → shim（远处 MPPI、近目标 RPP，照轮式 128.23）** → 实机到点 31/38 → 49/50，collision ahead 468 → 4 次。
  - 3.7.10 **换动作不等站稳（急停后立刻转、转完立刻走）有摔倒风险** → shim 加 `settle_time` + `final_rotate`，10-27 已部署、已提交；30524 15:55 换成 settle 0.3 + 只在起止点等，未实机测完；30423 仍是 10-26 版。
  - 3.7.11 运维杂项 → 语义服务被停栈清理误杀、ssh 里 restart 会杀掉自己、30423 整包、提交作者。

**4. v3.0.0 已知问题（按优先级）**
- 4.1 导航时 NDT 沿走廊拉偏定位：根因已修并实机验证（3.2.9，10-08）；剩守护本身的老问题。10-24 加跳变检查锁死解除 `JumpRelock`，**10-24 16:05 已部署 Orin 30524**；10-25 仍出现 NDT 持续要 0.25~0.5 m 修正、被拒几百次 → NEED RELOC（疑与 4.15 有关）。
- 4.2 定位总从地图原点初始化，不看分数：**10-27 Orin 两次初始化都错**（真值离原点 2.4 m / 78°，错解 NDT 分还高过门限）；全图俯视 FFT 搜索能找到真值，**方案 A（独立包 `src/test/loc_global_init`，结果经 `/initialpose` 发进去，不改 lightning-lm）10-27 本地跑通**（单测、0930b 离线、本地编 lightning 真联调：原点错初始化 4.5~5.3 m 都被纠正、本来对的不动）；**10-27 15:36 已部署 Orin 30524 并实机纠正一次**（原点初始化错 5.7 m / 104° → 4.5 s 纠正），手动启动、未进自启；lightning 自己的航向搜索仍把结果沿走廊拉偏 0.24 m，待定。
- 4.3 到点原地转时身体位移，是剩余到点误差的主要来源（Orin 13~41 cm，比带背包的大）。
- 4.4 RPP 不会绕障：带背包的 G1 仍是 RPP；Orin 已换 shim（远处 MPPI 能局部绕障，3.7.9）。
- 4.5 仿真：Orin 版已有（`nav_goal_sim` 的 Orin 模型、实机回放、障碍场景，3.7）；带背包 G1 的 v1.0.0 方案仍没重跑。
- 4.6 `min_vx 0.3` 没验证，造成进点冲过头（两种 G1 都没测过走路最低速）。
- 4.7 2D 栅格多层墙 / 相对 3D 转 1.25°：根因已修并部署 128.146（3.1.7）；在用地图的 map.pgm 要重建图或从 3D 重生成。
- 4.8 global costmap 对矮障碍看不全。
- 4.9 112.70 没更新。
- 4.10 Thor 终端的 DDS 参与者编号不够。
- 4.11 其他小问题。
- 4.12 collision_monitor 按仓库默认配置不启动（速度桥直接吃 `/cmd_vel`），和链路图不符；实机状态待核实。
- 4.13 **Orin LIO 偶尔丢航向 ~8°**（直走 + 小幅摆头时，不是大转向时，不是每次都有），根因未定；有含 livox 的包可离线复现。
- 4.14 **Orin 切定位时机器人在挪步 → LIO 重力初始化歪 ~8°**，前方地面被标成障碍；修法用户说先不做，临时办法是站稳几秒再切。
- 4.15 **怀疑 Orin 地图沿走廊被压短 6~8%**（LIO、腿式里程计都比 NDT 多算），未证实。
- 4.16 **Orin 已部署、未实机验证的清单**：shim 站稳 / 到点转身、定时重规划的效果、BackUp 脱困、`service-stop` 清残留、开机自启、蟹行补偿配 MPPI；容差 0.15 还没收紧。
- 4.17 Orin 地图里的建图残留障碍：起点身后一块（疑似建图时站人）→ 起点 Start occupied、待机位在未知区；贴墙 0.3 m 被困。
- 4.18 Orin 30423：整包装好，导航没启动，地图还是 30524 场地的。

**5. 排查方法与工具**
- 5.1 出问题先录包（定位问题用 `--loc`）。
- 5.2 定位：**激光对地图核对是唯一可信的真值**；对比"输出 − LIO"区分地图匹配和 LIO 的问题；
  NDT Hessian 日志 / 输入导出两个诊断开关；开发机可编译做离线回放（需用户授权）；
  Orin 负载高时抓点云拷回开发机算，不知道在哪时用全图俯视搜索。
- 5.3 到点精度测量：`goal_err.py` + `plan_rec.py` + `fit_watch.py`，误差按激光真实位置算。
- 5.4 Costmap：先分层、再找来源；离线回放台。
- 5.5 远程操作的坑：`pgrep/pkill -f` 会杀掉自己的 ssh 会话；ssh 里直接 `g1_service.sh restart` 只停不启等。
- 5.6 Orin 导航分析：速度跳变 / collision ahead 归因、本机回放仿真（`run_replay_test.py`）对比"实机有 / 仿真有 / 共有"。

**6. 参考**
- 6.1 录包和测量数据目录；6.2 lightning 上游事实；6.3 其他文档；6.4 Lightning-LM / Nav2 数据流详解（在线文档链接）。

---

## 1. 系统与机器现状（v3.0.0）

### 1.1 数据链路
```text
MID360 /livox/lidar（CustomMsg，逐点 offset_time）+ IMU
  → lightning run_loc_online：LIO + NDT 地图匹配 + 一致性守护 loc_guard
  → TF map->base_link（平面）、base_link->body_link（躯干倾斜）；/base_link_pose
  → Nav2：SmacPlanner2D 全局规划 + Regulated Pure Pursuit 控制（起步/到点原地转向）
           costmap 用 STVL（MID360 + D435），global 另有静态层和禁行区
  → /cmd_vel_nav → velocity_smoother → /cmd_vel
  → [仅 use_collision_monitor:=true 时] collision_monitor → /cmd_vel_safe
  → g1_cmdvel_to_sport（最小有效速度改写）→ Unitree LocoClient::SetVelocity
```
- **collision_monitor 按仓库默认配置不启动**（4.12）：`robot.launch.py` 的 `use_collision_monitor` 默认 false，
  `g1_autostart.env` 的 `G1_ROBOT_ARGS` 为空；此时 `g1_navigation.launch.py` 让速度桥直接订阅 `/cmd_vel`，
  `/cmd_vel_safe` 没有发布者。实机是否被 `/opt/G1/g1_autostart.local.env` 覆盖成 true 未核实。
- 障碍点云：MID360 的 `/lightning/registered_scan` → `scan_range_filter` → `/lightning/registered_scan_nav`；
  D435 原始点云 → `d435_mark_filter` → `/camera/camera/depth/mark_points`（标记），原始点云另做清除源。
- 本体：`/odommodestate` → `sport_to_odom` → `/odom`（不发 TF，只在导航模式运行；10-24 起是 C++ 版，Python 版 `sport_to_odom.py` 保留作对照，Orin 上 Python 版 ~50 Hz 输入就占 15% CPU）；`/lf/bmsstate` → `/battery_state`。
- 模式管理：`robot.launch.py`（`start_backend:=true`）里由 robot_status_manager 经 launch_manager 启停定位/导航/建图，
  前端切模式走 `/mode_set`；地图取数据库当前地图（`~/maps/db.sqlite`），取不到才用 `map_dir`。
  `start_backend:=false` 时才由顶层 launch 直接启动定位和导航（前端就不能切模式了）。
- 语义地图 `semantic_map_ros`（BGE-M3 + Qdrant）跑在 Thor 上，开机自启服务 `g1-semantic-map`。
- 网页控制台 `src/g1_web` 跑在电脑上：`python3 serve.py 8080`，页面里连 `ws://<机器人IP>:9090`（rosbridge）；改网页不用部署到 Thor。
  v3.0.0 起导航模式下可叠加（左侧"导航显示"两个勾选框）：`/local_costmap/costmap`（6×6 m 滚动窗口，限流 500 ms、CBOR 传输）、
  全局路径 `/plan`、RPP 正在跟的一段 `/received_global_plan`、碰撞预测轨迹 `/lookahead_collision_arc`（未来约 1 s）。

### 1.2 机器
| 机器 | 状态 |
|---|---|
| **Thor 128.146**（`unitree@192.168.128.146`，`/opt/G1/lighting_ws`，aarch64 / Ubuntu 24.04 / jazzy） | 主测机，**v3.0.0**：v1.0.0 全部部署于 10-06，10-08 又部署了 lightning（定位修复 3.2.9，已实机验证；同日再部署 g2p5 扩容修复 3.1.7）。sudo 要密码（服务启停免密）；开机自启 unit 指向 install；D435 序列号与默认值一致 |
| **Thor 112.70**（10-15 起 `192.168.1.103` 直连；阿里云 10-20 起 `ssh g1-thor` = `-p 30422 unitree@47.108.223.180`，原 45022，tegra-ubuntu，wlP1p1s0） | 10-15 核对：`nav2_params.yaml` 与 `4213f93`（v1.0.0 导航）md5 一致，lightning 仍是 10-08 修复前的版本（无 3.2.9），src 已删。手已改装。D435 线易松（10-15 开机没认到，重插后 USB 3 正常）。sudo 免密。D435i 序列号 317622075180，写在 `/opt/G1/g1_autostart.local.env` |
| **Orin 30524**（无 Thor 的 G1，`ssh g1-orin` = `-p 30524 unitree@47.108.223.180`，WiFi 192.168.1.109，eth0 = 192.168.123.164 即 G1 自带 PC2） | 10-20 部署当前工作区（v3.0.0 + 未提交的 3.3.6 keepout 修复）。JetPack 5.1.1 / Ubuntu 20.04 装不了 jazzy → 栈跑在常驻容器 `g1`（镜像 `g1-jazzy`，24.04 + jazzy），见 `tools/deploy/orin_docker/README.md`；手动 ros2 用 `/opt/G1/docker/g1_docker.sh shell`，**不要执行 `g1_service.sh install`**。本机覆盖：`/opt/G1/cyclonedds_g1.local.xml`（eth0）、`G1_MID360s_config.local.json`（host_ip .164）、local.env（D435i 250122078132）。语义地图 CPU 推理。sudo 要密码（服务启停免密）。地图：10-23 `1792742282012` → 10-24 `1792812698887` → **现用 `1792839199393`**（开发机 `~/G1_bags/maps/` 有拷贝）。另有别人的 `xysj-robot` 服务（unitree_sdk2，DDS domain 0），不要动。**这台是没有 Thor 背包的 G1**，本文其余实测（2.6 运动特性、3.4 到点、桥接 / RPP 参数）都来自带 Thor 背包的 G1，不能直接照搬；本机参数（10-24 16:10 起）全部在覆盖文件里，**不再就地改 install**（定位：10-27 起 `/opt/G1/lightning.local.yaml` 经 `loc_params_overlay:=`，`init_yaw_search_deg 0`）：`/opt/G1/nav_bridge.local.yaml`（`min_wz 0.32`、`walk_min_wz 0`、`wz_stop 0.15`、`max_reverse_vx 0.3`、`min_reverse_vx 0.25`、蟹行补偿 `crab_deg 10`）和 `/opt/G1/nav2_params.local.yaml`（`rotate_to_heading_angular_vel 0.9`、`xy_goal_tolerance 0.3` / `yaw 0.30`、local costmap `robot_base_frame: base_link_nav`、BackUp 行为树、smoother 下限 −0.3），源在仓库 `tools/deploy/orin_docker/`，local.env 的 `G1_ROBOT_ARGS` 用 `bridge_params_overlay:=` / `nav_params_overlay:=` 加载；**本机仓库的文件是源、优先级最高**（用户要求先改本机再拷），改了源要手动拷到 `/opt/G1/`，拷完 md5 核对。**控制器 10-26 17:25 起是 shim**：`nav_params_overlay` 指向 `/opt/G1/nav2_params.shim.local.yaml`（= local.yaml 全部内容 + FollowPath 换 shim，两份互不引用，改一份要同步另一份），`bash /opt/G1/nav_controller_switch.sh status|rpp|shim` 切换（只改 local.env），切完要重启 g1-robot（3.7.9）。部署 g1_nav_bridge / aid_navigation2 不会再冲掉（旧 install 备份 `/opt/G1/backup_1024b/`）。D435 彩色同事已不用：local.env 里 `realsense_enable_color:=false realsense_depth_profile:=848x480x15`（10-24 部署 robot_bringup 后生效：realsense 节点 CPU 99% → 8%，整机空闲 22% → 43%）。重启照常用 `g1_service.sh restart`（清理脚本是纯 Python，主机上能跑）。**停服务不清残留会留下整套孤儿导航栈**（ROS 进程在容器 cgroup 里，systemd 够不着；10-24 实测 09:18 的导航栈与新栈同名并存，负载 20 → 35），10-24 起 `g1_docker.sh service-stop` 自带清理，见 `tools/deploy/orin_docker/README.md`。**ssh 里直接执行 `g1_service.sh restart` 会被清理脚本连同 ssh 会话一起杀掉，只停不启**：写成脚本用 `setsid nohup bash /tmp/x.sh &` 启动（5.5）。负载常年 14~20 / 8 核，跑分析脚本要考虑（5.2） |
| **Orin 30423**（第二台无 Thor 的 G1，`ssh -p 30423 unitree@47.108.223.180`，WiFi 192.168.1.116，sudo 要密码） | 10-26 用 30524 整包装好：容器快照镜像 `g1-jazzy:orin-20261026` + qdrant、`/opt/G1`（含 build/install、覆盖文件）、`unitree_ros2` / 模型 / 地图；包在两台的 `~/g1_bundle_1026/`（`install_bundle.sh` 可重跑，自动换 D435 序列号 254343062813）。10-27 已在跑（`loc_global_init`、`lightning.local.yaml` 同 30524），控制器是 shim 但仍是 10-26 15:25 版（没有站稳 / 到点转身），**地图是 30524 场地的，要在本场地重建**。打包时 `docker save` 不受 nice 限制，30524 负载冲到 34，要避开导航时段 |
| 开发机（本地） | x86 / Ubuntu 22.04 / humble。**默认不在本地编译**，本地 install 不完整、不可用；用户授权时可在 scratchpad 编 lightning 做离线回放（5.2）。Meteor Lake 核显在 6.8 内核下会 GPU hang 导致整机卡死（`journalctl -b -1 -k | grep -E 'i915|DMAR|GPU HANG'`） |

### 1.3 实测指标（128.146）
| 项目 | 版本 / 日期 | 结果 | 数据 |
|---|---|---|---|
| 定位（导航 1 h / 483 m，停稳后激光核对） | v3.0.0，10-08 | 有效 27 次 0~4.2 cm、航向 ≤0.3°，不随路程增大；守护全程 GOOD，跳变检查 0 次 | Thor `/opt/G1/bags/replay/stop_fit_1008.log` |
| 定位（v1.0.0，正常时到点后激光核对；导航中会被拉偏 0.2 m 以上，3.2.8） | v1.0.0，10-06 | 1~4 cm | `goal_err_1006c/fit.txt` |
| 到点精度（前台↔厕所 10 次，按激光真实位置） | v1.0.0，10-06；**v3.0.0 未重测** | 平均 12.9 cm、最大 19.8 cm，航向 ≤2.8°，10/10 成功，单程 21~30 s | Thor `/opt/G1/bags/replay/goal_err_1006d/` |
| costmap 动态障碍残影 | 09-23 起未变 | local 0（清除延迟 p90 0.3 s），global 约 2 s 清掉 | 见 3.3 |
| costmap 近身噪点 | 09-24 起未变 | base_link 0.5 m 内 D435 致命格 0；走路时手的误标 41 次 → 0 | 见 3.3 |

## 2. 硬性规则和固定事实（不要重新验证）

### 2.1 安全：绝对禁止后退（最高优先级）
**G1 收到负的前进速度会直接摔倒。** 以下防线都不得放宽，`test_never_reverse` 等测试守着：
- RPP `allow_reversing: false`；保留的 MPPI 参数 `vx_min: 0.0`；velocity_smoother `min_velocity[0]: 0.0`。
- BT 里的 `BackUp` 恢复节点已删除，`backup` / `drive_on_heading` 同样不得进 BT。被困时只能靠清图 / Spin / Wait。
- `g1_cmdvel_to_sport` 对 `vx<0` 硬钳位为 0。
- 规划器不能用 REEDS_SHEPP 之类会规划倒车的模型。
- **例外：无 Thor 背包的 Orin 30524**（10-24 实测能稳定后退，2.6）。只用本机覆盖文件放开 BT 脱困后退，仓库默认值不变，测试 `test_never_reverse` 照旧守着：
  `tools/deploy/orin_docker/nav2_params.local.yaml`（换成带 `BackUp` 的行为树 `navigate_w_recovery_backup_…xml`，退 0.25 m / 0.3 m/s；smoother `min_velocity[0] −0.3`）
  + `nav_bridge.local.yaml`（`max_reverse_vx 0.3`、`min_reverse_vx 0.25`，只允许直线后退）。RPP `allow_reversing` 仍为 false。三处数值由 `test_orin_backup_overlay` 守着要一致。
  **不得把这两个文件或带 BackUp 的行为树用到带背包的 G1 上。**

### 2.2 编译与部署
- **不在本地编译**（用户明确要求）：本地只改代码，编译、跑 gtest/pytest 一律在 Thor 上做；Thor 离线就如实说"未编译"。
- **部署前必须征得用户同意**。验证时不主动发导航目标、不发速度；到点测试由用户在前端发点，我们只订阅测量。
- 部署脚本：`tools/deploy/deploy_to_robot.sh <IP> [包名 ...]`（默认 lightning semantic_map_ros robot_bringup）：
  rsync 按内容同步 → `g1_service.sh stop` → 只编译指定包 → `start` → 打印守护状态。
  排除 `bin/`、`thirdparty/`、`script/g1_humble_env.sh`。重启会让定位重新初始化，见 2.3。
- **部署 lightning 前先补 thirdparty**：脚本不传 `lightning-lm/thirdparty`，而部署后删 src 会把它一起删掉，下次编译就缺 Sophus / livox_ros_driver。
  用 Thor 自己的原件：`mkdir -p $WS/src/lightning-lm && cp -a /opt/G1/exp/base/src/lightning-lm/thirdparty $WS/src/lightning-lm/`
  （`/opt/G1/exp/*` 下 4 份内容一致；本机 `G1_deps` 那份是从上游补的，和它不同，不要传）。Thor 上编 lightning 约 3 分钟。
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
- **以上都是带 Thor 背包的 G1 的数据。没有 Thor 背包的 G1（Orin 30524）10-24 实测原地转**（临时起第二个 `cmdvel_to_sport` 实例，
  `min_wz 0 / wz_stop 0`，只发 vx=0；转角取 `/odommodestate` IMU 航向，位移取 map->base_link，每档 6 s）：
  - **|wz| ≤ 0.29 完全不转**（0.15~0.29 共 7 档，航向变化 ≤0.1°），**0.30 起转**，0.1~0.2 s 响应：是硬死区，不是慢慢转。
  - 实际/指令约 0.7~0.8：0.30 → 0.21 rad/s、0.35 → 0.28、0.40 → 0.42、0.50 → 0.41、0.60 → 0.47。
  - 身体位移转 70~170° 时 13~41 cm，**不随转速降低而减小**（0.30~0.35 档 27~41 cm），比带背包的 4~17 cm 大。
  - 据此该机改：桥接 `min_wz 0.6 → 0.32`、RPP `rotate_to_heading_angular_vel 0.6 → 0.35`（10-25 再改到 0.9，见下面"原地转漂移"），都在覆盖文件里；仓库默认仍是 0.6，给带 Thor 背包的 G1。
    改的起因：该机 10-22 / 10-23 到点前在目标周围半径 ~0.23 m 绕圈 50~75 s 进不了 0.10 容差（RPP 给 wz 0.3~0.56，被抬到 0.6 后转过头），
    `progress_checker`（0.10 m / 15 s）把绕圈当作有进展，不会自动中止。
  - **改 min_wz 后（11:42 lmw 新图）仍绕圈**：`/cmd_vel` 实测是纯原地转 `vx 0 / wz -0.35`，但该机身体绕自己右侧约 0.28 m 的点画圈（LIO 雷达位置同样在走，不是定位算错），
    目标落在圈心附近时目标方位始终在右侧约 90°（>45°）→ RPP 一直原地转，最近只到 0.15 m，绕了 2.5 min。
    于是该机 install 再改 `xy_goal_tolerance 0.10 → 0.20`（`stateful: true` 进容差即锁位置、只转航向），代价是到点精度约 0.2 m。**改后效果未验证**。
  - **后退实测（10-24 15:50，该机，有人扶）**：直接发 `SetVelocity` 负 vx（绕过桥，10 Hz 续发 0.5 s），每档 1.5 s，脚本 / 数据 `/opt/G1/bags/replay/reverse_test.py`、`reverse_test_20261024_155044/`：
    vx −0.1 完全不动（死区）；−0.2 退 0.11~0.13 m（起步延迟 0.79 s，稳态约 0.13 m/s）；−0.3 退 0.25 m（延迟 0.55 s，稳态约 0.21 m/s）；
    姿态偏差 ≤3.8°，侧漂 ≤3 cm、航向 ≤1.6°，发零速后 0.25 s 内停住（再滑 2~4 cm）。**没摔**。2.1 的禁令来自带 Thor 背包的 G1，仓库防线未改。
    据此 10-24 16:10 在该机放开 BT BackUp 脱困（2.1 例外）：清图 → 退 0.25 m → Spin → Wait。`ros2 param get` 已核对生效；**真实被困时的 BackUp 效果未验证**。
  - **走路时转向（10-25，`motion_test_1025_2031`，vx 0.4，直接调 SetVelocity）**：没有原地转那样的死区，基本线性：wz 0.10 → 实测 0.054~0.066、0.20 → 0.13、0.30 → 0.21~0.23。
    之前从导航包推出的"走路时 |wz|<0.15 不转"是测量方法错了（那段指令每 0.1 s 正负切换）。所以桥接走路时不再抬 wz（`walk_min_wz 0`，3.7.5）。
  - **走路蟹行（10-25）**：只发 vx、wz=0 时，实际行进方向比机体航向**偏左约 9.5°**：导航包定位 +9.2°、`/odommodestate` +9.8°、motion_test 直走 +10.3~11.1°。
    纯追踪的平衡点 = 前视 0.6 × tan 9.5° ≈ 0.10 m，正是实机"一直偏路径一侧 ~9 cm"（3.7.5）。另外每步上身左右晃 ±7 cm（步态本身，遥控时也有），导航去不掉，只能不对它反应。
  - **原地转漂移（10-25，`motion_test_1025_2027`，从静止转 120°，腿式里程计）**：逆时针 0.6 → 0.22~0.33 m、0.8 → 0.15~0.20、1.0 → 0.08~0.10；顺时针 0.6/0.8/1.0 都在 0.06~0.15；过冲 1~7°。
    即**转得越快漂得越少**（和上面 0.30~0.35 档的结论一致），所以原地转 0.35 → 0.9（= smoother `max_velocity[2]` 上限）。
  - **停步 / 换动作的惯性（10-26 shim 实机包 50 次到点统计，中位数）**：vx 0.3 一步降到 0 后，0.1 s 内就开始原地转，那时身体还在以 0.27 m/s 往前冲；
    0.9 rad/s 一步降到 0 时身体还在转 0.6 rad/s，多转 ~9°；转完 0.5 s 就起步。桥接把 0.05~0.3 的 vx 抬到 0.3，所以发给 G1 的速度只有"≥0.3"和"0"两档，没法渐变停车。
    用户反馈急停后重心前倾、转向时重心偏一侧，再接着换动作容易摔（S 弯、出发点附近最明显）。骨盆 IMU 前倾 / 侧倾偏差最大 4~8°，对重心偏移不敏感，不能据此说没风险（3.7.10）。

## 3. 问题与解决记录（截至 v3.0.0）

v1.0.0 之前的问题都在这里；v3.0.0 新解决的只有 3.2.9（定位拉偏根因），3.2.6 / 3.2.8 是它之前的过程记录。
3.7 是 10-24~10-27 在无 Thor 背包的 G1（Orin 30524）上的导航调试；那台机器上的定位问题大多还没解决，在第 4 章（4.1、4.2、4.13~4.15）。

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
  坐标系与旧图 `1789975513067` 差 ~20°，点位要重标。2D 栅格多层墙、整体拟合像转了 1.25°，根因是 g2p5 扩容错位（3.1.7），3D 没问题。

#### 3.1.7 2D 栅格多层墙、相对 3D 像转了 1.25°：g2p5 扩容错位（10-08）——已修，已部署 128.146
- 现象：地图 `1790676615248` 的 map.pgm 远处墙相对 3D 偏 0.3~0.7 m，有多层墙；3D 单层墙。整体拟合最优是转 −1.3° + 平移 (0.05, 0.15)，
  但近处（0~5 m）用这个变换反而变差 → 不是刚体转，是几块内容各自错位。
- 排查：从 `mapping_corridor_0929`（这张图的建图包，db3 有损坏，要顺序读到出错为止）抽出 165 条 `/map`：
  第 163 条就是存下的 map.pgm（逐格一致）；重绘前的增量图和 3D 吻合 0.89，**每次重绘或扩容后整体平移一次**，
  和位姿、回环、存图时机都无关（`/lightning/registered_scan` 累积的 LIO 点云和 3D 只差 0.4°）。
- 根因：`G2P5Map::Resize` 按整块子网格（16 格 × 0.05 = 0.8 m）把旧数据搬到新数组，偏移量 `round(Δ原点 / 0.8)`；
  而 `G2P5::ResizeMap` 给的新原点只对齐到 0.05 m。所以每次扩容旧内容整体错位 `|Δ/0.8 − round(Δ/0.8)| × 0.8`，最多 0.4 m，
  之后新画的帧位置是对的 → 新旧内容错开。建图时地图不断向各方向扩，错位叠加成多层墙；回环重绘是从空图重新扩一遍，换一套错位。
  验证公式：包里 11 次扩容，预测错位与实测（两张相邻 /map 互相配准）全部一致到 5 cm（例：Δ −1.80 → 0.20，Δ −0.45 → 0.35，Δ −6.40 → 0）。
- 解决：`g2p5_map.cc` `Resize` 把新原点向外扩到与旧原点相差整数个子网格（空图以 0 为基准，原点始终在 0.8 m 格点上）。
- 验证：`test/test_g2p5_map.cc`（gtest，用包里实际出现过的原点变化量连续扩容，检查旧点、新点都在原格）。
  在 Thor `/opt/G1/bags/replay/g2p5_map_test/`（`build_test.sh`，独立 g++ 编译，不碰 install）：旧代码失败（扩容后旧点全部移位），新代码通过。
  **10-08 已部署 128.146**（只部署 lightning）：`build/lightning/bin/test_g2p5_map` 通过、`test_loc_guard` 15/15；
  重启后激光核对：输出 (−0.10, −0.24, −4.3°) vs 激光最优 (−0.06, −0.24, −4.5°)，10 cm 内吻合 0.77。**还没有用新代码实机建过图**。分析脚本和抽出的 165 条 /map 在同目录 `ana/`（`dump_maps.py` 抽包、`verify_shift.py` 验公式、`eval_maps.py` 逐条对 3D；脚本里路径按开发机写的）。
- 影响：只影响 2D 栅格（网页显示、Nav2 global 静态层、禁行区底图），不影响定位。在用地图的 map.pgm 不会自己变好：
  要么部署后重建图（坐标系会变，点位要重标），要么从 global.pcd 重生成 pgm（保留坐标系）。
- 另（未查证、不是这次的原因）：`SlamSystem::SaveMap` 不等 g2p5 后台重绘完成就取栅格，回环后马上存图可能存到旧位姿画的图。

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

#### 3.2.6 定位漂移 / 地图双解（09-29~30）——当时只加了防护，根因见 3.2.9
- 现象：机器人回到原点，`/base_link_pose` 偏 1.9 m / 19°，glog 无告警，NDT 分值仍有 1.96。
- 漂移主要来自 lidar_loc 的地图修正（LIO 85 分钟回起点只差 9 cm；但单趟沿走廊会少算约 4%，同样是机身点造成的，见 3.2.9）。
  - 旧图走廊尽头**双解**：纵向最优解在 x≈−4.5（对）和 x≈−5.0（错）之间跳；该图行走区有大量现场已不存在的点团，
    只有一面沿走廊的长直墙约束纵向。同一个包实机落在 −4.5、离线回放落在 −4.97，激光吻合度 49% vs 24%。
  - 新图走廊**导航时必现**（09-30）：~0.3 m/s 导航、两端快速转身时，输出沿走廊被往原点方向拉 1.46 m、2.2 m；
    同时 LIO 与腿式里程计互差 ≈0.1 m。手动遥控慢走、原地快转只偏 0.1~0.2 m。不是 DDS（spdp、0 断档、0 丢包），
    离线回放无 Nav2 也复现（峰值 1.32 m）→ CPU 竞争不是主因。
  - 走行中 NDT 残差常卡 0.55~0.6 m：线搜索被关掉后沿牛顿方向盲走 6 步 × 0.1 m，得分不升反降（3.2.9），不是收敛。
- 解决（已部署）：
  - **一致性守护 `lidar_loc/loc_guard.{h,cc}`**：窗口（60 s 或 LIO 行程 5 m）内比较"输出相对运动"与"LIO 相对运动"，
    超 0.2 m / 5°（静止时 0.1 m）判 NDT 可疑；腿式里程计投票区分 NDT 错 / LIO 错；`action: freeze` + `rollback`：
    冻结地图修正只跟 LIO，并退回窗口内累积的修正；连续 10 帧 NDT 残差 <0.15 m / 2° 才恢复；冻结超过 30 帧提示 NEED RELOC。
    输出 `/lightning/loc_status`（JSON）和 glog 每帧 `loc guard frame:` 行（lo / out / odom 位姿、drift、res、state）。
  - **NDT 单帧跳变检查**（`max_ndt_jump_m 0.2` / `_deg 3`）：NDT 结果离 LIO 预测超过该值本帧不采信。
  - NDT 输入投影历史关键帧的实现修正：原实现在 `scan_undistort_` 上原地追加，而关键帧与它共用 CloudPtr，
    当前帧是新关键帧时遍历自身同时 push_back（未定义行为）；改为副本上投影并跳过自身，`loc_input.proj_kfs` 开关（默认 true）。
  - `/initialpose` 重定位（map 系 base_link 平面位姿经 `BaseTFPublisher::BasePoseToLidar` 换成雷达位姿，±15° / 3° 航向搜索，
    成功后 `pgo_->Reset()`，失败时不退回功能点初始化）：**10-24 Orin 30524 实机验证可用**——部署重启时机器人不在原点，原点初始化收敛到错误位姿（横滚 -40°、激光吻合 0.47），前端"模式"页"在地图上拖动设置位姿"给大致位姿 (3.61, 0.2, 180°) 后，收敛到 base_link (3.24, 0.59, -171°)，激光吻合 0.948、峰值清晰（初值差 ~0.45 m / 9° 由航向搜索 + NDT 纠正）。
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

#### 3.2.8 导航时 NDT 又被沿走廊拉偏（10-06，两次）——根因见 3.2.9，10-08 修复后实机未再出现
- 10:56 第一次导航到前台后，输出 − LIO 的修正量跳到 ~0.19 m，之后在前台一侧保持 0.15~0.25 m。
  守护**没报**：每个 60 s 窗口的增量都低于 0.2 m。直到重启定位后才发现，新旧位姿差 0.25 m（腿式里程计证明机器人没动），
  激光吻合度新位姿 82%、旧位姿 48%。
- 14:23 去厕所途中被拉偏 ~0.2 m，守护 14:24:08 冻结 + 回退 0.19 m，之后一直 NEED RELOC，纯 LIO 下定位偏 5~7 cm。

#### 3.2.9 导航时被拉偏的根因：机身点 + NDT 线搜索被关掉（10-08）——已修，已部署 128.146 实机验证
- 现象复现：`corridor_newmap_0930b` 包里的线上输出 +648 s 起被拉偏，+700 s 偏 1.9 m / 14°；本地离线回放
  （关掉单帧跳变检查 = 09-30 线上配置）在同一时刻同样偏 1.9 m → 由数据决定，不是 CPU / 丢帧。开机初始化也偏 0.42 m / 2°
  （4.2 的 0.33 m 同类）。所有结论都按激光对地图吻合度判真值。
- 根因 1（主因）：**`pointcloud_preprocess.cc` Livox 分支的盲区判断少一层括号**（`a || b || c && 距离 > blind`），
  盲区从未生效。MID360 打到正下方 0.25 m 内的机身，每帧约 4000 / 10000 点进了 LIO 和 NDT（`point_filter.yaml` 早就注明过，costmap 那边另行过滤）。
  - 对 NDT：机身点随雷达走，把目标函数拉歪。初始化那帧 6 自由度 NDT 的全局最优在 1.8 m 外；去掉机身点后从两个起点都收敛到激光真值（吻合 0.96）。
  - 对 LIO：机身点在机体系里不动，LIO 少算位移，第一趟走廊 6 m 少算 0.24 m（4%）。
- 根因 2（放大器）：**pclomp `computeStepLengthMT` 的 `interval_converged = (step_max - step_min) > 0`**（PCL 旧版 bug，1.12 改成 `< 0`），
  More-Thuente 线搜索一次都不跑：每步沿牛顿方向盲走 [epsilon/2, 0.1]，不检查得分升降。初值偏 0.3 m 以上、Hessian 不定时（走廊轴向曲率只有横向的 1/3~1/6），
  NDT 越迭代得分越低，沿走廊走出 0.6 m（导出输入后复算：出错帧初值得分 1.5~1.8，NDT 结果 1.07~1.47，激光最优 1.7~2.0）。
  当前帧和投影关键帧各自的激光最优位姿只差 0.09 m / 0.3°，运动中当前帧吻合度 0.92，**去畸变和关键帧投影都没问题**。
- 修复（10-08 提交并部署 128.146）：
  - `pointcloud_preprocess.cc`：补括号；去重改为和 `msg->points[i - 1]` 比（`point_filter_num > 1` 时 `cloud_full_[i - 1]` 是上一帧残留）。
  - `default_livox.yaml`：`fasterlio.blind 0.1 → 0.4`（机身点 ≤0.25 m，0.25~0.6 m 实测没有点）。
  - `ndt_omp_impl.hpp`：改回 `< 0`；线搜索内循环后重算 Hessian（原来被注释，`computeHessian` 没实现，用 `computeDerivatives(..., true)`）；
    `trialValueSelectionMT` 补 PCL 1.12 的 `a_t == a_l` 两处保护（否则除零出 NaN，Sophus abort）；`a_t` 非有限时退回 `step_min`。
  - `lidar_loc.cc`：`UpdateGlobalMap` 补 `setTransformationEpsilon(0.01)`（与构造时一致，原来替换对象时漏设，默认 0.1）；
    `Localize()` 结果非有限时本帧按失败处理，不交给 Sophus。
  - 诊断开关（默认关，5.2）：`lidar_loc.log_ndt_hessian`、`lidar_loc.ndt_dump_dir`。
- 验证（本地离线回放 0930b，6 个时刻激光连续优化求偏差）：
  - 线上配置（冻结 + 回退 + 跳变检查 + 投影）：全程 ≤2.3 cm / 0.23°（+20 s 刚初始化 5.5 cm），守护一次没冻结；修复前同配置偏 0.4~0.6 m。
  - 09-30 线上配置（无跳变检查）：+760 / +950 s 从 1.9 m 降到 0；只用当前帧也全程准。LIO 第一趟误差 4% → 0.8%。
  - 只修盲区、NDT 用旧版：这个包也准。但故意给错初值（1.36 m / 10°、0.85 m / 15°），新版 18 / 10 帧收敛，
    旧版 50 帧内不收敛 / 44 帧 → 线搜索修复保留。
  - 代价：NDT 单次耗时中位 21 → 29 ms，p90 34 → 78 ms（x86、两路回放并行；Thor 上要实测）。
- **实机验证（10-08，128.146，只部署 lightning）**：原点重启后激光核对吻合 0.80（正常值），初始化正确。用户前端发点导航约 1 h、
  定位轨迹累计 483 m（走廊来回、两端转身，单段最长连续 91 m），每次停稳后自动激光核对（`stop_fit.py`，5.2）：
  - 30 次核对，3 次采集时机器人在动作废，**有效 27 次误差 0~4.2 cm、航向 ≤0.3°**，不随路程增大；原点处固定约 −3 cm（x）。
  - 守护 3659 帧全部 GOOD；跳变拒绝 / 冻结 / 回退 / NaN / NEED RELOC 均 0 次（10-06 两段旧日志分别拒绝 3、8 次）。
  - Thor 上 NDT 耗时（glog `loc from` → `confidence` 间隔）：中位 38 ms、p90 91 ms、max 153 ms；旧版中位 29~34、p90 36~37 ms。
    p90 变长来自线搜索和 epsilon 0.01 的额外迭代；跟踪定位只在关键帧跑（行走时约 2 Hz），不影响实时性。
  - 观察（未查证）：行走时相邻定位帧的航向交替摆约 ±5°（LIO 和 NDT 同步摆，二者差值恒定），推测是雷达所在上身随步态扭转，
    腿式里程计（骨盆）只摆约 2°。旧版是否同样没有对照数据。
- 遗留：地图是用有 bug 的代码建的，沿建图轨迹有一串机身点（雷达高度），目前不影响定位，重建地图时会自然消失。

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
- 当前膨胀代价（r = robot_radius 0.50，inflation_radius 0.85，10-25 由 0.75 加大）：local k=8 → 0.55 m 168、0.60 113、0.65 75、0.70 50、0.75 34、0.85 15；
  global k=6 → 0.55 186、0.65 102、0.75 56、0.85 31；再外为 0。梯度带宽 0.35 m。`nav2_params.yaml` 里旧注释按 r=0.30 算（0.45→76、边界 51），10-14 已订正。
- 代价常识：`252·exp(-k(d-r))` 只在 inflation_radius 内成立；254 lethal、253 内切带、255 未知。nav2 没有"被困"检测；
  `collision_margin_distance` 是软惩罚，不是硬安全圈。

#### 3.3.6 全局规划穿过地图上没扫到的灰色区域（10-15，112.70）——已改代码，10-20 部署 Orin 30524，未实机验证
- 现象：前方被障碍挡住后，SmacPlanner2D 重规划的路线穿过 map.pgm 的未知区（灰色），而配置是 `allow_unknown: false`、`track_unknown_space: true`。
- 证据：10-15 112.70 同一时刻 `/map` 未知格 1,406,802、空闲 190,646；`/global_costmap/costmap` 未知只剩 430,019、空闲 972,297。
- 根因：`forbidden_map_create.cpp` 收到 `/map` 后把掩码每格填 0、只在禁行线上填 100；`KeepoutLayer::updateCosts` 照搬 nav2 KeepoutFilter 的
  `data > old || old == NO_INFORMATION`，于是掩码的 0 把 global costmap 所有未知格写成空闲。两台机器都有这个问题。
  local costmap 顶层 `track_unknown_space` 未设（默认 false），底色本来就是空闲，不受影响。
- 解决：`keepout_layer.cpp` 未知格只允许被 LETHAL（掩码 100 → 254）覆盖，其余掩码值跳过。
- 部署：10-20 随 Orin 30524 首次部署（那台还没有地图）；Thor 两台都没部署。
- 待做：建图后核对 global costmap 未知格数与 `/map` 一致；地图边缘 / 灰色里的点位会规划失败（这是预期行为）。

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

#### 3.4.4 当前导航配置（v1.0.0 定下，v3.0.0 未改）
（`aid_navigation2/param/nav2_params.yaml`、`g1_nav_bridge/config/nav_bridge.yaml`）
- 规划：SmacPlanner2D（tolerance 0.125，`use_final_approach_orientation: false`）。
- 控制：RPP，`desired_linear_vel 0.40`，前视 0.6（0.4~0.9），`max_robot_pose_search_dist 1.0`（路径绕回目标附近时不跳到末段），
  `use_rotate_to_heading: true`（起步偏差 >45° 先原地转，到点后原地转到目标朝向），`rotate_to_heading_angular_vel 0.6`（= 桥接 min_wz），
  `allow_reversing: false`，弯道按曲率降速、最低 0.25，碰撞预测 1.0 s 内会撞就停车。
- 原 MPPI 参数整段保留为 `FollowPathMPPI`（不在 `controller_plugins` 里，不会加载）；回退时把名字改回 `FollowPath`。
- `goal_checker`：`xy_goal_tolerance 0.10`、`yaw_goal_tolerance 0.10`、`stateful: true`。
- velocity_smoother：`max_velocity [0.45, 0, 0.9]`，`max_decel [-2.0, -0.35, -5.0]`，OPEN_LOOP，死区 0。
- 桥接：`turn_min_vx 0.0`、`min_wz 0.6`、`min_vx 0.3`（单机覆盖用 `robot.launch.py bridge_params_overlay:=<yaml>`，叠在 nav_bridge.yaml 之后）（带 Thor 背包的 G1；无背包的 Orin 30524 本机值见 1.2 和 3.7，控制器已换 shim）。
- 10-25 / 10-26 起仓库默认值另有改动（两种 G1 都生效）：`cost_travel_multiplier 2.0 → 10.0`、`inflation_radius 0.75 → 0.85`（local / global / MPPI critic 同改）、
  `robot_radius 0.50 → 0.40`、行为树加 `PathExpiringTimer 2 s` 定时重规划（3.7.3、3.7.6、3.7.7）。带背包的 G1 部署前要知道这些变化，Thor 上都还没部署。

#### 3.4.5 别再试
- `goal_checker stateful: false`（用户实测到不了点）。
- MPPI + 带调头圈的路径（阈值调多少都会在目标附近抄近路；再降其他 0.5 的阈值会让 MPPI 在终点附近几乎只剩避障约束）。
- Hybrid-A* 调头圈（能原地转之后没必要，而且窄处规划失败）。
- 收紧容差到 0.12 m 而不改原地转（10-03 仿真：基本没帮助）。

#### 3.4.6 仿真
`src/test/nav_goal_sim/`（humble Nav2 + 照搬抬速规则的 G1 运动学，见其 README）。它低估了横移（理想圆弧，没有步态），
  只能比较方案之间的相对好坏；v1.0.0 的方案没有在仿真里重跑。10-25 起加了 Orin 30524 的实测模型（`g1_orin_model.py`：死区、增益、蟹行、原地转漂移、延迟）、
  跟线 / 重规划 / shim 对比 / 实机目标回放几套脚本，结果见 3.7 和该目录 README。

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

### 3.7 无 Thor 背包的 G1（Orin 30524）导航调试（10-24~10-27）

机器和本机参数见 1.2，运动特性实测见 2.6，还没解决的见 4.1、4.2、4.13~4.18。本节的值除特别说明外，都只在 `tools/deploy/orin_docker/*.local.yaml` 里，
**不要改回仓库默认值，也不要用到带背包的 G1 上**。仿真都在 `src/test/nav_goal_sim/`（README 有表）。

#### 3.7.1 到点前绕圈、进不了容差（10-22~10-24）
- 现象：目标周围半径 ~0.23 m 绕圈 50~75 s，最长 2.5 min。`progress_checker`（0.10 m / 15 s）把绕圈算作有进展，不会自动中止，只能人工取消。
- 根因：① 原地转 |wz|≤0.29 是硬死区，RPP 给的 0.3~0.56 被桥抬到 0.6，转过头；② 改 `min_wz 0.32` 后仍绕：纯原地转时身体绕右侧 ~0.28 m 的点画圈，目标落在圈心附近时方位总在右侧 ~90°，RPP 就一直原地转（2.6）。
- 解决：`min_wz 0.32`；`xy_goal_tolerance 0.10 → 0.20`（10-24 19:26 有人在机器上放到 0.3、yaw 0.30，10-25 挪进覆盖文件）。代价是到点精度 ~0.2~0.3 m。
- 验证：10-26 shim 实机 49/50 到点，停稳后离目标中位 0.25 m（在 0.3 容差内）。要更准得收紧容差，仿真显示必须配合蟹行补偿（3.7.5）。

#### 3.7.2 停服务留下整套孤儿导航栈（10-24）
- 现象：原地转时 LIO 航向丢 ~20°；run_loc 丢帧，但同时段 `point_filter` 10 Hz 正常 → 是进程被 CPU 抢占（livox 队列只有 10）。负载 20 → 35。
- 根因：systemctl 停服务清不掉容器里 launch_manager 起的子 launch（ROS 进程在容器 cgroup 里），09:18 的整套导航栈和新栈同名并存。
- 解决：`stop_ros_processes.py` 加 `--only/--exclude/--dry-run`，`g1_docker.sh service-stop` 自动清残留（10-24 部署，备份 `/opt/G1/backup_1024`）。
  10-26 发现清理会误杀 semantic 服务的外壳进程（`docker exec` 客户端命令行含 install/、不含 semantic_map），语义地图跟着停 → 改为放过它（`983a36f`，已部署）。
- 验证：**实机停服务这条路径没单独验证**（4.16）。

#### 3.7.3 转弯切内角、原地转一甩就进障碍层（10-25）
- 现象：转弯时总卡进障碍物层；原地转身体漂移把机器人推进 253 内切带，RPP / Spin 都报 collision。
- 根因：RPP 严格沿路径走，`cost_travel_multiplier 2.0` 时转弯路径贴着膨胀带内侧（离障碍 0.6~0.7 m）；原地转 0.35 时逆时针漂 0.26~0.67 m（2.6）。
- 解决（仓库默认值）：`cost_travel_multiplier 2.0 → 5.0`（实测路径贴在膨胀带外沿 0.84~0.90 m）→ **10.0**；`inflation_radius 0.85`（local / global / MPPI critic 同改，10-25 16:15 部署）。
  本机：原地转 0.35 → 0.6 → **0.9**（转得越快漂得越少）。
- 验证：10-25 晚同一位置出发的几次，用户看了路径离障碍正常；之后又卡障，主因是 3.7.6。

#### 3.7.4 只原地转、不走弧线（10-25）——已撤回，别再提
- 起因：用户觉得边走边转（弧线）容易摔，要求只保留原地转和直走。10-25 16:15 部署 `turn_in_place_only`，用户随即要求撤回，16:38 已部署撤回版（install 与部署前一致）。
- 后来查明弧线上"不稳"的主因是 collision ahead 急停（3.7.8），不是弧线本身。

#### 3.7.5 走路蟹行：一直偏在路径一侧 ~9 cm（10-25）——已部署，实测有效
- 现象：RPP 跟线一直偏路径一侧约 9 cm，每步左右晃 ±7 cm；直线上越走越偏左，最后卡进障碍物。用户：遥控走得好好的，导航走得很差。
- 排查过程：
  - 怀疑"转向死区让 RPP 的小修正发不出去"→ 2.6 实测走路转向没有死区、基本线性。用户提的"桥接把每个速度都加上死区值"不需要，原地转已经靠 `min_wz` 处理。
  - 怀疑"定位航向随步态 1 Hz 摆动，RPP 跟着反打"→ 讨论过"按步态周期平滑位姿"（航向晚 ~0.5 s），**没做**，因为根因不在这里。
  - **根因：蟹行**。只发 vx 时实际行进方向偏左 ~9.5°，纯追踪平衡点 = 0.6 × tan 9.5° ≈ 0.10 m（2.6）。晃动是步态本身，去不掉，只能不让导航对它反应。
- 仿真（`run_track_test.py`，2 种子 × 9 段，Orin 实测模型；横向偏差 p50 / p95 / 带符号均值、到点误差、锁死段数）：
  | 方案 | 横向偏差 | 到点 | 锁死 |
  |---|---|---|---|
  | 现状（抬速 0.32、原地转 0.6、容差 0.3） | 0.105 / 0.25 / +0.11 m | 0.24 m | 1/18 |
  | A：走路不抬 wz | 0.100 / 0.24 / +0.10 | 0.22 | 2/18 |
  | A+B：再加原地转 0.9 | 0.103 / 0.25 / +0.11 | 0.20 | 0/18 |
  | A+B+`base_link_nav`（10°） | 0.064 / 0.19 / ±0.03 | 0.18 | 0/18 |
  | A+B+`base_link_nav`+容差 0.15 | 0.068 / 0.195 / +0.01 | 0.10（最大 0.18） | 0/18 |
  - 只做 A 改不了偏一侧；锁死都发生在往西走、蟹行把机器人推向南墙 (7.8,−1.2)~(8.9,−1.6) 时，加补偿后 0 次；**单独收紧容差 0.15 会多出锁死，必须和补偿一起**。
- 解决（`36dad49`，10-25 21:10 部署，旧覆盖文件备份 `/opt/G1/backup_1025c`）：
  - **A**：`walk_min_wz 0`（走路时 wz 原样透传）、`wz_stop 0.15`（只决定原地时多小的 wz 算停）。
  - **B**：原地转 `rotate_to_heading_angular_vel 0.9`。
  - **航向补偿**：`g1_nav_bridge` 里的新节点 `g1_crab_comp_frame` 发布 `base_link → base_link_nav`（走路时转 +`crab_deg 10`°，
    g = clamp((|v|−0.10)/0.10)、τ 0.3 s、20 Hz；停下 / 原地转时 = base_link）。**只有 local costmap 的 `robot_base_frame` 用它**
    （controller / goal_checker 取的位姿来自这里）；global costmap / planner / behavior_server / bt_navigator 仍用 base_link。
- 验证：10-25 21:3x 实测直线跟线 xte p50 0.015 m / p95 0.09 m，补偿方向对。部署时机器人不在原点，激光核对差 ≤2 cm。
  **容差收紧到 0.15 还没做**；10-26 换 shim 后 MPPI 也用 `base_link_nav`，**补偿配 MPPI 的效果没单独评估**（4.16）。
- 别再试：`wz_stop 0.05`（那轮跟线更差，p95 0.21 → 0.25 m）。

#### 3.7.6 卡障：网页"避障半径"把膨胀也改小了（10-25）；robot_radius 0.50 → 0.40（10-26）
- 现象：跟线变好后仍总卡障，路径贴墙 0.6 m 走；planner 报 "no valid path found"。
- 根因：有人在网页设避障半径 0.4，网页按"膨胀 = 半径 + 0.20"联动改成 0.6，梯度带只剩 0.2 m。10-26 从 3 个包反推实机当时的半径 / 膨胀：0.4/0.85、0.4/0.6、前 0.5/0.85 后 0.4/0.6，说明一直有人在改。
- 解决：当场 `ros2 param set` 改回 0.50/0.85（运行时改，重启回到 yaml）；网页 `app.js` 改成**只改 robot_radius，膨胀只读**，膨胀 < 半径 + 0.10 时拒绝（`aafff3d`）。
  网页同时给 G1D 用，G1 不支持的功能也不能从前端删。
  10-26 用户要求仓库默认 `robot_radius 0.50 → 0.40`（= 网页常用值，窄道能挤进去；`5fd1a9a`）：膨胀不动，路径离障碍不变；
  MID360 足迹剔除、D435 手部剔除仍 0.50（手离 base_link 最远 0.48 m）；collision_monitor SafetyZone 0.70 → 0.60。是否已部署到 Orin install 未核对，10-26 16:12 录包时机器上就是 0.40/0.85。
- 排查卡障先 `ros2 param get` 两张 costmap 的 `robot_radius` 和 `inflation_layer.inflation_radius`，别先怀疑规划器。

#### 3.7.7 路径贴障、collision ahead 走走停停：规划后才看到的障碍不触发重规划（10-26）——已部署，效果未统计
- 现象：路径有时离障碍 0.4~0.5 m（10-25 21:54 "2 号点 S 弯贴南墙"）；网页截图里 /plan 起点在机器人身后，说明是很久以前规划的（截图底图是 local costmap）。
- 排除平滑：按 jazzy 源码复现 SmacPlanner2D（A* + 内置平滑），干净 S 弯 / 直角弯里平滑只往内拉 2 cm，A* 守在膨胀外沿 0.85 m。**不是平滑的锅**。
- collision ahead 归因（3 个包 30 次）：29/30 是实物（前后持续、D435 和 MID360 都标），机器人离路径 ≤0.1 m，路径离障碍 0.38~0.73 m（jazzy RPP 足迹压到 254 就判撞，r 0.4 时中心 ~0.45 m）：
  ① r050 21:54：87.9 s 规划时走廊中间有 1~4 格临时障碍 (8.4,−1.2)，路径被挤到离南墙 0.41 m，旧 BT 11~26 s 不重规划，同一处急停 5 次；
  ② crab10_2122：41~56 s 走廊中间有地图里没有的实物（40~169 格）；③ A* 贴着 0 代价区边缘走（弯道内侧），而实时传感器看到的墙比地图上的墙近 0.1~0.3 m。
- 根因：旧行为树只在路径进 253 时才重规划，规划后才看到、离路径 0.4~0.8 m 的障碍不会让它改道。
- 解决：两棵行为树都加 `PathExpiringTimer 2 s`（`8659cf0`），10-26 14:3x 部署到 Orin（只同步 + 编 aid_navigation2，旧 BT 备份 `/opt/G1/backup_1026/`）。
- 仿真（`run_replan_test.py`，6 场景 × 2 种子）：路径旁晚看到的箱子，旧树机器人 0.43~0.65 m、2/6 次触发 9 次 collision ahead；新树 2 s 内改道，0.76~0.86 m，0 次。
  障碍一开始就在 global costmap 里时，规划器自己走中线（0.83 m）。
- 验证：**实机还没统计部署后 collision ahead 次数是否下降、原地转是否变多**（之后很快换了 shim，3.7.9 的实机数据已包含新 BT）。

#### 3.7.8 弧线上步态不稳（10-26）——主因是急停；`walk_max_wz`、CLOSED_LOOP 别再试
- 现象：用户觉得走弧线时步态不稳（没摔），速度有跳变。
- 试过：桥接加 `walk_max_wz`（走路转向上限 0.35）。看完速度曲线后用户要求**去掉，已全部撤回**。
- 速度曲线（r050_1025_2154 + crab10_1025_2122/2133，CSV 在 Orin `/opt/G1/bags/replay/vel_1026/`）：加减速本身是线性的（vx 0.035/拍、wz 0.06/拍）；
  走路 226 s 里"走着急停"40 次（每 ~6 s 一次），其中 32 次是 RPP collision ahead（vx、wz 同时清零，站 1~2 s 再起步），88% 在弧线上；躯干横滚 / 俯仰在弧线和直线上几乎一样。
  → 不稳的主因是急停后再起步，`walk_max_wz` 管不到（RPP 的碰撞预测用的是桥之前的指令）。`/odommodestate` 的 velocity 是机体系（已核对）。
- 别再试：velocity_smoother `feedback: CLOSED_LOOP`（静止起步死锁，仿真 40 s 位移 0，nav2_params 注释里写了）。

#### 3.7.9 控制器 RPP → shim：远处 MPPI、近目标 RPP（10-26）——实机明显变好
- 起因：轮式参考机 128.23（`aidlux@192.168.128.23`，humble）用同一个 SmacPlanner2D，路径不贴障，障碍逼近时会让。对照下来主要差在控制器：
  它用自研 `aid_pure_pursuit_shim_controller`（离目标 >0.6 m 交给 MPPI 跟线 + 避障，<0.6 m 切纯追踪 + 原地转）。
  另外它 robot_radius 0.225、local costmap 在 odom 系、有 360° 2D 激光的射线清除。nav2 自带的 RotationShimController 只管起步原地转、不管终点，所以自己写；
  它的 .so 是 humble 编的，G1 跑 jazzy，不能直接用。
- 实现：新包 `src/aid_nav_shim_controller`（`6a7c8ae`）：离目标 ≥ `near_goal_distance 0.6` m 用 MPPI（参数照搬 `FollowPathMPPI`），< 0.6 m 用 RPP（退出滞回 0.3）；
  新目标起步朝向偏 >45° 先原地转到 <0.3 rad（0.9 rad/s）；**卡住检测**：MPPI 阶段 10 s 内沿路径剩余长度没缩短 0.25 m 就报失败，
  而且要一直报到 controller_server 中止，否则会被 `failure_tolerance` 吞掉。
- 仿真（`run_shim_test.py`，7 个障碍场景，Orin 模型 + 新 BT）：迎面来人 RPP 两次都撞、shim 不撞；窄道两者都基本过不去（属于 robot_radius 的问题）。
- 部署：10-26 装进 install（备份 `/opt/G1/backup_1026_shim/`），**17:25 切为生效**；`nav_controller_switch.sh rpp` 可切回（要重启 g1-robot）。
- 实机对比（RPP 16:12 包 `~/G1_bags/veljump_1026_1612` vs shim 17:34 包 `~/G1_bags/shim_1026_1734`，各自的 `_csv` 是导出）：
  | 指标 | RPP | shim |
  |---|---|---|
  | 到点成功 | 31/38 | **49/50**（最后 1 趟是手动取消） |
  | collision ahead 停车 | 468 次 / 45 段 | **4 次 / 3 段** |
  | 卡住总时长 | 437 s | **10 s** |
  | 恢复行为 / BackUp 失败 | 57 / 18 | **0 / 0** |
  | 单程用时中位数 | 19.4 s | 19.5 s |
  | 速度跳变（每分钟 / 每趟） | 24.9 / 12.7 | 21.8 / 7.4 |
  RPP 那次的主要问题：到点原地转把机器人挪进墙 0.4 m 内，接着 RPP、BackUp、Spin 全判碰撞，一卡 30~40 s。
- 还剩的问题：
  - 33 次 "Optimizer fail"（MPPI 找不到可行轨迹），集中在 7 段、每段 0.5~2 s，其中 6 段发展成 "Controller patience exceeded"。出事时最近障碍 0.40~0.6 m（平时 0.65~0.8），是临时出现的东西，都靠重规划自己恢复了。人多的场地以后再看要不要调 MPPI 障碍代价。
  - 控制循环掉帧 22 次 / 17 min（多为 7~9.7 Hz，最低 5 Hz），Orin 负载 ~14 / 8 核、MPPI batch 2000。偶发，暂不改。
  - 到点转身冲过头、换动作不等站稳 → 3.7.10。

#### 3.7.10 换动作不等站稳，有摔倒风险 → shim 加站稳等待和到点转身（10-26~10-27）——已部署（30524；30423 仍是 10-26 版）、已提交，未实机测完
- 现象：用户反馈有障碍急停后重心前倾、再起步容易摔；转弯时重心偏一侧、停住再转也有风险，S 弯和进出出发点那段最不稳。
- 数据（shim 17:34 包，50 次到点，2.6 有数值）：到 D 点（出发点）从走廊进来时朝向 ~160°，目标朝向 −8°，每次都要原地转 ~170°，前后都接着急停和起步。
  刚开始转的 1.5 s 内中断或反向：D 点 9/17、C 点 8/16、B 点 7/16 次，原因是 RPP 先转向路径方向、再转向目标朝向，两者常相反。
  到点转身用满速 0.9 rad/s，一进 yaw 容差就清零，停时航向误差中位 4°，停稳 1.5 s 后变成 9°（p90 18°、最大 27°）；369 次速度跳变里 315 次发生在这段。RPP 那份包也一样，**根子不在 shim，在动作衔接**。
- 解决（shim 外壳输出端加，不动桥接、不动 G1 参数）：
  - `settle_time 0.6`：在 走 / 后退 / 左转 / 右转 之间切换，或停下后再动时，先发 0，至少站 0.6 s，且 odom 降到 |vx| < `settle_max_linear 0.15`、|wz| < `settle_max_angular 0.3`（最多等 `settle_max_time 1.2` s）才放行。
    动作类别按发给桥之前的指令判（`motion_vx_threshold 0.05`、`motion_wz_threshold 0.15`）。
  - `final_rotate true`：进到 xy 容差内（留 `final_rotate_xy_margin 0.05`）后不再交给 RPP，原地转向目标朝向；方向一开始定下（误差 >2.6 rad 时不换向）；
    快到位时按 `final_rotate_decel 1.5` 减速，到位时只剩 `final_rotate_min_vel 0.35` rad/s；漂出 xy 容差就交回 RPP。
  - **代码默认是开的，覆盖文件里显式写出；删掉那几行 ≠ 关**。要关：在 `nav2_params.shim.local.yaml` 的 `controller_server.FollowPath` 下写 `settle_time: 0.0`、`final_rotate: false`。
- 仿真（`run_replay_test.py` 按 10-26 实机目标顺序回放 + 7 个障碍场景；对照 `ctrl_shim.yaml`，新方案 `ctrl_shim_settle.yaml`）：
  | | 旧 shim | 新 shim |
  |---|---|---|
  | 到点成功 | 75/76 | **114/114** |
  | 走路直接变原地转 / 原地转直接起步 | 88 / 107 次 | 1 / 1 次 |
  | 身体没停稳就换动作（每趟） | 3.2 | **0.05~0.2** |
  | 停稳后航向误差 | 9.4° | **4.1°** |
  | 每趟用时中位数 | 18.6 s | 20.5 s（多 ~2 s 花在站稳上） |
  | 7 个障碍场景 | 12/14 | 12/14，结果和离障距离不变 |
- 部署：10-27 09:50。本机仓库的 `tools/deploy/orin_docker/nav2_params.shim.local.yaml` 是源，与 `/opt/G1` 那份 md5 一致；
  .so 先编到 `/opt/G1/stage_settle` 再改名替换（栈在跑时不能原地覆盖 .so）；旧版备份 `/opt/G1/backup_1027_settle/`。
- **待做**：实机测（看急停后起步、S 弯、到 D 点转身）；通过后把 30423 也部署成同一版（`aid_nav_shim_controller` + `nav2_params.shim.local.yaml`）。

#### 3.7.11 运维杂项（10-24~10-27）
- 语义地图服务 10-26 停了：见 3.7.2，停 g1-robot 时的清残留误杀了 semantic 服务的外壳进程，已修（`983a36f`）。
- ssh 里直接跑 `g1_service.sh restart` 会被清理脚本（命令行含 install 路径）连同 ssh 会话一起杀掉，只停不启（1.2、5.5）。
- 开发机连 Orin 的网页：`ssh -f -N -o ExitOnForwardFailure=yes -o ServerAliveInterval=15 -L 9090:localhost:9090 g1-orin`，然后开 `?host=localhost`（仓库根 README）。
- Orin 30423 整包部署见 1.2；Orin 上 Thor↔Orin WiFi 只有 ~300 KB/s，大文件走本机中转。
- git 提交：作者用 `lucifermooc` + GitHub 隐私邮箱，**不加 Co-Authored-By**（10-26 已重写历史并强推）。

## 4. v3.0.0 已知问题（按优先级）

### 4.1 导航时 NDT 沿走廊把定位拉偏——根因已修，10-08 实机验证通过
- 根因和修复见 3.2.9（机身点 + NDT 线搜索被关掉）。10-08 部署 128.146，导航 1 h / 483 m，有效激光核对 27 次全部 0~4.2 cm，
  守护全程 GOOD、跳变检查 0 次。10-06 的两次（3.2.8）没有原始数据，实机未再复现，按同因处理。
- 112.70 还是 09-30 的代码，没有这个修复（4.9）。
- 守护的老问题仍在：慢拉（每个 60 s 窗口 <0.2 m）发现不了；冻结后不会自动恢复；前端不显示 `/lightning/loc_status` 的 need_reloc。
- 单帧跳变检查（0.2 m）在初值偏差大时会挡住所有正确修正（3.2.9：修盲区前，线上配置因此一直偏 0.4~0.6 m）。128.146 修复后没再触发；**Orin 30524 上反复锁死**（10-23 17:53 偏 0.55 m 锁 25 min，10-24 15:00 偏 0.5 m 锁到结束，守护全程 GOOD）。
  10-24 加 `JumpRelock`（`loc_guard.h`，配置 `loc_guard.relock_*`）：连续 5 帧被拒、地图系修正量一致（±0.15 m / 2°）、期间 LIO **净位移** ≥0.5 m（原地转 / 静止不算，起点开阔区的歧义不能放）、分值 ≥0.95×近 50 帧中位数、修正 ≤1 m / 10° 时整帧采信 NDT 并重置守护 / PGO；采信后 5 帧内又被拒 3 帧（NDT 在相对当前位姿往一个方向拉，09-29 走廊那种）就撤回并停用到人工重定位；连续被拒 15 帧或停用时 `need_reloc`=true（前端红色"需要重定位"），`/lightning/loc_status` 多 `relocks`、`jump_reject_run`。离线复算 `tools/loc_debug/sim_relock.py`（16 段 glog）：两次锁死在 17:56:43、15:01:05 解除且验证通过，静止 / 原地转的歧义都不解除。gtest 23/23（Orin 容器 build 目录编译）。
  **10-24 16:05 已部署 Orin 30524**（部署时机器人不在原点 → 初始化错、横滚 −40°，用前端 `/initialpose` 重定位恢复，吻合 0.948）。
- **Orin 部署 JumpRelock 后仍出现的两次（未解决）**：
  - 10-25 16:22~16:25（地图 1792839199393）：远端 x≈10 原地转一圈多，LIO 位置游走 ~2 m；回程一次转向后 NDT 航向残差 0.2° → 4°（LIO 丢航向 ~4°，4.13），
    relock 采信 0.49 m 一次，停到原点附近后 NDT 持续要 0.5 m / 4°（门限 0.2 m / 3°），被拒 200 多次。激光核对：输出 (−0.20, 0.99, 16.4°)，真值约 (0.24, −0.13, 23.5°)，差 1.2 m / 7°；原点附近 x/y 剖面平（约束弱）。
  - 10-25 21:5x NEED RELOC（包 `r050_1025_2154` + `_livox`）：被拒的是平移 0.25~0.45 m、航向 <2°。跳变门限比较的是同一帧，不是时间错位。见 4.15。
  - 用户问过"把跳变门限放大能不能解决"：**不能**。16:2x 那次 NDT 每帧稳定要拉去的 (−0.05, 1.46, 12.5°) 离真值 1.6 m / 11°，比当时输出还远（分值 2.06，正常 2.4~2.5），
    门限拦下是对的。根因在前面的 LIO 航向丢失（4.13），初值偏 1 m 以上时，原点这片约束弱的区域 NDT 自己就收敛到错的地方。要先查清 4.13 / 4.15。

### 4.2 定位初始化总是从地图原点开始，没有防呆
- `run_loc_online` 写死 `SetInitPose(SE3())`，`Localize()` 不看分数；前端"定位"按钮在已定位时也会重启定位。
- **10-06 部署 semantic_map_ros 后，机器人就在原点附近（朝向 −4°）重启，定位仍偏了 0.33 m**（激光最优 (0.05, 0.02) vs 输出 (0.37, −0.08)），
  守护显示 GOOD，没发现。（10-08 部署修复后在原点 (0.12, −0.35, −11°) 重启，激光核对初始化正确。）
  0930b 离线复现了同类现象（原点初始化偏 0.42 m / 2°），根因是机身点（3.2.9），修复后初值偏 1.36 m / 10° 也能收敛；
  但在离原点几米处重启（3.2.7）仍会错，防呆仍要做。
- 待做：重启时用上次的位姿初始化，或多候选比分；`ModeSet("localization")` 已定位时不重启；前端显示 need_reloc。`/initialpose` 10-24 已在 Orin 实机验证可用。
- **10-24 `/initialpose` 的航向搜索也会错**：YawSearch 的粗匹配（5 m 体素）把准初值带到 1 m 外的错解，3 次落在同一处。
  试过"每个航向直接做精匹配"（本地改 `lidar_loc.cc`），用户说"停止，先不使用"，Orin src 已还原、未提交。
  另一个只看"候选点附近有没有地图点"的全局搜索脚本会被密集区骗（给出 11 m 外吻合 0.99 的假解），**别用**。
- **10-27 Orin 两次初始化都错**（09:43、09:50，机器人没动：输出 (0.61, 0.04, 0°)、(1.18, −0.38, 11°)）。
  激光真值 base (2.23, 0.87, 78°)，**离原点 2.4 m、转了 78°**；去地面后 10 cm 吻合：真值 0.41（x/y/航向单峰清晰），输出处 0.08。
  错解的 NDT 分 2.0~2.5，比初始化门限 1.8 还高：NDT 分是所有点的似然和，大部分来自地面，偶然对上几面墙就够了，**NDT 分只能说明局部对得上，判断不了位置对不对**。
  NDT 从原点爬坡的收敛范围约一个体素、航向十几度；初始化后快速收敛阶段每帧把 NDT 结果的 50% 加进输出，NDT 在几个局部最优之间跳（0.5~1.3 m），把输出一路带偏。
  用户最初提的"在原点 ±1 m / ±30° 铺候选、粗 NDT 后精 NDT"覆盖不到 2.4 m / 78°，而且和 10-24 一样会被粗匹配带偏，不采用。
- **全图俯视搜索能找到真值**（10-27 离线，几秒）：叠 10 帧静止点云 → 地图和激光都只留离地 0.3~2 m 的点 → 地图画成 0.1 m 栅格并膨胀 1 格 →
  每 2° 一个航向（180 个），激光栅格与地图栅格做 FFT 互相关，一次得到整张图所有平移的命中数 → 取最高并要求明显高于第二名（这次 0.57 vs 0.44）→ ±0.3 m / ±6° 内 3D 细化、看剖面单峰。
  不需要初值，每个位姿只打分不移动，不会被局部最优带走。长走廊等对称场景会有多峰，必须看第一名和第二名的差距，差距不够就不接受、提示人工重定位。
- **方案 A（10-27 用户选定，本地已跑通）**：独立包 `src/test/loc_global_init`（算法、参数、验证数据见其 README），**不改 lightning-lm**：
  只在采集期间订阅 `/livox/lidar` + `/livox/imu`（和 LIO 同源，不依赖 point_filter / lightning 状态；常订阅时 rclpy 反序列化 CustomMsg 占 GIL，搜索 3 s → 19 s），
  静止（|gyro| ≤0.15）~1 s 叠 10 帧、IMU 重力摆平 → 全图搜索：2D 俯视 FFT 粗搜（0.2 m / 3°）+ **3D 体素精搜**（2D 精分区分不开：10-27 错解 0.43 / 真值 0.59，3D 0.17 / 0.60）
  → 第一名 ≥0.35 且第二名 ≤0.85× → 与 lightning 当前 map->base_link 比，差 >0.3 m / 5° 才发 `/initialpose` 并核对收敛；`dry_run:=true` 只报不发；状态 `/loc_global_init/status`，`~/trigger` 重跑。
  本地验证：单测 10/10（合成房间任意位姿 <0.1 m / 2°；中心对称房间必须拒绝）；0930b 9 个静止时刻全部找对（精分 0.90~0.96、第二名 ≤0.64×；与 lightning 正常时差 0.06~0.12 m，
  +10 s 初始化偏、+675 s 起被拉偏 1.9 m 都和 3.2.9 的激光真值一致）；10-27 现场数据找回 (2.18, 0.87, 78°)；回放 + 替身端到端：错初始化 → relocalized、本来就对 → 不动、dry_run、起步在走 → 等静止，单次搜索 x86 ~3.5 s。
  **本地真 lightning 联调**（scratchpad 编 lightning + 回放 0930b + 本节点）：+700 s 原点初始化错 4.5 m → 发 `/initialpose` → 1.5 s 收敛到 (−4.69, 0.24, 10.6°)、之后稳在 ±5 mm；
  +672 s 错 5.3 m / 52° → 2.1 s 纠正；+10 s 原点初始化本来就对 → 不发；发布后机器人马上起步 → 判"核对时在动"、静止后重采判对。守护全程 GOOD。
  lightning 自己的 ±15° 航向搜索（粗 NDT）仍会把给定位姿拉偏 ~0.14 m，随后跟踪纠正。数据见包 README。
  还没做：`need_reloc` 自动触发；launch 接进 `robot_bringup`；前端显示；Orin 耗时。
  已知代价：lightning 仍会先从原点错初始化几秒，再被纠正，这段时间不能发导航目标（新节点应往前端报"初始化中"）；`/initialpose` 还会再做 ±15° 的 YawSearch，可以把 `init_yaw_search_deg` 设 0（会影响前端手动重定位）。
  方案 B（改 `run_loc_online.cc:72` 和 `lidar_loc.cc` 的初始化分支）从一开始就不会错，但要改 C++ 核心、重编 lightning，等 A 验证稳定后再考虑。
  **Orin 30524 实机（10-27 15:3x）**：同步到 `/opt/G1/lighting_ws/src/test/loc_global_init`，容器里只编这一个包（纯 Python，不停栈）。
  首次运行 30 s 只收到 10 帧点云、0 条 IMU：Orin 上 rclpy 反序列化 CustomMsg 每帧 119 ms，执行器被点云回调占满，IMU 回调饿死（x86 上没暴露）→
  改为 `raw=True` 订阅、numpy 解析 CDR（0.4 ms / 帧，与 rclpy 逐点一致），IMU 单独回调组。之后采集 1 s、**搜索 13~14 s**。
  开机后机器人没动，lightning 原点初始化 (−0.65, 0.28, 15°)、守护 NDT_SUSPECT；本节点找到 base (5.09, 0.77, 119°)（第二名 0.72×），发 `/initialpose` 后 4.5 s `relocalized`，
  守护 GOOD、NDT 分 2.87。**但 lightning 收到位姿后沿走廊停在 (4.85, 0.82)，静止时不纠正**（关掉航向搜索后 15:59 重测照样 4.83：是初始化那次 6 自由度精 NDT
  横滚跑到 −7°、平移挪 0.31 m，重力约束只改姿态；要修得在 lightning 初始化 NDT 时锁住横滚 / 俯仰，方案 B）；独立核对（点到地图最近点距离沿 x 剖面）单峰在本节点结果处
  （0.106 m / 10 cm 内 0.705，lightning 当前 0.123 / 0.585）。航向搜索用户确认去掉（10-24 粗匹配带偏 1 m 的风险，但它不是这 0.24 m 的原因）：加 `robot.launch.py loc_params_overlay:=`（lightning 本机覆盖，按键合并到 `default_livox.yaml` 之上，只作用于定位），
  Orin 本机 `tools/deploy/orin_docker/lightning.local.yaml` 设 `init_yaw_search_deg: 0`，仓库默认仍 15（Thor 无本节点）；Orin 上前端拖位姿的航向要拖准。
  **状态：已部署 Orin 30524（手动 `ros2 launch`，日志 `/opt/G1/logs/loc_global_init*.log`），未进开机自启 / `robot_bringup`，重启后要手动起**。
  10-27 15:5x 两台 Orin 都部署了 `loc_params_overlay` + `/opt/G1/lightning.local.yaml`（航向搜索 0，glog `init yaw search +-0 deg` 已核对；
  备份 `/opt/G1/backup_1027_locovl/`）。整包拷过去的 src 里 `lidar_loc.cc` 比 `.o` 新（10-24 改回原样没重编），两台各重编了一次 lightning（~3.5 min），内容与仓库一致。
  30423 也装了 `loc_global_init`，但那台坐着，没跑。
  **15:3x、15:5x 两次 30524 是吊着的**（雷达离地 1.385 m，落地 1.234 m），上面的 −7° 横滚 / 0.24 m 可能有吊装的份；16:06 落地换朝向后 trigger：
  本节点 (4.96, 0.73, −16.5°)（第二名 0.45×），lightning (4.82, 0.78, −16.9°)，差 0.15 m → `ok_already`。

### 4.3 到点后原地转向的身体位移
剩余到点误差的主要来源（v1.0.0 实测：转 90° 常从 6 cm 被甩到 20~30 cm；v3.0.0 定位更准后没重测到点误差）。可选：转完补一小段位置修正；
  或转向时加少量前进速度抵消后退（需要标定）。
  Orin 30524 更严重：转 70~170° 位移 13~41 cm，逆时针比顺时针大，转得越快越小（2.6），所以该机原地转用 0.9。shim 的 `final_rotate` 只解决冲过头和换向，不减少位移。

### 4.4 RPP 不会局部绕障
只按碰撞预测停车，绕障靠全局重规划（BT 每 2 s 或路径失效时重规划；2026-10-26 前只在失效时，规划后才出现的障碍离路径 0.4~0.5 m 也不会改道，3.7.7）。
带背包的 G1 **还没测过有人 / 障碍物挡路的情况**。Orin 30524 已换 shim（远处 MPPI 能局部绕障），仿真里迎面来人 RPP 两次都撞、shim 不撞，实机 collision ahead 468 → 4 次（3.7.9）；
实机还没专门测过有人迎面走来。

### 4.5 仿真覆盖
`nav_goal_sim` 是 humble 版。带背包 G1 的当前方案（2D + RPP + 原地转，v1.0.0 起）没有重跑。
Orin 30524 已有实测模型和几套仿真（跟线、重规划、shim 对比、实机目标回放，3.7）。仿真和实机的已知差别：模型没有步态、原地转起转延迟按 0.35 s 算（实测 0.1~0.2 s）、
直线模型复现不出"wz_stop 0.05 更差"；传感器只有模拟激光层，没有 D435 / STVL 的噪点和延迟。

### 4.6 `min_vx 0.3` 没有实测依据
限制了慢速靠近，造成进点冲过头。应像原地转一样实测 G1 能否稳定低速行走。Orin 30524 也只测了后退死区（−0.1 不动）和转向，没测走路最低速；
桥接把 0.05~0.3 抬到 0.3，所以停车只能从 0.3 一步到 0（2.6）。

### 4.7 2D 栅格多层墙 / 相对 3D 转 ~1.25°——根因已修（3.1.7），已部署 128.146
g2p5 扩容错位，`g2p5_map.cc` 已修、单测通过，10-08 部署。还要做：用新代码实机建一次图确认；在用地图 `1790676615248` 的 map.pgm 仍是错的
  （远处墙偏 0.3~0.7 m，影响网页和 Nav2 global 静态层），需要重建图或从 global.pcd 重生成。

### 4.8 global costmap 的矮障碍
（用户决定暂不做）：D435 视场内 1.5~2.5 m、低于 0.7 m、MID360 从侧面标过的矮障碍会被 2D 射线擦掉
  （回放视场内丢失率比噪声底高约 40 个百分点，多为 1~3 s，最长 11 s）；1.5 m 外的前方矮障碍 global 看不到。根治要把 global 的 D435 也放进 STVL。

### 4.9 112.70 没有更新
10-15 核对：导航配置已是 v1.0.0（`4213f93`），lightning 没有 3.2.9 的定位修复（当天定位 `NDT_SUSPECT` + 冻结、回退 7 次），src 已删。
10-15 实测导航卡顿 / 不动（`/opt/G1/bags/replay/vel_mon_1015.jsonl`）：起步处身边 0.5 m 内有障碍，RPP 反复 `collision ahead` 清零速度；
接近目标时 RPP 给 vx 0.05~0.09 + |wz|≥0.15，速度桥把 wz 抬到 0.6、vx 不抬，原地左右扭 11 s。未改。

### 4.10 Thor `~/.bashrc` 的 DDS 配置
新开终端执行 `ros2` 会报参与者编号不够（2.4）。可在第 124 行加
  `<Discovery><MaxAutoParticipantIndex>120</MaxAutoParticipantIndex></Discovery>`，或改为 source `dds_env.sh`；需要用户同意。

### 4.11 小问题
- `walk_monitor.py`（Thor）第 46 行订阅回调要写成 `lambda m: self.q.append(m)`，Thor 上是坏的版本。
- `test_g1_g2p5_config.py` 有 3 个早已存在的失败（URDF 关节已改名 `body_to_mid360_joint`）。
- `laser_mapping.cc` MakeKF 的 `20 / 180 * M_PI` 是整数除法（=0），投影关键帧只按 3 m 更新；修它会改关键帧策略，未动。
- lightning 自己的 tag 过滤（`pointcloud_preprocess.cc:53`）是注释掉的（盲区 / 去重的优先级 bug 10-08 已修，3.2.9）。
- `loc_input.proj_kfs` 不是漂移原因：修复后开 / 关都准（3.2.9）。
- 3.6 的遗留项未核实。

### 4.12 collision_monitor 按默认配置不启动（10-14 对照代码发现，实机未核实）
- 现象：本文 1.1 和 `aid_navigation2/G1_DIRECT_MAP.md`、`g1_nav_bridge/README.md` 的链路图都画了 collision_monitor，
  但 `robot.launch.py` 里 `use_collision_monitor` 默认 false（`robot_bringup/README.md` 写明"完成验收前默认关闭"），
  `g1_autostart.env` 也没打开。所以按仓库配置，Nav2 速度经 velocity_smoother 直接进速度桥，没有 0.70 m 停车圈。
  `g1_navigation_direct.launch.py`、`g1_navigation.launch.py` 单独启动时默认是 true，两种入口不一致。
- 疑点：5.3 记录 10-06 用 `plan_rec.py` 录到了 `/cmd_vel_safe`，说明当时实机可能开着（`/opt/G1/g1_autostart.local.env`？）。
- 要做：在 128.146 上 `ros2 node list | grep collision`、`ros2 topic info /cmd_vel_safe -v` 和查看 local.env，确认实际状态后再改本条。
  若要默认启用，注意其参数用的是 `/lightning/registered_scan`（未经 `scan_range_filter`），停车圈 0.70 m、`min_points 4`，需先验收。
  （10-26 起仓库停车圈随 robot_radius 改为 0.60，3.7.6。）

### 4.13 Orin 30524：LIO 偶尔丢航向 ~8°（10-25）——根因未定
- 现象：10-25 16:15 那趟 LIO 比机身 IMU 少转 ~8°，发生在远端 x≈8~10 m **直走 + 小幅摆头**的 12 s（16:22:29~41，当时是只原地转模式），不是那一圈多的大转向；
  NDT 跟着 LIO 一起偏，残差很小，所以守护看不出来。同图 13:39 那趟在同一区域来回 24 m / 2637° 只差 2.3° → 不是每次都发生。
- 比较方法的坑：腿式里程计航向**静止时**也会跳 5~16°，只能在静止段之间比较；转动中比较会被两路采样时刻差污染（±40° 假跳变）。
- 和 3.7.2 的区别：10-24 那次是孤儿栈抢 CPU 导致丢帧；这次 LIO 没丢帧。
- 待做：用 10-25 21:54 录的 `r050_1025_2154_livox`（含 `/livox/lidar`、`/livox/imu`、`/odommodestate`）离线复现。分析脚本在当时会话的 scratchpad，未入库。

### 4.14 Orin 30524：切定位时机器人在挪步，LIO 重力初始化歪 ~8°（10-24）——用户说先不做
- 现象：10-24 16:5x 走起来满屏噪点、collision ahead。切图后定位重启时 G1 在挪步，LIO 重力初始化歪 ~8°（LIO z 随位移涨，坡度 8.3°；往常 0.4~2.4°），
  `gravity_constrain` 把这个倾角带进定位输出和 TF，MID360 点云里前方地面被抬高（1.2 m 处高 0.15 m），全标成障碍。`scan_tilt.py` 逐帧拟合 map 系地面，得 7.3°。
- 修法（IMU 静止时才对齐重力 + LIO 与 NDT 倾角长期不一致时停用重力约束）用户说**先不做**。临时办法：**站稳几秒再切定位**。
- 别误判：建图时 g2p5 报 "floor is not horizontal ~7°" 是雷达自身的安装俯仰，不是地图斜（两张图点云地面 ≤1.3°），不能当证据。

### 4.15 Orin 30524：怀疑地图沿走廊被压短 6~8%（10-25）——未证实
- 依据：10-25 21:5x NEED RELOC 前后的静止段之间，NDT 测得位移 4.96 / 5.25 m，LIO 多 8.5% / 7.4%，腿式里程计多 4.5% / 6.6%，都是沿前进方向多算。
  两路独立里程计都比 NDT 长 → 怀疑是地图短了，这也能解释 4.1 里 NDT 持续要 0.25~0.45 m 的平移修正。
- 验证方法：`scan_vs_map.py`（原始点云按 NDT 位姿放进 `global.pcd`，看偏差是否随离原点距离线性增长）已拷到 Orin `/opt/G1/scalechk_tmp/`，
  22:1x ssh 断开没跑成。**跑完要删 `/opt/G1/scalechk_tmp`**。

### 4.16 Orin 30524：已部署、未实机验证的清单（10-27）
| 项 | 部署时间 | 要看什么 |
|---|---|---|
| shim 站稳 + 到点转身（3.7.10） | 10-27 09:50；15:55 改 settle 0.3 + `settle_mid_path false` | 急停后起步、S 弯、到 D 点转身是否稳；每趟多 ~2 s 能否接受；通过后同步到 30423 |
| 定时重规划 BT（3.7.7） | 10-26 14:3x | collision ahead 次数是否下降、原地转是否变多 |
| 蟹行补偿配 MPPI（3.7.5、3.7.9） | 10-26 17:25 切 shim | MPPI 阶段的横向偏差；补偿是 RPP 时代标定的 |
| 容差 0.3 → 0.15（3.7.5） | 未做 | 仿真要求和补偿一起收 |
| BackUp 脱困（2.6） | 10-24 16:10 | 真实被困时能否退出来 |
| `service-stop` 清残留（3.7.2） | 10-24 | 停服务后容器里是否还有 ROS 进程 |
| 开机自启 | 10-20 | 10-24 12:15 重启过一次，单栈正常；之后改过的东西没再验证 |
| 本机 robot_radius 0.40（3.7.6） | 未核对 | install 里两张 costmap 的值 |

### 4.17 Orin 30524：地图里的建图残留障碍（10-24 排查）——未处理
- 地图 1792742282012：建图起点身后 (−0.42, 0.07~0.28) 有一块实时传感器看不到的障碍（疑似建图时站了人）。起点和目标 (−0.08, 0.03) 离它 0.33 m < 当时的 robot_radius 0.5 →
  "Start occupied" / no valid path；待机位 (−1~−1.6, 0.2~0.5) 在它身后，是未知区。
- 10-23 16:11~16:14 贴墙 0.3 m 被困：Start occupied + Spin 撞检 + 当时没有后退（之后放开了 BackUp）。
- 之后换过两次图（1.2），新图有没有同类残留没有记录。建图时起点周围别站人。

### 4.18 Orin 30423：装好了，导航没启动
整包 10-26 18:45 装完（1.2），服务已 enable、没启动。地图是 30524 场地的，在本场地要重新建图；第一次启动后按 2.3 核对定位。

## 5. 排查方法与工具

### 5.1 出现问题先录包
`tools/field_test/record_nav_bag.sh [名字] [-t 秒] [--light|--full|--loc]`：只录在线的话题，默认不压缩（zstd 吃 CPU）。
**定位问题用 `--loc`**（livox 原始云 + IMU + /odom + TF，sqlite3，可直接给 `run_loc_offline` 回放）。
09-21 那次排查全程没录包，只能靠 glog 反推，很多结论无法复验——别再犯。

### 5.2 定位
- **激光对地图核对是唯一可信的真值**（NDT 分值 confidence 不能当作位置正确的依据）：
  - `/opt/G1/bags/replay/loc_check.py <map_dir> [cx cy]`：抓当前 scan，体素查表，在当前位姿 / 原点 / 指定点附近搜 x-y-yaw，打 x/y 剖面看是否单峰。
    点云地图地面在建图雷达高度下方，z 偏移约 −1.1~−1.25。正常时 10 cm 内吻合 0.8 左右、25 cm 内 0.97。
  - **Orin 负载 ~20 时 `loc_check.py` 120 s 跑不完，被 timeout 杀掉时也没有任何输出**。改用：容器里 `cap_scan.py` 抓 `/lightning/registered_scan` + TF 存 npz，拷回开发机算
    （10-27 脚本在当时会话的 scratchpad，未入库；`src/test/loc_global_init/bev_search.py` 是它的入库版）。
  - **不知道机器人在哪时用全图俯视搜索**（4.2）：去地面、0.1 m 栅格、2° 航向、FFT 互相关，开发机几秒；看第一名与第二名的差距和剖面单峰。
    比较时用"去地面后的吻合率"，带地面的吻合率和 NDT 分都会被地面撑高。现场和建图时有变化时真值处吻合会低于 0.8（10-27 只有 0.41），但峰值仍清晰。
  - `fit_once.py <map_dir>`（单次，输出 JSON）、`fit_watch.py`（每次到点后自动核对，靠 `goal_err` 的 log.txt 触发）。
  - `stop_fit.py <map_dir> <glog INFO>`（`run_stop_fit.sh` 带环境 nohup 启动）：用户从前端发点时用。读 glog 判断停稳
    （离上次核对 >0.3 m、3 s 内 xy <2 cm 且航向 <1°），抓激光核对；采集期间位姿跨度 >3 cm / 1° 判"无效（采集时在动）"——
    不加这条时起步那一刻抓的帧会报出 12 cm 的假误差。静止时 NDT 约 2 s 一帧，判停稳的窗口不能要求帧数多。
  - `glog_stats.py <glog INFO>`：NDT 耗时（`loc from` → `confidence` 间隔）、置信度、跳变拒绝 / NaN / NEED RELOC 计数。
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
- **NDT 诊断开关**（默认关，只打日志 / 导出，不改结果）：
  - `lidar_loc.log_ndt_hessian: true`：每次匹配一行 `ndt hess:`（点数、迭代次数、分值、xy 块和舒尔补的特征值与弱方向角、6 维特征值）。
    条件数 = 强 / 弱特征值；新图走廊 2.5~6，不是奇异。
  - `lidar_loc.ndt_dump_dir: <目录>`：每次跟踪匹配导出 NDT 输入 PCD（雷达系，前 `header.seq` 个点是当前帧，其余是投影关键帧）
    和 `index.txt`（时刻、当前帧点数、总点数、初值、NDT 结果、分值）。拿它在 Python 里复算 NDT 得分，判断"目标函数错了"还是"优化器走错了"。
  - 判断 NDT 是否往下坡走：对比初值和结果处的 NDT 得分（pclomp 同规则重算），结果更低就是优化器的问题。
- **开发机离线回放**（需用户授权编译，10-08 用过）：步骤和脚本在 `/home/ap/G1_bags/replay_scripts/ndt_1008/README.md`。
  要点：源码副本放 scratchpad、补 `/home/ap/G1_deps/lightning_thirdparty`；链接加 `-L/home/ap/G1_deps/lib`（否则链到系统 glog 0.4）；
  jazzy 录的包要改写 `metadata.yaml` 才能被 humble 读；回放有随机性（多线程时序），同一配置两次结果可能不同，结论要多跑几组。
- **离线回放对有些地图不具代表性**（旧图在线 NDT 残差 p90 0.064 m，离线 0.42 m）。`run_loc_offline` 退出时会往地图目录写动态图层，
  回放一律用地图拷贝；临时启动配置 `/tmp/g1_lightning_*.yaml` 停栈时会被清理，离线要用 install 里的 `default_livox.yaml` 自己改。

### 5.3 到点精度测量（10-06 用的方法）
Thor `/opt/G1/bags/replay/` 下，用 `run_goal_err*.sh` / `run_plan_rec*.sh` / `run_fit_watch*.sh` nohup 启动（自带 ROS 和 DDS 环境），都只订阅：
- `goal_err.py <输出目录>`：每次 `/nav_to_pose` 开一轮，`/task_status` 变成功时记"判到达"，位姿稳定后记"站稳"，按目标朝向拆成纵向 / 横向误差；
  点位坐标写在脚本里（前台、厕所）。
- `plan_rec.py`：录 `/plan` 和速度指令（`/cmd_vel_safe` 要用 best-effort QoS 才收得到；collision_monitor 没开时它没有发布者，改录 `/cmd_vel`，见 4.12）。
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
- 机器人会突然掉线（10-06 下午出现过一次，ping 不通、ARP 失败），重新上线后测量进程要重启。Orin 30524 10-25 22:1x 也断过（Connection refused）。
- **Orin：ssh 里直接执行 `g1_service.sh restart` 只停不启**：清理脚本按 install 路径匹配，把带这条命令的 ssh 会话一起杀掉。
  写成脚本，用 `setsid nohup bash /tmp/x.sh &` 启动。Orin 上手动 ros2 一律走 `g1_docker.sh shell` / `run`（主机 `~/.bashrc` 是 foxy）。
- 有别人也在用 Orin 30524（密码登录、调自己的 `xysj-robot`，网页上改半径），排查前先 `ros2 param get` 核对运行中的值，别信 yaml（3.7.6）。

### 5.6 Orin 导航分析（10-26 用的方法）
- 录包：`record_nav_bag.sh`（5.1）；定位问题要带 `/livox/lidar`、`/livox/imu`、`/odommodestate`，否则 LIO 问题没法离线复现（10-25 有一个包就是没带）。
- 速度跳变 / 急停：`tools/nav2_debug/vel_jump_extract.py`（容器里跑，读 mcap）把速度链各级（控制器 → smoother → 发给 G1 → G1 实际）、腿式里程计、位姿、/rosout、/plan 导出成 CSV；
  `vel_jump_analyze.py`（开发机跑）找跳变，并判断是控制器、平滑器、桥（抬速 / 死区）还是断流引入的。
  开发机 `~/G1_bags/<包>_csv/` 是导出结果。collision ahead 归因（墙还是噪点、离路径多远）的脚本在 10-26 会话 scratchpad，未入库。
- 本机回放仿真：`src/test/nav_goal_sim/run_replay_test.py` 用实机包的地图、起点、目标序列和机器上实际生效的配置（`~/G1_bags/veljump_1026_1612_ctx`）重跑，
  导出同格式 CSV，用同一套分析脚本对比"实机有 / 仿真有 / 共有"。只需要定位坐标和目标，不需要点云。
- 仿真和实机的差别见 4.5：仿真有、实机没有的一般是模型简化造成的，先怀疑模型。

## 6. 参考

### 6.1 数据（Thor `/opt/G1/bags/`）
- 建图：`map500d_0919`（8 分钟 0 断档）、`mapping_corridor_0929`（覆盖建图，已 reindex）。
- 定位：`locguard_0929`（旧图 20 min 行走）、`corridor_newmap_0930`、`corridor_newmap_0930b`（导航时拉偏复现，`--loc`）；离线 A/B 结果 `replay/ab0930/`。
  开发机 `/home/ap/G1_bags/` 有 `corridor_newmap_0930b`（完整）、地图 `maps/1790676615248`、Thor 脚本拷贝；`corridor_newmap_0930` 拷贝中断，不能用。
- costmap：`walkby_0923`（修复前）、`walkby_s2_0923`（修复后）、`demo2_0922_1350`、`d435_ghost_0922`（错深度）、`d435near_0924` / `d435near2_0924`（修手前 / 后）、
  `hang_0924`（机身回波）、`move_0924`（错深度 + 飞点）。
- 到点测量：`replay/goal_err_1006`（MPPI）、`goal_err_1006b`（MPPI 阈值 0.3 / RPP + Hybrid）、`goal_err_1006c`（原地转，容差 0.2）、
  `goal_err_1006d`（v1.0.0，容差 0.1）；原地转：`replay/spin_1006`。
- **Orin 30524**（`/opt/G1/bags/` 与 `/opt/G1/bags/replay/`）：后退 `reverse_test_20261024_155044/`；运动测试 `motion_test_1025_2027`（原地转）、`motion_test_1025_2031`（走路转向 / 蟹行）；
  蟹行补偿后的导航 `crab10_1025_2122`、`crab10_1025_2133`；NEED RELOC `r050_1025_2154` + `r050_1025_2154_livox`（含 livox 原始数据，4.13 / 4.15 用）；速度曲线导出 `replay/vel_1026/`。
  开发机 `~/G1_bags/`：`veljump_1026_1612`（RPP）、`shim_1026_1734`（shim，50 趟）及各自的 `_csv`，`veljump_1026_1612_ctx`（录包时机器上的配置），`maps/1792839199393`。
  仿真结果在 `src/test/nav_goal_sim/results/`（不入库）。

### 6.2 lightning 上游事实（对照 github 上游 1325fed 确认，不是我们改坏的）
- `Localize()` 永远返回 true（issue #127：原本靠 RTK 重置，开源版"定位丢了没办法处理"）。
- `YawSearch` 上游注释掉（#123：会选错角度）；我们只在 `/initialpose` 重定位时用（粗匹配前 3 名做精匹配后按精分判定）。
- 跟踪 NDT 在 `UpdateGlobalMap` 里 4 次迭代（循环条件 `>`，实际最多 6 步）、步长 0.1；pclomp 构造时 `transformation_epsilon_ = 0.1`，
  线搜索步长下限 = epsilon/2。pclomp 的 More-Thuente 线搜索原本是关掉的（`> 0`），10-08 已修（3.2.9）。
- `pointcloud_preprocess.cc` Livox 分支的盲区 / 去重判断有优先级 bug，`blind` 从未生效，10-08 已修（3.2.9）。
- 退化检测 / DR 多初值比较上游都注释掉了；lightning 的 "DR" 就是 LIO 自身 IMU 状态，不独立。
- NDT 输入不是纯当前帧：`GetProjCloud()` = 当前帧 + 最多 5 个历史关键帧各取最早 1001 点；`fasterlio.proj_kfs` 管不到（上游判断被注释）。

### 6.3 其他文档
- `launchreadmd.md`：手动启动 / 建图 / 定位 / 导航的命令，新机器首次部署（RealSense SDK、DDS 接收缓冲）。
- `AI_HANDOFF_2026-09-17.md`：早期交接（TF 设计、桥接包、电池、Unitree 官方 SLAM 黑盒检查）。
- `README_SLAM_NAV.md`、`G1_3D_NAV_MIGRATION_ANALYSIS.md`：工作区精简和迁移分析。
- `src/semantic_map_ros/README.md`、`DATA_FLOW.md`：语义地图部署和数据流；仓库根目录 `README.md`、`HANDOFF.md`：网页控制台和仿真。
- `src/test/nav_goal_sim/README.md`：到点精度仿真；10-25 起的 Orin 跟线 / 重规划仿真结果表。
- `tools/deploy/orin_docker/README.md`：Orin 容器部署、覆盖文件、停服务清残留；`src/aid_nav_shim_controller/`（头文件注释写了全部行为和参数）：shim 控制器。
- VLN（LightNav-0）评估在独立工作区 `/home/ap/project/G1_VLN`，不在本仓库。

### 6.4 数据流详解（在线文档，不在本仓库）
两份都是 claude.ai 上的 Claude Docs 文档（私有，只有账号本人能看）；也可在 claude.ai/code/artifacts 或 Claude Code 终端 `/artifacts` 里找到。
按数据流组织，每段对照代码、公式和本文第 3 章的实际调试结果。

| 文档 | 链接 | 内容 |
|---|---|---|
| Lightning-LM 数据流详解（对照代码与公式） | https://claude.ai/artifact/9k6Yn1agyvZpCPVwmkFxT9 | 预处理 → LIO → 建图后端 → NDT 定位 → g2p5，对应 3.1 / 3.2 |
| Nav2 数据流详解（对照代码与公式） | https://claude.ai/artifact/2SE5jkNmKTza81qWfcc9Tw | 输入 / TF → costmap → SmacPlanner2D → RPP / MPPI → 速度链 → BT，对应 3.3 / 3.4 / 3.5 |
| G1 定位错题集（Lightning-LM） | https://claude.ai/artifact/Ukyh45QcVPA1YBS85u6QHV | 按表象自顶向下：4 个表象 → 17 个候选原因 → 定位输出的 3 个前提 + 防护层，含 09 月起全部定位问题，对应 3.1 / 3.2 / 4.1 / 4.2 / 4.13~4.15 |
| G1 导航错题集（Orin 30524） | https://claude.ai/artifact/X7SFAh86t9s544nT2gB1dE | 按表象自顶向下：4 个表象 → 17 个候选原因 → 4 个底盘假设不成立 + 避障余量账，对应 2.6 / 3.7 / 4.16 |
