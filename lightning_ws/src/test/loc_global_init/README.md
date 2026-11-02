# loc_global_init — 开机全局定位初始化

lightning 开机永远从地图原点做一次局部 NDT（`run_loc_online.cc` 的 `SetInitPose(SE3())`），机器人不在原点附近就静默初始化到错误位姿，
NDT 分数还可能高于门限（2026-10-27 Orin 30524：真值离原点 2.4 m / 78°，两次启动输出 (0.61, 0.04, 0°)、(1.18, −0.38, 11°)，分数 2.0~2.5）。

本包**不改 lightning**：独立节点全图搜索当前位姿，lightning 输出不对时发 `/initialpose`（lightning 已支持，10-24 Orin 实机验证）。

## 算法（`loc_global_init/bev_search.py`）
1. 雷达点按 IMU 重力摆平；地图按 `gravity_up.txt` 摆平；两边各自估地面，只留离地 0.3~2.0 m 的点。
2. **粗搜**：俯视 0.2 m 占据栅格，航向每 3°，扫描画成以雷达为中心的小栅格，与地图 FFT 互相关
   = 一次算出整张图所有平移的命中格数。只打分不移动候选（lightning 的粗 NDT 会把候选拉走，10-24 实测）。
3. 每个航向取局部极大，跨航向非极大抑制（0.75 m / 10°），留 8 个候选。
4. **精搜**：0.1 m 3D 体素（离地高度，膨胀 1 格）上对每个候选 ±0.3 m / ±4° 网格搜索，再细化一轮。
   必须 3D：俯视 2D 精分在 10-27 数据上错误位姿 0.43 / 真值 0.59，3D 是 0.17 / 0.60。
5. **判定**：最高精分 ≥ `min_score`（0.35），且第二名（离第一名 > 0.75 m 或航向差 > 10°）≤ `max_second_ratio`（0.85）× 第一名，
   否则拒绝（对称场景宁可不给，交给人工重定位）。

## 节点（`loc_global_init/node.py`）
- 启动后加载 `<map_dir>/global.pcd`，订阅 `/livox/lidar` + `/livox/imu`，等到 10 帧（~1 s）期间 |gyro| ≤ 0.15 rad/s，叠加后搜索，**然后立即取消订阅**。
  TF 监听也只在一次初始化期间开着。
- **点云用 `raw=True` 订阅、numpy 直接解析 CDR**（`geom.livox_raw_to_xyz`），不经 rclpy 反序列化：Orin 上 rclpy 每帧 ~2 万个点对象要 119 ms，
  10 Hz 就占满 1.2 核，执行器被点云回调占住，同组的 IMU 回调一次都轮不到（10-27 Orin 首次运行 30 s 只收到 10 帧点云、0 条 IMU）。
  raw 解析 0.4 ms / 帧，与 rclpy 结果逐点一致（Orin 实时 8 帧）；IMU 另放一个回调组。
- 搜索结果（雷达位姿）经 `body_link->mid360_link` 外参和摆平旋转换成 base_link 平面位姿，与 lightning 当前 `map->base_link` 比：
  - 差 ≤ 0.3 m / 5° → `ok_already`，不动 lightning；
  - 否则发 `/initialpose` → `published`，等输出进 0.3 m / 5° 并稳住 1 s → `relocalized`；超时 `verify_failed`，最多 2 次；
    核对期间 IMU 显示在动 → `verify_interrupted_moving`，等静止后重新采集（不计入失败次数，lightning 跟对了会判 `ok_already`）。
- `dry_run:=true`：只算、只报，不发 `/initialpose`（状态 `dry_run_would_publish`）。
- 状态：`/loc_global_init/status`（std_msgs/String JSON，transient local）；重新跑：`ros2 service call /loc_global_init/trigger std_srvs/srv/Trigger`。
- 启动：`ros2 launch loc_global_init loc_global_init.launch.py map_dir:=<地图目录> [dry_run:=true]`。

## 验证
- 单测（不需要 ROS）：`python3 -B -m pytest -q test`（合成房间：任意位姿无初值找回 <0.1 m / 2°；中心对称房间必须拒绝；别的房间的扫描必须拒绝）。
- 离线（读 db3，不需要 rosbag2 版本匹配）：
  `python3 -B -m loc_global_init.offline_bag <bag> <map_dir> --list-static`、`... --at <包内秒> ...`。
  - 0930b 包（地图 1790676615248）9 个静止时刻全部接受，精分 0.90~0.96、第二名 ≤0.64× 第一名，单次 ~3 s（x86）。
    lightning 正常的时刻（80~450 s）与其输出差 0.06~0.12 m / ≤1°；+10 s（已知初始化偏 0.42 m）差 0.33 m；
    +675 s 起（已知被拉偏 1.9 m / 14°）差 1.8~2.0 m / 6~14° —— 都和 CLAUDE.md 3.2.9 的激光真值结论一致。
  - 10-27 Orin 30524 现场数据（地图 1792839199393）：找回 (2.18, 0.87, 78°)，激光真值 (2.23, 0.87, 78°)；第二名 0.72×。
- **本地真 lightning 联调（10-27）**：开发机在 scratchpad 编 lightning（`ndt_1008/README.md` 的步骤），`g1_online.launch.py mode:=localization` +
  从 db3 按包内节奏回放 `/livox/lidar`、`/livox/imu`、`/odom`、`/tf_static` + 本节点 launch。lightning 每次都先从原点初始化：

  | 包内起点 | 机器人真实位置 | lightning 原点初始化 | 本节点 | lightning 最终 |
  |---|---|---|---|---|
  | +10 s | (0.07, −0.50, 5.5°) | (0.00, −0.49, 5.5°)，对 | `ok_already`（差 0.06 m），不发 | 不变，守护 GOOD |
  | +700 s | (−4.71, 0.24, 10.5°) | (−0.18, −0.19, 9.9°)，错 4.5 m | 发 `/initialpose` → `relocalized` 1.5 s | (−4.69, 0.24, 10.6°)，之后 70 s 稳在 ±5 mm，守护 GOOD |
  | +672 s | (−5.10, −0.08, −86°) | 错 5.3 m / 52° | 发 → `relocalized` 2.1 s（误差 0.03 m / 0.2°） | 之后机器人起步转身，lightning 正常跟踪 |
  | +675.5 s | 同上，发布后马上起步 | 错 5.3 m / 99° | 发 → `verify_interrupted_moving` → 静止后重采 → `ok_already`（0.02 m） | 跟踪正常，守护 GOOD |

  lightning 收到 `/initialpose` 后约 0.6 s 完成重定位；它自己的 ±15° 航向搜索（粗 NDT）在 +700 s 把 x 拉偏 0.14 m，随后快速收敛阶段纠正，不影响结果。
  联调脚本（回放、lightning 替身、TF 记录、编译）在当时会话的 scratchpad，未入库。

- **Orin 30524 实机（10-27 15:3x，地图 1792839199393，开机后机器人没动）**：lightning 原点初始化输出 (−0.65, 0.28, 15°)，守护 `NDT_SUSPECT` 冻结。
  - `dry_run`：采集 1 s，搜索 **13 s**（负载 ~20），找到 base (5.07, 0.79, 119°)，精分 0.59、第二名 0.72×，与当前输出差 5.7 m / 104°。
  - 正式：搜索 14 s，(5.09, 0.77, 119°)，发 `/initialpose` → 4.5 s `relocalized`，lightning 输出 (4.85, 0.82, 119.5°)，守护 GOOD、NDT 分 2.87。
  - 剩 0.24 m 差（沿走廊 x），静止时不会被跟踪纠正。起初以为是 lightning 的 ±15° 航向搜索（glog：雷达 x 5.09 → 4.77），
    15:59 关掉航向搜索（`init_yaw_search_deg 0`）重启再试，**照样拉到 4.75**：是 lightning 初始化那一次 6 自由度精 NDT 本身——
    姿态不受约束，横滚跑到 −7°，平移沿走廊挪 0.31 m，之后重力约束只改姿态、不改平移。
    独立核对（离地 0.3~2 m 点到地图最近点距离，KD 树，沿 x 剖面、每个 x 对 y / 航向取优）：单峰在雷达 x 4.95~5.00（均值 0.105 m、10 cm 内 0.70），
    本节点结果 0.106 / 0.705，lightning 当前 0.123 / 0.585 → 本节点的位姿更准。但 lightning 之后的跟踪 NDT（有重力约束）在 4.83 处残差也只有 0.006 m，
    两种方法沿走廊差 0.2 m，走廊方向本来约束弱；也可能和 4.15（怀疑地图沿走廊被压短 6~8%）有关，未查清。
  - **注意：15:3x / 15:5x 两次机器人是吊着的**（用户事后说明；本节点估的雷达离地 1.385 m，落地后 1.234 m，高 15 cm），初始化 NDT 横滚跑到 −7° 可能与此有关。
    16:06 放下、原地换朝向后 `trigger` 重跑：找到 (4.96, 0.73, −16.5°)，第二名 0.45×（比吊着时清楚得多），lightning 跟着转身后是 (4.82, 0.78, −16.9°)，
    差 0.15 m / 0.35° < 0.3 m → `ok_already`，不发。搜索 15 s。
  - **发布位姿补偿躯干倾斜**（16:09 发现）：lightning `BasePoseToLidar` 只用静态外参换算 /initialpose，不管躯干倾斜；机身前倾 ~4° 时雷达在 base 前 ~0.09 m，
    直接发真实 base 位姿会把雷达放到后面 0.09 m。现在发 `geom.base_pose_for_lightning`（让 lightning 换出的雷达位置正好是搜索结果），核对仍按真实 base 位姿；状态里多 `published_base`。
  - 16:11 那次重启时机器人正被搬动（LIO `imu init done, bg` 1.3 rad/s、重力歪 30°+，`tilt error ... skip`），结果作废。**重启前机器人必须静止**（同 4.14）。
  - **16:14 落地静止重启（有效）**：原点初始化错 6.56 m → 搜索 11 s 找到 (6.31, −1.41, 1.0°)（第二名 0.62×）→ lightning 换出的雷达 6.405 正好落在搜索结果上 →
    1.3 s `relocalized`，lightning 稳在 (6.16, −1.35, 0.5°)，守护 GOOD、NDT 分 2.50。剩 0.15 m / 0.5°：lightning 初始化精 NDT 把雷达 6.405 → 6.253，跟踪 NDT 在那里残差 0.004 m。
    独立核对沿 x 剖面谷底在 6.40~6.45（本节点 0.124 m / 10 cm 内 0.63，lightning 0.147 / 0.49）。两个地点（x≈5、x≈6.3，航向差 120°）lightning 都比剖面谷底偏 −x 0.15~0.2 m，
    方向在地图系一致 → 更像 lightning NDT 与"点到最近地图点"两种度量沿走廊的分歧（或地图沿走廊比例问题，4.15），改初始化消不掉（跟踪 NDT 会再拉回去）；要判谁对得用卷尺量真值。
  - **16:23 原点附近静止重启**：lightning 原点初始化自己就对，本节点 (−0.36, 0.13, −7.5°)（第二名 0.60×）vs lightning (−0.27, 0.12, −7.3°)，差 0.09 m → `ok_already`。
    剖面：y 单峰尖（0.14），x 谷底宽（−0.27~−0.22 几乎一样），吻合 0.555（x≈5~6.3 处 0.63~0.70）→ 原点附近沿 x 约束弱、吻合更低，并不比别处准；
    lightning 与谷底差 ~0.05~0.09 m，三处都是 lightning 的 x 更靠近原点，但不成比例（0.09/0.27、0.2/5、0.15/6.3），不像单纯的地图比例问题。

## 未做 / 待定
- 只在启动时跑一次（+ trigger 服务）；没接 `/lightning/loc_status` 的 `need_reloc` 自动触发，没进前端；没接进 `robot_bringup` 的 launch。
- lightning 先从原点错初始化、几秒后被纠正（联调里从启动到纠正约 13 s：采集 ~8 s 含等数据、搜索 ~4 s），这段时间不能导航（Nav2 不知道）。
- `/initialpose` 进 lightning 后会沿走廊偏：本地 0.14 m（随后跟踪纠正），**Orin 实机 0.24 m 且静止时不纠正**；原因是初始化精 NDT 不约束姿态（见上），
  关掉航向搜索不解决，真要修得改 lightning 初始化（NDT 时锁住横滚 / 俯仰），属于方案 B。航向搜索 10-27 用户确认去掉（理由仍成立：10-24 粗匹配把准初值带到 1 m 外）：新增 `robot.launch.py loc_params_overlay:=`（合并到 lightning yaml 之上），Orin 本机 `tools/deploy/orin_docker/lightning.local.yaml`
  设 `init_yaw_search_deg: 0`；仓库默认仍 15（Thor 上没有本节点，前端拖位姿要航向搜索）。代价：Orin 上前端拖位姿的航向要拖得比较准。
- Orin 搜索 13~14 s（x86 3.5 s），加采集，开机后从节点启动到纠正约 20 s。
- 节点是手动起的（`ros2 launch`，日志 `/opt/G1/logs/loc_global_init*.log`），没进开机自启；Orin 重启后要再手动起或接进 `robot_bringup`。
