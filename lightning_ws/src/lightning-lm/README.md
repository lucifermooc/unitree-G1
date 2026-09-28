# Lightning-LM 在 G1 上的工作原理（定位 / 建图）

> 2026-09-23 按源码通读整理。代码位置用「文件 + 函数名」标注（行号会随修改漂移）。
> 上游项目说明、编译和数据集测试见 [README_CN.md](README_CN.md)。

- [一、定位](#一定位)
- [二、建图](#二建图)
- [三、建图产物如何被定位使用](#三建图产物如何被定位使用)
- [四、已知问题（读代码发现，未修）](#四已知问题读代码发现未修)
- [五、参数速查](#五参数速查)

---

## 一、定位

**一句话**：LIO 负责算相对运动，NDT 负责把位置锚到先验地图上，PGO 把两者融合，最后用 IMU 把位姿外推到 200 Hz 发布成 TF。

### 1.0 入口

- `robot.launch.py` → `g1_localization.launch.py` → [launch/g1_online.launch.py](launch/g1_online.launch.py)（`mode=localization`）。
  launch 把 yaml 改写成临时文件（写入话题名、`map_path`、`pub_registered_scan` 等），再用 `ExecuteProcess` 启动
  `run_loc_online --config=...`。可执行文件用 gflags 解析参数，**不能传 `--ros-args`**，诊断开关只能经 yaml 下发。
- [src/app/run_loc_online.cc](src/app/run_loc_online.cc) `main`：`LocSystem::Init` → `SetInitPose(SE3())`（**以单位位姿为初值**）→ `Spin()`。

### 1.1 线程与数据流总览

```
/livox/imu 200Hz ──[ROS 回调线程]── LIO.ProcessIMU：kf_imu_ 做 IMU 预测 = "DR"
                                      ├─> LidarLoc.dr_queue（插值用）
                                      └─> PGO.ProcessDR ─> PubResult ─> TF 200Hz
/livox/lidar 10Hz ─[ROS 回调线程]── 预处理 ─> [LIO 队列 max=1]
                                                 │ LIO 线程（"激光里程计"）
          SyncPackages → IMU 去畸变 → ESKF 对"LIO 自己的局部 ivox 地图"配准 → LO 位姿
            ├─> LidarLoc.lo_queue、PGO.lidar_odom_queue
            ├─> /lightning/registered_scan（雷达系，stamp = lidar_end_time）
            └─ 有新关键帧（或尚未收敛）时：当前帧 + ≤5 个关键帧拼成一片 ─> [定位队列 max=1]
                                                 │ 激光定位线程（"激光定位"）
          NDT 与先验分块地图配准 → 只取残差的 10% → 重力约束 → LocalizationResult
                                                 ▼
          PGO 滑窗（5 帧）：定位先验边 + LO 相对边 → LM 优化 → result_（时刻 = 该帧点云时间）
```

另有 `LidarLoc::UpdateMapThread`：每 10 ms 检查地图分块是否变化，变化就重建 NDT 目标。
两个异步队列都是 `SetMaxSize(1)`（`Localization::Init`），处理不过来时只保留最新一帧。

相关文件：
[src/core/system/loc_system.cc](src/core/system/loc_system.cc)、
[src/core/system/ros_io.cc](src/core/system/ros_io.cc)、
[src/core/localization/localization.cpp](src/core/localization/localization.cpp)、
[src/core/localization/lidar_loc/lidar_loc.cc](src/core/localization/lidar_loc/lidar_loc.cc)、
[src/core/localization/pose_graph/pgo.cc](src/core/localization/pose_graph/pgo.cc)、
[src/core/localization/pose_graph/pgo_impl.cc](src/core/localization/pose_graph/pgo_impl.cc)、
[src/core/localization/pose_graph/smoother.h](src/core/localization/pose_graph/smoother.h)。

### 1.2 IMU 链路（200 Hz，决定输出频率）

`Localization::ProcessIMUMsg` → `LaserMapping::ProcessIMU`：用 `kf_imu_` 做纯 IMU 预测。
每处理完一帧雷达，`LaserMapping::Run` 末尾把 `kf_imu_` 重置为 LIO 最新状态，再把缓冲里的 IMU 重放一遍。
这个预测状态就是代码里的 **DR**，同时送给 LidarLoc（插值用）和 PGO（外推输出用）。
IMU 订阅队列深度 1000（`SubscribeSensors`），原因见工作区 CLAUDE.md 的 IMU 断档条目。

### 1.3 LIO 链路（10 Hz）

- **预处理**（`PointCloudPreprocess` Livox 分支）：每 `point_filter_num=2` 个点取 1 个，保留雷达系 z∈[`roi.height_min`, `roi.height_max`] = [-2, 10] 的点。
- **`LaserMapping::Run`**：同步 IMU → 去畸变到帧尾 → 迭代 ESKF 对 ivox 局部地图做点到面配准。
  **这张局部地图是 LIO 在定位过程中自己增量建的，不是先验地图。**
  LO 世界系 = 开机位置（已对齐重力），与先验地图无关；**LidarLoc 只用 LO 的相对运动**（两次定位之间的 ΔLO）。
- **定位模式的关键帧条件**：移动 1 m、转 10°，或**静止满 2 s**（`is_in_slam_mode_=false` 分支）。
- **`Localization::LidarOdomProcCloud`**：
  - 先把缓冲里积压的帧补跑完（每帧都 `ProcessLO` + 发布 `registered_scan`），解决 IMU 晚到导致的永久滞后。
  - 已收敛：只有出现新关键帧才送 NDT（`loc_on_kf: true`）。
  - 快速收敛阶段：每帧都送。
  - 送给 NDT 的点云是 `LaserMapping::GetProjCloud()`：当前帧 + 最近几个关键帧（每个最多 1000 点，变换到当前帧坐标）。
    注意它会**原地追加**到 `scan_undistort_`，所以 `PublishRegisteredScan` 必须在它之前调用。

### 1.4 激光定位 `LidarLoc::Align`

1. 时刻取点云 stamp + `lidar_time_interval`（≈帧尾）。在 LO 队列上插值得到 `current_lo_pose_`，同时从 LIO 状态取重力方向。
2. **预测**：`guess = last_abs_pose × (last_lo⁻¹ × current_lo)`。
3. `map_->LoadOnPose(guess)`：地图按 100 m 分块，加载曼哈顿距离 ≤2 的块；块变化时由 UpdateMapThread 换 NDT 目标。
4. **NDT**（`LidarLoc::Localize`）：体素 1 m，最多 20 次迭代，4 线程，分值 = `getTransformationProbability()`。
5. **融合**：`esti = guess × exp(balance × log(guess⁻¹ × ndt))`，即**每次只修正 NDT 残差的 `balance_factor`（0.1）**。
   - 快速收敛阶段：分值 ≥ `fast_converge_min_score`(1.5) 时系数用 `fast_converge_factor`(0.5)；
     残差连续 3 次 <0.5° / 5 cm 即收敛；最多 50 帧后强制退出。
   - 收敛后分值 < `track_min_score`（默认 0 = 关闭）时系数置 0，只沿 LO 递推。
6. **重力约束** `ApplyGravityConstraint`：用最小旋转把 LIO 的"上方向"转到 `<map>/gravity_up.txt` 的方向，航向和平移不动。
   没有该文件则自动关闭。`force_2d=false`，所以结果仍是 6DoF。
7. 每次定位把当前位姿写到 `./data/recover_pose.txt`（相对进程工作目录）。

### 1.5 PGO（每次定位触发一次）

- `PGO::ProcessLidarLoc` 新建一帧 → `PGOImpl::AddPGOFrame` → `RunOptimization`。
- 图里的边：
  - 定位先验边：σ 0.3 m / 1°，Huber δ=30（**写死在 `pgo_impl.h`，yaml 的 `pgo:` 段不生效**）。
  - LO 相对边：连到最近 4 帧，σ 0.3 m / 1°，Cauchy。
  - 边缘化先验：挂在窗口首帧，实现是简化版（直接取它当前的优化位姿）。
  - DR 边的函数 `AddDRFactors` 写了但**没被调用**。
- 窗口 5 帧，LM 迭代 5 次，最新一帧的优化位姿写入 `result_`。
- `PGOImpl::PGOFrameToResult`：激光定位端到端延迟 >3 s 的结果丢弃。

### 1.6 高频输出与 TF（每条 IMU 触发一次）

1. `PGO::PubResult` → `ExtrapolateLocResult`：`result_ × DR(result 时刻)⁻¹ × DR(最新)`，把 PGO 结果外推到最新 IMU 时刻。
2. **`PoseSmoother`**：先用 DR 的运动做预测，再往输入方向只拉 **1%**（每条 IMU 一次，τ≈0.5 s）。
   输出与输入差 >2 m 时系数改 0.2；>5 m 直接跳过去。
3. 回调链：`LocSystem::PublishBaseTF` → `BaseTFPublisher::Publish`（[ros_io.cc](src/core/system/ros_io.cc)）：
   1. `T_map_body = T_map_lidar × T_lidar_body`，`T_lidar_body` 从 URDF 的 `/tf_static` 读一次（未就绪时不发 TF）。
   2. 取 body 的 x 轴在水平面的投影作为航向，发布 `map->base_link`：只有 x、y、yaw，**z 强制为 0**。
   3. 剩下的倾斜发布成 `base_link->body_link`（平移为 0）。
   4. 额外发一个静态的 `base_link->base_footprint`（单位变换）。
   - stamp = 最新 IMU 时间（传感器时钟）。
4. `robot_bringup/robot_pose_pub` 每 100 ms 查一次 `map->base_link`，发布 `/base_link_pose`（TF 时间戳停止前进则停发）。

**输出一共被平滑了三层**：NDT 残差只取 10%（且只在关键帧上做）→ PGO 与 LO 融合 → 输出端 1% 平滑器。
推论：持续运动时输出相对 NDT 解有系统性滞后（τ ≈ 10 个定位周期）。glog 里 `loc using lo guess` 与 `confidence: ..., t:`
之差是**原始残差**，实际注入要乘 0.1。

### 1.7 初始化

- `LidarLoc::Init`：读 `index.txt`，先载入第一个功能点附近的块。第一个功能点是 `start` = 建图时第一个关键帧的位姿（≈原点）。
- 第一帧定位走 `InitWithFP(单位位姿)`：只做一次细分辨率 NDT，**没有航向搜索**（`YawSearch` 已注释掉）。
- 结论：**定位必须在建图起点附近、朝向与建图起步时大致相同的地方开机。**前端"设置初始位姿"目前不起作用（见已知问题 3）。
- 初始化后进入快速收敛阶段（1.4 第 5 步）。

---

## 二、建图

**一句话**：建图的主体是 LIO（迭代 ESKF + ivox 局部地图）。每个关键帧存下来后交给回环线程和栅格线程；
回环修正关键帧位姿，保存时把所有关键帧点云按修正后的位姿拼起来，再切成 100 m 分块。

### 2.0 入口与线程

`mode=mapping` → [src/app/run_slam_online.cc](src/app/run_slam_online.cc)：`SlamSystem::Init` → `StartSLAM("new_map")` → `Spin()`。

```
/livox/imu ─┐   [ROS 回调线程，单线程执行]
/livox/lidar┴─> LIO.ProcessPointCloud2 + Run()（同步执行，每来一帧点云跑一次）
                  └─ AfterLidarProcessed：registered_scan、TF（10 Hz）
                       └─ 出现新关键帧 ─┬─> [回环线程] 检测 → NDT → 位姿图优化 → 写回 KF.opt_pose
                                      │                                   └─ 回调：触发重绘
                                      └─> [g2p5 前端线程] 增量绘制栅格 → /map
                                          [g2p5 后端线程] 回环后整图重绘 → 替换前端地图 → /map
```

**与定位不同：建图时 LIO 不在单独线程，直接在 ROS 回调线程里同步运行**（`SlamSystem::ProcessLidar`）。

相关文件：
[src/core/system/slam.cc](src/core/system/slam.cc)、
[src/core/lio/laser_mapping.cc](src/core/lio/laser_mapping.cc)、
[src/core/lio/imu_processing.hpp](src/core/lio/imu_processing.hpp)、
[src/core/lio/eskf.cc](src/core/lio/eskf.cc)、
[src/core/loop_closing/loop_closing.cc](src/core/loop_closing/loop_closing.cc)、
[src/core/g2p5/g2p5.cc](src/core/g2p5/g2p5.cc)、
[src/core/maps/tiled_map.cc](src/core/maps/tiled_map.cc)。

### 2.1 LIO（地图的来源）

- **IMU 初始化**（`ImuProcess::IMUInit` / `Process`）：
  - 累计 20 多条 IMU（约 0.1 s），这段时间机器人必须静止。
  - `gravity_align_init: true`：用平均加速度确定"上"方向，求最小旋转把它转到 +z（不引入航向），重力固定为 -z。
  - 自动判断加速度计单位是 g 还是 m/s²（MID360 输出 g）。
  - **结果：地图坐标系 = 建图起点处 IMU 的位置，z 对齐重力，x 是起步时的朝向。**
- **每一帧**（`LaserMapping::Run`）：
  1. `SyncPackages`：取最早一帧点云，等 IMU 覆盖到帧尾；否则打 `sync failed` 并留待下次。
  2. `ImuProcess::UndistortPcl`：向前积分 IMU，再把每个点反向补偿到帧尾时刻。
     两条 IMU 间隔 >0.1 s 打 `get abnormal dt`（= IMU 断档，快转时会让航向跳变、地图多层墙）。
  3. 按 `filter_size_scan`(0.1 m) 体素降采样。
  4. `ESKF::Update` 迭代最多 `max_iteration`(4) 次，每次调用 `LaserMapping::ObsModel`：
     每点在 ivox 找 5 近邻拟合平面，残差 = 点到平面距离；再对 HᵀH 做特征分解，只在可观测方向上更新（退化处理）。
- **关键帧**：移动 `kf_dis_th`(1 m) 或转过 `kf_angle_th`(10°) 就建一个（建图模式没有"静止 2 s"规则）。
  - `LaserMapping::MakeKF` 保存：去畸变后的整帧点云（雷达系）、LIO 位姿、opt 位姿
    （初值 = 上一关键帧 opt 位姿 × LIO 相对运动）。
  - **ivox 局部地图只在建关键帧时加点**（`MapIncremental`），不是每帧都加。

### 2.2 回环（`LoopClosing`，每个关键帧处理一次）

- **找候选**（`DetectLoopCandidates`）：
  - 距上次找到候选超过 `loop_kf_gap`(20) 个关键帧才重新找。
  - 只看 ID 相差 ≥ `closest_id_th`(50)、水平距离 < `max_range`(20 m) 的历史关键帧。
  - 同一段轨迹里每 `min_id_interval`(20) 个 ID 最多取一个。
- **配准**（`ComputeForCandidate`）：
  - 目标：候选关键帧前后 ±40 个关键帧（每 4 个取 1）拼成的世界系子图。
  - 源：当前关键帧前后 ±`src_submap_range`(20) 个关键帧拼成的子图。
  - NDT 分辨率依次 10 / 5 / 2 / 1 m，分值 > `ndt_score_th`(1.3) 才接受。
  - `loop_4dof: true`：只保留 x、y、航向修正（roll/pitch/高度沿用 LIO）。
- **位姿图**（`PoseOptimization`）：
  - 所有关键帧都是顶点；每个关键帧与前 2 个关键帧之间加 LIO 相对运动边（σ 0.1 m / 3°）；
    回环边 σ 0.2 m / 3°，Cauchy 核；`with_height: false` 时不加高度先验。
  - 有新回环时才跑 LM（20 次迭代）；chi2 超阈值的回环边标成外点。
  - 优化后写回**所有**关键帧的 opt 位姿，并触发 g2p5 重绘。

### 2.3 g2p5 2D 栅格

- 前端线程：每个关键帧增量绘制。后端线程：回环后从头重绘，画完替换前端地图。经 `/map` 发布（transient_local）。
- **高度在雷达系里计算**（`G2P5::Convert3DTo2DScan`）：
  - 地面方程：`esti_floor: true` 时用 RANSAC 拟合（输入 = 雷达系 z < `lidar_height + floor_height` 的点，要求法向 z 分量 ≥0.99），
    失败就退回 `floor_height`(-1.2) 的水平面。
  - 离地高度在 [`min_th_floor`, `max_th_floor`] = [0.30, 1.0] m 的点记为 hit（障碍）。
  - 每 1° 一条射线，从雷达位置画 miss。2.5D 规则：格子记录的高度低于射线在该处的高度时才刷成空闲。
- 输出（`G2P5Map::ToROS`）：访问次数 >3 且 hit 比例超过阈值 → 占据（100）。分辨率 `grid_map_resolution`(0.05 m)。
- launch 参数 `floor_height` / `min_obstacle_height` / `max_obstacle_height` 可覆盖上述三个值。

### 2.4 建图时的 TF

`SlamSystem::PublishBaseTF`：`T_map_lidar = KF_opt × KF_lio⁻¹ × LIO_now`（用最新关键帧的回环修正量修正 LIO 当前位姿），
然后走同一个 `BaseTFPublisher`，发平面的 `map->base_link` 和 `base_link->body_link`。
**频率只有 10 Hz，stamp = lidar_end_time**（定位模式是 200 Hz）。

### 2.5 保存（服务 `/lightning/save_map`，参数 `map_id` → `map_save_root/map_id`）

`SlamSystem::SaveMap`：
1. 目标目录已存在则**先删除再重建**。
2. 所有关键帧点云 × 各自 opt 位姿 → 全局地图，0.1 m 体素降采样（`LaserMapping::GetGlobalMap`）。
3. `TiledMap::ConvertFromFullPCD` → `SaveToBin`：切成 100 m 分块，每块再按 0.1 m 降采样后存盘。
4. 写 `global.pcd`、`gravity_up.txt`、`map.pgm` / `map.yaml`。

---

## 三、建图产物如何被定位使用

| 产物 | 内容 | 定位怎么用 |
|---|---|---|
| `index.txt` + `<id>.pcd` | 第一行是原点；每块一行：id、格坐标、路径；之后是 `# functional points` 段，含 `start` = 第一个关键帧 opt 位姿（≈原点） | `TiledMap::LoadMapIndex` 读入；开机先载入 `start` 附近的块；运行中按位姿加载曼哈顿距离 ≤2 的块，作为 NDT 目标 |
| `gravity_up.txt` | `0 0 1`（`gravity_align_init` 开着才写） | 有该文件 `gravity_constrain` 才生效 |
| `map.pgm` / `map.yaml` | g2p5 最新栅格 | 给 Nav2 map_server，**不参与定位** |
| `global.pcd` | 全局点云 | 定位不用 |
| `<id>_dyn.pcd` | 定位退出时保存的动态图层（只写非空块） | `update_dynamic_cloud: false`，动态图层不会被填充，通常不会产生该文件 |

- 地图的原点和朝向 = 建图起步时的位姿；定位用单位位姿作初值且不做航向搜索 → **定位要在建图起点、同一朝向开机**。
- 高度：地图 z=0 在建图起点的雷达高度，地面在 `g2p5.floor_height`（每张图不同）。平面模式下 `map->base_link` 的 z 恒为 0，不用这个偏移。
- `robot.launch.py` 默认由 robot_status_manager 按数据库（`~/maps/db.sqlite`）当前地图决定 `map_path`。

---

## 四、已知问题（读代码发现，未修）

定位：

1. **yaml 的 `pgo:` 段没有任何代码读取**。PGO 用 `pgo_impl.h` 里 `PGOImpl::Options` 的默认值（例如 outlier 阈值实际 30，yaml 写的是 5）。
2. **`LidarLoc::Localize` 两个分支都返回 true**：
   - `min_init_confidence: 1.8` 拦不住初始化，初值附近 NDT 收敛到哪就用哪。
   - `match_fail_count_` 永不增加，状态恒为 GOOD。
   - 功能点和 `recover_pose.txt` 两种初始化后备方案实际用不到。
3. **lightning 不订阅 `/initialpose`**：robot_status_manager 把前端给的位姿转发到 `/initialpose`，但没人接收，前端"设置初始位姿"不起作用。
4. DR 的 `is_parking_` 从未被置 true（检测代码已注释），**`enable_parking_static: true` 实际不起作用**。
5. `LaserMapping::MakeKF` 里 `20 / 180 * M_PI` 是整数除法（恒为 0），拼接用的关键帧只有离上一个超过 3 m 时才会更换。
6. `PointCloudPreprocess` Livox 分支的有效点判断 `||` / `&&` 优先级写错，`blind` 距离过滤基本不生效
   （blind=0.1 本来也挡不住打在 G1 头上的点）。

建图：

7. **建图模式没有补跑循环**：每来一帧点云只跑一次 `Run()`，同步失败一次就永久多滞后一帧。
   定位路径已修（`Localization::LidarOdomProcCloud` 的 while 循环），建图没修。只影响建图时 TF / registered_scan 延迟，不影响地图几何。
8. ivox 局部地图只在关键帧时更新（1 m / 10°），快速转向进入新区域时配准用的局部地图会落后。属原设计，查建图重影时值得记住。
9. **待确认**：g2p5 地面 RANSAC 的法向正负号不固定，拟合成朝下法向时会被 `values[2] < 0.99` 判为"不水平"而退回默认水平面，
   躯干倾斜在远处造成高度误差。需查建图 glog 中 `floor is not horizontal` 的出现频率确认影响。

---

## 五、参数速查

均在 [config/default_livox.yaml](config/default_livox.yaml)（G1 使用）。"生效"一栏是读代码的结论。

| 参数 | 当前值 | 作用 | 生效 |
|---|---|---|---|
| `roi.height_min` / `height_max` | -2.0 / 10.0 | 雷达系 z 裁剪（必须为负，否则地面全被裁掉） | ✅ |
| `fasterlio.point_filter_num` | 2 | 点云抽稀 | ✅ |
| `fasterlio.filter_size_scan` / `filter_size_map` | 0.1 / 0.2 | 单帧降采样 / ivox 加点降采样 | ✅ |
| `fasterlio.kf_dis_th` / `kf_angle_th` | 1.0 m / 10° | 关键帧间隔（定位另加静止 2 s） | ✅ |
| `fasterlio.gravity_align_init` | true | LIO 世界系对齐重力，建图写 `gravity_up.txt` | ✅ |
| `fasterlio.blind` | 0.1 | 近距离盲区 | ❌ 见问题 6 |
| `lidar_loc.loc_on_kf` | true | 收敛后只在关键帧做 NDT | ✅ |
| `lidar_loc.balance_factor` | 0.1 | NDT 残差注入比例 | ✅ |
| `lidar_loc.fast_converge_*` | 0.5 / 1.5 / 0.5° / 0.05 m / 3 / 50 | 初始化后快速收敛 | ✅ |
| `lidar_loc.track_min_score` | 未设（0） | 低分不融合 | ✅（默认关） |
| `lidar_loc.gravity_constrain` | true | 用 LIO 重力锁 roll/pitch | ✅（需 `gravity_up.txt`） |
| `lidar_loc.min_init_confidence` | 1.8 | 初始化分值门限 | ❌ 见问题 2 |
| `lidar_loc.enable_parking_static` | true | 静止时固定输出 | ❌ 见问题 4 |
| `lidar_loc.update_dynamic_cloud` | false | 动态图层更新 | ✅（关） |
| `pgo.*` | — | PGO 噪声/阈值 | ❌ 见问题 1 |
| `loop_closing.loop_kf_gap` / `closest_id_th` / `max_range` | 20 / 50 / 20 m | 回环候选 | ✅ |
| `loop_closing.ndt_score_th` / `src_submap_range` / `loop_4dof` | 1.3 / 20 / true | 回环配准 | ✅ |
| `loop_closing.with_height` | false | 高度先验 | ✅（关） |
| `g2p5.floor_height` | -1.2 | 默认地面高度（雷达系） | ✅ |
| `g2p5.min_th_floor` / `max_th_floor` | 0.30 / 1.0 | 障碍离地高度区间 | ✅ |
| `g2p5.esti_floor` | true | RANSAC 估计地面 | ✅（见问题 9） |
| `g2p5.grid_map_resolution` | 0.05 | 栅格分辨率 | ✅ |
| `system.with_ui` | true（launch 默认改为 false） | Pangolin 3D 窗口，吃 1 核 | ✅ |
| `system.pub_registered_scan` | launch 参数下发 | 发布 `/lightning/registered_scan` | ✅ |
