# AGV 3D Nav → G1 迁移分析

日期：2026-09-14。范围：本工作空间源码、配置与启动文件静态对比；未修改运行代码，未编译新栈、未连接机器人或下发运动命令。已尝试 DeepSeek MCP 复核，但服务器返回 model_not_found：账号组不支持 deepseek-v4-pro，未获得复核意见。

## 结论

可将 FAST-LIO-SAM + map_create_2d + aid_localization 作为 Lightning 的替代路线，并将 AGV 的导航配置和控制器移植到 G1。这里是可实施的方案判断，并非已经验证可用。

3d_nav_slam_nav_only 自身仍使用 Nav2；替换当前导航工程并不等于移除 Nav2 框架。如果目标是完全不用 Nav2，需要另选或实现规划、控制、代价地图和任务执行，超出这份工程的直接迁移范围。建议复用这份已实现的导航栈，集中处理 G1 适配。

## 保留、替换与收口

| 当前部分 | 建议 |
| --- | --- |
| lightning-lm 建图、定位 | 由 FAST-LIO-SAM、map_create_2d、aid_localization 替换 |
| 当前 aid_navigation2 | 以 AGV 版本为基础另建 G1 参数与入口，保留其 MPPI + PurePursuitShim 路线 |
| g1_nav_bridge 速度控制 | 保留，与新导航最终速度输出连接；保留默认禁用、限速与超时停车 |
| g1_nav_bridge 里程计 | 保留原始状态适配能力，重新明确 odom 与 TF 发布职责 |
| Livox 驱动与 SDK | 保留，为新栈建立原生 PointCloud2 启动入口 |
| Lightning 中的 g1.urdf | 迁到独立 G1 描述/启动包，核验外参，解除算法包依赖 |
| robot_bringup/aid_robot_py 业务 | 首次跑通用独立启动入口；之后改地图保存、模式切换、地图管理接口 |
| RGBD、回充等 | 首版按真实使用需要接入，不照搬 AGV 设备配置 |

## 源码已确认的迁移阻塞项

### 1. MID360 消息类型与时间字段

- 当前 `src/livox_ros_driver2/launch_ROS2/msg_MID360_launch.py` 设置 `xfer_format=1`，输出 CustomMsg。
- 新 `fast_lio_sam/FAST_LIO_SAM/src/laserMapping.cpp` 实际只创建 PointCloud2 订阅，CustomMsg 分支已注释。仅改话题名无效。
- 新 `preprocess.cpp` 的 LIVOX 分支需要包括 line、tag、timestamp 的点结构；其时间计算为 `(timestamp - header时间纳秒) / 1e6`，得到毫秒。
- 本地驱动 `lddc.cpp` 和 `comm/pub_handler.cpp` 显示原生 PointCloud2 保存绝对纳秒点时间，可以作为适配起点。仍须用实际消息核对字段、扫描时长、时钟和 IMU 覆盖。

首选新增 `xfer_format=0` 的 G1 驱动入口，避免继续依赖 Lightning 的点云转换程序。不能用只含 XYZ/强度的显示点云替代有逐点时间的 SLAM 输入。

### 2. 默认 MID360 launch 并非完整 G1 建图入口

`mapping_mid360.launch.py` 默认仿真时间为 true，将 `/Odometry` 重映射到 `/odom`，且没有启动二维建图。与此同时，wheel odom 默认订阅 `/odom`，会产生输入输出混用风险。`localization.launch.py` 又固定加载 rs128.yaml。

需要独立 G1 建图/定位入口：明确实时时钟、MID360 参数、唯一的里程计话题所有者，启动 map_create_2d，取消固定 taskset 核心绑定或改成可选。

### 3. TF 与轮速融合开关耦合，是首要架构改动

当前 `g1_nav_bridge/sport_to_odom.py` 明确不广播 TF。新定位必须查询 `odom -> base_footprint`，因此当前桥不能直接满足要求。

更深一层的问题在新 `laserMapping.cpp::publish_odometry`：

- 定位前端模式直接返回，不广播 TF。
- 建图开启 wheel_odom 时，反算并发布 map→odom。
- 建图关闭 wheel_odom 时，直接发布 map→base_link。

因此“关闭轮速融合，然后增加 odom→base TF”还不够，会让 base_link 出现冲突父节点。应将 **是否融合外部里程计** 与 **TF 发布策略** 分开控制。

建议首版目标树：`map -> odom -> base_footprint -> base_link -> mid360_link`。map→odom 在建图由建图节点负责，在定位由 aid_localization 负责，两模式互斥；odom→base_footprint 始终只有一个发布者。

现有 URDF 是 base_link→base_footprint，二者零变换。迁移时统一方向和语义，并修正算法中混用 base_link/base_footprint 的查询，不能叠加一条反向静态 TF。

### 4. G1 状态里程计不能未经验证按轮速使用

AGV 默认启用 wheel_odom.enabled、tight_coupling、anchor_pose_in_front_end；定位前端会用外部里程计绝对位姿锚定状态。G1 当前桥是平面 XY/yaw 近似，速度按机体系解释，仍须核验机器人真实话题语义、时间戳和姿态变化。

建议先关闭紧耦合与绝对位姿锚定，拆开 TF 开关，验证纯 LiDAR+IMU 建图。导航 odom 可选择：

1. 若 G1 状态里程计经验证连续且时间正确，由适配器发布平面 odom 与对应 TF；不必立刻把它作为 FAST-LIO 约束。
2. 若状态里程计不足以支撑导航，改由连续 LIO 位姿提供 odom，处理 IMU 到导航基座外参、原点初始化和速度坐标系。不能把建图模式下 map 系的 `/Odometry` 直接改名为 odom，也不能把回环跳变送进连续 odom。

两条路线都必须确保定位使用的 odom、TF 与前端坐标定义一致；该选择应由录包评估确定。

### 5. 倒装、步态和三维姿态

G1 驱动 JSON 包含约 181.19° roll，URDF 注明倒装已由驱动处理；URDF 雷达高度约 1.23618 m。新 mid360.yaml 的 IMU–LiDAR 外参不能未经核验直接套用。需核对驱动究竟对点云、IMU 各做了何种旋转，以及是否重复补偿。

AGV 定位默认 constrain_to_planar=true。G1 胸部姿态与高度随步态变化，固定地面到雷达变换只是近似。首版限定平地低速导航，实测静止、转身、行走时地图与点云对齐情况，再决定是否引入动态姿态补偿。不能将本次二维导航移植视为具备上下楼梯能力。

### 6. 同名包不能直接覆盖或混编

两棵源码都包含 aid_robot_msgs、aid_navigation2、aid_costmap_plugin。根目录递归发现包时存在同名冲突风险；使用显式构建路径/隔离源码及单独 build/install/log，不能混用旧产物。

对比确认：两份 SaveMap.srv 字段顺序不同；旧消息包独有 MapMark，新版增加若干消息服务；多个既有消息也不同。需统一接口并重编译消费者。新 costmap 的 StaticLayer 源码和插件注册在旧版中不存在，不能用旧库配新参数。

### 7. 导航障碍物与速度配置

AGV 当前启用 STVL，既订阅过滤点云作障碍标记，也订阅原始点云清除障碍。两个通道都需要改为 MID360 输入。默认 pointcloud_filter.launch.py 仅启动双 RGBD，雷达过滤启动需要从 reference 中提取，并正确配置生命周期。

AGV 半径 0.225 m、前进上限 0.6 m/s、角速度 1 rad/s 不应照搬；G1 当前桥限制分别为 0.15 m/s 和 0.25 rad/s，首版控制器、平滑器与桥应一致，vy 先保持 0。机器人 footprint 应覆盖实际行走包络，再调整膨胀距离和障碍高度。

Keepout 默认启用；首版若没有禁行区数据端，应同步关闭插件及相关启动/生命周期项。最终速度链明确为控制器→平滑器→碰撞监控→G1 桥，避免旁路或重复发布。

### 8. Lightning 依赖不仅在 launch

`robot_bringup/CMakeLists.txt`、package.xml、robot_status_manager.cpp 硬依赖 lightning 及其 SaveMap 服务；sensor_driver.launch.py 还启动 Lightning 点云转换。aid_robot_py 的地图管理包含 Lightning 专用目录处理。因此直接删除 lightning-lm 会破坏旧 bringup 编译及业务流程。

新地图流程应统一保存 GlobalMap.pcd、trajectory.pcd 和同坐标系二维地图 YAML/图像，分别调用新栈的 3D/2D 保存服务并检查结果。旧 Lightning 地图不能仅改目录名当作新定位地图；首版优先重新建图。地图保存服务请求应以最终统一后的 srv 定义为准。

## 实施顺序与验收

1. **独立编译最小新栈**：统一消息包，加入 FAST-LIO-SAM、二维建图、定位和独立 G1 bringup，保留驱动与 G1 桥。确认目标机 ROS/Nav2、GTSAM、PCL、GeographicLib 等依赖；本地主机有 Humble，不代表目标机已验证。OpenCL 包已有 CPU fallback，可先使用 CPU。
2. **传感器与建图**：验证 PointCloud2 逐点时间和 IMU 坐标，修正 TF 发布逻辑；先录包回放，随后实机建图。验收静止不漂转、移动无明显重影、回环与二维地图一致，保存产物齐全。
3. **定位**：使用新地图重新启动前端与 NDT/ICP；验收 initialpose 生效、odom 连续、map→odom 单一发布者、停止再启动可复现，不能只以出现 TF 为成功。
4. **导航**：接入 AGV 控制器和适配后的点云过滤，先保持 G1 运动禁用。验收地图、规划、障碍出现及清除、速度输出和停车链，再做平地低速行走。
5. **业务切换与清理**：替换保存地图/地图切换/总启动入口，迁出 URDF，解除剩余 Lightning 依赖，最后从活动构建中移除旧栈。保留源码版本用于回退，不直接覆盖旧工程。

建议最终提供三个统一入口：建图、保存地图、定位导航；名称可在实施时确定。本分析中的目标结构并非已存在的可运行命令。

## 下一步优先工作

先做“独立 G1 bringup + 原生 MID360 PointCloud2 + TF/里程计职责拆分 + FAST-LIO-SAM 二维/三维建图”这一闭环，再接定位与导航。最大的工作量在传感器坐标、时间和 odom 适配，而非复制导航参数。DeepSeek 复核仍待服务器模型配置恢复后补做。
