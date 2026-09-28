# 3D Nav 工作空间：SLAM + Navigation 精简整理

本目录从原 `haier-robot-ws-3d_nav` 中只保留 SLAM、定位和 Nav2 核心代码，以及它们编译所需的内部消息/插件包。未修改核心源码逻辑。

## 1. 保留的包

### SLAM / 建图

- `fast_lio_sam/FAST_LIO_SAM`
  - 3D LiDAR-IMU(-wheel odom) 前端 + GTSAM 后端/回环。
  - 当前机器人配置使用 `config/rs128.yaml` 和 `launch/mapping_rs.launch.py`。
  - 输入：`/rslidar_points`、`/rslidar_imu_data`、`/odom`（轮速里程计）。
  - 关键输出：`/Odometry`、`/cloud_registered_body`、`fast_lio_sam/key_frame`、`fast_lio_sam/increment_pos_pub`、`fast_lio_sam/path_update`。
  - 建图阶段发布 map 相关 TF，并提供 3D 地图保存服务。

- `map_create_2d`
  - 根据 FAST-LIO-SAM 的关键帧/轨迹生成 2D OccupancyGrid。
  - `mapping_rs.launch.py` 会直接 include `mapping_2d.launch.py`。
  - 提供 `/save_2d_map`，与 3D PCD 保存到同一地图目录。

### 定位

- `aid_localization`
  - 定位阶段复用 `fast_lio_sam` 的 `front_end_mode=true` 作为激光惯导前端。
  - 加载 `GlobalMap.pcd` / `trajectory.pcd`，使用 NDT/ICP 做全局点云匹配。
  - 从 TF 获取 `odom -> base_footprint`，维护并发布 `map -> odom`。
  - 接收 `/initialpose` 完成重定位初始化。

### Navigation

- `aid_navigation2`
  - Nav2 bringup、BT、planner/controller/costmap/velocity smoother 参数。
  - 当前 controller 为自研 PurePursuitShim 包装 MPPI。
  - global costmap 使用 `map`；local costmap 使用 `odom`；机器人基座为 `base_footprint`。

- `aid_pure_pursuit_shim_controller`
  - `controller_server` 当前实际启用的自研控制器插件。
  - 远距离使用 MPPI，靠近目标切换纯跟踪并处理减速/转向/碰撞预测。

- `aid_costmap_plugin`
  - Nav2 自定义 costmap layer。
  - 当前参数实际启用了自定义 `StaticLayer` 和 `KeepoutLayer`；包内还包含其他 layer，但为同一个插件库，保留整个包避免改源码/构建逻辑。

### Navigation 点云预处理

- `aid_pointcloud_filter`
  - 当前 Nav2 costmap 实际订阅 `/rslidar_points/points2_filted`。
  - 这个话题由原 `robot_bringup/collision_monitor.launch.py` 中启动的 `cloud_filter_rslidar_node` 生成。
  - 因此它虽然不是 SLAM 算法本体，但对当前导航障碍物层是实际运行依赖，不能删。

- `aid_opencl_process`
  - `aid_pointcloud_filter` 的编译依赖。当前 RS LiDAR 参数中 `use_opencl_point_filters=false`，但 CMake/package 仍要求该包存在。

### 内部公共依赖

- `aid_robot_msgs`
  - FAST-LIO-SAM、2D 建图和 costmap 插件的内部消息/服务依赖。
  - 保留整个接口包，避免为“精简接口”修改原源码和 CMake。

## 2. 原工程中明确排除的包/功能

以下不属于 SLAM/Nav 核心，未放入本精简工作空间：

- `sensor_diagnostics`：传感器诊断
- `charge_pile_control`：充电桩
- `servo_axix_arm` / `hair_arm_control`：机械臂/舵机
- `haier_opt_driver` / `aid-ros-driver`：整机驱动
- `apriltag_ros` / `opennav_docking`：AprilTag 与自动回充
- `rgbd_calibration`：RGBD 标定
- `aid_robot_py`：地图管理、launch 管理、waypoint 管理等整机业务
- `aid_waypoint_follower`：当前 `navigation2.launch.py` 中启动项已注释
- `odom_laser_calibra_tool`：里程计/激光外参标定工具
- `loca_confidence`：定位置信度业务
- `laser_undistortion`：独立激光去畸变；FAST-LIO-SAM 已有自己的去畸变前端，且原总 launch 中该节点已注释
- `aid_sensor_monitor`：传感器监控
- `aid_aging_test`：老化测试
- `robot_bringup`：混合整机业务的总启动包，不作为核心包保留
- `haier_robot_urdf`：原实际 `robot_state_publisher.launch.py` 使用的是 `robot_bringup/urdf/robot.urdf`，不是该包

`reference/robot.urdf` 与 `reference/odom_laser_calibration.yaml` 仅作为 TF/外参参考保留，不参与本精简工作空间构建。

另外保留了 `reference/collision_monitor_params.yaml` 和原始 `collision_monitor.launch.py` 作为导航点云过滤/碰撞监控参考。原 launch 仍引用 `robot_bringup`，因此这里只把它当参考，不把整个混合业务包带回来。

## 3. 实际数据链路

### 建图

```text
/rslidar_points -----------+
/rslidar_imu_data ---------+--> fast_lio_sam_mapping
/odom (wheel odom) --------+        |
                                    +--> /Odometry
                                    +--> /cloud_registered_body
                                    +--> key_frame / path_update / increment_pos_pub
                                    +--> 3D PCD map
                                              |
                                              +--> map_create_2d --> 2D OccupancyGrid
                                                                  --> /save_2d_map
```

原工程实际入口：

```bash
ros2 launch fast_lio_sam mapping_rs.launch.py use_sim_time:=false
```

`mapping_rs.launch.py` 已自动启动 `map_create_2d/mapping_2d.launch.py`。

### 定位

```text
/rslidar_points + /rslidar_imu_data + /odom
                  |
                  v
fast_lio_sam_mapping(front_end_mode=true)
                  |
                  +--> /cloud_registered_body (lidar frame)
                  |
                  v
aid_localization + GlobalMap.pcd + trajectory.pcd
                  |
                  +--> map -> odom TF
                  +--> localization pose / global map cloud
```

运行示例：

```bash
ros2 launch aid_localization localization.launch.py \
  map_dir:=/home/aidlux/maps/<map_name> \
  use_sim_time:=false
```

### 导航

```text
2D map yaml --> map_server --> /map
                              |
map -> odom -> base_footprint TF
                              |
/rslidar_points --> aid_pointcloud_filter --> /rslidar_points/points2_filted
                                                |
                                                +--> local/global costmap
                                   planner_server
                                   controller_server
                                   BT navigator
                                   velocity_smoother
                                          |
                                          +--> cmd_vel
```

运行示例：

```bash
ros2 launch aid_navigation2 navigation2.launch.py \
  map:=/home/aidlux/maps/<map_name>/<map_name>.yaml \
  use_sim_time:=false
```

## 4. TF 最低要求

这套代码不是“只有 map -> base_link”结构。定位和 Nav2 明确依赖标准链路：

```text
map -> odom -> base_footprint -> base_link -> rslidar
```

其中：

- `aid_localization` 发布 `map -> odom`。
- `/odom` 对应的底盘/里程计侧必须提供可查询的 `odom -> base_footprint` TF。
- `base_footprint -> base_link -> rslidar` 为机器人静态 TF。
- FAST-LIO 的 wheel-odom 紧耦合还会通过 TF 解析 LiDAR 与 wheel odom child frame 的外参。

如果只启动本精简工作空间但没有底盘 TF / robot_state_publisher，SLAM/定位/Nav2 不会完整工作。

当前 Nav2 障碍物层还要求 `/rslidar_points/points2_filted`。原工程通过 `aid_pointcloud_filter` 的 `cloud_filter_rslidar_node` 生成该话题，参数已保存在 `reference/collision_monitor_params.yaml`。

## 5. 原工程的地图切换逻辑（已从 robot_bringup 中抽离为说明）

原 `robot_status_manager` 做的事情本质上是：

1. mapping 模式启动 `fast_lio_sam/mapping_rs.launch.py`；
2. 调 `/save_3d_map` 保存 `GlobalMap.pcd`、`trajectory.pcd` 等；
3. 调 `/save_2d_map` 保存 `<map_name>.png + <map_name>.yaml`；
4. 停止 mapping；
5. 启动 `aid_localization/localization.launch.py map_dir:=...`；
6. 必要时向 `/initialpose` 重发建图结束时的当前位姿；
7. Nav2 通过 `map_server/load_map` 切到 `<map_name>.yaml`。

这部分属于整机“模式/地图管理”业务，不是 SLAM/Nav 算法本身，因此没有把整个 `robot_bringup` 包带进来。

## 6. 编译时仍需要的系统/外部 ROS 依赖

保留下来的源码并不代表所有第三方依赖都在本 zip 内。至少包括：

- ROS 2 / Nav2 对应发行版
- PCL / pcl_conversions / pcl_ros
- Eigen3
- GTSAM
- GeographicLib
- `livox_ros_driver2`（FAST-LIO-SAM 源码/CMake 有硬依赖，即使当前传感器是 RS128）
- `spatio_temporal_voxel_layer`
- Nav2 MPPI controller 及 Nav2 标准组件

另外原 `aid_navigation2/package.xml` 仍声明了 `teb_local_planner`，但当前 `nav2_params.yaml` 实际 controller 使用 MPPI + `aid_pure_pursuit_shim_controller`，并没有启用 TEB。
