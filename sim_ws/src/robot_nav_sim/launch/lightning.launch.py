"""仿真 + 你自己的 Lightning-LM（建图/定位）+ 你自己的 Nav2（aid_navigation2）。

仿真只顶替传感器驱动：/livox/lidar（Livox CustomMsg）、/livox/imu、/odom、/clock。
其余全部复用 lightning_ws 里的原文件，不修改它们：
  - 建图/定位：lightning/launch/g1_online.launch.py（URDF 换成仿真机器人，配置在 default_livox.yaml 上做仿真覆盖）
  - 导航：aid_navigation2/launch/navigation2.launch.py + param/nav2_params.yaml
          + g1_nav_bridge/scan_range_filter（和 g1_navigation_direct.launch.py 一样）

前提：先 source lightning_ws，再 source sim_ws。

    # 1) 建图：遥控走一圈，然后保存
    ros2 launch robot_nav_sim lightning.launch.py mode:=mapping robot:=go2 world:=flat
    ros2 service call /lightning/save_map lightning/srv/SaveMap "{map_id: sim_go2_flat}"
    # 2) 定位 + 导航（出生点必须和建图时一样，Lightning 从建图起点开始定位）
    ros2 launch robot_nav_sim lightning.launch.py mode:=navigation robot:=go2 world:=flat map_dir:=~/maps/sim_go2_flat
"""
import copy
import importlib.util
import os
import tempfile

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription, LogInfo, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

# 仿真 Mid-360 离地高度（URDF 里算出来的），给 Lightning 的 2D 栅格（g2p5）当默认地面高度
LIDAR_HEIGHT = {"go2": 0.439, "diff": 0.290, "ackermann": 0.325}

# 仿真机器人的真实外形半径（以 base_link 为圆心，含腿/轮）。你的 nav2_params.yaml 是 G1 的 0.50
# （为覆盖 D435 盲区特意放大），在 1.2 m 的门洞里只剩 0.2 m 可走的中线，MPPI 过不去。
# 阿克曼车的 base_link 在后轴，车身前伸 0.675 m，用多边形足迹。
ROBOT_RADIUS = {"go2": 0.33, "diff": 0.32}
ACKERMANN_FOOTPRINT = "[[0.70, 0.25], [0.70, -0.25], [-0.15, -0.25], [-0.15, 0.25]]"
INFLATION_MARGIN = 0.20   # 和你的规则一致：inflation_radius = robot_radius + 0.20

# Humble 和 Jazzy 的插件名写法不同；nav2_params.yaml 是按 Jazzy（Thor）写的
HUMBLE_PLUGIN_PREFIXES = ("nav2_behaviors", "nav2_smac_planner", "nav2_navfn_planner", "nav2_theta_star_planner")


def load_module(path, name):
    spec = importlib.util.spec_from_file_location(name, path)
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod


def lightning_config(robot):
    """在你的 default_livox.yaml 上只改仿真必须改的项。"""
    src = os.path.join(get_package_share_directory("lightning"), "config", "default_livox.yaml")
    with open(src) as f:
        cfg = yaml.safe_load(f)
    # 真机驱动把点云和 IMU 都做了 Rx(pi) 且有几厘米偏移；仿真的雷达和 IMU 是同一个坐标系
    cfg["fasterlio"]["extrinsic_T"] = [0.0, 0.0, 0.0]
    cfg["fasterlio"]["extrinsic_R"] = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
    path = os.path.join(tempfile.gettempdir(), f"robot_nav_sim_lightning_{robot}.yaml")
    with open(path, "w") as f:
        yaml.safe_dump(cfg, f, sort_keys=False, allow_unicode=True)
    return path, src


def humble_plugin(name):
    for prefix in HUMBLE_PLUGIN_PREFIXES:
        if isinstance(name, str) and name.startswith(prefix + "::"):
            return prefix + "/" + name[len(prefix) + 2:]
    return name


def set_robot_size(nav, robot, radius_arg):
    """把 G1 的外形换成仿真机器人的外形，膨胀半径和 MPPI ObstaclesCritic 按你的规则同步。"""
    radius = float(radius_arg) if radius_arg else ROBOT_RADIUS.get(robot)
    inflation = None
    for name in ("local_costmap", "global_costmap"):
        params = nav.get(name, {}).get(name, {}).get("ros__parameters")
        if params is None:
            continue
        if radius is not None:
            params["robot_radius"] = radius
            params.pop("footprint", None)
            inflation = radius + INFLATION_MARGIN
            for layer in params.get("plugins", []):
                cfg = params.get(layer, {})
                if "InflationLayer" in str(cfg.get("plugin", "")):
                    cfg["inflation_radius"] = inflation
        else:
            params["footprint"] = ACKERMANN_FOOTPRINT
    ctrl = nav.get("controller_server", {}).get("ros__parameters", {}).get("FollowPath", {})
    if inflation is not None and "ObstaclesCritic" in ctrl:
        ctrl["ObstaclesCritic"]["inflation_radius"] = inflation   # 必须和 local_costmap 的膨胀层一致
    if radius is None:
        for critic in ("CostCritic", "ObstaclesCritic"):
            if critic in ctrl:
                ctrl[critic]["consider_footprint"] = True
    return f"robot_radius {radius:.2f} m，inflation_radius {inflation:.2f} m" if radius is not None \
        else f"多边形足迹 {ACKERMANN_FOOTPRINT}"


def sim_nav_params(robot, nav_mod, radius_arg=""):
    """你的 nav2_params.yaml -> 仿真版：去掉 D435 和禁行区层、全部用仿真时钟、插件名改成 Humble 写法、
    外形换成仿真机器人。"""
    nav_share = get_package_share_directory("aid_navigation2")
    src = os.path.join(nav_share, "param", "nav2_params.yaml")
    with open(src) as f:
        nav = yaml.safe_load(f)
    # 仿真没有 D435，也没有前端画的禁行区：和你的 launch 开关 use_realsense_obstacles/use_keepout:=false 一样
    nav_mod.drop_layers(nav, ["keepout_layer", "stvl_voxel_layer"])

    def walk(node):
        if isinstance(node, dict):
            for k, v in list(node.items()):
                if k == "ros__parameters" and isinstance(v, dict):
                    v["use_sim_time"] = True
                if k == "plugin":
                    node[k] = humble_plugin(v)
                else:
                    walk(v)
        elif isinstance(node, list):
            for v in node:
                walk(v)
    walk(nav)
    ctrl = nav.get("controller_server", {}).get("ros__parameters", {})
    if "progress_checker_plugins" in ctrl and "progress_checker_plugin" not in ctrl:
        ctrl["progress_checker_plugin"] = ctrl["progress_checker_plugins"][0]   # Humble 是单数
    if robot == "ackermann" and "FollowPath" in ctrl:
        # 阿克曼车不能原地转：MPPI 换成阿克曼运动模型（最小转弯半径约 0.8 m）
        ctrl["FollowPath"]["motion_model"] = "Ackermann"
        ctrl["FollowPath"].setdefault("AckermannConstraints", {})["min_turning_r"] = 0.85
    size = set_robot_size(nav, robot, radius_arg)
    out = os.path.join(tempfile.mkdtemp(prefix="robot_nav_sim_nav_"), "nav2_params_sim.yaml")
    with open(out, "w") as f:
        yaml.safe_dump(nav, f, sort_keys=False, allow_unicode=True)
    return out, src, size


def setup(context):
    get = lambda k: LaunchConfiguration(k).perform(context)  # noqa: E731
    pkg = get_package_share_directory("robot_nav_sim")
    robot, world, mode = get("robot"), get("world"), get("mode")
    if mode not in ("mapping", "localization", "navigation"):
        raise RuntimeError("mode 只能是 mapping / localization / navigation")
    map_dir = os.path.abspath(os.path.expanduser(get("map_dir") or f"~/maps/sim_{robot}_{world}"))
    map_save_root = os.path.abspath(os.path.expanduser(get("map_save_root")))

    sim_mod = load_module(os.path.join(pkg, "launch", "sim.launch.py"), "robot_nav_sim_sim")
    urdf_file = os.path.join(tempfile.gettempdir(), f"robot_nav_sim_{robot}_lightning.urdf")
    with open(urdf_file, "w") as f:
        f.write(sim_mod.robot_description(pkg, robot, "lightning"))

    cfg_path, cfg_src = lightning_config(robot)
    actions = [
        LogInfo(msg=f"[lightning 仿真] 模式 {mode}，地图目录 {map_dir}，Lightning 配置基于 {cfg_src}"),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(pkg, "launch", "sim.launch.py")),
            launch_arguments={"robot": robot, "world": world, "stack": "lightning",
                              "gui": get("gui"), "rviz": "false", "start_rsp": "false",
                              "urdf_file": urdf_file,
                              "x": get("x"), "y": get("y"), "yaw": get("yaw")}.items()),
        # 你的 Lightning launch：建图 run_slam_online / 定位 run_loc_online，并由它启动 robot_state_publisher
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(get_package_share_directory("lightning"), "launch", "g1_online.launch.py")),
            launch_arguments={"mode": "mapping" if mode == "mapping" else "localization",
                              "urdf": urdf_file, "config": cfg_path,
                              "map_path": map_dir, "map_save_root": map_save_root,
                              "start_livox": "false", "pub_registered_scan": "true",
                              "with_ui": "false", "with_2dui": "false", "start_rviz": "false",
                              "floor_height": f"{-LIDAR_HEIGHT[robot]:.3f}",
                              "min_obstacle_height": "0.15", "max_obstacle_height": "1.0"}.items()),
        # run_slam_online / run_loc_online 由 gflags 解析命令行，不能带 --ros-args，
        # 所以节点起来后再用参数服务把 use_sim_time 打开（节点名都是 lightning_slam）。
        # 不打开的话 LocGuard 用墙钟和仿真时间戳比较 /odom，仿真实时率不是 1 时会反复判定"时钟跳变"。
        ExecuteProcess(name="lightning_use_sim_time", output="screen", shell=True, cmd=[
            "for i in $(seq 60); do "
            "ros2 param set /lightning_slam use_sim_time true >/dev/null 2>&1 "
            "&& echo '[lightning 仿真] /lightning_slam 已切到仿真时钟' && exit 0; sleep 1; done; "
            "echo '[lightning 仿真] 警告：60 s 内没等到 /lightning_slam，use_sim_time 未设置'"]),
    ]

    if mode == "navigation":
        nav_share = get_package_share_directory("aid_navigation2")
        nav_mod = load_module(os.path.join(nav_share, "launch", "g1_navigation_direct.launch.py"), "g1_nav_direct")
        nav_params, nav_src, size = sim_nav_params(robot, nav_mod, get("robot_radius"))
        nav_map = os.path.join(map_dir, "map.yaml")
        if not os.path.isfile(nav_map):
            raise RuntimeError(f"找不到导航栅格地图 {nav_map}：先用 mode:=mapping 建图并调用 /lightning/save_map 保存")
        actions += [
            LogInfo(msg=f"[lightning 仿真] Nav2 参数基于 {nav_src}，仿真改写后：{nav_params}（{size}）"),
            # 和 g1_navigation_direct.launch.py 里同一个节点、同一组参数
            Node(package="g1_nav_bridge", executable="scan_range_filter", name="scan_range_filter",
                 output="screen", respawn=True, respawn_delay=1.0,
                 parameters=[{"input_topic": nav_mod.MID360_RAW, "output_topic": nav_mod.MID360_NAV,
                              "min_range": nav_mod.MID360_MIN_RANGE, "base_frame": "base_link",
                              "crop_radius": nav_mod.MID360_CROP_RADIUS, "use_sim_time": True}]),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(os.path.join(nav_share, "launch", "navigation2.launch.py")),
                launch_arguments={"map": nav_map, "params_file": nav_params,
                                  "use_sim_time": "true", "use_composition": "False"}.items()),
        ]

    if get("rviz").lower() == "true":
        actions.append(Node(package="rviz2", executable="rviz2", output="log",
                            arguments=["-d", os.path.join(pkg, "rviz", "lightning.rviz")],
                            parameters=[{"use_sim_time": True}]))
    return actions


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("mode", default_value="navigation",
                              description="mapping | localization | navigation（navigation = 定位 + Nav2）"),
        DeclareLaunchArgument("robot", default_value="go2", description="go2 | diff | ackermann"),
        DeclareLaunchArgument("world", default_value="flat", description="flat | two_floor"),
        DeclareLaunchArgument("map_dir", default_value="",
                              description="地图目录，默认 ~/maps/sim_<robot>_<world>"),
        DeclareLaunchArgument("map_save_root", default_value="~/maps",
                              description="/lightning/save_map 保存到 <map_save_root>/<map_id>"),
        DeclareLaunchArgument("robot_radius", default_value="",
                              description="留空用仿真机器人的真实半径；robot_radius:=0.50 还原 G1 的值"),
        DeclareLaunchArgument("gui", default_value="true"),
        DeclareLaunchArgument("rviz", default_value="true"),
        DeclareLaunchArgument("x", default_value="3.0"),
        DeclareLaunchArgument("y", default_value="4.2"),
        DeclareLaunchArgument("yaw", default_value="0.0"),
        OpaqueFunction(function=setup),
    ])
