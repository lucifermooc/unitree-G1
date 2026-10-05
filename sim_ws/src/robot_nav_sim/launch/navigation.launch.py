"""Nav2 导航：仿真 + Nav2（AMCL + 地图）+ 楼层管理 + RViz。

    ros2 launch robot_nav_sim navigation.launch.py robot:=go2 world:=two_floor
    ros2 launch robot_nav_sim navigation.launch.py robot:=ackermann world:=flat

地图默认用 tools/gen_world.py 生成的真值地图（two_floor 用一楼地图，flat 用 flat 地图），
也可以用 map:=/绝对路径/my_map.yaml 换成自己建的图。
在 RViz 里用 "2D Goal Pose" 发目标；跨楼层用：ros2 run robot_nav_sim go_to --floor 2 --x 3 --y 2
"""
import copy
import os
import tempfile

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def deep_merge(base, over):
    out = copy.deepcopy(base)
    for k, v in over.items():
        if isinstance(v, dict) and isinstance(out.get(k), dict):
            out[k] = deep_merge(out[k], v)
        else:
            out[k] = copy.deepcopy(v)
    return out


def merged_params(pkg, robot, x, y, yaw):
    with open(os.path.join(pkg, "config", "nav2_base.yaml")) as f:
        base = yaml.safe_load(f)
    with open(os.path.join(pkg, "config", f"nav2_{robot}.yaml")) as f:
        over = yaml.safe_load(f) or {}
    p = deep_merge(base, over)
    p["amcl"]["ros__parameters"]["initial_pose"] = {"x": float(x), "y": float(y), "z": 0.0, "yaw": float(yaw)}
    path = os.path.join(tempfile.gettempdir(), f"robot_nav_sim_nav2_{robot}.yaml")
    with open(path, "w") as f:
        yaml.safe_dump(p, f, sort_keys=False)
    return path


def setup(context):
    pkg = get_package_share_directory("robot_nav_sim")
    robot = LaunchConfiguration("robot").perform(context)
    world = LaunchConfiguration("world").perform(context)
    x, y, yaw = (LaunchConfiguration(k).perform(context) for k in ("x", "y", "yaw"))
    map_yaml = LaunchConfiguration("map").perform(context) or os.path.join(
        pkg, "maps", "floor1.yaml" if world == "two_floor" else f"{world}.yaml")
    params = merged_params(pkg, robot, x, y, yaw)
    nav_rviz = os.path.join(pkg, "rviz", "nav.rviz")

    actions = []
    if LaunchConfiguration("sim").perform(context).lower() == "true":
        actions.append(IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(pkg, "launch", "sim.launch.py")),
            launch_arguments={"robot": robot, "world": world, "x": x, "y": y, "yaw": yaw,
                              "gui": LaunchConfiguration("gui").perform(context),
                              "rviz": LaunchConfiguration("rviz").perform(context),
                              "rviz_config": nav_rviz}.items()))
    elif LaunchConfiguration("rviz").perform(context).lower() == "true":
        actions.append(Node(package="rviz2", executable="rviz2", output="log", arguments=["-d", nav_rviz],
                            parameters=[{"use_sim_time": True}]))

    actions.append(IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory("nav2_bringup"), "launch", "bringup_launch.py")),
        launch_arguments={"map": map_yaml, "params_file": params, "use_sim_time": "true",
                          "autostart": "true", "use_composition": "False"}.items()))

    if world == "two_floor":
        actions.append(Node(package="robot_nav_sim", executable="floor_manager", output="screen",
                            parameters=[{"use_sim_time": True,
                                         "floors_file": os.path.join(pkg, "config", "floors.yaml"),
                                         "maps_dir": os.path.join(pkg, "maps"),
                                         "initial_floor": 1,
                                         "auto_detect": LaunchConfiguration("auto_floor").perform(context).lower() == "true"}]))
    return actions


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("robot", default_value="go2", description="go2 | diff | ackermann"),
        DeclareLaunchArgument("world", default_value="two_floor", description="two_floor | flat"),
        DeclareLaunchArgument("map", default_value="", description="地图 yaml，默认用生成的真值地图"),
        DeclareLaunchArgument("gui", default_value="true"),
        DeclareLaunchArgument("rviz", default_value="true"),
        DeclareLaunchArgument("sim", default_value="true", description="false：仿真已经在运行"),
        DeclareLaunchArgument("auto_floor", default_value="true",
                              description="遥控上下楼时，按仿真真值高度自动切换楼层地图"),
        DeclareLaunchArgument("x", default_value="3.0"),
        DeclareLaunchArgument("y", default_value="4.2"),
        DeclareLaunchArgument("yaw", default_value="0.0"),
        OpaqueFunction(function=setup),
    ])
