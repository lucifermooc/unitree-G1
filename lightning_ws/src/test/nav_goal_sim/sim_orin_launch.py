"""Orin 30524 跟线 / 到点仿真：Nav2（humble，RPP）+ g1_orin_sim.py（速度桥 + 无背包 G1 模型）。

ros2 launch sim_orin_launch.py map:=<map.yaml> x:=0.6 y:=0.1 yaw_deg:=-7 sim_args:="-p bridge.mode:=pulse -p model.yaw_comp_deg:=9.5"
速度链路：controller -> cmd_vel_nav -> velocity_smoother -> cmd_vel -> g1_orin_sim。
"""
import os
import shlex

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

HERE = os.path.dirname(os.path.realpath(__file__))
SRC = os.path.realpath(os.path.join(HERE, "..", ".."))
BT_XML = os.path.join(SRC, "aid_navigation2", "behavior_trees",
                      "navigate_w_recovery_and_replanning_only_if_path_becomes_invalid.xml")
NAV_NODES = ["map_server", "planner_server", "controller_server", "velocity_smoother",
             "behavior_server", "bt_navigator"]


def sim_node(context):
    a = lambda k: LaunchConfiguration(k).perform(context)  # noqa: E731
    cmd = ["python3", "-B", os.path.join(HERE, "g1_orin_sim.py"), "--ros-args",
           "-p", f"x:={a('x')}", "-p", f"y:={a('y')}", "-p", f"yaw_deg:={a('yaw_deg')}"] + shlex.split(a("sim_args"))
    procs = [ExecuteProcess(cmd=cmd, cwd=HERE, output="log")]
    crab = a("crab_frame_deg")
    if float(crab) != 0.0:   # g1_crab_comp_frame 的对照版：发 base_link -> base_link_nav
        procs.append(ExecuteProcess(cmd=["python3", "-B", os.path.join(HERE, "crab_comp_sim.py"), "--ros-args",
                                         "-p", f"crab_deg:={crab}"], cwd=HERE, output="log"))
    return procs


def generate_launch_description():
    base = os.path.join(HERE, "nav2_sim_params.yaml")
    orin = os.path.join(HERE, "nav2_orin_rpp.yaml")
    arg = LaunchConfiguration
    return LaunchDescription([
        DeclareLaunchArgument("map"),
        DeclareLaunchArgument("planner", default_value=os.path.join(HERE, "planner_orin.yaml")),
        DeclareLaunchArgument("x", default_value="0.0"),
        DeclareLaunchArgument("y", default_value="0.0"),
        DeclareLaunchArgument("yaw_deg", default_value="0.0"),
        DeclareLaunchArgument("bt_xml", default_value=BT_XML),
        DeclareLaunchArgument("controller_extra", default_value=os.path.join(HERE, "empty_controller.yaml")),
        DeclareLaunchArgument("sim_args", default_value=""),
        # 非 0 时起 crab_comp_sim.py；同时要给 local_extra:=local_base_link_nav.yaml
        DeclareLaunchArgument("crab_frame_deg", default_value="0.0"),
        DeclareLaunchArgument("local_extra", default_value=os.path.join(HERE, "empty.yaml")),
        # 叠加到 planner_server（global costmap）和 controller_server（local costmap）的额外参数，如 obstacle_layer_sim.yaml
        DeclareLaunchArgument("costmap_extra", default_value=os.path.join(HERE, "empty.yaml")),
        DeclareLaunchArgument("smoother_extra", default_value=os.path.join(HERE, "empty.yaml")),
        Node(package="nav2_map_server", executable="map_server", name="map_server", output="log",
             parameters=[base, {"yaml_filename": arg("map")}]),
        Node(package="nav2_planner", executable="planner_server", name="planner_server", output="log",
             parameters=[base, orin, arg("planner"), arg("costmap_extra")]),
        Node(package="nav2_controller", executable="controller_server", name="controller_server", output="log",
             parameters=[base, orin, arg("controller_extra"), arg("local_extra"), arg("costmap_extra")], remappings=[("cmd_vel", "cmd_vel_nav")]),
        Node(package="nav2_velocity_smoother", executable="velocity_smoother", name="velocity_smoother",
             output="log", parameters=[base, orin, arg("smoother_extra")],
             remappings=[("cmd_vel", "cmd_vel_nav"), ("cmd_vel_smoothed", "cmd_vel")]),
        Node(package="nav2_behaviors", executable="behavior_server", name="behavior_server", output="log",
             parameters=[base, orin]),
        Node(package="nav2_bt_navigator", executable="bt_navigator", name="bt_navigator", output="log",
             parameters=[base, {"default_nav_to_pose_bt_xml": arg("bt_xml")}]),
        Node(package="nav2_lifecycle_manager", executable="lifecycle_manager", name="lifecycle_manager_navigation",
             output="log", parameters=[{"autostart": True, "node_names": NAV_NODES, "bond_timeout": 4.0}]),
        OpaqueFunction(function=sim_node),
    ])
