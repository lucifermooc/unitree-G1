"""Nav2 到点仿真：map_server + planner/controller/smoother/behavior/bt_navigator + G1 运动学仿真。

ros2 launch sim_launch.py map:=<map.yaml> planner:=planner_hybrid.yaml x:=-0.16 y:=-0.21 yaw_deg:=-14
速度链路：controller -> cmd_vel_nav -> velocity_smoother -> cmd_vel -> g1_motion_sim（cmdvel_to_sport 抬速规则）。
行为树用 Thor 的 aid_navigation2/behavior_trees/navigate_w_recovery_and_replanning_only_if_path_becomes_invalid.xml。
"""
import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

HERE = os.path.dirname(os.path.realpath(__file__))
SRC = os.path.realpath(os.path.join(HERE, "..", ".."))
BT_XML = os.path.join(SRC, "aid_navigation2", "behavior_trees",
                      "navigate_w_recovery_and_replanning_only_if_path_becomes_invalid.xml")
NAV_NODES = ["map_server", "planner_server", "controller_server", "velocity_smoother",
             "behavior_server", "bt_navigator"]


def generate_launch_description():
    params = os.path.join(HERE, "nav2_sim_params.yaml")
    arg = LaunchConfiguration
    planner = arg("planner")
    return LaunchDescription([
        DeclareLaunchArgument("map", default_value="/home/ap/G1_bags/maps/1790676615248/map.yaml"),
        DeclareLaunchArgument("planner", default_value=os.path.join(HERE, "planner_2d.yaml")),
        DeclareLaunchArgument("x", default_value="0.0"),
        DeclareLaunchArgument("y", default_value="0.0"),
        DeclareLaunchArgument("yaw_deg", default_value="0.0"),
        DeclareLaunchArgument("lift", default_value="true"),
        DeclareLaunchArgument("bt_xml", default_value=BT_XML),
        # 额外参数文件（叠加在速度平滑器上，如 decel_fast.yaml）；默认用空文件
        DeclareLaunchArgument("smoother_extra", default_value=os.path.join(HERE, "empty.yaml")),
        # 额外参数文件（叠加在控制器上，如 goal_tight.yaml）
        DeclareLaunchArgument("controller_extra", default_value=os.path.join(HERE, "empty_controller.yaml")),
        Node(package="nav2_map_server", executable="map_server", name="map_server", output="log",
             parameters=[params, {"yaml_filename": arg("map")}]),
        Node(package="nav2_planner", executable="planner_server", name="planner_server", output="log",
             parameters=[params, planner]),
        Node(package="nav2_controller", executable="controller_server", name="controller_server", output="log",
             parameters=[params, arg("controller_extra")], remappings=[("cmd_vel", "cmd_vel_nav")]),
        Node(package="nav2_velocity_smoother", executable="velocity_smoother", name="velocity_smoother",
             output="log", parameters=[params, arg("smoother_extra")],
             remappings=[("cmd_vel", "cmd_vel_nav"), ("cmd_vel_smoothed", "cmd_vel")]),
        Node(package="nav2_behaviors", executable="behavior_server", name="behavior_server", output="log",
             parameters=[params]),
        Node(package="nav2_bt_navigator", executable="bt_navigator", name="bt_navigator", output="log",
             parameters=[params, {"default_nav_to_pose_bt_xml": arg("bt_xml")}]),
        Node(package="nav2_lifecycle_manager", executable="lifecycle_manager", name="lifecycle_manager_navigation",
             output="log", parameters=[{"autostart": True, "node_names": NAV_NODES, "bond_timeout": 4.0}]),
        ExecuteProcess(cmd=["python3", os.path.join(HERE, "g1_motion_sim.py"), "--ros-args",
                            "-p", ["x:=", arg("x")], "-p", ["y:=", arg("y")],
                            "-p", ["yaw_deg:=", arg("yaw_deg")], "-p", ["lift:=", arg("lift")]],
                       output="log"),
    ])
