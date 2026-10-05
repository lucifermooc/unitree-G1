"""启动 Gazebo Harmonic 仿真：场景 + 机器人 + ros_gz 桥 + 点云转 2D 激光（+ RViz）。

    ros2 launch robot_nav_sim sim.launch.py robot:=go2 world:=two_floor
    robot: go2 | diff | ackermann        world: two_floor | flat
    gui:=false 不开 Gazebo 窗口（无界面运行）；rviz:=false 不开 RViz
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import xacro

ROBOTS = {
    "go2": "go2.urdf.xacro",
    "diff": "diff_car.urdf.xacro",
    "ackermann": "ackermann_car.urdf.xacro",
}
# 出生点：一楼西侧房间
DEFAULT_SPAWN = {"x": "3.0", "y": "4.2", "yaw": "0.0"}


def robot_description(pkg, robot):
    if robot not in ROBOTS:
        raise RuntimeError(f"robot 只能是 {list(ROBOTS)}，收到 {robot}")
    mappings = {}
    if robot == "go2":
        has_mesh = os.path.isfile(os.path.join(pkg, "meshes", "go2", "base.dae"))
        mappings["use_meshes"] = "true" if has_mesh else "false"
    return xacro.process_file(os.path.join(pkg, "urdf", ROBOTS[robot]), mappings=mappings).toxml()


def setup(context):
    pkg = get_package_share_directory("robot_nav_sim")
    robot = LaunchConfiguration("robot").perform(context)
    world = LaunchConfiguration("world").perform(context)
    gui = LaunchConfiguration("gui").perform(context).lower() == "true"
    rviz = LaunchConfiguration("rviz").perform(context).lower() == "true"
    rviz_config = LaunchConfiguration("rviz_config").perform(context) or os.path.join(pkg, "rviz", "sim.rviz")
    x, y, yaw = (LaunchConfiguration(k).perform(context) for k in ("x", "y", "yaw"))

    world_file = os.path.join(pkg, "worlds", f"{world}.sdf")
    if not os.path.isfile(world_file):
        raise RuntimeError(f"找不到场景 {world_file}")
    gz_args = f"-r {world_file}" if gui else f"-r -s --headless-rendering {world_file}"

    desc = robot_description(pkg, robot)
    actions = [
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(get_package_share_directory("ros_gz_sim"), "launch", "gz_sim.launch.py")),
            launch_arguments={"gz_args": gz_args, "on_exit_shutdown": "true"}.items()),
        Node(package="robot_state_publisher", executable="robot_state_publisher", output="screen",
             parameters=[{"robot_description": desc, "use_sim_time": True}]),
        Node(package="ros_gz_sim", executable="create", output="screen",
             arguments=["-topic", "robot_description", "-name", "robot",
                        "-x", x, "-y", y, "-z", "0.03", "-Y", yaw]),
        Node(package="ros_gz_bridge", executable="parameter_bridge", output="screen",
             parameters=[{"config_file": os.path.join(pkg, "config", "bridge.yaml"), "use_sim_time": True}]),
        Node(package="pointcloud_to_laserscan", executable="pointcloud_to_laserscan_node",
             name="pointcloud_to_laserscan", output="screen",
             parameters=[os.path.join(pkg, "config", "pointcloud_to_laserscan.yaml")],
             remappings=[("cloud_in", "/livox/points"), ("scan", "/scan")]),
    ]
    if rviz:
        actions.append(Node(package="rviz2", executable="rviz2", output="log",
                            arguments=["-d", rviz_config], parameters=[{"use_sim_time": True}]))
    return actions


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("robot", default_value="go2", description="go2 | diff | ackermann"),
        DeclareLaunchArgument("world", default_value="two_floor", description="two_floor | flat"),
        DeclareLaunchArgument("gui", default_value="true", description="是否打开 Gazebo 窗口"),
        DeclareLaunchArgument("rviz", default_value="true", description="是否打开 RViz"),
        DeclareLaunchArgument("rviz_config", default_value="", description="RViz 配置文件，默认 rviz/sim.rviz"),
        DeclareLaunchArgument("x", default_value=DEFAULT_SPAWN["x"]),
        DeclareLaunchArgument("y", default_value=DEFAULT_SPAWN["y"]),
        DeclareLaunchArgument("yaw", default_value=DEFAULT_SPAWN["yaw"]),
        OpaqueFunction(function=setup),
    ])
