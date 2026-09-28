"""语义导航：一句话 → 语义地图查坐标 → 发给 Nav2 → 机器人走过去。

用法（需要 Nav2 已经在运行、定位正常）：
    python semantic_nav.py "我想喝水"
    python semantic_nav.py                 # 交互模式，连续输入

在别的程序里用：
    from semantic_nav import SemanticNavigator
    nav = SemanticNavigator()
    result = nav.go("去充电")
"""
import math
import sys

import rclpy
from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult

from query import search


def goal_to_pose(best, stamp):
    """语义地图查询结果（x, y, yaw）→ Nav2 的目标 PoseStamped。"""
    pose = PoseStamped()
    pose.header.frame_id = best["frame_id"]
    pose.header.stamp = stamp
    pose.pose.position.x = float(best["x"])
    pose.pose.position.y = float(best["y"])
    # ROS 用四元数表示朝向；平面上只绕 z 轴转：z = sin(yaw/2)，w = cos(yaw/2)
    pose.pose.orientation.z = math.sin(best["yaw"] / 2)
    pose.pose.orientation.w = math.cos(best["yaw"] / 2)
    return pose


class SemanticNavigator:
    def __init__(self, use_sim_time=False):
        if not rclpy.ok():
            rclpy.init()
        self.navigator = BasicNavigator(node_name="semantic_navigator")
        if use_sim_time:
            self.navigator.set_parameters(
                [rclpy.parameter.Parameter("use_sim_time", rclpy.Parameter.Type.BOOL, True)])
        # 定位由 Cartographer 提供（不是 AMCL），所以不等待 AMCL
        self.navigator.waitUntilNav2Active(localizer="robot_localization")

    def go(self, text, timeout_sec=180.0):
        """查询并导航，返回 {"query", "found", "best", "nav_result"}；没找到地点时不动。"""
        result = search(text)
        if not result["found"]:
            result["nav_result"] = "NOT_FOUND"
            return result

        goal = goal_to_pose(result["best"], self.navigator.get_clock().now().to_msg())
        self.navigator.goToPose(goal)
        start = self.navigator.get_clock().now()
        canceled = False
        while not self.navigator.isTaskComplete():
            elapsed = (self.navigator.get_clock().now() - start).nanoseconds / 1e9
            if elapsed > timeout_sec and not canceled:
                self.navigator.cancelTask()
                canceled = True
        status = self.navigator.getResult()
        result["nav_result"] = {TaskResult.SUCCEEDED: "SUCCEEDED", TaskResult.CANCELED: "CANCELED",
                                TaskResult.FAILED: "FAILED"}.get(status, "UNKNOWN")
        return result


def _print(result):
    if result["nav_result"] == "NOT_FOUND":
        print(f"「{result['query']}」没有匹配的地点，不导航")
    else:
        b = result["best"]
        print(f"「{result['query']}」→ {b['title']} (x={b['x']}, y={b['y']}, yaw={b['yaw']}, "
              f"score={b['score']}) → 导航结果：{result['nav_result']}")


def main():
    nav = SemanticNavigator(use_sim_time="--sim" in sys.argv)
    args = [a for a in sys.argv[1:] if a != "--sim"]
    if args:
        _print(nav.go(" ".join(args)))
    else:
        print("输入一句话让机器人去对应地点，输入 q 退出")
        while True:
            try:
                text = input("> ").strip()
            except (EOFError, KeyboardInterrupt):
                break
            if text.lower() in ("q", "quit", "exit"):
                break
            if text:
                _print(nav.go(text))
    rclpy.try_shutdown()


if __name__ == "__main__":
    main()
