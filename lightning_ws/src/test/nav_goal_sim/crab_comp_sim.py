#!/usr/bin/env python3
"""g1_nav_bridge/crab_comp_frame 的 Python 对照版（逻辑逐行照搬 include/g1_nav_bridge/crab_comp.hpp，本地不编译 C++）。
订阅 /odom，20 Hz 发 TF base_link -> base_link_nav = 绕 z 转 crab × g；g 目标 = clamp((|v| - v0)/(v_full - v0), 0, 1)，一阶平滑 tau；
/odom 超过 0.5 s 没数据按停下处理。"""
import math

import rclpy
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from tf2_ros import TransformBroadcaster


class CrabCompSim(Node):
    def __init__(self):
        super().__init__("g1_crab_comp_frame")
        self.crab = math.radians(self.declare_parameter("crab_deg", 10.0).value)
        self.v0 = self.declare_parameter("v0", 0.10).value
        self.v_full = self.declare_parameter("v_full", 0.20).value
        self.tau = self.declare_parameter("tau", 0.3).value
        self.gain = 0.0
        self.speed, self.last_odom = 0.0, None
        self.last = self.get_clock().now()
        self.tf = TransformBroadcaster(self)
        self.create_subscription(Odometry, "odom", self.on_odom, 10)
        self.create_timer(0.05, self.step)

    def on_odom(self, m):
        self.speed = math.hypot(m.twist.twist.linear.x, m.twist.twist.linear.y)
        self.last_odom = self.get_clock().now()

    def step(self):
        now = self.get_clock().now()
        dt = (now - self.last).nanoseconds * 1e-9
        self.last = now
        stale = self.last_odom is None or (now - self.last_odom).nanoseconds * 1e-9 > 0.5
        speed = 0.0 if stale else self.speed
        target = min(1.0, max(0.0, (speed - self.v0) / (self.v_full - self.v0))) if self.v_full > self.v0 else 0.0
        if dt > 0:
            self.gain += (target - self.gain) * (1.0 - math.exp(-dt / self.tau))
        yaw = self.crab * self.gain
        t = TransformStamped()
        t.header.stamp = now.to_msg()
        t.header.frame_id, t.child_frame_id = "base_link", "base_link_nav"
        t.transform.rotation.z, t.transform.rotation.w = math.sin(yaw / 2), math.cos(yaw / 2)
        self.tf.sendTransform(t)


def main():
    rclpy.init()
    n = CrabCompSim()
    try:
        rclpy.spin(n)
    except KeyboardInterrupt:
        pass
    finally:
        n.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
