#!/usr/bin/env python3
"""G1 运动学仿真（替代真机做 Nav2 到点测试）。

订阅 /cmd_vel，按 g1_nav_bridge/src/cmdvel_to_sport.cpp 的规则改写速度（G1 不能原地转、不能慢速蠕动、不能倒走），
再经一阶响应积分成位姿，发布 /odom 和 TF（map->odom 恒等，odom->base_link）。

抬速规则（与 cmdvel_to_sport.cpp ApplyMinimumVelocity 一一对应，参数默认值取 g1_nav_bridge/config/nav_bridge.yaml）：
  vx < 0           -> 0（G1 倒走会摔倒）
  |wz| >= wz_stop  -> wz = sign * max(|wz|, min_wz)，vx = max(vx, turn_min_vx)      （转弯必须带前进速度）
  否则 |vx| < vx_stop -> 完全停止；否则 vx = max(vx, min_vx)                         （不能慢速蠕动）
SetVelocity 的 duration=0.5 s：超过 cmd_timeout 没有新指令就停。
"""
import math

import rclpy
from geometry_msgs.msg import TransformStamped, Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from tf2_ros import StaticTransformBroadcaster, TransformBroadcaster


def lift(vx, wz, min_vx=0.3, min_wz=0.6, turn_min_vx=0.2, vx_stop=0.05, wz_stop=0.15):
    """cmdvel_to_sport 的抬速规则。返回 (vx, wz)。"""
    vx = max(vx, 0.0)
    if abs(wz) >= wz_stop:
        return max(vx, turn_min_vx), math.copysign(max(abs(wz), min_wz), wz)
    if abs(vx) < vx_stop:
        return 0.0, 0.0
    return max(vx, min_vx), wz


class G1MotionSim(Node):
    def __init__(self):
        super().__init__("g1_motion_sim")
        p = self.declare_parameter
        self.x = p("x", 0.0).value
        self.y = p("y", 0.0).value
        self.yaw = math.radians(p("yaw_deg", 0.0).value)
        self.tau = p("tau", 0.25).value              # 速度一阶响应时间常数（s）
        self.cmd_timeout = p("cmd_timeout", 0.5).value
        self.lift_on = p("lift", True).value         # false = 理想差速底盘（对照用）
        self.lp = {k: p(k, v).value for k, v in
                   dict(min_vx=0.3, min_wz=0.6, turn_min_vx=0.2, vx_stop=0.05, wz_stop=0.15).items()}
        rate = p("rate", 50.0).value

        self.target = (0.0, 0.0)
        self.v, self.w = 0.0, 0.0
        self.last_cmd = None
        self.tf = TransformBroadcaster(self)
        self.static_tf = StaticTransformBroadcaster(self)
        self.odom_pub = self.create_publisher(Odometry, "odom", 10)
        self.create_subscription(Twist, "cmd_vel", self.on_cmd, 10)

        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id, t.child_frame_id = "map", "odom"
        t.transform.rotation.w = 1.0
        self.static_tf.sendTransform(t)

        self.dt = 1.0 / rate
        self.create_timer(self.dt, self.step)
        self.get_logger().info(
            f"start ({self.x:.2f}, {self.y:.2f}, {math.degrees(self.yaw):.1f} deg), lift={self.lift_on}, tau={self.tau}")

    def on_cmd(self, msg):
        vx, wz = msg.linear.x, msg.angular.z
        self.target = lift(vx, wz, **self.lp) if self.lift_on else (max(vx, 0.0), wz)
        self.last_cmd = self.get_clock().now()

    def step(self):
        now = self.get_clock().now()
        target = self.target
        if self.last_cmd is None or (now - self.last_cmd).nanoseconds * 1e-9 > self.cmd_timeout:
            target = (0.0, 0.0)
        a = min(1.0, self.dt / self.tau) if self.tau > 0 else 1.0
        self.v += (target[0] - self.v) * a
        self.w += (target[1] - self.w) * a
        self.yaw += self.w * self.dt
        self.x += self.v * math.cos(self.yaw) * self.dt
        self.y += self.v * math.sin(self.yaw) * self.dt

        stamp = now.to_msg()
        q = (0.0, 0.0, math.sin(self.yaw / 2), math.cos(self.yaw / 2))
        t = TransformStamped()
        t.header.stamp = stamp
        t.header.frame_id, t.child_frame_id = "odom", "base_link"
        t.transform.translation.x, t.transform.translation.y = self.x, self.y
        t.transform.rotation.z, t.transform.rotation.w = q[2], q[3]
        self.tf.sendTransform(t)

        o = Odometry()
        o.header.stamp = stamp
        o.header.frame_id, o.child_frame_id = "odom", "base_link"
        o.pose.pose.position.x, o.pose.pose.position.y = self.x, self.y
        o.pose.pose.orientation.z, o.pose.pose.orientation.w = q[2], q[3]
        o.twist.twist.linear.x, o.twist.twist.angular.z = self.v, self.w
        self.odom_pub.publish(o)


def main():
    rclpy.init()
    node = G1MotionSim()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
