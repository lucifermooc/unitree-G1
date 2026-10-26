#!/usr/bin/env python3
"""Orin 30524（无背包 G1）仿真节点：/cmd_vel -> 速度桥（g1_orin_model.Bridge）-> G1 模型（g1_orin_model.G1）-> TF / odom。

TF：map->odom 恒等（静态），odom->base_link = 叠加了步态晃动的"定位位姿"（RPP、costmap 看到的就是它）。
另发 /sim/truth（骨盆真实位姿）、/sim/pub_pose（发布位姿）、/sim/sent（桥发给 G1 的 vx/vy/wz）供 run_track_test.py 统计。
参数：bridge.* 传给 Bridge，model.* 传给 G1（如 -p bridge.mode:=pulse -p model.yaw_comp_deg:=9.5）。
"""
import math

import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped, TransformStamped, Twist
from nav_msgs.msg import Odometry, Path
from rclpy.node import Node
from rclpy.parameter import Parameter
from tf2_ros import StaticTransformBroadcaster, TransformBroadcaster

from g1_orin_model import G1, Bridge

BRIDGE_DEFAULTS = dict(mode="current", wz_stop=0.15, min_wz=0.32, min_vx=0.3, vx_stop=0.05, turn_min_vx=0.0,
                       walk_min_wz=0.15, walk_wz_stop=0.03, smooth_s=0.0, vy_gain=0.8, vy_max=0.2, vy_ff=0.0)
MODEL_DEFAULTS = dict(walk_resp="measured", walk_yaw_bias=0.014, latency=0.35, tau_v=0.3, tau_w=0.2, dist_sigma=0.10, dist_tau=5.0,
                      sway_y=0.06, sway_yaw_deg=2.5, sway_f=1.0, spin_drift=True, vy_dead=0.10, crab_deg=9.5,
                      yaw_comp_deg=0.0, seed=0)


def quat(yaw):
    return 0.0, 0.0, math.sin(yaw / 2), math.cos(yaw / 2)


class G1OrinSim(Node):
    def __init__(self):
        super().__init__("g1_orin_sim")
        x = self.declare_parameter("x", 0.0).value
        y = self.declare_parameter("y", 0.0).value
        yaw = math.radians(self.declare_parameter("yaw_deg", 0.0).value)
        bk = {k: self.declare_parameter(f"bridge.{k}", v).value for k, v in BRIDGE_DEFAULTS.items()}
        mk = {k: self.declare_parameter(f"model.{k}", v).value for k, v in MODEL_DEFAULTS.items()}
        self.bridge = Bridge(dt=0.1, **bk)
        self.g1 = G1(x=x, y=y, yaw=yaw, **mk)
        self.get_logger().info(f"bridge {bk}")
        self.get_logger().info(f"model {mk}")
        self.t0 = self.get_clock().now()
        self.plan = None
        self.tf = TransformBroadcaster(self)
        self.static_tf = StaticTransformBroadcaster(self)
        self.odom_pub = self.create_publisher(Odometry, "odom", 10)
        self.truth_pub = self.create_publisher(PoseStamped, "sim/truth", 10)
        self.pubpose_pub = self.create_publisher(PoseStamped, "sim/pub_pose", 10)
        self.sent_pub = self.create_publisher(Twist, "sim/sent", 10)
        self.create_subscription(Twist, "cmd_vel", self.on_cmd, 10)
        self.create_subscription(Path, "plan", self.on_plan, 1)
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id, t.child_frame_id = "map", "odom"
        t.transform.rotation.w = 1.0
        self.static_tf.sendTransform(t)
        self.dt = 0.02
        self.last = self.now()
        self.create_timer(self.dt, self.step)

    def now(self):
        return (self.get_clock().now() - self.t0).nanoseconds * 1e-9

    def on_plan(self, msg):
        if len(msg.poses) >= 2:
            self.plan = np.array([(p.pose.position.x, p.pose.position.y) for p in msg.poses])

    def cte(self):
        if self.plan is None:
            return None
        x, y, _ = self.g1.published()
        a, b = self.plan[:-1], self.plan[1:]
        ab = b - a
        l2 = np.maximum((ab ** 2).sum(1), 1e-9)
        q = np.array([x, y])
        u = np.clip(((q - a) * ab).sum(1) / l2, 0, 1)
        d = np.hypot(*(q - (a + ab * u[:, None])).T)
        k = int(np.argmin(d))
        cr = ab[k, 0] * (q[1] - a[k, 1]) - ab[k, 1] * (q[0] - a[k, 0])
        return math.copysign(d[k], cr)

    def on_cmd(self, msg):
        vx, vy, wz = self.bridge(msg.linear.x, msg.angular.z, cte=self.cte())
        self.g1.command(self.g1.t, vx, vy, wz, duration=0.5)
        s = Twist()
        s.linear.x, s.linear.y, s.angular.z = vx, vy, wz
        self.sent_pub.publish(s)

    def pose_msg(self, stamp, x, y, yaw, frame="map"):
        p = PoseStamped()
        p.header.stamp, p.header.frame_id = stamp, frame
        p.pose.position.x, p.pose.position.y = x, y
        _, _, p.pose.orientation.z, p.pose.orientation.w = quat(yaw)
        return p

    def step(self):
        # 按墙钟推进模型（定时器抖动时补步，保持实时）
        now = self.now()
        while self.g1.t + self.dt <= now:
            self.g1.step(self.dt)
        stamp = self.get_clock().now().to_msg()
        px, py, pyaw = self.g1.published()
        tx, ty, tyaw = self.g1.truth()
        t = TransformStamped()
        t.header.stamp = stamp
        t.header.frame_id, t.child_frame_id = "odom", "base_link"
        t.transform.translation.x, t.transform.translation.y = px, py
        _, _, t.transform.rotation.z, t.transform.rotation.w = quat(pyaw)
        self.tf.sendTransform(t)
        o = Odometry()
        o.header.stamp = stamp
        o.header.frame_id, o.child_frame_id = "odom", "base_link"
        o.pose.pose = self.pose_msg(stamp, px, py, pyaw).pose
        o.twist.twist.linear.x, o.twist.twist.angular.z = self.g1.v, self.g1.w
        self.odom_pub.publish(o)
        self.truth_pub.publish(self.pose_msg(stamp, tx, ty, tyaw))
        self.pubpose_pub.publish(self.pose_msg(stamp, px, py, pyaw))


def main():
    rclpy.init()
    node = G1OrinSim()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
