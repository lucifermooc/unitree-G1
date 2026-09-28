"""建图时自动开车：根据激光雷达避障，在场地里到处走，代替人手动遥控。

用法：python auto_explore.py --duration 240   （仿真时间，秒）
真机上建图一般是人用手柄/键盘遥控，不需要这个脚本。
"""
import argparse
import math

import rclpy
from geometry_msgs.msg import TwistStamped
from rclpy.node import Node
from sensor_msgs.msg import LaserScan

FORWARD_SPEED = 0.18   # m/s
TURN_SPEED = 0.9       # rad/s
SAFE_DIST = 0.45       # 前方小于这个距离就原地转向


class AutoExplore(Node):
    def __init__(self, duration):
        super().__init__('auto_explore', parameter_overrides=[
            rclpy.parameter.Parameter('use_sim_time', value=True)])
        self.duration = duration
        self.start = None
        self.scan = None
        self.turn_dir = 1.0
        self.turning = False
        self.pub = self.create_publisher(TwistStamped, 'cmd_vel', 10)
        self.create_subscription(LaserScan, 'scan', self.on_scan, 10)
        self.create_timer(0.1, self.step)

    def on_scan(self, msg):
        self.scan = msg

    def sector_min(self, center_deg, half_width_deg):
        """某个方向扇区内的最近距离（0°=正前方，逆时针为正）。"""
        s = self.scan
        values = []
        for i, r in enumerate(s.ranges):
            angle = math.degrees(s.angle_min + i * s.angle_increment)
            diff = (angle - center_deg + 180) % 360 - 180
            if abs(diff) <= half_width_deg:
                values.append(r if math.isfinite(r) and r > s.range_min else s.range_max)
        return min(values) if values else s.range_max

    def step(self):
        now = self.get_clock().now()
        if self.scan is None or now.nanoseconds == 0:
            return
        if self.start is None:
            self.start = now
        elapsed = (now - self.start).nanoseconds / 1e9
        cmd = TwistStamped()
        cmd.header.stamp = now.to_msg()
        if elapsed > self.duration:
            self.pub.publish(cmd)  # 停车
            self.get_logger().info('探索结束')
            raise SystemExit

        front = self.sector_min(0, 30)
        left = self.sector_min(60, 30)
        right = self.sector_min(-60, 30)
        if front < SAFE_DIST:
            if not self.turning:  # 刚开始转时决定方向，转完之前不改，避免左右抖动
                self.turn_dir = 1.0 if left >= right else -1.0
                self.turning = True
            cmd.twist.angular.z = TURN_SPEED * self.turn_dir
        else:
            self.turning = False
            cmd.twist.linear.x = FORWARD_SPEED
            # 往更空旷的一侧稍微偏，走得更分散
            cmd.twist.angular.z = max(-0.5, min(0.5, 0.6 * (left - right)))
        self.pub.publish(cmd)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--duration', type=float, default=240.0)
    args = parser.parse_args()
    rclpy.init()
    node = AutoExplore(args.duration)
    try:
        rclpy.spin(node)
    except SystemExit:
        pass
    node.destroy_node()
    rclpy.try_shutdown()


if __name__ == '__main__':
    main()
