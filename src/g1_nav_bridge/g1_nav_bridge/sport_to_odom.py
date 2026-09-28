#!/usr/bin/env python3
"""Read-only planar Unitree odometry adapter. Never broadcasts TF or commands."""
import math

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from nav_msgs.msg import Odometry
from unitree_go.msg import SportModeState


def convert_state(state, odom_frame='odom', base_frame='base_link'):
    """Planar approximation: boot-relative XY/yaw and body-frame XY/yaw velocity.

    Unitree quaternion order is w,x,y,z. Position Z and body_height are not
    interchangeable; neither is used for the ground-projected navigation frame.
    Covariances are intentionally supplied by the node, not inferred from data.
    """
    values = [*state.position, *state.velocity, *state.imu_state.quaternion, state.yaw_speed]
    if state.error_code or not all(math.isfinite(v) for v in values):
        raise ValueError('Unitree error_code or nonfinite odometry')
    sec, nsec = int(state.stamp.sec), int(state.stamp.nanosec)
    if sec < 0 or not 0 <= nsec < 1_000_000_000 or sec * 1_000_000_000 + nsec <= 0:
        raise ValueError('Invalid source timestamp')
    w, x, y, z = state.imu_state.quaternion
    norm = math.sqrt(w*w+x*x+y*y+z*z)
    if norm < 1e-6:
        raise ValueError('Invalid quaternion')
    w, x, y, z = (v/norm for v in (w, x, y, z))
    yaw = math.atan2(2*(w*z+x*y), 1-2*(y*y+z*z))
    out = Odometry()
    out.header.stamp.sec, out.header.stamp.nanosec = sec, nsec
    out.header.frame_id, out.child_frame_id = odom_frame, base_frame
    out.pose.pose.position.x = float(state.position[0])
    out.pose.pose.position.y = float(state.position[1])
    out.pose.pose.orientation.z = math.sin(yaw/2)
    out.pose.pose.orientation.w = math.cos(yaw/2)
    out.twist.twist.linear.x = float(state.velocity[0])
    out.twist.twist.linear.y = float(state.velocity[1])
    out.twist.twist.angular.z = float(state.yaw_speed)
    return out


class SportToOdom(Node):
    def __init__(self):
        super().__init__('g1_sport_to_odom')
        defaults = dict(input_topic='/odommodestate', output_topic='/odom',
                        odom_frame='odom', base_frame='base_link',
                        stale_timeout=0.5,
                        pose_covariance_diagonal=[0.1, 0.1, 1e6, 1e6, 1e6, 0.1],
                        twist_covariance_diagonal=[0.1, 0.1, 1e6, 1e6, 1e6, 0.1])
        for name, value in defaults.items():
            self.declare_parameter(name, value)
        p = lambda key: self.get_parameter(key).value
        self.odom_frame, self.base_frame = p('odom_frame'), p('base_frame')
        if not self.odom_frame or not self.base_frame or self.odom_frame == self.base_frame:
            raise ValueError('Distinct nonempty odom/base frame IDs required')
        self.pose_cov, self.twist_cov = p('pose_covariance_diagonal'), p('twist_covariance_diagonal')
        for diagonal in (self.pose_cov, self.twist_cov):
            if len(diagonal) != 6 or any(not math.isfinite(v) or v < 0 for v in diagonal):
                raise ValueError('Covariance diagonal must have six finite nonnegative values')
        self.timeout = p('stale_timeout')
        if not math.isfinite(self.timeout) or self.timeout <= 0:
            raise ValueError('stale_timeout must be positive')
        qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE,
                         durability=DurabilityPolicy.VOLATILE)
        self.publisher = self.create_publisher(Odometry, p('output_topic'), qos)
        self.subscription = self.create_subscription(SportModeState, p('input_topic'), self.on_state, qos)
        self.last_stamp = -1
        self.last_receive = None
        self.timer = self.create_timer(self.timeout, self.check_stale)

    def on_state(self, state):
        try:
            out = convert_state(state, self.odom_frame, self.base_frame)
            stamp = out.header.stamp.sec * 1_000_000_000 + out.header.stamp.nanosec
            if stamp <= self.last_stamp:
                raise ValueError('Nonincreasing source timestamp; restart adapter after robot clock reset')
        except ValueError as error:
            self.get_logger().warning(str(error), throttle_duration_sec=5.0)
            return
        for i in range(6):
            out.pose.covariance[i*7] = self.pose_cov[i]
            out.twist.covariance[i*7] = self.twist_cov[i]
        self.last_stamp = stamp
        self.last_receive = self.get_clock().now()
        self.publisher.publish(out)

    def check_stale(self):
        if self.last_receive is None or (self.get_clock().now()-self.last_receive).nanoseconds > self.timeout*1e9:
            self.get_logger().warning('No fresh Unitree odometry; no replacement/zero data published',
                                      throttle_duration_sec=5.0)


def main(args=None):
    rclpy.init(args=args)
    node = SportToOdom()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    # CMakeLists 用 install(PROGRAMS ...) 把本文件直接装成可执行，setup.py 的
    # console_scripts 入口点不生效，所以必须自带 __main__ 守卫；否则 launch 起它时
    # Python 从头跑到文件末尾就以 0 退出（"process has finished cleanly"，无日志），
    # 节点看似启动实则从未运行。2026-09-22 由此发现 /odom 一直没有发布者。
    main()
