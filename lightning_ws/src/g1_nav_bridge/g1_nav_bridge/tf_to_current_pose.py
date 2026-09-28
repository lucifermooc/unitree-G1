#!/usr/bin/env python3
"""只读 map → base_link，发布定位 PoseStamped；不生成或修改 TF。"""
import math

import rclpy
from rclpy.node import Node
from rclpy.time import Time
from geometry_msgs.msg import PoseStamped
from tf2_ros import Buffer, TransformListener, TransformException


def to_pose(transform):
    """保留 TF 的参考系、原始时间戳和完整 6DoF 位姿，不重复乘外参。"""
    t, q = transform.transform.translation, transform.transform.rotation
    if not all(math.isfinite(v) for v in (t.x, t.y, t.z, q.x, q.y, q.z, q.w)):
        raise ValueError('TF 含非有限值')
    if q.x*q.x + q.y*q.y + q.z*q.z + q.w*q.w < 1e-12:
        raise ValueError('TF 四元数无效')
    msg = PoseStamped()
    msg.header = transform.header
    msg.pose.position.x, msg.pose.position.y, msg.pose.position.z = t.x, t.y, t.z
    msg.pose.orientation = q
    return msg


class TfToCurrentPose(Node):
    def __init__(self):
        super().__init__('g1_tf_to_current_pose')
        defaults = dict(global_frame='map', base_frame='base_link',
                        output_topic='/current_pose', rate_hz=50.0, max_tf_age=0.5)
        for key, value in defaults.items():
            self.declare_parameter(key, value)
        p = lambda key: self.get_parameter(key).value
        self.global_frame, self.base_frame = p('global_frame'), p('base_frame')
        rate, self.max_age = float(p('rate_hz')), float(p('max_tf_age'))
        if not all(math.isfinite(v) and v > 0 for v in (rate, self.max_age)):
            raise ValueError('rate_hz 和 max_tf_age 必须为正有限值')
        if not self.global_frame or not self.base_frame or self.global_frame == self.base_frame:
            raise ValueError('global_frame 和 base_frame 必须非空且不同')
        self.buffer = Buffer()
        self.listener = TransformListener(self.buffer, self)
        self.publisher = self.create_publisher(PoseStamped, p('output_topic'), 10)
        self.last_stamp = None
        self.create_timer(1.0/rate, self.publish_pose)

    def publish_pose(self):
        try:
            # timeout 默认为零；不会在定时回调中阻塞等待 TF。
            transform = self.buffer.lookup_transform(self.global_frame, self.base_frame, Time())
            stamp = Time.from_msg(transform.header.stamp).nanoseconds
            age = (self.get_clock().now().nanoseconds - stamp) / 1e9
            if age > self.max_age or age < -0.1:
                raise ValueError('定位 TF 过期或时间超前，请检查定位与时钟同步')
            if stamp == self.last_stamp:
                return  # 未更新的 TF 不伪装成新的定位结果。
            msg = to_pose(transform)
        except (TransformException, ValueError) as error:
            self.get_logger().warning(str(error), throttle_duration_sec=5.0)
            return
        self.publisher.publish(msg)
        self.last_stamp = stamp


def main(args=None):
    rclpy.init(args=args)
    node = TfToCurrentPose()
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
