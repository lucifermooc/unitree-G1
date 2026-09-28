#!/usr/bin/env python3
"""在线回放时记录定位实际发布的 TF 与 /base_link_pose（每条带接收时刻）。用法: tfrec.py <输出目录>
输出:
  map_base.txt   stamp x y z qx qy qz qw recv    map->base_link
  base_body.txt  同上                             base_link->body_link
  static.txt     parent child x y z qx qy qz qw   /tf_static 全部
  blp.txt        stamp x y z qx qy qz qw recv    /base_link_pose
"""
import os
import sys
import time

import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from tf2_msgs.msg import TFMessage


def stamp(s):
    return s.sec + s.nanosec * 1e-9


def row(st, t, r, recv):
    return f'{st:.6f} {t.x:.6f} {t.y:.6f} {t.z:.6f} {r.x:.8f} {r.y:.8f} {r.z:.8f} {r.w:.8f} {recv:.6f}\n'


def main():
    out = sys.argv[1]
    os.makedirs(out, exist_ok=True)
    f = {k: open(os.path.join(out, k + '.txt'), 'w', buffering=1) for k in ('map_base', 'base_body', 'static', 'blp')}
    rclpy.init()
    node = rclpy.create_node('tfrec')

    def on_tf(msg):
        now = time.time()
        for tr in msg.transforms:
            key = {('map', 'base_link'): 'map_base', ('base_link', 'body_link'): 'base_body'}.get(
                (tr.header.frame_id, tr.child_frame_id))
            if key:
                f[key].write(row(stamp(tr.header.stamp), tr.transform.translation, tr.transform.rotation, now))

    def on_static(msg):
        for tr in msg.transforms:
            t, r = tr.transform.translation, tr.transform.rotation
            f['static'].write(f'{tr.header.frame_id} {tr.child_frame_id} {t.x} {t.y} {t.z} {r.x} {r.y} {r.z} {r.w}\n')

    def on_blp(msg):
        f['blp'].write(row(stamp(msg.header.stamp), msg.pose.position, msg.pose.orientation, time.time()))

    node.create_subscription(TFMessage, '/tf', on_tf, 1000)
    node.create_subscription(TFMessage, '/tf_static', on_static, QoSProfile(
        depth=100, durability=DurabilityPolicy.TRANSIENT_LOCAL, reliability=ReliabilityPolicy.RELIABLE))
    node.create_subscription(PoseStamped, '/base_link_pose', on_blp, 100)
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass


if __name__ == '__main__':
    main()
