#!/usr/bin/env python3
"""把 bag 里的 /tf_static 单独 latched 发出来，给离线回放用。

    python3 replay_tf_static.py <bag>

为什么需要：`ros2 bag play --start-offset N` 会跳过 N 秒之前的所有消息，
而 /tf_static 只在录制开头发一次 —— 于是回放里 body_link->mid360_link /
camera_depth_optical_frame 这些静态外参全没有，两个传感器的点云都会被
tf2 MessageFilter 丢掉（日志里是 "timestamp earlier than all the data in the
transform cache"），代价地图全空，A/B 结果全是 0。踩过一次，别再踩。
"""
import sys
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSHistoryPolicy
import rosbag2_py
from rclpy.serialization import deserialize_message
from tf2_msgs.msg import TFMessage


def main():
    bag = sys.argv[1]
    r = rosbag2_py.SequentialReader()
    r.open(rosbag2_py.StorageOptions(uri=bag, storage_id='mcap'),
           rosbag2_py.ConverterOptions('', ''))
    r.set_filter(rosbag2_py.StorageFilter(topics=['/tf_static']))
    trs = []
    while r.has_next():
        _, data, _ = r.read_next()
        trs.extend(deserialize_message(data, TFMessage).transforms)
    if not trs:
        print('bag 里没有 /tf_static'); return 1
    rclpy.init()
    n = Node('replay_tf_static')
    q = QoSProfile(depth=1, history=QoSHistoryPolicy.KEEP_LAST,
                   durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)
    pub = n.create_publisher(TFMessage, '/tf_static', q)
    msg = TFMessage(transforms=trs)
    pub.publish(msg)
    print(f'发布 {len(trs)} 条静态 TF: ' + ', '.join(
        f'{t.header.frame_id}->{t.child_frame_id}' for t in trs[:8]))
    # 保持存活，并每 2 s 重发一次（订阅者晚到也能拿到）
    n.create_timer(2.0, lambda: pub.publish(msg))
    rclpy.spin(n)
    return 0


if __name__ == '__main__':
    sys.exit(main())
