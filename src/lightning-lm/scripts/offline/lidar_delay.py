#!/usr/bin/env python3
"""把 /livox/lidar_raw 延迟固定秒数后原样转发到 /livox/lidar（转发序列化字节，不改 stamp），
复现实车点云传输滞后（2026-09-19 实测到达定位时落后 1.8~2.3 s，IMU 基本不滞后）。
用法: lidar_delay.py <延迟秒> [附加随机抖动秒]
"""
import collections
import random
import sys
import threading
import time

import rclpy
from livox_ros_driver2.msg import CustomMsg


def main():
    delay = float(sys.argv[1])
    jitter = float(sys.argv[2]) if len(sys.argv) > 2 else 0.0
    rclpy.init()
    node = rclpy.create_node('lidar_delay')
    pub = node.create_publisher(CustomMsg, '/livox/lidar', 10)
    queue = collections.deque()
    cv = threading.Condition()
    last_due = [0.0]
    stop = threading.Event()

    def on_msg(data):
        with cv:
            # 保持先后顺序：抖动只推迟、不让后到的帧越过前一帧
            due = max(last_due[0], time.monotonic() + delay + random.uniform(0.0, jitter))
            last_due[0] = due
            queue.append((due, data))
            cv.notify()

    def sender():
        while not stop.is_set():
            with cv:
                while not queue and not stop.is_set():
                    cv.wait(0.1)
                if not queue or stop.is_set():
                    continue
                due, data = queue[0]
                wait = due - time.monotonic()
                if wait > 0:
                    cv.wait(wait)
                    continue
                queue.popleft()
            pub.publish(data)

    node.create_subscription(CustomMsg, '/livox/lidar_raw', on_msg, 100, raw=True)
    th = threading.Thread(target=sender, daemon=True)
    th.start()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    # 先停发送线程再销毁节点：退出时线程仍在 publish 会在解释器收尾阶段段错误
    stop.set()
    with cv:
        cv.notify()
    th.join(1.0)
    node.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()


if __name__ == '__main__':
    main()
