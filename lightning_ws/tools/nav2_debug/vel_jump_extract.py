#!/usr/bin/env python3
"""速度跳变分析，第 1 步（机器人容器里跑，读 mcap）：把速度链各级导出成 CSV。

  python3 vel_jump_extract.py <bag_dir> <out_dir>

导出（t = 录包接收时刻，epoch s）：
  cmd_vel_nav.csv   控制器输出            t vx wz
  cmd_vel.csv       velocity_smoother 输出 t vx wz
  sport.csv         桥发给 G1（/api/sport/request，LocoClient.SetVelocity） t api_id vx vy wz duration
  odommodestate.csv G1 实际（世界系速度 + 航向角速度） t vx_w vy_w yaw_speed yaw
  odom.csv          腿式里程计（机体系） t vx vy wz
  pose.csv          /base_link_pose     t x y yaw
  rosout.csv        controller / bt / behavior / 桥 的日志 t name level msg
  plan.csv          /plan 每次发布的时刻和点数 t n x0 y0 x1 y1
第 2 步在本机跑 vel_jump_analyze.py。
"""
import csv
import json
import math
import os
import sys

import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message

TOPICS = ["/cmd_vel_nav", "/cmd_vel", "/api/sport/request", "/odommodestate", "/odom", "/base_link_pose",
          "/rosout", "/plan"]
LOG_NODES = ("controller_server", "bt_navigator", "behavior_server", "velocity_smoother", "planner_server",
             "g1_cmdvel_to_sport", "cmdvel_to_sport")


def yaw_of(q):
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


def main():
    bag, out = sys.argv[1], sys.argv[2]
    os.makedirs(out, exist_ok=True)
    meta = rosbag2_py.Info().read_metadata(bag, "")
    r = rosbag2_py.SequentialReader()
    r.open(rosbag2_py.StorageOptions(uri=bag, storage_id=meta.storage_identifier), rosbag2_py.ConverterOptions("", ""))
    types = {t.name: t.type for t in r.get_all_topics_and_types()}
    use = [t for t in TOPICS if t in types]
    r.set_filter(rosbag2_py.StorageFilter(topics=use))
    cls = {t: get_message(types[t]) for t in use}
    names = {"/cmd_vel_nav": "cmd_vel_nav", "/cmd_vel": "cmd_vel", "/api/sport/request": "sport",
             "/odommodestate": "odommodestate", "/odom": "odom", "/base_link_pose": "pose", "/rosout": "rosout",
             "/plan": "plan"}
    files = {t: open(os.path.join(out, names[t] + ".csv"), "w", newline="") for t in use}
    w = {t: csv.writer(f) for t, f in files.items()}
    n = dict.fromkeys(use, 0)
    while r.has_next():
        topic, data, ts = r.read_next()
        m = deserialize_message(data, cls[topic])
        t = f"{ts * 1e-9:.4f}"
        if topic in ("/cmd_vel_nav", "/cmd_vel"):
            row = [t, f"{m.linear.x:.4f}", f"{m.angular.z:.4f}"]
        elif topic == "/api/sport/request":
            api = m.header.identity.api_id
            try:
                p = json.loads(m.parameter) if m.parameter else {}
            except ValueError:
                p = {}
            v = p.get("velocity", [float("nan")] * 3)
            row = [t, api, *(f"{x:.4f}" for x in v[:3]), p.get("duration", "")]
        elif topic == "/odommodestate":
            row = [t, f"{m.velocity[0]:.4f}", f"{m.velocity[1]:.4f}", f"{m.yaw_speed:.4f}", f"{m.imu_state.rpy[2]:.4f}"]
        elif topic == "/odom":
            tw = m.twist.twist
            row = [t, f"{tw.linear.x:.4f}", f"{tw.linear.y:.4f}", f"{tw.angular.z:.4f}"]
        elif topic == "/base_link_pose":
            p = m.pose.pose if hasattr(m.pose, "pose") else m.pose
            row = [t, f"{p.position.x:.4f}", f"{p.position.y:.4f}", f"{yaw_of(p.orientation):.4f}"]
        elif topic == "/rosout":
            if not any(k in m.name for k in LOG_NODES):
                continue
            row = [t, m.name, m.level, m.msg.replace("\n", " ")[:300]]
        else:  # /plan
            P = m.poses
            if not P:
                row = [t, 0, "", "", "", ""]
            else:
                a, b = P[0].pose.position, P[-1].pose.position
                row = [t, len(P), f"{a.x:.3f}", f"{a.y:.3f}", f"{b.x:.3f}", f"{b.y:.3f}"]
        w[topic].writerow(row)
        n[topic] += 1
    for f in files.values():
        f.close()
    print(bag, n)


if __name__ == "__main__":
    main()
