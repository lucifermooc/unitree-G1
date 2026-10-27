"""离线验证：从录包（sqlite3 db3，直接读，jazzy / humble 都行）取某时刻的 /livox/lidar + /livox/imu，
走和节点相同的流程（IMU 摆平 → 全图搜索 → 换成 base_link 位姿），并和包里当时 lightning 输出的 map->base_link 对比。

  offline_bag <bag 目录或 db3> <map_dir> --at 120 300 600   # 包内相对秒
  offline_bag <bag> <map_dir> --list-static                  # 列出 IMU 静止段
"""
import argparse
import glob
import math
import os
import sqlite3
import time

import numpy as np
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message

from . import bev_search
from .geom import lidar_pose_to_base, livox_to_xyz, quat_to_rot, yaw_diff_deg
from .pcd_io import load_pcd_xyz


class Bag:
    def __init__(self, path):
        if os.path.isdir(path):
            dbs = sorted(glob.glob(os.path.join(path, '*.db3')))
            if not dbs:
                raise FileNotFoundError('no .db3 in ' + path)
            path = dbs[0]
        self.con = sqlite3.connect(f'file:{path}?mode=ro', uri=True)
        self.topics = {name: (tid, typ) for tid, name, typ in
                       self.con.execute('select id, name, type from topics')}
        self.t0, self.t1 = self.con.execute('select min(timestamp), max(timestamp) from messages').fetchone()
        self._types = {}

    def _cls(self, typ):
        if typ not in self._types:
            self._types[typ] = get_message(typ)
        return self._types[typ]

    def read(self, topic, t_from=None, t_to=None, limit=None):
        """返回 [(bag 时间 ns, msg)]；t_from / t_to 是包内相对秒"""
        if topic not in self.topics:
            return []
        tid, typ = self.topics[topic]
        q = 'select timestamp, data from messages where topic_id = ?'
        args = [tid]
        if t_from is not None:
            q += ' and timestamp >= ?'
            args.append(self.t0 + int(t_from * 1e9))
        if t_to is not None:
            q += ' and timestamp <= ?'
            args.append(self.t0 + int(t_to * 1e9))
        q += ' order by timestamp'
        if limit:
            q += f' limit {int(limit)}'
        cls = self._cls(typ)
        return [(t, deserialize_message(bytes(d), cls)) for t, d in self.con.execute(q, args)]


def static_tf(bag, parent, child):
    for _, m in bag.read('/tf_static'):
        for tr in m.transforms:
            if tr.header.frame_id == parent and tr.child_frame_id == child:
                t, q = tr.transform.translation, tr.transform.rotation
                return quat_to_rot(q), np.array([t.x, t.y, t.z])
    return None


def tf_at(bag, parent, child, t_rel, span=0.5):
    best = None
    for t, m in bag.read('/tf', t_rel - span, t_rel + span):
        for tr in m.transforms:
            if tr.header.frame_id == parent and tr.child_frame_id == child:
                dt = abs((t - bag.t0) * 1e-9 - t_rel)
                if best is None or dt < best[0]:
                    R = quat_to_rot(tr.transform.rotation)
                    best = (dt, tr.transform.translation.x, tr.transform.translation.y, math.atan2(R[1, 0], R[0, 0]))
    return None if best is None else best[1:]


def list_static(bag, win=2.0, gyro_max=0.15):
    imu = bag.read('/livox/imu')
    ts = np.array([(t - bag.t0) * 1e-9 for t, _ in imu])
    g = np.array([np.linalg.norm([m.angular_velocity.x, m.angular_velocity.y, m.angular_velocity.z]) for _, m in imu])
    segs = []
    start = prev = None
    for t, still in zip(ts, g <= gyro_max):
        if still:
            start = t if start is None else start
            prev = t
        else:
            if start is not None and prev - start >= win:
                segs.append((start, prev))
            start = None
    if start is not None and prev - start >= win:
        segs.append((start, prev))
    return segs


def run_at(bag, mi, sp, t_rel, n_frames, ext, frames):
    scans = bag.read('/livox/lidar', t_rel, t_rel + 3.0, limit=n_frames)
    if len(scans) < n_frames:
        return dict(t=t_rel, error=f'only {len(scans)} scans')
    a = (scans[0][0] - bag.t0) * 1e-9 - 0.1
    b = (scans[-1][0] - bag.t0) * 1e-9 + 0.05
    imu = bag.read('/livox/imu', a, b)
    acc = np.array([[m.linear_acceleration.x, m.linear_acceleration.y, m.linear_acceleration.z] for _, m in imu])
    gyr = np.array([[m.angular_velocity.x, m.angular_velocity.y, m.angular_velocity.z] for _, m in imu])
    R_level = bev_search.rot_to_up(acc.mean(0))
    pts = np.concatenate([livox_to_xyz(m) for _, m in scans]) @ R_level.T
    t0 = time.time()
    res = bev_search.search(mi, pts, sp)
    out = dict(t=round(t_rel, 1), gyro_max=round(float(np.linalg.norm(gyr, axis=1).max()), 3),
               search_s=round(time.time() - t0, 2), **res.summary())
    if res.best is not None:
        p = mi.R_level.T @ np.array([res.best.x, res.best.y, 0.0])
        bx, by, byaw = lidar_pose_to_base(p[0], p[1], res.best.yaw, R_level, *ext)
        out['target_base'] = (round(bx, 3), round(by, 3), round(math.degrees(byaw), 2))
        cur = tf_at(bag, frames[0], frames[1], b)
        if cur is not None:
            out['bag_base'] = (round(cur[0], 3), round(cur[1], 3), round(math.degrees(cur[2]), 2))
            out['diff'] = (round(math.hypot(cur[0] - bx, cur[1] - by), 3), round(yaw_diff_deg(cur[2], byaw), 2))
    return out


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('bag')
    ap.add_argument('map_dir')
    ap.add_argument('--at', type=float, nargs='*', default=[])
    ap.add_argument('--list-static', action='store_true')
    ap.add_argument('--n-frames', type=int, default=10)
    ap.add_argument('--set', nargs='*', default=[], help='搜索参数 key=value')
    a = ap.parse_args()
    bag = Bag(a.bag)
    print(f'bag duration {(bag.t1 - bag.t0) * 1e-9:.1f} s, topics: {sorted(bag.topics)}')
    if a.list_static:
        for s, e in list_static(bag):
            print(f'  static {s:8.1f} .. {e:8.1f}  ({e - s:.1f} s)')
        if not a.at:
            return
    sp = bev_search.SearchParams()
    for kv in a.set:
        k, v = kv.split('=')
        setattr(sp, k, type(getattr(sp, k))(v))
    up = (0.0, 0.0, 1.0)
    gu = os.path.join(a.map_dir, 'gravity_up.txt')
    if os.path.isfile(gu):
        up = tuple(float(v) for v in open(gu).read().split()[:3])
    mi = bev_search.MapIndex(load_pcd_xyz(os.path.join(a.map_dir, 'global.pcd')), sp, up)
    ext = static_tf(bag, 'body_link', 'mid360_link')
    if ext is None:
        raise SystemExit('no body_link->mid360_link in /tf_static')
    for t in a.at:
        print(run_at(bag, mi, sp, t, a.n_frames, ext, ('map', 'base_link')), flush=True)


if __name__ == '__main__':
    main()
