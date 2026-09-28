#!/usr/bin/env python3
"""近处"会被标记成障碍"的点是哪来的 —— 按传感器几何 + 时间稳定性取证。

    python3 near_obstacle_forensics.py <bag> [--t0 S] [--t1 S] [--radius M]

对每帧 /lightning/registered_scan，挑出**会被 obstacle_layer 标记**的点
（map 系 z∈[min_h,1.8]、传感器距离>obstacle_min_range、水平距离<radius），然后分两个维度看：

  几何：传感器距离 / 俯仰角(负=向下看) / 方位角 —— 认出"打在自己身上"“扫到地面”等
  时间：按 5 cm 世界格聚合，看同一个格子在连续多少帧里出现 ——
        真障碍每帧都在；噪点只出现 1~2 帧。

输出最后给出"噪点占比"和噪点的几何画像，用来决定该加哪种过滤。
"""
import argparse, math, sys
from collections import defaultdict
import numpy as np
import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('bag')
    ap.add_argument('--t0', type=float, default=60.0)
    ap.add_argument('--t1', type=float, default=180.0)
    ap.add_argument('--radius', type=float, default=1.5, help='只看 base_link 这个水平距离内')
    ap.add_argument('--min-h', type=float, default=0.10)
    ap.add_argument('--min-range', type=float, default=0.25)
    ap.add_argument('--topic', default='/lightning/registered_scan')
    args = ap.parse_args()

    import tf2_ros, rclpy.time, rclpy.duration, sensor_msgs_py.point_cloud2 as pc2
    buf = tf2_ros.Buffer(cache_time=rclpy.duration.Duration(seconds=60))
    want = [args.topic, '/base_link_pose', '/tf', '/tf_static']
    r = rosbag2_py.SequentialReader()
    r.open(rosbag2_py.StorageOptions(uri=args.bag, storage_id='mcap'),
           rosbag2_py.ConverterOptions('', ''))
    types = {t.name: t.type for t in r.get_all_topics_and_types()}
    r.set_filter(rosbag2_py.StorageFilter(topics=[t for t in want if t in types]))
    cls = {t: get_message(types[t]) for t in want if t in types}

    t0 = None
    scans, poses = [], []
    while r.has_next():
        tp, data, stamp = r.read_next()
        t = stamp / 1e9
        if t0 is None:
            t0 = t
        rel = t - t0
        if rel > args.t1 + 1:
            break
        m = deserialize_message(data, cls[tp])
        if tp == '/tf_static':
            for tr in m.transforms:
                buf.set_transform_static(tr, 'bag')
        elif tp == '/tf':
            for tr in m.transforms:
                buf.set_transform(tr, 'bag')
        elif rel < args.t0:
            continue
        elif tp == args.topic:
            scans.append((rel, m))
        elif tp == '/base_link_pose':
            q = m.pose.orientation
            yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y ** 2 + q.z ** 2))
            poses.append((rel, m.pose.position.x, m.pose.position.y, yaw))
    P = np.array(poses)
    print(f'{args.topic}: {len(scans)} 帧, 窗口 {args.t0}~{args.t1}s, 半径 {args.radius} m')
    if not scans:
        return 1
    fields = [f.name for f in scans[0][1].fields]
    print('点云字段:', fields)

    def pose_at(t):
        i = min(max(int(np.searchsorted(P[:, 0], t)), 0), len(P) - 1)
        return P[i, 1], P[i, 2], P[i, 3]

    seen = defaultdict(list)      # 世界格键 -> [帧号]
    rec = []                      # 每个被标记点的 (帧号, 键, range, elev, azim(相对机头), z)
    nframe = 0
    for k, (t, sm) in enumerate(scans):
        try:
            tr = buf.lookup_transform('map', sm.header.frame_id,
                                      rclpy.time.Time.from_msg(sm.header.stamp))
        except Exception:
            continue
        q = tr.transform.rotation; tv = tr.transform.translation
        w_, x_, y_, z_ = q.w, q.x, q.y, q.z
        R = np.array([[1-2*(y_*y_+z_*z_), 2*(x_*y_-z_*w_), 2*(x_*z_+y_*w_)],
                      [2*(x_*y_+z_*w_), 1-2*(x_*x_+z_*z_), 2*(y_*z_-x_*w_)],
                      [2*(x_*z_-y_*w_), 2*(y_*z_+x_*w_), 1-2*(x_*x_+y_*y_)]])
        a = pc2.read_points(sm, field_names=('x', 'y', 'z'), skip_nans=True)
        p = np.stack([a['x'], a['y'], a['z']], -1).astype(float)
        rng = np.linalg.norm(p, axis=1)
        elev = np.degrees(np.arcsin(np.clip(p[:, 2] / np.maximum(rng, 1e-6), -1, 1)))
        azim = np.degrees(np.arctan2(p[:, 1], p[:, 0]))
        pm = (R @ p.T).T + np.array([tv.x, tv.y, tv.z])
        cx, cy, yaw = pose_at(t)
        d = np.hypot(pm[:, 0] - cx, pm[:, 1] - cy)
        sel = (pm[:, 2] > args.min_h) & (pm[:, 2] < 1.8) & (rng > args.min_range) & (d < args.radius)
        if not sel.any():
            nframe += 1
            continue
        gx = np.round(pm[sel, 0] / 0.05).astype(np.int64)
        gy = np.round(pm[sel, 1] / 0.05).astype(np.int64)
        for i, (a_, b_) in enumerate(zip(gx.tolist(), gy.tolist())):
            seen[(a_, b_)].append(nframe)
        idx = np.nonzero(sel)[0]
        for i, ii in enumerate(idx):
            rec.append((nframe, (gx[i], gy[i]), rng[ii], elev[ii], azim[ii], pm[ii, 2], d[ii]))
        nframe += 1

    if not rec:
        print('窗口内没有会被标记的近处点'); return 0
    A = np.array([(f, rg, el, az, z, dd) for f, _, rg, el, az, z, dd in rec])
    keys = [k for _, k, *_ in rec]
    # 每个格子出现的帧数 -> 稳定性
    nhit = {k: len(set(v)) for k, v in seen.items()}
    stable = np.array([nhit[k] for k in keys])
    print(f'\n被标记的近处点总数 {len(A)}，落在 {len(seen)} 个 5cm 格里，扫描帧 {nframe}')
    print('\n=== 按"该格被几帧看到"分档 ===')
    for lo, hi, name in [(1, 1, '只出现1帧(噪点)'), (2, 3, '2~3帧'), (4, 10, '4~10帧'),
                         (11, 10 ** 9, '>10帧(稳定障碍)')]:
        s = (stable >= lo) & (stable <= hi)
        ncell = sum(1 for k, n in nhit.items() if lo <= n <= hi)
        if s.sum():
            print(f'  {name:18s} 点{int(s.sum()):7d} ({100*s.sum()/len(A):4.1f}%) 格{ncell:6d}  '
                  f'传感器距离 p50={np.median(A[s,1]):.2f}m  俯仰 p10/p50/p90='
                  f'{np.percentile(A[s,2],10):+.0f}/{np.percentile(A[s,2],50):+.0f}/{np.percentile(A[s,2],90):+.0f}°  '
                  f'map高度 p50={np.median(A[s,4]):.2f}m  离机器人 p50={np.median(A[s,5]):.2f}m')
    noise = stable <= 2
    print(f'\n=== 噪点(<=2帧) 的几何画像  n={int(noise.sum())} ===')
    if noise.sum():
        N = A[noise]
        for name, col, bins in [('传感器距离(m)', 1, [0.25, 0.4, 0.6, 0.8, 1.0, 1.5, 2.0, 99]),
                                ('俯仰角(度,负=向下)', 2, [-90, -60, -40, -25, -15, -7, 0, 15, 90]),
                                ('map高度(m)', 4, [0.1, 0.2, 0.4, 0.7, 1.0, 1.3, 1.8])]:
            h, _ = np.histogram(N[:, col], bins=bins)
            print(f'  {name}: ' + '  '.join(f'[{bins[i]:g},{bins[i+1]:g})={h[i]}'
                                            for i in range(len(h)) if h[i]))
        az = np.array([a for (_, _, _, _, a, _, _) in rec])[noise]
        hh, _ = np.histogram(az, bins=[-180, -135, -90, -45, 0, 45, 90, 135, 180])
        print('  传感器方位角(度,0=正前): ' + '  '.join(
            f'[{b},{b+45})={hh[i]}' for i, b in enumerate(range(-180, 180, 45)) if hh[i]))
    return 0


if __name__ == '__main__':
    sys.exit(main())
