#!/usr/bin/env python3
"""幻影 lethal 格是"哪一帧点云"画上去的？—— 支撑率 vs 时间偏移 Δt 曲线。

    python3 phantom_origin_probe.py <bag> [--t0 S] [--t1 S] [--radius M]

做法：取每一帧 costmap 里**新出现**的 lethal 格，再取 t+Δt 时刻的 registered_scan，
把点云用 bag 里的 TF 变到 map 系、按同一套 5 cm 栅格取整，看新格是否命中点所在格
（允许 ±1 格容差）。扫一串 Δt：

  * 峰值在 Δt≈0        -> 标记来自当帧点云，噪声是传感器/算法真报出来的
  * 峰值在 Δt<0(负)    -> costmap 画的是 |Δt| 秒前的点云 = 点云到达滞后(DDS/队列)
  * 处处都低           -> 这些格子不是任何一帧的直接标记（陈旧残影，清除失败）
"""
import argparse, math, sys
import numpy as np
import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message

C_RAW = '/local_costmap/costmap_raw'
SCAN = '/lightning/registered_scan'
POSE = '/base_link_pose'


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('bag')
    ap.add_argument('--t0', type=float, default=60.0)
    ap.add_argument('--t1', type=float, default=180.0)
    ap.add_argument('--radius', type=float, default=2.0)
    ap.add_argument('--stride', type=int, default=5, help='每隔几帧 costmap 检查一次')
    ap.add_argument('--dts', default='-3,-2.5,-2,-1.5,-1,-0.6,-0.3,-0.1,0,0.1,0.3,0.6,1')
    args = ap.parse_args()
    DTS = [float(x) for x in args.dts.split(',')]

    import tf2_ros, rclpy.time, rclpy.duration
    import sensor_msgs_py.point_cloud2 as pc2
    buf = tf2_ros.Buffer(cache_time=rclpy.duration.Duration(seconds=60))

    r = rosbag2_py.SequentialReader()
    r.open(rosbag2_py.StorageOptions(uri=args.bag, storage_id='mcap'),
           rosbag2_py.ConverterOptions('', ''))
    types = {t.name: t.type for t in r.get_all_topics_and_types()}
    want = [C_RAW, SCAN, POSE, '/tf', '/tf_static']
    r.set_filter(rosbag2_py.StorageFilter(topics=[t for t in want if t in types]))
    cls = {t: get_message(types[t]) for t in want if t in types}

    t_start = None
    frames, scans, poses = [], [], []
    while r.has_next():
        topic, data, stamp = r.read_next()
        t = stamp / 1e9
        if t_start is None:
            t_start = t
        rel = t - t_start
        if rel > args.t1 + 2:
            break
        m = deserialize_message(data, cls[topic])
        if topic == '/tf_static':
            for tr in m.transforms:
                buf.set_transform_static(tr, 'bag')
            continue
        if topic == '/tf':
            for tr in m.transforms:
                buf.set_transform(tr, 'bag')
            continue
        if rel < args.t0 - 4:
            continue
        if topic == C_RAW:
            frames.append((rel, m))
        elif topic == SCAN:
            scans.append((rel, m))
        elif topic == POSE:
            q = m.pose.orientation
            poses.append((rel, m.pose.position.x, m.pose.position.y))
    P = np.array(poses)
    st = np.array([s[0] for s in scans])
    print(f'costmap帧 {len(frames)}  scan帧 {len(scans)}  (TF cache 已载入)')

    def pose_at(t):
        i = min(max(int(np.searchsorted(P[:, 0], t)), 0), len(P) - 1)
        return P[i, 1], P[i, 2]

    def keys(msg, cx, cy):
        md = msg.metadata
        res, w, h = md.resolution, md.size_x, md.size_y
        ox, oy = md.origin.position.x, md.origin.position.y
        a = np.frombuffer(bytes(msg.data), dtype=np.uint8).reshape(h, w)
        ys, xs = np.nonzero(a == 254)
        wx = ox + (xs + 0.5) * res
        wy = oy + (ys + 0.5) * res
        sel = np.hypot(wx - cx, wy - cy) <= args.radius
        return set(zip(np.round(wx[sel] / res).astype(np.int64).tolist(),
                       np.round(wy[sel] / res).astype(np.int64).tolist())), res

    def scan_keys(sm, res):
        """点云 -> map 系 5 cm 栅格键集合（含 ±1 格膨胀，容忍取整边界）。"""
        try:
            tr = buf.lookup_transform('map', sm.header.frame_id,
                                      rclpy.time.Time.from_msg(sm.header.stamp))
        except Exception:
            return None
        q = tr.transform.rotation; tv = tr.transform.translation
        w_, x_, y_, z_ = q.w, q.x, q.y, q.z
        R = np.array([[1-2*(y_*y_+z_*z_), 2*(x_*y_-z_*w_), 2*(x_*z_+y_*w_)],
                      [2*(x_*y_+z_*w_), 1-2*(x_*x_+z_*z_), 2*(y_*z_-x_*w_)],
                      [2*(x_*z_-y_*w_), 2*(y_*z_+x_*w_), 1-2*(x_*x_+y_*y_)]])
        a = pc2.read_points(sm, field_names=('x', 'y', 'z'), skip_nans=True)
        p = np.stack([a['x'], a['y'], a['z']], -1).astype(float)
        rng = np.linalg.norm(p, axis=1)
        p = (R @ p.T).T + np.array([tv.x, tv.y, tv.z])
        sel = (p[:, 2] > 0.10) & (p[:, 2] < 1.8) & (rng > 0.25)   # 同 costmap 的高度/自身点过滤
        gx = np.round(p[sel, 0] / res).astype(np.int64)
        gy = np.round(p[sel, 1] / res).astype(np.int64)
        base = set(zip(gx.tolist(), gy.tolist()))
        out = set()
        for dx in (-1, 0, 1):
            for dy in (-1, 0, 1):
                out |= {(a_ + dx, b_ + dy) for a_, b_ in base}
        return out

    cache = {}
    hits = {d: [0, 0] for d in DTS}
    prev = set()
    n_used = 0
    for idx, (t, m) in enumerate(frames):
        cx, cy = pose_at(t)
        cells, res = keys(m, cx, cy)
        new = cells - prev
        prev = cells
        if t < args.t0 or t > args.t1 or idx % args.stride or not new:
            continue
        n_used += 1
        for d in DTS:
            j = int(np.argmin(np.abs(st - (t + d))))
            if abs(st[j] - (t + d)) > 0.15:
                continue
            if j not in cache:
                cache[j] = scan_keys(scans[j][1], res)
                if len(cache) > 80:
                    cache.pop(next(iter(cache)))
            sk = cache[j]
            if sk is None:
                continue
            hits[d][0] += len(new & sk)
            hits[d][1] += len(new)

    print(f'\n=== 新生 lethal 格的点云支撑率 vs Δt (基于 {n_used} 帧, 半径{args.radius}m) ===')
    print('  Δt(s)   支撑率     命中/总数')
    best = max(hits, key=lambda d: hits[d][0] / hits[d][1] if hits[d][1] else 0)
    for d in DTS:
        h, n = hits[d]
        if not n:
            print(f'  {d:+5.2f}      -(无对应scan)')
            continue
        bar = '#' * int(60 * h / n)
        print(f'  {d:+5.2f}   {100*h/n:5.1f}%   {h:7d}/{n:<7d} {bar}')
    if hits[best][1]:
        print(f'\n  峰值在 Δt={best:+.2f}s ({100*hits[best][0]/hits[best][1]:.1f}%)')
    return 0


if __name__ == '__main__':
    sys.exit(main())
