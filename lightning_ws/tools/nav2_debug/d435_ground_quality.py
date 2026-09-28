#!/usr/bin/env python3
"""D435 原始深度质量：地面平不平、空洞多不多、有没有打穿、假点是否稳定。

    python3 d435_ground_quality.py <bag> [--t0 S] [--t1 S] [--filtered]

用来做驱动深度滤波（realsense_depth_filters）的开/关对照：同一站位、机器人静止、
**正前方 2.5 m 内保持空地**，各录 60 s（record_nav_bag.sh --full），分别跑本脚本对比。
--filtered 改读 /camera/camera/depth/mark_points，看 d435_mark_filter 之后还剩多少假点。

所有量都在重力水平的 base_link 系（地面 z=0），按点云自己的 stamp 查 TF。
指标：
  有效点率      每帧点数 / 深度图像素数（空洞越多越低；需要 camera_info）
  地面带 z      水平距离 1.0~2.5 m、|z|<0.30 的点：std、p95|z|（地面不平）
  抬升率        地面带里 z>=0.15（会被 STVL 标记）的点占比
  打穿率        z<-0.10 的点占比（深度偏大，落到地面以下）
  掠射比值      z>=0.15 且射线下倾<30° 的点：实测距离 / 按地面应到距离（<1 = 深度被砍短）
  格子稳定性    z>=0.15 的 5 cm 格按出现帧占比分：闪烁(<50%) / 稳定(>=80%)
                —— 空地上"稳定"的格子就是稳定错深度，时间滤波和 decay 都治不了
  假点区        与 d435_mark_filter 候选同口径：离相机 >1.5 m、下倾 <30°、z∈[0.15,0.65)。
                已取证的假薄片（0.37~0.59 m）就在这里；空地上此区每帧点数与稳定格数都应≈0。
"""
import argparse
from collections import defaultdict, deque

import numpy as np
import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message


def mat(tr):
    q = tr.transform.rotation; tv = tr.transform.translation
    w, x, y, z = q.w, q.x, q.y, q.z
    R = np.array([[1-2*(y*y+z*z), 2*(x*y-z*w), 2*(x*z+y*w)],
                  [2*(x*y+z*w), 1-2*(x*x+z*z), 2*(y*z-x*w)],
                  [2*(x*z-y*w), 2*(y*z+x*w), 1-2*(x*x+y*y)]])
    return R, np.array([tv.x, tv.y, tv.z])


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('bag')
    ap.add_argument('--t0', type=float, default=0.0)
    ap.add_argument('--t1', type=float, default=1e9)
    ap.add_argument('--filtered', action='store_true')
    ap.add_argument('--base', default='base_link')
    args = ap.parse_args()

    import tf2_ros, rclpy.time, rclpy.duration, sensor_msgs_py.point_cloud2 as pc2
    buf = tf2_ros.Buffer(cache_time=rclpy.duration.Duration(seconds=3600))
    CAM = '/camera/camera/depth/mark_points' if args.filtered else '/camera/camera/depth/color/points'
    INFO = '/camera/camera/depth/camera_info'
    want = [CAM, INFO, '/tf', '/tf_static']
    r = rosbag2_py.SequentialReader()
    storage = 'mcap' if any(p.endswith('.mcap') for p in __import__('os').listdir(args.bag)) else 'sqlite3'
    r.open(rosbag2_py.StorageOptions(uri=args.bag, storage_id=storage),
           rosbag2_py.ConverterOptions('', ''))
    types = {t.name: t.type for t in r.get_all_topics_and_types()}
    if CAM not in types:
        raise SystemExit(f'bag 里没有 {CAM}')
    r.set_filter(rosbag2_py.StorageFilter(topics=[t for t in want if t in types]))
    cls = {t: get_message(types[t]) for t in want if t in types}

    npix = None
    pending = deque()          # 点云等 TF 到齐（0.3 s 后）再处理
    S = defaultdict(list)
    cell_frames = defaultdict(int)
    ghost_frames = defaultdict(int)
    nfr = 0; nfail = 0

    def process(m):
        nonlocal nfr, nfail
        try:
            tr = buf.lookup_transform(args.base, m.header.frame_id,
                                      rclpy.time.Time.from_msg(m.header.stamp))
        except Exception:
            nfail += 1; return
        R, tv = mat(tr)
        a = pc2.read_points(m, field_names=('x', 'y', 'z'), skip_nans=True)
        p = np.stack([a['x'], a['y'], a['z']], -1).astype(float)
        if len(p) == 0:
            return
        q = p @ R.T + tv
        nfr += 1
        if npix:
            S['valid'].append(len(p) / npix)
        hr = np.hypot(q[:, 0], q[:, 1])
        band = (hr > 1.0) & (hr < 2.5) & (np.abs(q[:, 2]) < 0.30)
        if band.any():
            S['z_std'].append(q[band, 2].std())
            S['z_p95'].append(np.percentile(np.abs(q[band, 2]), 95))
            S['lift'].append((q[band, 2] >= 0.15).mean())
        S['punch'].append((q[:, 2] < -0.10).mean())
        d = q - tv
        dh = np.hypot(d[:, 0], d[:, 1]); dz = -d[:, 2]
        down = np.arctan2(dz, dh)
        hi = q[:, 2] >= 0.15
        g = hi & (down > np.radians(2)) & (down < np.radians(30))
        if g.any():
            S['ratio'].extend((np.linalg.norm(d[g], axis=1) / (tv[2] / np.sin(down[g]))).tolist())
        for c in set(map(tuple, np.floor(q[hi & (hr < 2.5), :2] / 0.05).astype(int))):
            cell_frames[c] += 1
        gz = (q[:, 2] >= 0.15) & (q[:, 2] < 0.65) & (np.linalg.norm(d, axis=1) > 1.5) & \
             (down < np.radians(30))
        S['ghost_pts'].append(int(gz.sum()))
        for c in set(map(tuple, np.floor(q[gz, :2] / 0.05).astype(int))):
            ghost_frames[c] += 1

    while r.has_next():
        tp, data, stamp = r.read_next()
        t = stamp / 1e9
        m = deserialize_message(data, cls[tp])
        if tp == '/tf_static':
            for tr in m.transforms:
                buf.set_transform_static(tr, 'bag')
        elif tp == '/tf':
            for tr in m.transforms:
                buf.set_transform(tr, 'bag')
        elif tp == INFO:
            npix = m.width * m.height
        elif tp == CAM:
            pending.append((t, m))
        while pending and t - pending[0][0] > 0.3:
            tc, mc = pending.popleft()
            if not hasattr(main, 'T0'):
                main.T0 = tc
            if args.t0 <= tc - main.T0 <= args.t1:
                process(mc)
    for _, mc in pending:
        process(mc)

    def pct(k, qs=(50, 95)):
        v = np.asarray(S[k])
        return '  '.join(f'p{q}={np.percentile(v, q):.3f}' for q in qs) if len(v) else '无数据'

    print(f'{CAM}  帧数 {nfr}（TF 失败 {nfail}）')
    print(f'有效点率      {pct("valid", (5, 50)) if npix else "无 camera_info"}')
    print(f'地面带 z std  {pct("z_std")}')
    print(f'地面带 p95|z| {pct("z_p95")}')
    print(f'抬升率(>=.15) {pct("lift")}')
    print(f'打穿率(<-.10) {pct("punch")}')
    print(f'掠射比值      {pct("ratio", (5, 25, 50))}   (<0.8 的占 '
          f'{100*(np.asarray(S["ratio"]) < 0.8).mean() if S["ratio"] else 0:.1f}%)')
    print(f'假点区点数/帧 {pct("ghost_pts", (50, 95))}')
    if nfr and ghost_frames:
        occ = np.array(list(ghost_frames.values())) / nfr
        print(f'假点区格子    共 {len(occ)}  稳定(>=80%帧) {(occ >= 0.8).sum()}')
    if nfr and cell_frames:
        occ = np.array(list(cell_frames.values())) / nfr
        print(f'>=0.15m 格子  共 {len(occ)}  闪烁(<50%帧) {(occ < 0.5).sum()}  '
              f'稳定(>=80%帧) {(occ >= 0.8).sum()}')


if __name__ == '__main__':
    main()
