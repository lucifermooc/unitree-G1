#!/usr/bin/env python3
"""MID360 在 costmap 里的"噪点障碍"取证（实录 bag，需 rec_move.sh 录的话题）。

    python3 lidar_noise_probe.py <bag> [--skip 5]

A. 点：/lightning/registered_scan_nav（已剔除离雷达 0.25 m 内）转到 base_link（扫描时刻），高 0.10~1.8 m 的点
   按水平距离 / 方位 / 高度分布，运动与静止分开（base_link 平移 >0.08 m/s 或转角 >0.15 rad/s 算运动）。
B. 格：两张图 mid360_voxel_layer_raw 的 254 格离机器人多远、当下（0.35 s 内 ±7.5 cm）有没有雷达点支撑。
C. 来源：无支撑格往前追最后一次被雷达点打中的时刻，那一刻机器人离它多远（d_mark）。
   d_mark < robot_radius → 走动时身体附近的点被标进去，footprint 自清只遮当前足迹，走开后露出来（拖尾）。
"""
import argparse, math
from collections import defaultdict
import numpy as np
import sensor_msgs_py.point_cloud2 as pc2
from scipy.spatial import cKDTree
from costmap_recall import read, stamp, mat, load_rep, grid_cells, polar

NAV = '/lightning/registered_scan_nav'
BANDS = (0, 0.3, 0.4, 0.5, 0.6, 0.8, 1.0, 1.5, 2.0, 2.5, 99)


def pose_fn(buf):
    def f(t):
        import rclpy.time
        R, tv = mat(buf.lookup_transform('map', 'base_link', rclpy.time.Time(nanoseconds=int(t * 1e9))))
        return tv, math.atan2(R[1, 0], R[0, 0])
    return f


def moving(pose, t):
    try:
        (p0, y0), (p1, y1) = pose(t - 0.25), pose(t + 0.25)
    except Exception:
        return None
    dy = abs((y1 - y0 + math.pi) % (2 * math.pi) - math.pi)
    return np.hypot(*(p1 - p0)[:2]) / 0.5 > 0.08 or dy / 0.5 > 0.15


def hist(x, bins=BANDS, div=1):
    h = np.histogram(x, bins=bins)[0] / div
    return ' '.join(f'{bins[i]:g}-{bins[i + 1]:g}:{h[i]:.0f}' for i in range(len(h)) if h[i])


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('bag')
    ap.add_argument('--skip', type=float, default=5.0)
    ap.add_argument('--radius', type=float, default=0.50, help='robot_radius（footprint 自清半径）')
    a = ap.parse_args()
    buf, _ = load_rep(a.bag, '/__none__', 0)
    pose = pose_fn(buf)

    # ---------------- A. 点
    scans = []            # (t, map 系点, 机器人位置) 给 B/C 用
    st = {True: defaultdict(list), False: defaultdict(list)}; nfr = {True: 0, False: 0}
    t0 = None
    for _, m in read(a.bag, [NAV]):
        t = stamp(m); t0 = t if t0 is None else t0
        if t - t0 < a.skip: continue
        try:
            Rb, tb = mat(buf.lookup_transform('base_link', m.header.frame_id, m.header.stamp))
            Rm, tm = mat(buf.lookup_transform('map', m.header.frame_id, m.header.stamp))
        except Exception:
            continue
        p = pc2.read_points(m, field_names=('x', 'y', 'z'), skip_nans=True)
        p = np.stack([p['x'], p['y'], p['z']], -1).astype(np.float64)
        qb = p @ Rb.T + tb
        k = (qb[:, 2] >= 0.10) & (qb[:, 2] <= 1.8)
        qm = (p @ Rm.T + tm)[k]
        scans.append((t, qm.astype(np.float32)))
        mv = moving(pose, t)
        if mv is None: continue
        q = qb[k]; hb = np.hypot(q[:, 0], q[:, 1]); az = np.degrees(np.arctan2(q[:, 1], q[:, 0]))
        nfr[mv] += 1
        s = st[mv]
        s['hb'].append(hb); s['near_az'].append(az[hb < 0.6]); s['near_z'].append(q[hb < 0.6, 2])
        s['near_n'].append(int((hb < 0.6).sum()))
        s['az_all'].append(az[(hb >= 0.6) & (hb < 2.5)])
    print(f'== {a.bag.rstrip("/").split("/")[-1]}')
    print('A. registered_scan_nav 高 0.10~1.8 m 的点（base_link 系，扫描时刻）')
    for mv in (False, True):
        s = st[mv]
        if not nfr[mv]: continue
        hb = np.concatenate(s['hb'])
        print(f'  [{"运动" if mv else "静止"}] {nfr[mv]} 帧  每帧点数按水平距离: ' + hist(hb, div=nfr[mv]))
        print(f'     水平 <0.6 m 的点: 每帧均值 {np.mean(s["near_n"]):.1f}  p90 {np.percentile(s["near_n"], 90):.0f}'
              f'  {100 * np.mean(np.array(s["near_n"]) > 0):.0f}% 的帧有')
        naz = np.concatenate(s['near_az']); nz = np.concatenate(s['near_z'])
        if len(naz):
            print(f'     其方位(30°): {np.histogram(naz, bins=np.arange(-180, 181, 30))[0].tolist()}  (-180..180)')
            print(f'     其高度(0.1..1.8 /0.2): {np.histogram(nz, bins=np.arange(0.1, 1.81, 0.2))[0].tolist()}')
        az = np.concatenate(s['az_all'])
        h5 = np.histogram(az, bins=np.arange(-60, 61, 5))[0] / nfr[mv]
        print('     0.6~2.5 m 点的前方方位密度(每帧, -60..60 每 5°): ' + ' '.join(f'{x:.0f}' for x in h5))

    # ---------------- B/C. 格
    ts = np.array([x[0] for x in scans]); trees = {}
    def tree(i):
        if i not in trees: trees[i] = cKDTree(scans[i][1][:, :2]) if len(scans[i][1]) else None
        return trees[i]
    for cm, look in (('/local_costmap/mid360_voxel_layer_raw', 2.0), ('/global_costmap/mid360_voxel_layer_raw', 6.0),
                     ('/local_costmap/stvl_voxel_layer_raw', 0), ('/global_costmap/stvl_voxel_layer_raw', 0)):
        _, fr = load_rep(a.bag, cm, a.skip)
        if not fr: print(f'{cm}: 无'); continue
        rows = []; per = []; dmark = []
        for m in fr:
            t = stamp(m)
            try: tb, yaw = pose(t)
            except Exception: continue
            _, P = grid_cells(m)
            d, b = polar(P, tb, yaw)
            per.append(len(P))
            if not look or not len(P):
                rows += [(x, y, 1) for x, y in zip(d, b)]; continue
            j = np.searchsorted(ts, t + 0.05)
            sup = np.zeros(len(P), bool)
            for i in range(max(0, j - 50), j):
                if ts[i] >= t - 0.35 and tree(i) is not None:
                    sup |= tree(i).query_ball_point(P, 0.075, return_length=True) > 0
            rows += [(x, y, s) for x, y, s in zip(d, b, sup)]
            U = P[~sup]; last = np.full(len(U), -1.0)
            i = j - 1
            while i >= 0 and ts[i] >= t - look and (last < 0).any():
                if tree(i) is not None:
                    hit = (tree(i).query_ball_point(U, 0.075, return_length=True) > 0) & (last < 0)
                    last[hit] = ts[i]
                i -= 1
            for u, tl in zip(U, last):
                if tl < 0: dmark.append(np.nan); continue
                try: pb, _ = pose(tl)
                except Exception: continue
                dmark.append(np.hypot(*(u - pb[:2])))
        r = np.array(rows)
        print(f'B. {cm}  {len(per)} 帧  254/帧 均值 {np.mean(per):.0f}  p90 {np.percentile(per, 90):.0f}')
        print(f'   离机器人距离: {hist(r[:, 0])}  (格·帧)')
        if look:
            un = r[:, 2] == 0
            print(f'   无支撑 {100 * un.mean():.0f}%，其距离: {hist(r[un, 0])}')
            dm = np.array(dmark); ok = ~np.isnan(dm)
            if ok.any():
                print(f'C. 无支撑格最后被打中时离机器人 d_mark: {hist(dm[ok])}  ；{look:g} s 内找不到来源 {100 * (~ok).mean():.0f}%')
                print(f'   d_mark < {a.radius} m（走动时身体附近的点留下的拖尾）占 {100 * (dm[ok] < a.radius).mean():.0f}%')


if __name__ == '__main__':
    main()
