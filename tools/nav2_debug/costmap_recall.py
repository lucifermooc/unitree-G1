#!/usr/bin/env python3
"""代价地图召回 / 残影 / 误清 —— 给"加清除源"类改动做 A/B 用（local 与 global 都能用）。

    # 每个被测 bag：D435 高物体召回（分距离段）+ 正前方无支撑残影
    python3 costmap_recall.py <原始bag> <被测bag1> [被测bag2 ...] [--topic /costmap_raw]
    # 误清真障碍：base 有、new 没有的 254 格中，当时仍有传感器点支撑的比例（按高度/距离/方位切片）
    python3 costmap_recall.py <原始bag> <base> <new> --overclear

原始 bag 提供 D435 点云 /camera/camera/depth/color/points 与 MID360 /lightning/registered_scan；
被测 bag 是回放台输出（合成 costmap 在 /costmap_raw，--section global_costmap 时也是它），需含 /tf。
线上实录用 --topic /local_costmap/costmap_raw 或 /global_costmap/costmap_raw。

口径（2026-09-23 walkby_0923 local 回放用的就是这套）：
  高物体召回：D435 当前帧附近 0.2 s 内 z∈[0.7,1.8]、方位 ±43° 的点，≥3 点的格里有多少是 254。
  前方无支撑残影：方位 ±30°、0.3~2.5 m 的 254 格，0.3 s 内无 D435(z≥0.15) 点、0.35 s 内无 MID360 点支撑。
  误清：两个被测 bag 按最近时刻对齐（≤0.15 s）；"有支撑" = 0.35 s 内 ±7.5 cm 有 D435(z 0.15~1.8) 或 MID360(z 0.10~1.8) 点。
"""
import argparse, math
from collections import defaultdict
import numpy as np
import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
import tf2_ros, rclpy.time, rclpy.duration, sensor_msgs_py.point_cloud2 as pc2
from scipy.spatial import cKDTree

CAM = '/camera/camera/depth/color/points'
FOOTPRINT_R = 0.50   # robot_radius
LID = '/lightning/registered_scan'


def mat(tr):
    q = tr.transform.rotation; t = tr.transform.translation; w, x, y, z = q.w, q.x, q.y, q.z
    return (np.array([[1-2*(y*y+z*z), 2*(x*y-z*w), 2*(x*z+y*w)],
                      [2*(x*y+z*w), 1-2*(x*x+z*z), 2*(y*z-x*w)],
                      [2*(x*z-y*w), 2*(y*z+x*w), 1-2*(x*x+y*y)]]), np.array([t.x, t.y, t.z]))


def read(bag, topics):
    r = rosbag2_py.SequentialReader()
    r.open(rosbag2_py.StorageOptions(uri=bag, storage_id='mcap'), rosbag2_py.ConverterOptions('', ''))
    ty = {t.name: t.type for t in r.get_all_topics_and_types()}
    r.set_filter(rosbag2_py.StorageFilter(topics=[t for t in topics if t in ty]))
    while r.has_next():
        tp, d, _ = r.read_next()
        yield tp, deserialize_message(d, get_message(ty[tp]))


def stamp(m):
    return rclpy.time.Time.from_msg(m.header.stamp).nanoseconds / 1e9


def load_rep(bag, topic, skip):
    buf = tf2_ros.Buffer(cache_time=rclpy.duration.Duration(seconds=7200)); fr = []
    for tp, m in read(bag, ['/tf', '/tf_static', topic]):
        if tp == '/tf_static':
            for x in m.transforms: buf.set_transform_static(x, 'b')
        elif tp == '/tf':
            for x in m.transforms: buf.set_transform(x, 'b')
        else:
            fr.append(m)
    if fr:
        t0 = stamp(fr[0]); fr = [m for m in fr if stamp(m) >= t0 + skip]
    return buf, fr


def load_sensors(src, buf, t_lo, t_hi):
    """D435（~10 Hz，每 3 点取 1）与 MID360，转到 map 系，float32。"""
    cam, lid = [], []; n_cam = 0
    for tp, m in read(src, [CAM, LID]):
        t = stamp(m)
        if t < t_lo or t > t_hi: continue
        if tp == CAM:
            n_cam += 1
            if n_cam % 3: continue
        try:
            R, tv = mat(buf.lookup_transform('map', m.header.frame_id, m.header.stamp))
        except Exception:
            continue
        a = pc2.read_points(m, field_names=('x', 'y', 'z'), skip_nans=True)
        p = np.stack([a['x'], a['y'], a['z']], -1).astype(np.float64)
        if tp == CAM:
            q = (p[::3] @ R.T + tv).astype(np.float32); cam.append((t, q))
        else:
            p = p[np.linalg.norm(p, axis=1) > 0.25]; q = p @ R.T + tv
            lid.append((t, q[(q[:, 2] >= 0.10) & (q[:, 2] <= 1.8)].astype(np.float32)))
    return cam, lid


def window(lst, t, back, fwd=0.05):
    xs = [q for tt, q in lst if t - back <= tt <= t + fwd and len(q)]
    return np.vstack(xs) if xs else np.zeros((0, 3), np.float32)


def grid_cells(m, thr=254):
    a = np.array(m.data, dtype=np.uint8).reshape(m.metadata.size_y, m.metadata.size_x)
    ys, xs = np.nonzero(a == thr); r = m.metadata.resolution   # 只数 254：255 是未知格（track_unknown_space），不是障碍
    return a, np.c_[m.metadata.origin.position.x + (xs + .5) * r, m.metadata.origin.position.y + (ys + .5) * r]


def robot(buf, m):
    Rb, tb = mat(buf.lookup_transform('map', 'base_link', m.header.stamp))
    return tb, math.atan2(Rb[1, 0], Rb[0, 0])


def polar(P, tb, yaw):
    d = P[:, :2] - tb[:2]
    return np.hypot(d[:, 0], d[:, 1]), (np.degrees(np.arctan2(d[:, 1], d[:, 0]) - yaw) + 180) % 360 - 180


def recall_and_stale(name, buf, fr, cam, lid, lag=0.0):
    hit = defaultdict(int); tot = defaultdict(int); stale = []; selfm = []; self_t = []
    runs = []; cur = {}   # 无支撑格(世界系整数格) -> (首次, 最近) 时刻：同一格连续无支撑多久
    for m in fr:
        t = stamp(m)
        try: tb, yaw = robot(buf, m)
        except Exception: continue
        a, P = grid_cells(m)
        res = m.metadata.resolution; ox = m.metadata.origin.position.x; oy = m.metadata.origin.position.y
        Q = window(cam, t - lag, 0.2, 0.02)
        if len(Q):
            hd, br = polar(Q, tb, yaw)
            tall = (Q[:, 2] >= 0.7) & (Q[:, 2] <= 1.8) & (np.abs(br) <= 43)
            ix = ((Q[:, 0] - ox) / res).astype(int); iy = ((Q[:, 1] - oy) / res).astype(int)
            ok = (ix >= 0) & (iy >= 0) & (ix < a.shape[1]) & (iy < a.shape[0])
            for band, (lo, hi) in (('0.5~1.5m', (0.5, 1.5)), ('1.5~2.5m', (1.5, 2.5))):
                sel = tall & ok & (hd >= lo) & (hd < hi)
                if not sel.any(): continue
                cells, cnt = np.unique(np.c_[ix[sel], iy[sel]], axis=0, return_counts=True); cells = cells[cnt >= 3]
                tot[band] += len(cells); hit[band] += int((a[cells[:, 1], cells[:, 0]] == 254).sum())
        dist, brg = polar(P, tb, yaw)
        selfm.append(int((dist < FOOTPRINT_R).sum()))   # 足迹内的 254：多半是机身回波被标成了障碍
        if selfm[-1]: self_t.append(t)
        F = P[(np.abs(brg) <= 30) & (dist > 0.3) & (dist < 2.5)]
        sup = np.zeros(len(F), bool)
        if len(F):
            C = window(cam, t, 0.3, 0.02); C = C[C[:, 2] >= 0.15]; L = window(lid, t, 0.35)
            for S in (C, L):
                if len(S): sup |= cKDTree(S[:, :2]).query_ball_point(F, 0.075, return_length=True) > 0
        stale.append(int((~sup).sum()))
        nxt = {k: (cur.get(k, (t, t))[0], t) for k in map(tuple, np.floor(F[~sup] / m.metadata.resolution).astype(int))}
        runs += [tl - t0 for k, (t0, tl) in cur.items() if k not in nxt]
        cur = nxt
    runs += [tl - t0 for t0, tl in cur.values()]
    s = '  '.join(f'{k} 召回 {100*hit[k]/max(tot[k],1):5.1f}% ({hit[k]}/{tot[k]})' for k in ('0.5~1.5m', '1.5~2.5m'))
    print(f'{name:14s} 高物体(0.7~1.8m) {s} | 正前方无支撑残影 每帧均值 {np.mean(stale) if stale else 0:5.1f} '
          f'p90 {np.percentile(stale, 90) if stale else 0:.0f} | 足迹内({FOOTPRINT_R} m) 254 每帧均值 '
          f'{np.mean(selfm) if selfm else 0:.2f} 最大 {max(selfm) if selfm else 0}')
    if runs:
        print(f'{"":14s} 前方无支撑格连续存在 p90/p99/max = ' + '/'.join(f'{x:.1f}' for x in np.percentile(runs, [90, 99, 100])) + ' s')
    if self_t:
        segs = [[self_t[0], self_t[0]]]
        for x in self_t[1:]:
            if x - segs[-1][1] > 2: segs.append([x, x])
            else: segs[-1][1] = x
        t0 = stamp(fr[0])
        print(f'{"":14s} 足迹内有 254 的时段(相对首帧 s): ' + ' '.join(f'{a - t0:.0f}-{b - t0:.0f}' for a, b in segs[:12]))


def overclear(buf0, fr0, fr1, cam, lid, align=0.15):
    T1 = np.array([stamp(m) for m in fr1]); tot = 0; tot_fov = [0, 0]; lost = []; streak = defaultdict(int); longest = defaultdict(int)
    for m in fr0:
        t = stamp(m); j = int(np.argmin(np.abs(T1 - t)))
        if abs(T1[j] - t) > align: continue
        _, P0 = grid_cells(m); _, P1 = grid_cells(fr1[j])
        if len(P0) == 0: continue
        C = window(cam, t, 0.35); C = C[(C[:, 2] >= 0.15) & (C[:, 2] <= 1.8)]
        parts = [x for x in (C, window(lid, t, 0.35)) if len(x)]
        if not parts: continue
        S = np.vstack(parts)
        idx = cKDTree(S[:, :2]).query_ball_point(P0, 0.075)
        sup = np.array([len(i) > 0 for i in idx]); tot += int(sup.sum())
        try: tb, yaw = robot(buf0, m)
        except Exception: continue
        _, b0 = polar(P0, tb, yaw); infov = np.abs(b0) <= 43
        tot_fov[0] += int((sup & infov).sum()); tot_fov[1] += int((sup & ~infov).sum())
        # 用世界系整数格索引比对（格中心恰在 x.xx25/x.xx75，round() 与 np.round 取整方向可能不同）
        res = m.metadata.resolution
        in_new = set(map(tuple, np.floor(P1 / res).astype(int)))
        K0 = np.floor(P0 / res).astype(int)
        cur = set()
        for p, kk, s, ii in zip(P0, K0, sup, idx):
            key = (int(kk[0]), int(kk[1]))
            if s and key not in in_new:
                d, b = polar(p[None], tb, yaw)
                lost.append((S[ii, 2].min(), S[ii, 2].max(), d[0], b[0])); cur.add(key)
        for key in list(streak):
            if key not in cur: longest[key] = max(longest[key], streak.pop(key))
        for key in cur: streak[key] += 1
    for key, v in streak.items(): longest[key] = max(longest[key], v)
    print(f'误清：base 中有支撑的 254 格·帧 {tot}，new 中缺失 {len(lost)}（{100*len(lost)/max(tot,1):.2f}%）')
    if lost:
        a = np.array(lost); low = a[:, 0] < 0.7
        # D435 射线只可能擦到 ±43° 视场内的格；视场外的"缺失"是两次回放的相位/时序差 = 噪声底，
        # 视场内高出噪声底的部分才是清除源造成的误清
        fov = np.abs(a[:, 3]) <= 43
        for tag, k, t in (('D435 视场内 ±43°', fov, tot_fov[0]), ('视场外（噪声底）', ~fov, tot_fov[1])):
            print(f'  {tag}: 有支撑 {t} 格·帧，缺失 {k.sum()}（{100*k.sum()/max(t,1):.2f}%），'
                  f'其中矮物体 <0.7 m {int((k & low).sum())}（{100*(k & low).sum()/max(t,1):.2f}%）')
        for tag, k in (('支撑点最低 <0.7 m（矮物体）', low), ('支撑点最低 ≥0.7 m', ~low)):
            if k.any():
                print(f'  {tag}: {k.sum()} 格·帧  距离 p10/50/90 = ' + '/'.join(f'{x:.2f}' for x in np.percentile(a[k, 2], [10, 50, 90]))
                      + '  方位 p10/50/90 = ' + '/'.join(f'{x:.0f}' for x in np.percentile(a[k, 3], [10, 50, 90])) + '°')
        L = np.array(list(longest.values()))
        print(f'  同一格连续缺失帧数 p50/p90/max = {np.percentile(L,50):.0f}/{np.percentile(L,90):.0f}/{L.max()}（乘以 costmap 帧间隔得秒数）')


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('src'); ap.add_argument('reps', nargs='+')
    ap.add_argument('--topic', default='/costmap_raw')
    ap.add_argument('--skip', type=float, default=4.0, help='跳过开头冷启动秒数')
    ap.add_argument('--overclear', action='store_true', help='reps 恰为 base new 两个')
    ap.add_argument('--align', type=float, default=0.15,
                    help='--overclear 两个被测 bag 按最近时刻配对的容差。local(5 Hz) 0.15；global(1 Hz，两次回放相位随机) 用 0.6')
    ap.add_argument('--cam-lag', type=float, default=0.0,
                    help='召回只看 costmap 时刻再往前 lag 秒的 D435 帧。回放用 0；线上实录用 0.4 '
                         '（2026-09-23 实测 D435 采集→线上 costmap 发布 0.3~0.5 s，窗口只看 0.2 s 会得到 0%% 召回）')
    args = ap.parse_args()
    reps = [load_rep(b, args.topic, args.skip) for b in args.reps]
    buf0, fr0 = reps[0]
    if not fr0: raise SystemExit(f'{args.reps[0]} 里没有 {args.topic}')
    t_lo = stamp(fr0[0]) - 2; t_hi = stamp(fr0[-1]) + 1
    cam, lid = load_sensors(args.src, buf0, t_lo, t_hi)
    print(f'{args.topic}  窗口 {t_hi - t_lo:.0f}s  D435 帧 {len(cam)}  MID360 帧 {len(lid)}')
    if args.overclear:
        assert len(reps) == 2, '--overclear 需要恰好 base new 两个被测 bag'
        overclear(buf0, fr0, reps[1][1], cam, lid, args.align)
    else:
        for b, (buf, fr) in zip(args.reps, reps):
            recall_and_stale(b.rstrip('/').split('/')[-1], buf, fr, cam, lid, args.cam_lag)


if __name__ == '__main__':
    main()
