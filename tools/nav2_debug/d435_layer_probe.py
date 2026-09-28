#!/usr/bin/env python3
"""D435 层（stvl_voxel_layer）的障碍从哪来、为什么清得慢。

    python3 d435_layer_probe.py <bag> [--layer /global_costmap/stvl_voxel_layer_raw]

bag 需含 /tf /tf_static /camera/camera/depth/color/points /camera/camera/depth/camera_info 和被测层。
按 STVL 的实际做法逐帧重放 D435 标记：点转到 map 系按高度裁剪，5 cm 体素每帧 ≥4 点，
再按离相机的 3D 距离裁剪（realsense_mark 0.15~1.8 m / 1.5 m 内；realsense_mark_tall 0.7~1.8 m / 2.5 m 内）。
A. 被标记的体素：按"整段录制里有多少帧标到它"分成稳定（真实物体）和偶发（噪点），看偶发体素的
   位置（离 base_link、方位、高度）和来源像素（深度图 u/v、深度）。
B. 图层里的 254 格：离机器人多远，最后一次被标记是多久前，是否还在清除视锥里
   （视锥里 ~1.15 s 清掉，视锥外要等 voxel_decay）。
"""
import argparse, math, sys
from collections import defaultdict
import numpy as np
import sensor_msgs_py.point_cloud2 as pc2
from costmap_recall import read, stamp, mat, load_rep, grid_cells, robot

CAM = '/camera/camera/depth/color/points'
INFO = '/camera/camera/depth/camera_info'
VS = 0.05
SRC = ((0.15, 1.8, 1.5), (0.70, 1.8, 2.5))     # (min_h, max_h, obstacle_range)
MINPTS = 4
HF, VF, ZMIN, ZMAX = 1.518, 1.012, 0.20, 2.5   # realsense_clear 视锥（global）


def marks(p, R, t):
    """返回本帧被标记体素 {STVL 索引: 光学系质心}。STVL 用 Coord(double) 截断取整。"""
    q = p @ R.T + t
    out = {}
    for hmin, hmax, rng in SRC:
        k = (q[:, 2] >= hmin) & (q[:, 2] <= hmax)
        if not k.any(): continue
        qq, po = q[k], p[k]
        uk, inv, cnt = np.unique(np.floor(qq / VS).astype(np.int64), axis=0, return_inverse=True, return_counts=True)
        inv = inv.ravel()
        cen = np.zeros((len(uk), 3)); np.add.at(cen, inv, qq); cen /= cnt[:, None]
        ceo = np.zeros((len(uk), 3)); np.add.at(ceo, inv, po); ceo /= cnt[:, None]
        ok = (cnt >= MINPTS) & (np.linalg.norm(cen - t, axis=1) <= rng)
        for c, co in zip(cen[ok], ceo[ok]):
            out[tuple(np.trunc(c / VS).astype(int))] = co
    return out


def in_frustum(po):
    x, y, z = po[..., 0], po[..., 1], po[..., 2]
    return (z >= ZMIN) & (z <= ZMAX) & (np.abs(np.arctan2(x, z)) <= HF / 2) & (np.abs(np.arctan2(y, z)) <= VF / 2)


def hist(x, bins):
    h = np.histogram(x, bins=bins)[0]
    return ' '.join(f'{bins[i]:g}-{bins[i + 1]:g}:{h[i]}' for i in range(len(h)) if h[i])


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('bag')
    ap.add_argument('--layer', default='/global_costmap/stvl_voxel_layer_raw')
    ap.add_argument('--info-bag', help='bag 里没录 camera_info 时，从这个 bag 取内参')
    a = ap.parse_args()
    buf, fr = load_rep(a.bag, a.layer, 0)
    K = None; cam_frame = None
    ev = []                      # (t, {key: 光学系质心}, 相机 map 位姿 R,t, base xy, yaw)
    npts = []
    if a.info_bag:
        m = next(m for _, m in read(a.info_bag, [INFO]))
        K = np.array(m.k).reshape(3, 3); W, H = m.width, m.height; info_frame = m.header.frame_id
    for tp, m in read(a.bag, [INFO, CAM]):
        if tp == INFO:
            if K is None: K = np.array(m.k).reshape(3, 3); W, H = m.width, m.height; info_frame = m.header.frame_id
            continue
        cam_frame = m.header.frame_id
        try:
            R, tv = mat(buf.lookup_transform('map', m.header.frame_id, m.header.stamp))
            tb, yaw = robot(buf, m)
        except Exception:
            continue
        p = pc2.read_points(m, field_names=('x', 'y', 'z'), skip_nans=True)
        p = np.stack([p['x'], p['y'], p['z']], -1).astype(np.float64)
        npts.append(len(p))
        ev.append((stamp(m), marks(p, R, tv), R, tv, tb, yaw))
    print(f'== {a.bag.rstrip("/").split("/")[-1]}  D435 {len(ev)} 帧（每帧 ~{np.mean(npts):.0f} 点），点云 frame={cam_frame}，内参 frame={info_frame} {W}x{H}')
    B = np.array([e[4][:2] for e in ev]); Y = np.array([e[5] for e in ev])
    print(f'   base_link 位移范围 {np.ptp(B[:, 0]):.2f} x {np.ptp(B[:, 1]):.2f} m，航向范围 {math.degrees(np.ptp(Y)):.1f}°')

    # ---------------- A. 体素稳定性
    N = len(ev); cnt = defaultdict(int); first = {}
    for i, (_, mk, *_r) in enumerate(ev):
        for k in mk: cnt[k] += 1; first.setdefault(k, i)
    frac = {k: c / N for k, c in cnt.items()}
    print(f'A. 被标记过的体素 {len(cnt)} 个；每帧标记 {np.mean([len(e[1]) for e in ev]):.0f} 个')
    cls = lambda f: '稳定(≥50%帧)' if f >= 0.5 else ('间歇(5~50%)' if f >= 0.05 else '偶发(<5%)')
    rows = defaultdict(list)     # 类别 -> (水平距 base, 方位, 高度, u, v, 深度)
    for t, mk, R, tv, tb, yaw in ev:
        for k, co in mk.items():
            w = np.array(k) * VS
            d = w[:2] - tb[:2]
            hb = math.hypot(*d); az = (math.degrees(math.atan2(d[1], d[0]) - yaw) + 180) % 360 - 180
            u = K[0, 0] * co[0] / co[2] + K[0, 2]; v = K[1, 1] * co[1] / co[2] + K[1, 2]
            rows[cls(frac[k])].append((hb, az, w[2], u, v, co[2]))
    for c in ('稳定(≥50%帧)', '间歇(5~50%)', '偶发(<5%)'):
        r = np.array(rows[c])
        if not len(r): print(f'  [{c}] 无'); continue
        nv = sum(1 for k in cnt if cls(frac[k]) == c)
        print(f'  [{c}] {nv} 个体素，{len(r)} 次标记（{len(r) / N:.1f} 次/帧）')
        print(f'     离 base_link 水平: {hist(r[:, 0], [0, .3, .5, .7, 1, 1.5, 2, 2.5, 3])}')
        print(f'     方位: {hist(r[:, 1], [-60, -45, -30, -15, 0, 15, 30, 45, 60])}')
        print(f'     高度: {hist(r[:, 2], [.15, .3, .5, .7, .9, 1.2, 1.5, 1.8])}')
        print(f'     深度(光轴): {hist(r[:, 5], [0, .3, .5, .7, 1, 1.5, 2, 2.5])}')
        print(f'     像素 u(0..{W}): {hist(r[:, 3], list(np.linspace(0, W, 9).round()))}')
        print(f'     像素 v(0..{H}): {hist(r[:, 4], list(np.linspace(0, H, 9).round()))}')

    # ---------------- B. 图层 254 格
    ts = np.array([e[0] for e in ev])
    last2d = defaultdict(list)   # 2D 格 -> [(t, 体素索引)]
    res = fr[0].metadata.resolution; ox, oy = fr[0].metadata.origin.position.x, fr[0].metadata.origin.position.y
    for t, mk, *_r in ev:
        for k in mk:
            w = np.array(k) * VS
            last2d[(int(math.floor((w[0] - ox) / res)), int(math.floor((w[1] - oy) / res)))].append((t, k))
    T = np.array([stamp(m) for m in fr])
    print(f'B. {a.layer}  {len(fr)} 帧，{len(fr) / (T[-1] - T[0]):.2f} Hz')
    per = []; near = []; ages = []; agecls = defaultdict(list); inside = []; seen = {}
    for m in fr:
        t = stamp(m)
        try: tb, yaw = robot(buf, m)
        except Exception: continue
        arr, P = grid_cells(m)
        ys, xs = np.nonzero(arr == 254)
        d = np.hypot(P[:, 0] - tb[0], P[:, 1] - tb[1])
        per.append(len(P)); near.append(int((d < 0.5).sum()))
        j = int(np.searchsorted(ts, t)) - 1
        R, tv = ev[j][2], ev[j][3]
        for cx, cy, dd in zip(xs, ys, d):
            best = None
            for dx in (-1, 0, 1):
                for dy in (-1, 0, 1):
                    for tt, k in reversed(last2d.get((cx + dx, cy + dy), [])):
                        if tt <= t + 0.05:
                            if best is None or tt > best[0]: best = (tt, k)
                            break
            if best is None: ages.append(99); agecls['无'].append(dd); continue
            age = t - best[0]; ages.append(age)
            po = (np.array(best[1]) * VS - tv) @ R          # 体素转到当前相机光学系
            fin = bool(in_frustum(po))
            inside.append(fin)
            agecls[('视锥内' if fin else '视锥外') + ('' if age < 0.5 else ' 挂着>0.5s')].append(dd)
            seen[(cx, cy)] = cls(frac[best[1]])
    ages = np.array(ages)
    print(f'   254/帧 均值 {np.mean(per):.0f} p90 {np.percentile(per, 90):.0f} max {max(per)}；base_link 0.5 m 内 均值 {np.mean(near):.1f} max {max(near)}')
    print(f'   距上次被标记: {hist(ages, [0, .2, .5, 1, 1.5, 2, 3, 5, 10, 100])}（格·帧；99=录制内没被标过）')
    for k, v in sorted(agecls.items()):
        print(f'   [{k}] {len(v)} 格·帧，离机器人: {hist(np.array(v), [0, .3, .5, .7, 1, 1.5, 2, 2.5, 3, 5])}')
    c = defaultdict(int)
    for v in seen.values(): c[v] += 1
    print(f'   出现过的 254 格按支撑体素类别: {dict(c)}')


if __name__ == '__main__':
    main()
