#!/usr/bin/env python3
"""MID360 迁 STVL 前的参数取证：机身回波、垂直视场、同一格的重访间隔。

    python3 mid360_stvl_probe.py <bag> [--start S] [--dur D] [--min-range 0.25] [--radius 0.50]

1. 雷达位姿：base_link 系下 mid360_link 的高度与俯仰/横滚（STVL 3D 雷达视锥按雷达系对称）。
2. 近距点：离雷达 <0.1 / 0.1~0.25 / 0.25~0.5 m 的比例；剔除 min_range 后，仍落在机器人足迹
   （离 base_link 水平 < radius）且高度 0.10~1.8 m 的点 = STVL 会标在脚下的机身回波（自清是关的）。
3. 垂直视场：雷达系仰角分布（剔除近距后）→ vertical_fov_angle 需要的对称半角。
4. 重访间隔：map 系 5 cm 格（0.5~2.5 m、高 0.10~1.8 m）两次被打到的间隔分布
   → STVL voxel_decay / decay_acceleration 必须让真障碍活过 p95 间隔，否则非重复扫描下会闪。
"""
import argparse, math
from collections import defaultdict
import numpy as np
import rclpy.time
import sensor_msgs_py.point_cloud2 as pc2
from costmap_recall import read, stamp, mat, load_rep

LID = '/lightning/registered_scan'


def survival(decay, acc):
    """STVL 线性衰减 + 视锥加速：voxel_decay - t - acc·t³/6 = 0 的解（二分）。"""
    lo, hi = 0.0, decay
    for _ in range(60):
        mid = (lo + hi) / 2
        lo, hi = (mid, hi) if decay - mid - acc * mid ** 3 / 6 > 0 else (lo, mid)
    return lo


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('bag')
    ap.add_argument('--start', type=float, default=0.0, help='相对 bag 第一帧雷达的秒数')
    ap.add_argument('--dur', type=float, default=1e9)
    ap.add_argument('--min-range', type=float, default=0.25)
    ap.add_argument('--radius', type=float, default=0.50, help='robot_radius')
    a = ap.parse_args()
    buf, _ = load_rep(a.bag, '/__none__', 0)

    near = np.zeros(4); total = 0; foot = 0; foot_frames = 0; nfr = 0
    elev = []; az_cnt = np.zeros(36); hits = defaultdict(list); pose = []
    t0 = None
    for _, m in read(a.bag, [LID]):
        t = stamp(m); t0 = t if t0 is None else t0
        if t - t0 < a.start or t - t0 > a.start + a.dur: continue
        try:
            Rb, tb = mat(buf.lookup_transform('base_link', m.header.frame_id, m.header.stamp))
            Rm, tm = mat(buf.lookup_transform('map', m.header.frame_id, m.header.stamp))
        except Exception:
            continue
        p = pc2.read_points(m, field_names=('x', 'y', 'z'), skip_nans=True)
        p = np.stack([p['x'], p['y'], p['z']], -1).astype(np.float64)
        r = np.linalg.norm(p, axis=1)
        total += len(p); nfr += 1
        near += [(r < 0.1).sum(), ((r >= 0.1) & (r < a.min_range)).sum(), ((r >= a.min_range) & (r < 0.5)).sum(), 0]
        k = r >= a.min_range
        q = p[k]
        pose.append((tb[2], math.degrees(math.asin(-Rb[2, 0])), math.degrees(math.atan2(Rb[2, 1], Rb[2, 2]))))
        # 雷达系仰角
        elev.append(np.degrees(np.arctan2(q[:, 2], np.hypot(q[:, 0], q[:, 1])))[::5])
        # base_link 系：足迹内的机身回波 + 方位覆盖
        qb = q @ Rb.T + tb
        hb = np.hypot(qb[:, 0], qb[:, 1])
        fk = (hb < a.radius) & (qb[:, 2] >= 0.10) & (qb[:, 2] <= 1.8)
        foot += int(fk.sum()); foot_frames += int(fk.any())
        az = (np.degrees(np.arctan2(qb[:, 1], qb[:, 0])) + 180) % 360
        az_cnt += np.bincount((az // 10).astype(int) % 36, minlength=36)
        # map 系格子被打到的时刻
        qm = q @ Rm.T + tm
        sel = (qm[:, 2] >= 0.10) & (qm[:, 2] <= 1.8)
        d = np.hypot(qm[:, 0] - tm[0], qm[:, 1] - tm[1])
        sel &= (d >= 0.5) & (d <= 2.5)
        for key in set(map(tuple, np.floor(qm[sel, :2] / 0.05).astype(int))):
            hits[key].append(t)

    if nfr == 0:
        raise SystemExit('没有可用的雷达帧（查 TF / 时间窗）')
    z, pitch, roll = np.median(np.array(pose), axis=0)
    print(f'== {a.bag.rstrip("/").split("/")[-1]}  雷达帧 {nfr}')
    print(f'1) 雷达在 base_link 系：高 {z:.3f} m，俯仰 {pitch:+.1f}°，横滚 {roll:+.1f}°（中位数）')
    print(f'2) 离雷达 <0.1 m {100 * near[0] / total:.1f}%   0.1~{a.min_range} m {100 * near[1] / total:.1f}%'
          f'   {a.min_range}~0.5 m {100 * near[2] / total:.1f}%')
    print(f'   剔除 <{a.min_range} m 后仍在足迹内(<{a.radius} m)、高 0.10~1.8 m 的点：每帧 {foot / nfr:.2f} 个，'
          f'{100 * foot_frames / nfr:.1f}% 的帧有  ← 非 0 说明 STVL 会在脚下标格')
    e = np.concatenate(elev)
    lo, hi = np.percentile(e, [0.5, 99.5])
    print(f'3) 雷达系仰角 p0.5/p99.5 = {lo:+.1f}° / {hi:+.1f}°  → 对称视锥半角至少 {max(abs(lo), abs(hi)):.1f}°'
          f'（vertical_fov_angle ≥ {math.radians(2 * max(abs(lo), abs(hi))):.2f} rad）')
    share = az_cnt / az_cnt.sum()
    sparse = [f'{i * 10 - 180:+d}°' for i in range(36) if share[i] < 0.2 / 36]
    print(f'   base_link 系方位上点数不到均值 20% 的 10° 扇区：{" ".join(sparse) or "无"}')
    gaps = []
    for ts in hits.values():
        if len(ts) >= 5:
            gaps.extend(np.diff(np.sort(ts)))
    g = np.array(gaps)
    g = g[g < 5.0]   # 更长的是离开视野/被遮挡，不是扫描稀疏
    if len(g):
        print(f'4) 同一格重访间隔（被打到 ≥5 次的格，{len(g)} 个间隔）p50/p90/p95/p99 = '
              + '/'.join(f'{x:.2f}' for x in np.percentile(g, [50, 90, 95, 99])) + ' s')
        print('   视锥内真障碍的存活时间（没再被打到时）应大于 p95、最好大于 p99：')
        for decay, acc in ((1.5, 1.0), (1.5, 5.0), (1.5, 15.0), (5.0, 1.0)):
            print(f'     voxel_decay {decay} + decay_acceleration {acc:>4}: {survival(decay, acc):.2f} s')


if __name__ == '__main__':
    main()
