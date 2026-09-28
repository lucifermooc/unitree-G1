#!/usr/bin/env python3
"""代价地图清除滞后：一个 254 格最后一次被传感器点支撑之后，还挂多久才从图上消失。

    python3 costmap_clear_lag.py <bag> [--topics /global_costmap/mid360_voxel_layer_raw ...] [--skip 5]

bag 需含 /tf、/lightning/registered_scan、/camera/camera/depth/color/points 和被测 costmap 话题（实录或回放均可）。
- 消失滞后：第 k 帧是 254、第 k+1 帧不是的格，= t(k+1) - 最后一次 ±7.5 cm 内有 MID360(z 0.10~1.8) 或 D435(z 0.15~1.8) 点的时刻。
  按"人"（支撑点最高 ≥1.0 m）/ 矮物体分开。
- 挂着的残影：每帧里 1 s 以上没有支撑的 254 格数，及其已挂时长。
"""
import argparse
import numpy as np
from scipy.spatial import cKDTree
from costmap_recall import load_rep, load_sensors, grid_cells, stamp

LOOK = 15.0   # 往前找支撑的最长时间


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('bag')
    ap.add_argument('--topics', nargs='+', default=['/global_costmap/mid360_voxel_layer_raw',
                                                   '/global_costmap/stvl_voxel_layer_raw',
                                                   '/local_costmap/mid360_voxel_layer_raw'])
    ap.add_argument('--skip', type=float, default=5.0)
    a = ap.parse_args()
    buf, fr0 = load_rep(a.bag, a.topics[0], a.skip)
    cam, lid = load_sensors(a.bag, buf, stamp(fr0[0]) - LOOK, stamp(fr0[-1]) + 1)
    S = sorted([(t, q) for t, q in lid + cam if len(q)], key=lambda x: x[0])
    ts = np.array([t for t, _ in S]); trees = {}

    def last_support(P, t):
        """每个格最后一次有点支撑的时刻（-1 = LOOK 内没有）与那次支撑点的最高高度。"""
        last = np.full(len(P), -1.0); ztop = np.zeros(len(P))
        i = int(np.searchsorted(ts, t + 0.05)) - 1
        while i >= 0 and ts[i] >= t - LOOK and (last < 0).any():
            if i not in trees: trees[i] = cKDTree(S[i][1][:, :2])
            todo = np.nonzero(last < 0)[0]
            idx = trees[i].query_ball_point(P[todo], 0.075)
            for k, ii in zip(todo, idx):
                if ii: last[k] = ts[i]; ztop[k] = S[i][1][ii, 2].max()
            i -= 1
        return last, ztop

    for topic in a.topics:
        _, fr = load_rep(a.bag, topic, a.skip)
        if len(fr) < 2: print(f'{topic}: 帧不够'); continue
        T = np.array([stamp(m) for m in fr])
        lag = []; tall = []; stale_n = []; stale_age = []; never = 0
        prev = None
        for k, m in enumerate(fr):
            _, P = grid_cells(m); res = m.metadata.resolution
            keys = {tuple(x) for x in np.floor(P / res).astype(int)}
            if prev is not None:
                gone = np.array([p for p, kk in zip(prev[1], prev[2]) if kk not in keys])
                if len(gone):
                    last, zt = last_support(gone, prev[0])
                    ok = last >= 0; never += int((~ok).sum())
                    lag += list(T[k] - last[ok]); tall += list(zt[ok] >= 1.0)
            last, _ = last_support(P, T[k]) if len(P) else (np.zeros(0), None)
            age = np.where(last < 0, LOOK, T[k] - last)
            stale_n.append(int((age > 1.0).sum())); stale_age += list(age[age > 1.0])
            prev = (T[k], P, [tuple(x) for x in np.floor(P / res).astype(int)])
        lag = np.array(lag); tall = np.array(tall, bool)
        print(f'== {topic}  {len(fr)} 帧，实际 {len(fr) / (T[-1] - T[0]):.2f} Hz，帧间隔 p50/p90/max '
              + '/'.join(f'{x:.2f}' for x in np.percentile(np.diff(T), [50, 90, 100])) + ' s')
        for name, sel in (('全部', np.ones(len(lag), bool)), ('人(支撑点≥1.0 m)', tall), ('矮物体', ~tall)):
            if sel.any():
                print(f'  消失滞后[{name}] {sel.sum()} 格  p50/p90/p99/max = '
                      + '/'.join(f'{x:.1f}' for x in np.percentile(lag[sel], [50, 90, 99, 100])) + ' s')
        print(f'  消失的格里 {LOOK:g} s 内找不到任何支撑: {never}')
        sa = np.array(stale_age)
        print(f'  每帧挂着的 >1 s 无支撑格: 均值 {np.mean(stale_n):.0f}  p90 {np.percentile(stale_n, 90):.0f}  max {max(stale_n)}'
              + (f'；已挂时长 p50/p90 {np.percentile(sa, 50):.1f}/{np.percentile(sa, 90):.1f} s' if len(sa) else ''))


if __name__ == '__main__':
    main()
