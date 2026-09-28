#!/usr/bin/env python3
"""用静态地图当判据，量"局部代价地图里凭空长出来的障碍"。

    python3 phantom_vs_map.py <bag> [--t0 S] [--t1 S] [--radius M] [--prefix /local_costmap]

判据：local_costmap 的 254 格，如果在**静态地图里是空地**（且离任何地图占据格 > --map-tol），
就算"幻影"。真墙/真家具会落在地图占据格上或它旁边，不会被误判；
唯一会被算成幻影的真障碍是"地图建好之后才出现的东西"（人、临时箱子）——
所以还要看它的**存活时长**和**是否随机出现**：人是连续移动的一团，噪点是散点、生灭极快。

输出：
  * 幻影格占比、幻影格到机器人的距离分布
  * 幻影格的存活时长（分 MID360 / STVL 两层）
  * 幻影"团块"大小分布（连通域）：散点 vs 成片
"""
import argparse, math, sys
from collections import defaultdict, deque
import numpy as np
import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('bag')
    ap.add_argument('--t0', type=float, default=0.0)
    ap.add_argument('--t1', type=float, default=1e9)
    ap.add_argument('--radius', type=float, default=2.5)
    ap.add_argument('--prefix', default='/local_costmap')
    ap.add_argument('--map-tol', type=float, default=0.25,
                    help='离静态地图占据格这么近就不算幻影(m)，吸收定位误差与墙厚')
    ap.add_argument('--stride', type=int, default=2)
    args = ap.parse_args()

    C = args.prefix + '/costmap_raw'
    S = args.prefix + '/stvl_voxel_layer_raw'
    want = [C, S, '/map', '/base_link_pose']
    r = rosbag2_py.SequentialReader()
    r.open(rosbag2_py.StorageOptions(uri=args.bag, storage_id='mcap'),
           rosbag2_py.ConverterOptions('', ''))
    types = {t.name: t.type for t in r.get_all_topics_and_types()}
    r.set_filter(rosbag2_py.StorageFilter(topics=[t for t in want if t in types]))
    cls = {t: get_message(types[t]) for t in want if t in types}

    t0 = None
    gmap = None
    frames, stvl, poses = [], [], []
    while r.has_next():
        tp, data, stamp = r.read_next()
        t = stamp / 1e9
        if t0 is None:
            t0 = t
        rel = t - t0
        if rel > args.t1:
            break
        m = deserialize_message(data, cls[tp])
        if tp == '/map':
            gmap = m
        elif rel < args.t0:
            continue
        elif tp == C:
            frames.append((rel, m))
        elif tp == S:
            stvl.append((rel, m))
        elif tp == '/base_link_pose':
            poses.append((rel, m.pose.position.x, m.pose.position.y))
    if gmap is None:
        print('bag 里没有 /map，没法判定幻影'); return 1
    if not frames:
        print('bag 里没有', C); return 1
    P = np.array(poses)
    mi = gmap.info
    occ = np.array(gmap.data, dtype=np.int8).reshape(mi.height, mi.width)
    print(f'静态地图 {mi.width}x{mi.height} res={mi.resolution} 占据格={int((occ>=50).sum())}')

    # 把地图占据格按 map-tol 膨胀，做成"可以合法出现障碍"的掩膜
    from scipy.ndimage import binary_dilation, label
    rad = max(1, int(round(args.map_tol / mi.resolution)))
    yy, xx = np.ogrid[-rad:rad + 1, -rad:rad + 1]
    se = (xx ** 2 + yy ** 2) <= rad ** 2
    legal = binary_dilation(occ >= 50, structure=se)
    unknown = binary_dilation(occ < 0, structure=se)     # 未知区也不算幻影

    def pose_at(t):
        i = min(max(int(np.searchsorted(P[:, 0], t)), 0), len(P) - 1)
        return P[i, 1], P[i, 2]

    st = np.array([s[0] for s in stvl]) if stvl else None
    born = {}
    life = []
    prev = set()
    tot = ph = 0
    ph_d = []
    blob_sizes = []
    for k, (t, m) in enumerate(frames):
        if k % args.stride:
            continue
        cx, cy = pose_at(t)
        md = m.metadata; res = md.resolution
        ox, oy = md.origin.position.x, md.origin.position.y
        a = np.frombuffer(bytes(m.data), dtype=np.uint8).reshape(md.size_y, md.size_x)
        ys, xs = np.nonzero(a == 254)
        wx = ox + (xs + 0.5) * res; wy = oy + (ys + 0.5) * res
        d = np.hypot(wx - cx, wy - cy)
        sel = d <= args.radius
        wx, wy, d = wx[sel], wy[sel], d[sel]
        gx = ((wx - mi.origin.position.x) / mi.resolution).astype(int)
        gy = ((wy - mi.origin.position.y) / mi.resolution).astype(int)
        ok = (gx >= 0) & (gx < mi.width) & (gy >= 0) & (gy < mi.height)
        phantom = np.zeros(len(wx), bool)
        phantom[ok] = ~legal[gy[ok], gx[ok]] & ~unknown[gy[ok], gx[ok]]
        tot += len(wx); ph += int(phantom.sum())
        ph_d.extend(d[phantom].tolist())
        # 幻影格连通域大小
        if phantom.any():
            keys = set(zip(np.round(wx[phantom] / res).astype(np.int64).tolist(),
                           np.round(wy[phantom] / res).astype(np.int64).tolist()))
            seen = set()
            for c in keys:
                if c in seen:
                    continue
                q = deque([c]); seen.add(c); n = 0
                while q:
                    u = q.popleft(); n += 1
                    for dx in (-1, 0, 1):
                        for dy in (-1, 0, 1):
                            v = (u[0] + dx, u[1] + dy)
                            if v in keys and v not in seen:
                                seen.add(v); q.append(v)
                blob_sizes.append(n)
            # 存活时长
            s_cells = set()
            if st is not None and len(st):
                j = int(np.argmin(np.abs(st - t)))
                if abs(st[j] - t) < 0.4:
                    sm = stvl[j][1]; smd = sm.metadata
                    sa = np.frombuffer(bytes(sm.data), dtype=np.uint8).reshape(smd.size_y, smd.size_x)
                    sy, sx = np.nonzero(sa == 254)
                    s_cells = set(zip(np.round((smd.origin.position.x + (sx + 0.5) * smd.resolution) / res).astype(np.int64).tolist(),
                                      np.round((smd.origin.position.y + (sy + 0.5) * smd.resolution) / res).astype(np.int64).tolist()))
            for c in keys:
                born.setdefault(c, (t, 'stvl' if c in s_cells else 'mid360'))
            for c in list(prev - keys):
                bt, lay = born.pop(c, (t, '?'))
                life.append((t - bt, lay))
            prev = keys
    print(f'\n采样 {len(frames)//args.stride} 帧，{args.radius} m 内 254 格共 {tot} 个')
    if tot:
        print(f'  其中"静态地图里是空地"的幻影格: {ph} ({100*ph/tot:.1f}%)')
    if ph_d:
        D = np.array(ph_d)
        print('  幻影格到机器人的距离 p10/p50/p90: '
              f'{np.percentile(D,10):.2f}/{np.percentile(D,50):.2f}/{np.percentile(D,90):.2f} m'
              f'   <1.0m 的占 {100*(D<1.0).mean():.1f}%')
    if blob_sizes:
        B = np.array(blob_sizes)
        print(f'  幻影连通团块 {len(B)} 个: 1格={int((B==1).sum())}  2~4格={int(((B>1)&(B<5)).sum())}  '
              f'5~20格={int(((B>=5)&(B<21)).sum())}  >20格={int((B>20).sum())}  最大={B.max()}格')
    if life:
        L = np.array([x[0] for x in life]); lay = np.array([x[1] for x in life])
        print(f'  幻影格存活时长 p50={np.median(L):.2f}s p90={np.percentile(L,90):.2f}s  '
              f'>2s 的占 {100*(L>2).mean():.1f}%')
        for nm in ('mid360', 'stvl'):
            s = lay == nm
            if s.sum():
                print(f'    [{nm}] n={int(s.sum())} 中位存活 {np.median(L[s]):.2f}s')
    return 0


if __name__ == '__main__':
    sys.exit(main())
