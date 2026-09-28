#!/usr/bin/env python3
"""离线量化 local_costmap 的"幻影障碍"闪烁（读 bag，不连机器人）。

    python3 costmap_noise_stats.py <bag_dir> [--t0 S] [--t1 S] [--radius M] [--scan-check]

为什么要有这个工具：改一次 costmap 参数就上机器人跑一趟，既慢又没法复现。
这里直接读录好的 bag，按"世界坐标格"对齐相邻两帧代价地图，统计：

  * 每帧 lethal(>=253) 格的 生/灭 数量        -> 闪烁强度
  * 每个格子的存活时长分布                     -> 是瞬时噪点还是残影
  * 短命格（<1 s）到机器人的距离/方位分布      -> 噪点来自哪个传感器的哪个区域
  * 按层拆分：combined 减去 STVL 层 = MID360(obstacle_layer) 的贡献
  * 与机器人速度/角速度的相关性                -> 是不是步态摆动/转向引起

--scan-check 额外做"该格当帧有没有实测点支撑"：用 bag 里的 /tf 建 buffer，
把 /lightning/registered_scan 变换到 map 系，看短命格附近有没有点。
没有点 = 这个格子不是本帧测出来的（残影/清不掉），有点 = 传感器真的报了噪点。
"""
import argparse, math, sys
from collections import defaultdict

import numpy as np
import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message

# 代价值语义：254=lethal, 253=inscribed, 255=NO_INFORMATION(未知)。
# STVL 层 track_unknown_space:=true，整张图大部分是 255，
# 用 ">=253" 会把未知当成障碍（踩过一次）——必须显式排除 255。
# 代价值语义：254=lethal(传感器标记), 253=inscribed(**膨胀层**算出来的内切带),
# 255=NO_INFORMATION(未知)。只有 254 是"传感器说这里有东西"；
# 253 是 inflation_layer 按 robot_radius 画的环，障碍稍微动一下整圈 253 就重画，
# 把它算进"闪烁"会凭空多出几十倍的假格子（踩过一次）。
# STVL 层 track_unknown_space:=true，整张图大部分是 255，">=253" 会把未知也当障碍（也踩过）。
MARK = 254

C_RAW = '/local_costmap/costmap_raw'
S_RAW = '/local_costmap/stvl_voxel_layer_raw'
POSE = '/base_link_pose'
SCAN = '/lightning/registered_scan'
CMD = '/cmd_vel'


def reader(bag, topics):
    r = rosbag2_py.SequentialReader()
    r.open(rosbag2_py.StorageOptions(uri=bag, storage_id='mcap'),
           rosbag2_py.ConverterOptions('', ''))
    types = {t.name: t.type for t in r.get_all_topics_and_types()}
    r.set_filter(rosbag2_py.StorageFilter(topics=[t for t in topics if t in types]))
    return r, types


def lethal_cells(msg):
    """返回 (lethal格键集合, res, 窗口键范围) —— 键 = 以 resolution 取整的世界坐标。

    注意：local_costmap 是 rolling window，机器人一走，窗口就扫过原本就存在的格子。
    如果只按"离机器人多远"过滤再做帧间差分，这些格子会被误判成"新生"——
    走路时假新生会比静止时高好几倍，纯粹是几何假象。所以这里返回整窗，
    差分只在**两帧窗口的交集**里做，距离过滤放到统计输出时再做。
    """
    md = msg.metadata
    res, w, h = md.resolution, md.size_x, md.size_y
    ox, oy = md.origin.position.x, md.origin.position.y
    a = np.frombuffer(bytes(msg.data), dtype=np.uint8).reshape(h, w)
    ys, xs = np.nonzero(a == 254) if MARK else np.nonzero((a == 254) | (a == 253))
    wx = ox + (xs + 0.5) * res
    wy = oy + (ys + 0.5) * res
    gx = np.round(wx / res).astype(np.int64)
    gy = np.round(wy / res).astype(np.int64)
    bounds = (int(round(ox / res)) + 1, int(round((ox + w * res) / res)) - 1,
              int(round(oy / res)) + 1, int(round((oy + h * res) / res)) - 1)
    return set(zip(gx.tolist(), gy.tolist())), res, bounds


def in_bounds(cells, b):
    x0, x1, y0, y1 = b
    return {c for c in cells if x0 <= c[0] <= x1 and y0 <= c[1] <= y1}


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('bag')
    ap.add_argument('--t0', type=float, default=0.0, help='从 bag 开始的秒数')
    ap.add_argument('--t1', type=float, default=1e9)
    ap.add_argument('--radius', type=float, default=2.0, help='只统计机器人这个半径内的格子')
    ap.add_argument('--prefix', default='/local_costmap',
                    help='代价地图话题前缀；离线回放产物用 /costmap')
    ap.add_argument('--scan-check', action='store_true')
    ap.add_argument('--inscribed', action='store_true',
                    help='连 253(膨胀内切带)一起统计；默认只看传感器标记的 254')
    ap.add_argument('--scan-tol', type=float, default=0.12, help='格中心多远内算有点支撑(m)')
    args = ap.parse_args()

    global MARK
    if args.inscribed:
        MARK = None   # 见 lethal_cells
    global C_RAW, S_RAW
    C_RAW = args.prefix + '/costmap_raw'
    S_RAW = args.prefix + '/stvl_voxel_layer_raw'
    topics = [C_RAW, S_RAW, POSE, CMD]
    if args.scan_check:
        topics += [SCAN, '/tf', '/tf_static']
    rd, types = reader(args.bag, topics)
    msgcls = {t: get_message(ty) for t, ty in types.items() if t in topics}

    tf_buf = None
    if args.scan_check:
        import tf2_ros, rclpy.time
        tf_buf = tf2_ros.Buffer(cache_time=rclpy.duration.Duration(seconds=30))

    t_start = None
    poses = []            # (t, x, y, yaw)
    cmds = []             # (t, vx, wz)
    frames = []           # (t, cells:set, res, cx, cy, yaw)
    stvl = []             # (t, cells:set)
    scans = []            # (t, msg) —— 只在 scan-check 时留
    while rd.has_next():
        topic, data, stamp = rd.read_next()
        t = stamp / 1e9
        if t_start is None:
            t_start = t
        rel = t - t_start
        if rel < args.t0:
            if topic == '/tf_static' and tf_buf is not None:
                for tr in deserialize_message(data, msgcls[topic]).transforms:
                    tf_buf.set_transform_static(tr, 'bag')
            continue
        if rel > args.t1:
            break
        cls = msgcls.get(topic)
        if cls is None:
            continue
        m = deserialize_message(data, cls)
        if topic == POSE:
            q = m.pose.orientation
            yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y ** 2 + q.z ** 2))
            poses.append((rel, m.pose.position.x, m.pose.position.y, yaw))
        elif topic == CMD:
            cmds.append((rel, m.linear.x, m.angular.z))
        elif topic == C_RAW:
            frames.append((rel, m))
        elif topic == S_RAW:
            stvl.append((rel, m))
        elif topic == '/tf' and tf_buf is not None:
            for tr in m.transforms:
                tf_buf.set_transform(tr, 'bag')
        elif topic == '/tf_static' and tf_buf is not None:
            for tr in m.transforms:
                tf_buf.set_transform_static(tr, 'bag')
        elif topic == SCAN and args.scan_check:
            scans.append((rel, m))

    if not frames:
        print('bag 里没有 %s' % C_RAW); return 1
    P = np.array(poses) if poses else None
    print(f'窗口 {frames[0][0]:.1f}~{frames[-1][0]:.1f}s  costmap帧={len(frames)}  '
          f'stvl帧={len(stvl)}  pose={len(poses)}  半径={args.radius}m')

    def pose_at(t):
        if P is None or len(P) == 0:
            return 0.0, 0.0, 0.0
        i = int(np.searchsorted(P[:, 0], t))
        i = min(max(i, 0), len(P) - 1)
        return P[i, 1], P[i, 2], P[i, 3]

    # ---- 逐帧生/灭 + 存活时长 -------------------------------------------------
    stvl_t = np.array([s[0] for s in stvl]) if stvl else None
    born = {}                 # cell -> (t_first, layer)
    lifetimes = []            # (life_s, dist, bearing_deg, layer)
    prev = set(); prev_b = None; prev_t = None
    rows = []
    for t, m in frames:
        cx, cy, yaw = pose_at(t)
        cells, res, bnd = lethal_cells(m)
        s_cells = set()
        if stvl_t is not None and len(stvl_t):
            j = int(np.argmin(np.abs(stvl_t - t)))
            if abs(stvl_t[j] - t) < 0.4:
                s_cells, _, _ = lethal_cells(stvl[j][1])
        near = lambda S: {c for c in S if math.hypot(c[0]*res-cx, c[1]*res-cy) <= args.radius}
        if prev_t is not None:
            # 只在两帧窗口交集里做差分，排除 rolling window 扫进/扫出造成的假生灭
            ov = (max(bnd[0], prev_b[0]), min(bnd[1], prev_b[1]),
                  max(bnd[2], prev_b[2]), min(bnd[3], prev_b[3]))
            cur_o = in_bounds(cells, ov); prev_o = in_bounds(prev, ov)
            b = near(cur_o - prev_o)
            d = near(prev_o - cur_o)
            for c in b:
                born.setdefault(c, (t, 'stvl' if c in s_cells else 'mid360'))
            for c in d:
                t0, lay = born.pop(c, (t, '?'))
                wx, wy = c[0] * res, c[1] * res
                dist = math.hypot(wx - cx, wy - cy)
                bear = math.degrees(math.atan2(wy - cy, wx - cx) - yaw)
                bear = (bear + 180) % 360 - 180
                lifetimes.append((t - t0, dist, bear, lay))
            rows.append((t, len(near(cells)), len(near(s_cells)), len(b), len(d)))
        prev, prev_b, prev_t = cells, bnd, t

    R = np.array([r[:5] for r in rows])
    dt = R[-1, 0] - R[0, 0]
    print('\n=== 标记格(254)数量 (机器人 %.1fm 内) ===' % args.radius)
    print(f'  combined: 均值 {R[:,1].mean():6.1f}  p50 {np.percentile(R[:,1],50):6.1f}  max {R[:,1].max():6.0f}')
    print(f'  STVL(D435): 均值 {R[:,2].mean():6.1f}  p50 {np.percentile(R[:,2],50):6.1f}  max {R[:,2].max():6.0f}')
    print(f'  MID360 独有 ≈ combined-STVL: 均值 {(R[:,1]-R[:,2]).mean():6.1f}')
    print('\n=== 闪烁（相邻帧生/灭）===')
    print(f'  每帧新生 均值 {R[:,3].mean():6.1f}  p90 {np.percentile(R[:,3],90):6.1f}  max {R[:,3].max():5.0f}')
    print(f'  每帧消亡 均值 {R[:,4].mean():6.1f}  p90 {np.percentile(R[:,4],90):6.1f}  max {R[:,4].max():5.0f}')
    print(f'  换算成每秒: 新生 {R[:,3].sum()/dt:.0f} 格/s   消亡 {R[:,4].sum()/dt:.0f} 格/s')

    if lifetimes:
        L = np.array([(a, b, c) for a, b, c, _ in lifetimes])
        lay = np.array([d for *_, d in lifetimes])
        print('\n=== 已消亡格子的存活时长 (n=%d) ===' % len(L))
        for p in (10, 25, 50, 75, 90, 99):
            print(f'  p{p:<2d} = {np.percentile(L[:,0],p):6.2f} s', end='')
        print()
        short = L[L[:, 0] < 1.0]
        print(f'  存活<1s 的短命格: {len(short)} ({100*len(short)/len(L):.0f}%)  '
              f'<0.5s: {int((L[:,0]<0.5).sum())}  >5s(残影): {int((L[:,0]>5).sum())}')
        for name in ('mid360', 'stvl'):
            sel = lay == name
            if sel.sum():
                print(f'  [{name}] n={int(sel.sum()):6d} 中位存活 {np.median(L[sel,0]):.2f}s  '
                      f'中位距离 {np.median(L[sel,1]):.2f}m')
        if len(short):
            print('  短命格距离分布(m):', np.round(np.percentile(short[:, 1], [10, 50, 90]), 2),
                  ' 方位角分布(度,0=正前):', np.round(np.percentile(short[:, 2], [10, 50, 90]), 0))
            for lo, hi in [(0, 0.6), (0.6, 1.0), (1.0, 1.5), (1.5, 2.0), (2.0, 99)]:
                n = int(((short[:, 1] >= lo) & (short[:, 1] < hi)).sum())
                if n:
                    print(f'    d=[{lo},{hi}) {n:6d} 个 ({100*n/len(short):4.1f}%)')

    # ---- 运动相关性 -----------------------------------------------------------
    if P is not None and len(P) > 5:
        tt, xx, yy, yw = P[:, 0], P[:, 1], P[:, 2], P[:, 3]
        dtp = np.diff(tt)
        ok = dtp > 1e-3
        v = np.hypot(np.diff(xx), np.diff(yy))[ok] / dtp[ok]
        dyaw = (np.diff(yw)[ok] + np.pi) % (2 * np.pi) - np.pi
        w = np.abs(dyaw) / dtp[ok]
        tm = tt[1:][ok]
        vi = np.interp(R[:, 0], tm, v)
        wi = np.interp(R[:, 0], tm, w)
        moving = vi > 0.05
        print('\n=== 运动 vs 静止 ===')
        for name, sel in (('运动(v>0.05)', moving), ('静止', ~moving)):
            if sel.sum() > 5:
                print(f'  {name:12s} 帧={int(sel.sum()):5d}  新生/帧 {R[sel,3].mean():6.1f}  '
                      f'消亡/帧 {R[sel,4].mean():6.1f}  lethal数 {R[sel,1].mean():6.1f}')
        if moving.sum() > 20:
            c = np.corrcoef(wi[moving], R[moving, 3])[0, 1]
            print(f'  运动时 |角速度| 与 新生格数 的相关系数 = {c:+.2f}')

    # ---- 点云支撑检查 ---------------------------------------------------------
    if args.scan_check and scans:
        import rclpy.time, sensor_msgs_py.point_cloud2 as pc2
        st = np.array([s[0] for s in scans])
        checked = supported = 0
        # 对每一帧的新生格抽样检查（最多 400 个），看当帧点云有没有落在格子附近
        rng = np.random.default_rng(0)
        prev = set()
        for t, m in frames[::5]:
            cx, cy, yaw = pose_at(t)
            cells, res = lethal_cells(m, cx, cy, args.radius)
            new = list(cells - prev)
            prev = cells
            if not new:
                continue
            j = int(np.argmin(np.abs(st - t)))
            if abs(st[j] - t) > 0.3:
                continue
            sm = scans[j][1]
            try:
                tr = tf_buf.lookup_transform('map', sm.header.frame_id,
                                             rclpy.time.Time.from_msg(sm.header.stamp))
            except Exception:
                continue
            q = tr.transform.rotation
            tv = tr.transform.translation
            w_, x_, y_, z_ = q.w, q.x, q.y, q.z
            Rm = np.array([[1-2*(y_*y_+z_*z_), 2*(x_*y_-z_*w_), 2*(x_*z_+y_*w_)],
                           [2*(x_*y_+z_*w_), 1-2*(x_*x_+z_*z_), 2*(y_*z_-x_*w_)],
                           [2*(x_*z_-y_*w_), 2*(y_*z_+x_*w_), 1-2*(x_*x_+y_*y_)]])
            a = pc2.read_points(sm, field_names=('x', 'y', 'z'), skip_nans=True)
            pts = (Rm @ np.stack([a['x'], a['y'], a['z']], -1).astype(float).T).T + \
                  np.array([tv.x, tv.y, tv.z])
            pxy = pts[(pts[:, 2] > 0.10) & (pts[:, 2] < 1.8)][:, :2]
            if len(pxy) == 0:
                continue
            sample = new if len(new) <= 400 else [new[i] for i in rng.choice(len(new), 400, replace=False)]
            for gx, gy in sample:
                wx, wy = gx * res, gy * res
                checked += 1
                if np.min(np.hypot(pxy[:, 0] - wx, pxy[:, 1] - wy)) <= args.scan_tol:
                    supported += 1
        if checked:
            print(f'\n=== 新生格的点云支撑 (抽查 {checked} 个) ===')
            print(f'  当帧 {args.scan_tol*100:.0f}cm 内有实测点: {supported} '
                  f'({100*supported/checked:.0f}%)  无点支撑: {checked-supported} '
                  f'({100*(checked-supported)/checked:.0f}%)')
            print('  无点支撑 = 不是本帧测出来的（累积残影/清除失败）；'
                  '有点支撑 = 传感器确实报了这个点（真噪点或真障碍）')
    return 0


if __name__ == '__main__':
    sys.exit(main())
