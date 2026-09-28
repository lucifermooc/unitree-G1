#!/usr/bin/env python3
"""STVL(D435) 前方每个 254 格是怎么来的 —— 真障碍 / 地面误检 / 衰减残影 三分类。

    python3 stvl_cell_provenance.py <bag> [--t0 S] [--t1 S] [--stride N]

对每个采样时刻，取 STVL 层在机器人前方扇区内的 254 格，用**当帧**深度点云判定：

  真障碍   格子 12 cm 内有 z>=0.30 m 的点        （有高度的东西）
  地面误检 只有 z∈[min_h,0.30) 的矮点             （地面残差被抬过阈值）
  残影     当帧这个方向上根本没有点               （voxel_decay 留下的旧标记）
           再细分成"在相机视锥内"(STVL 本该把它清掉 = 清除没生效)
           和"在视锥外/盲区"(只能等衰减，是设计如此)

同时统计地面点的 z 误差分布：用"该方位角上最高点 < 0.30 m"的列当作真地面，
看它们的 z 有多少超过 min_obstacle_height —— 直接回答阈值该不该抬。
"""
import argparse, math, sys
import numpy as np
import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message

STVL = '/local_costmap/stvl_voxel_layer_raw'
CAM = '/camera/camera/depth/color/points'
POSE = '/base_link_pose'


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('bag')
    ap.add_argument('--t0', type=float, default=60.0)
    ap.add_argument('--t1', type=float, default=800.0)
    ap.add_argument('--stride', type=int, default=10, help='每隔几帧 STVL 采一次')
    ap.add_argument('--radius', type=float, default=1.5)
    ap.add_argument('--fov', type=float, default=50.0, help='前方扇区半角(度)')
    ap.add_argument('--min-h', type=float, default=0.15, help='当前 realsense_mark.min_obstacle_height')
    ap.add_argument('--tall', type=float, default=0.30, help='高于此高度才算"有高度的真障碍"')
    ap.add_argument('--tol', type=float, default=0.12)
    ap.add_argument('--prefix', default='/local_costmap',
                    help='代价地图话题前缀；离线回放产物用 /costmap')
    args = ap.parse_args()

    global STVL
    STVL = args.prefix + '/stvl_voxel_layer_raw'
    import tf2_ros, rclpy.time, rclpy.duration, sensor_msgs_py.point_cloud2 as pc2
    buf = tf2_ros.Buffer(cache_time=rclpy.duration.Duration(seconds=3600))
    want = [STVL, CAM, POSE, '/tf', '/tf_static']
    r = rosbag2_py.SequentialReader()
    r.open(rosbag2_py.StorageOptions(uri=args.bag, storage_id='mcap'),
           rosbag2_py.ConverterOptions('', ''))
    types = {t.name: t.type for t in r.get_all_topics_and_types()}
    r.set_filter(rosbag2_py.StorageFilter(topics=[t for t in want if t in types]))
    cls = {t: get_message(types[t]) for t in want if t in types}

    def mat(tr):
        q = tr.transform.rotation; tv = tr.transform.translation
        w, x, y, z = q.w, q.x, q.y, q.z
        R = np.array([[1-2*(y*y+z*z), 2*(x*y-z*w), 2*(x*z+y*w)],
                      [2*(x*y+z*w), 1-2*(x*x+z*z), 2*(y*z-x*w)],
                      [2*(x*z-y*w), 2*(y*z+x*w), 1-2*(x*x+y*y)]])
        return R, np.array([tv.x, tv.y, tv.z])

    t0 = None
    poses = []
    pending_cam = None        # 最近一帧相机点云（map 系）
    KINDS = ('real', 'ground', 'ghost_in_fov', 'ghost_out_fov')
    n_cell = {k: 0 for k in KINDS}
    n_frame = {k: 0 for k in KINDS}; n_frame.update({'any': 0, 'tot': 0})
    near_block = {k: 0 for k in KINDS}   # <1.0 m 的挡路格
    ground_z = []
    k = 0
    while r.has_next():
        tp, data, stamp = r.read_next()
        t = stamp / 1e9
        if t0 is None:
            t0 = t
        rel = t - t0
        if rel > args.t1:
            break
        m = deserialize_message(data, cls[tp])
        if tp == '/tf_static':
            for tr in m.transforms:
                buf.set_transform_static(tr, 'bag')
            continue
        if tp == '/tf':
            for tr in m.transforms:
                buf.set_transform(tr, 'bag')
            continue
        if rel < args.t0:
            continue
        if tp == POSE:
            q = m.pose.orientation
            yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y ** 2 + q.z ** 2))
            poses.append((rel, m.pose.position.x, m.pose.position.y, yaw))
        elif tp == CAM:
            try:
                tr = buf.lookup_transform('map', m.header.frame_id,
                                          rclpy.time.Time.from_msg(m.header.stamp))
            except Exception:
                continue
            R, tv = mat(tr)
            a = pc2.read_points(m, field_names=('x', 'y', 'z'), skip_nans=True)
            p = np.stack([a['x'], a['y'], a['z']], -1).astype(float)
            # 同时留下 map->相机光学系 的逆变换，用来判断某个格子在不在视锥里
            pending_cam = (rel, (R @ p.T).T + tv, R.T, tv)
        elif tp == STVL:
            k += 1
            if k % args.stride or pending_cam is None or not poses:
                continue
            if abs(pending_cam[0] - rel) > 0.3:
                continue
            cx, cy, yaw = poses[-1][1], poses[-1][2], poses[-1][3]
            md = m.metadata; res = md.resolution
            ox, oy = md.origin.position.x, md.origin.position.y
            arr = np.frombuffer(bytes(m.data), dtype=np.uint8).reshape(md.size_y, md.size_x)
            ys, xs = np.nonzero(arr == 254)
            wx = ox + (xs + 0.5) * res; wy = oy + (ys + 0.5) * res
            d = np.hypot(wx - cx, wy - cy)
            b = np.degrees(np.arctan2(wy - cy, wx - cx) - yaw); b = (b + 180) % 360 - 180
            f = (d < args.radius) & (np.abs(b) < args.fov)
            pm = pending_cam[1]
            Rinv, cam_t = pending_cam[2], pending_cam[3]
            dd = np.hypot(pm[:, 0] - cx, pm[:, 1] - cy)
            bb = np.degrees(np.arctan2(pm[:, 1] - cy, pm[:, 0] - cx) - yaw)
            bb = (bb + 180) % 360 - 180
            near = (dd < args.radius + 0.5) & (np.abs(bb) < args.fov + 10)
            hi = pm[near & (pm[:, 2] >= args.tall) & (pm[:, 2] < 1.8)][:, :2]
            lo = pm[near & (pm[:, 2] >= args.min_h) & (pm[:, 2] < args.tall)][:, :2]
            # 真地面的 z 误差：该方位角 ±3° 内最高点 <tall 的列，取其 z
            if near.sum() > 100:
                zb = pm[near][:, 2]
                ab = bb[near]
                bins = np.arange(-args.fov, args.fov + 1, 6)
                who = np.digitize(ab, bins)
                for bi in np.unique(who):
                    s = who == bi
                    if s.sum() > 20 and zb[s].max() < args.tall:
                        ground_z.append(np.percentile(zb[s], [50, 95, 99]))
            n_frame['tot'] += 1
            got = set()
            for wx_, wy_, d_ in zip(wx[f], wy[f], d[f]):
                if len(hi) and np.min(np.hypot(hi[:, 0] - wx_, hi[:, 1] - wy_)) <= args.tol:
                    c = 'real'
                elif len(lo) and np.min(np.hypot(lo[:, 0] - wx_, lo[:, 1] - wy_)) <= args.tol:
                    c = 'ground'
                else:
                    # 视锥判定：格子在 [0.15,1.8] 任一高度上落进相机水平/垂直 FOV 就算"看得见"
                    # (realsense_clear 的 horizontal_fov_angle=1.518rad / vertical=1.012rad)
                    hs = np.arange(0.15, 1.8, 0.1)
                    q = Rinv @ (np.stack([np.full_like(hs, wx_), np.full_like(hs, wy_), hs], -1)
                                - cam_t).T          # -> 相机光学系: z 前, x 右, y 下
                    rr = np.linalg.norm(q, axis=0)
                    ha = np.abs(np.arctan2(q[0], q[2]))
                    va = np.abs(np.arctan2(q[1], q[2]))
                    vis = (q[2] > 0.2) & (rr < 3.5) & (ha < 1.518 / 2) & (va < 1.012 / 2)
                    c = 'ghost_in_fov' if vis.any() else 'ghost_out_fov'
                n_cell[c] += 1; got.add(c)
                if d_ < 1.0:
                    near_block[c] += 1
            for c in got:
                n_frame[c] += 1
            if got:
                n_frame['any'] += 1

    tot = sum(n_cell.values())
    print(f'采样 {n_frame["tot"]} 帧，前方 ±{args.fov:.0f}° / {args.radius} m 内共 {tot} 个 STVL 254 格')
    if tot:
        for c, name in (('real', '真障碍(有>=%.2fm高点)' % args.tall),
                        ('ground', '地面误检(只有%.2f~%.2fm矮点)' % (args.min_h, args.tall)),
                        ('ghost_in_fov', '残影·在相机视锥内(清除失效)'),
                        ('ghost_out_fov', '残影·视锥外/盲区(只能等衰减)')):
            print(f'  {name:32s} 格 {n_cell[c]:7d} ({100*n_cell[c]/tot:4.1f}%)   '
                  f'出现在 {n_frame[c]:5d} 帧 ({100*n_frame[c]/n_frame["tot"]:4.1f}%)   '
                  f'其中 <1.0m 挡路 {near_block[c]}')
    if ground_z:
        G = np.array(ground_z)
        print(f'\n=== 真地面列的 z 分布 (n={len(G)} 个方位列) ===')
        print(f'  该列 z 中位数: p50={np.median(G[:,0]):.3f}  p95={np.percentile(G[:,0],95):.3f}')
        print(f'  该列 z 的 p95: p50={np.median(G[:,1]):.3f}  p95={np.percentile(G[:,1],95):.3f}  max={G[:,1].max():.3f}')
        print(f'  该列 z 的 p99: p50={np.median(G[:,2]):.3f}  p95={np.percentile(G[:,2],95):.3f}  max={G[:,2].max():.3f}')
        over = 100 * (G[:, 1] > args.min_h).mean()
        print(f'  地面列里 p95 高度已超过 min_obstacle_height({args.min_h}) 的占比: {over:.1f}%')
    return 0


if __name__ == '__main__':
    sys.exit(main())
