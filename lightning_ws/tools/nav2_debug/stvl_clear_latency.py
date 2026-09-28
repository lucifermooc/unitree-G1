#!/usr/bin/env python3
"""STVL 的清除延迟：一个格子"最后一次被看到有东西"之后，过多久才消失。

    python3 stvl_clear_latency.py <bag> [--prefix ""] [--t0 S] [--t1 S]

这是判断"D435 障碍不更新"最直接的数字：
  * 延迟 ≈ voxel_decay  -> 清除没生效，格子是自然衰减到期才没的
  * 延迟 << voxel_decay -> 视锥清除在工作
同时只统计"机器人当时看得见该格"的情形（格子在相机视锥内），盲区里的保留是设计行为、不算延迟。
"""
import argparse, math, sys
import numpy as np
import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('bag')
    ap.add_argument('--prefix', default='/local_costmap')
    ap.add_argument('--t0', type=float, default=0.0)
    ap.add_argument('--t1', type=float, default=1e9)
    ap.add_argument('--radius', type=float, default=2.0)
    ap.add_argument('--tol', type=float, default=0.12)
    # 只统计"相机确实看得见"的格子：近处盲区(<0.8m 时矮体素落在视场下缘外)和
    # 视野边缘的格子留着是设计行为，不该算成"清除失败"。
    ap.add_argument('--dmin', type=float, default=0.0)
    ap.add_argument('--dmax', type=float, default=99.0)
    ap.add_argument('--fov', type=float, default=180.0, help='只看前方 ±fov 度')
    args = ap.parse_args()

    import tf2_ros, rclpy.time, rclpy.duration, sensor_msgs_py.point_cloud2 as pc2
    buf = tf2_ros.Buffer(cache_time=rclpy.duration.Duration(seconds=3600))
    STVL = args.prefix + '/stvl_voxel_layer_raw'
    CAM = '/camera/camera/depth/color/points'
    want = [STVL, CAM, '/base_link_pose', '/tf', '/tf_static']
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

    t0 = None; cam = None; pose = None
    last_seen = {}          # cell -> 最后一次有点支撑的时刻
    alive = {}
    lat = []                # 清除延迟
    still = 0
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
        elif tp == '/tf':
            for tr in m.transforms:
                buf.set_transform(tr, 'bag')
        elif rel < args.t0:
            continue
        elif tp == '/base_link_pose':
            q = m.pose.orientation
            yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y ** 2 + q.z ** 2))
            pose = (m.pose.position.x, m.pose.position.y, yaw)
        elif tp == CAM:
            try:
                tr = buf.lookup_transform('map', m.header.frame_id,
                                          rclpy.time.Time.from_msg(m.header.stamp))
            except Exception:
                continue
            R, tv = mat(tr)
            a = pc2.read_points(m, field_names=('x', 'y', 'z'), skip_nans=True)
            p = np.stack([a['x'], a['y'], a['z']], -1).astype(float)
            pm = (R @ p.T).T + tv
            cam = (rel, pm[(pm[:, 2] > 0.15) & (pm[:, 2] < 1.8)][:, :2])
        elif tp == STVL and cam is not None and pose is not None:
            md = m.metadata; res = md.resolution
            ox, oy = md.origin.position.x, md.origin.position.y
            arr = np.frombuffer(bytes(m.data), dtype=np.uint8).reshape(md.size_y, md.size_x)
            ys, xs = np.nonzero(arr == 254)
            wx = ox + (xs + 0.5) * res; wy = oy + (ys + 0.5) * res
            d = np.hypot(wx - pose[0], wy - pose[1])
            b = np.degrees(np.arctan2(wy - pose[1], wx - pose[0]) - pose[2])
            b = (b + 180) % 360 - 180
            sel = (d <= args.radius) & (d >= args.dmin) & (d <= args.dmax) & (np.abs(b) <= args.fov)
            wx, wy = wx[sel], wy[sel]
            cur = set(zip(np.round(wx / res).astype(np.int64).tolist(),
                          np.round(wy / res).astype(np.int64).tolist()))
            pts = cam[1]
            for c in cur:
                cwx, cwy = c[0] * res, c[1] * res
                if len(pts) and np.min(np.hypot(pts[:, 0] - cwx, pts[:, 1] - cwy)) <= args.tol:
                    last_seen[c] = rel
                alive.setdefault(c, rel)
            for c in list(alive):
                if c not in cur:
                    ls = last_seen.pop(c, None)
                    alive.pop(c, None)
                    if ls is not None:
                        lat.append(rel - ls)
            still = len(alive)
    if lat:
        L = np.array(lat)
        print(f'样本 {len(L)} 个格子  清除延迟(最后一次有点 -> 格子消失):')
        print('  ' + '  '.join(f'p{p}={np.percentile(L,p):.2f}s' for p in (10, 25, 50, 75, 90, 99)))
        print(f'  >2s 的占 {100*(L>2).mean():.1f}%   >3s 的占 {100*(L>3).mean():.1f}%   最大 {L.max():.1f}s')
    else:
        print('没有可统计的格子')
    print(f'  窗口结束时仍存活 {still} 格')
    return 0


if __name__ == '__main__':
    sys.exit(main())
