#!/usr/bin/env python3
"""D435 假障碍取证：把撑起 STVL lethal 格的深度点反投影回像素与射线几何。

    python3 d435_false_obstacle_probe.py <bag> [--t0 S] [--t1 S] [--prefix /local_costmap]

回答三个问题：
  1. 这些点在深度图的哪个位置？（u,v 像素；顶部几行 = 掠射远地面的双目误匹配区）
  2. 射线与地面的夹角多小？（掠射角越小，亮面地板越容易误匹配）
  3. 这条射线按真实地面算应该打到多远？实测距离是不是明显偏短？
     （偏短 = 深度被"提前截断"，反投影后点就浮到地面以上）
以及这些格子在窗口里存在了多久（稳定 vs 闪烁）——稳定的错深度结构是没法靠时间滤波去掉的。

默认只看"静态地图里是空地"的格子（真墙/真家具不算），需要 bag 里有 /map。
"""
import argparse, math, sys
from collections import defaultdict
import numpy as np
import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('bag')
    ap.add_argument('--t0', type=float, default=0.0)
    ap.add_argument('--t1', type=float, default=1e9)
    ap.add_argument('--prefix', default='/local_costmap')
    ap.add_argument('--radius', type=float, default=2.5)
    ap.add_argument('--stride', type=int, default=5)
    ap.add_argument('--tol', type=float, default=0.10)
    ap.add_argument('--map-tol', type=float, default=0.25)
    args = ap.parse_args()

    import tf2_ros, rclpy.time, rclpy.duration, sensor_msgs_py.point_cloud2 as pc2
    from scipy.ndimage import binary_dilation
    buf = tf2_ros.Buffer(cache_time=rclpy.duration.Duration(seconds=3600))
    STVL = args.prefix + '/stvl_voxel_layer_raw'
    CAM = '/camera/camera/depth/color/points'
    INFO = '/camera/camera/depth/camera_info'
    want = [STVL, CAM, INFO, '/base_link_pose', '/map', '/tf', '/tf_static']
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

    t0 = None; cam = None; pose = None; K = None; gmap = None
    rec = []                     # 每个"撑起幻影格"的点的取证信息
    cell_frames = defaultdict(set)
    nfr = 0
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
        if tp == '/map':
            gmap = m; continue
        if tp == INFO:
            K = (m.k[0], m.k[4], m.k[2], m.k[5], m.width, m.height); continue
        if rel < args.t0:
            continue
        if tp == '/base_link_pose':
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
            p = np.stack([a['x'], a['y'], a['z']], -1).astype(float)   # 光学系
            cam = (rel, p, (R @ p.T).T + tv, R, tv)
        elif tp == STVL and cam is not None and pose is not None and K is not None:
            nfr += 1
            if nfr % args.stride:
                continue
            md = m.metadata; res = md.resolution
            ox, oy = md.origin.position.x, md.origin.position.y
            arr = np.frombuffer(bytes(m.data), dtype=np.uint8).reshape(md.size_y, md.size_x)
            ys, xs = np.nonzero(arr == 254)
            if len(xs) == 0:
                continue
            wx = ox + (xs + 0.5) * res; wy = oy + (ys + 0.5) * res
            d = np.hypot(wx - pose[0], wy - pose[1])
            sel = d <= args.radius
            wx, wy, d = wx[sel], wy[sel], d[sel]
            if gmap is not None:
                mi = gmap.info
                occ = np.array(gmap.data, dtype=np.int8).reshape(mi.height, mi.width)
                if 'legal' not in dir():
                    rad = max(1, int(round(args.map_tol / mi.resolution)))
                    yy, xx = np.ogrid[-rad:rad + 1, -rad:rad + 1]
                    se = (xx ** 2 + yy ** 2) <= rad ** 2
                    legal = binary_dilation(occ >= 50, structure=se)
                gx = ((wx - mi.origin.position.x) / mi.resolution).astype(int)
                gy = ((wy - mi.origin.position.y) / mi.resolution).astype(int)
                ok = (gx >= 0) & (gx < mi.width) & (gy >= 0) & (gy < mi.height)
                keep = np.zeros(len(wx), bool)
                keep[ok] = ~legal[gy[ok], gx[ok]]
                wx, wy, d = wx[keep], wy[keep], d[keep]
            if len(wx) == 0:
                continue
            popt, pmap, Rm, tv = cam[1], cam[2], cam[3], cam[4]
            fx, fy, cx_, cy_, W, H = K
            for cwx, cwy, cd in zip(wx, wy, d):
                cell_frames[(round(cwx / res), round(cwy / res))].add(nfr)
                dist = np.hypot(pmap[:, 0] - cwx, pmap[:, 1] - cwy)
                hit = np.nonzero(dist <= args.tol)[0]
                if len(hit) == 0:
                    continue
                for i in hit[:20]:
                    X, Y, Z = popt[i]
                    if Z <= 0.05:
                        continue
                    u = fx * X / Z + cx_; v = fy * Y / Z + cy_
                    rng = math.sqrt(X * X + Y * Y + Z * Z)
                    dirm = (pmap[i] - tv) / max(rng, 1e-6)
                    graz = math.degrees(math.asin(max(-1, min(1, -dirm[2]))))  # 射线下倾角
                    # 这条射线按真实地面(z=0)算应该打到多远
                    exp_r = (tv[2] / -dirm[2]) if dirm[2] < -1e-6 else float('inf')
                    rec.append((v, u, rng, exp_r, graz, pmap[i][2], cd))
    if not rec:
        print('窗口内没有"地图空地上的 STVL 格"，或没有点支撑它们'); return 0
    A = np.array(rec)
    print(f'撑起幻影格的深度点样本 {len(A)} 个（{len(cell_frames)} 个格子，采样 {nfr//args.stride} 帧）')
    print(f'深度图 {int(K[4])}x{int(K[5])}')
    v, u = A[:, 0], A[:, 1]
    print(f'\n=== 像素位置 ===')
    print(f'  行 v(0=顶部): p5={np.percentile(v,5):.0f} p25={np.percentile(v,25):.0f} '
          f'p50={np.percentile(v,50):.0f} p75={np.percentile(v,75):.0f} p95={np.percentile(v,95):.0f}'
          f'   (图高 {int(K[5])})   落在最上 1/4 的占 {100*(v<K[5]/4).mean():.0f}%')
    print(f'  列 u(0=左): p5={np.percentile(u,5):.0f} p50={np.percentile(u,50):.0f} '
          f'p95={np.percentile(u,95):.0f}   (图宽 {int(K[4])})'
          f'   落在最左/最右 1/5 的占 {100*((u<K[4]/5)|(u>4*K[4]/5)).mean():.0f}%')
    print(f'\n=== 射线几何 ===')
    print(f'  下倾角(度,越小越掠射): p5={np.percentile(A[:,4],5):.1f} p50={np.percentile(A[:,4],50):.1f} '
          f'p95={np.percentile(A[:,4],95):.1f}')
    print(f'  实测距离 p50={np.median(A[:,2]):.2f} m   按地面应打到 p50={np.median(A[:,3]):.2f} m   '
          f'比值 实测/应到 p50={np.median(A[:,2]/np.maximum(A[:,3],1e-6)):.2f}')
    print(f'  反投影后的 map 高度: p50={np.median(A[:,5]):.2f} m  p95={np.percentile(A[:,5],95):.2f} m')
    print(f'  格子离机器人: p50={np.median(A[:,6]):.2f} m')
    life = np.array([len(s) for s in cell_frames.values()])
    print(f'\n=== 格子稳定性（被几个采样帧看到，共 {nfr//args.stride} 帧）===')
    print(f'  1帧={int((life==1).sum())}  2~5帧={int(((life>1)&(life<6)).sum())}  '
          f'6~20帧={int(((life>=6)&(life<21)).sum())}  >20帧={int((life>20).sum())}  最长={life.max()}帧')
    return 0


if __name__ == '__main__':
    sys.exit(main())
