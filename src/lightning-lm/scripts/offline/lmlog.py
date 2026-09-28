#!/usr/bin/env python3
"""Lightning 建图/定位 glog 分析工具（合并原 locdrift / matchoff / ndtlag / liocmp / kfdelta / roomkf / locana）。

定位（run_loc_offline / run_loc_online 日志）:
  lmlog.py tfdrift <tf.txt> <log> [步长秒=0]  逐条定位输出（TF 输出，带时间戳）相对起步对齐的 LIO 轨迹的漂移（主指标）
  lmlog.py online  <目录>                在线回放（online_replay.sh）：实际发布的 TF 逐条漂移、平面 TF 检查、/base_link_pose 一致性
  lmlog.py tfref   <tf.txt> <ref tf.txt> x y yaw [步长秒=0]  与参考轨迹（自有地图定位 + gridreg.py 两图配准）逐条对比
  lmlog.py summary <log>                 一行汇总：定位输出漂移峰值/结束值、>1 m 秒数、低分匹配数（离线 A/B 对比用）
  lmlog.py outdrift <log> [步长秒=10]    定位输出相对起步对齐的 LIO 轨迹的误差随时间（1 s 中位数）
  lmlog.py offset  <log> [步长秒=5]      T_map_lio = 匹配位姿 × LIO^-1（2D）随时间的变化；两者都对时应为常量
  lmlog.py drift   <log> [窗口秒=10]     按数据时间分窗：LIO 航向相对匹配的漂移、输出与匹配偏差、输出跳变、分值
  lmlog.py ndtlag  <log>                 NDT 结果 - LO 预测，分解到沿程/横向（行走帧的系统偏差）
  lmlog.py wall    <log> [窗口秒=30]     按墙钟时间分窗：LIO 滞后、匹配与 hf 输出偏差、PGO/IMU 断档计数
  lmlog.py liocmp  <logA> <logB> [步长秒=20]  两份日志同一数据时刻的 LIO 状态对比
建图（run_slam_offline / run_slam_online 日志）:
  lmlog.py kfdelta <log> <kf起> <kf止> [步长=5]         关键帧回环优化后位姿 - LIO 创建时位姿
  lmlog.py roomkf  <log> x0 y0 x1 y1 [分值门限=1.3]    区域内关键帧的访问段及涉及的回环候选
"""
import datetime
import math
import re
import sys
import time

NUM = r'([-+0-9.eE]+)'
RE_END = re.compile(r'LIO get cloud at beg: ' + NUM + r', end: ' + NUM)
RE_LIO = re.compile(r'LIO state:\s+' + r'\s+'.join([NUM] * 3) + r', yaw ' + NUM + r'(?:, vel:\s+' + r'\s+'.join([NUM] * 3) + ')?')
RE_CF = re.compile(r'confidence: ' + NUM + r', t:\s+' + r'\s+'.join([NUM] * 3) + r', succ: (\d), rpy\(deg\):\s+' + r'\s+'.join([NUM] * 3))
RE_HF = re.compile(r'hf output t:\s+' + r'\s+'.join([NUM] * 3) + r', rpy\(deg\):\s+' + r'\s+'.join([NUM] * 3))
RE_LAST = re.compile(r'last abs pose:\s+' + r'\s+'.join([NUM] * 3))
RE_GUESS = re.compile(r'loc using lo guess:\s+' + r'\s+'.join([NUM] * 3))
RE_WALL = re.compile(r'^[IWEF](\d{8}) (\d\d:\d\d:\d\d\.\d+)')


def wrap(a):
    return (a + 180) % 360 - 180


def floats(m):
    return [float(v) if v is not None else 0.0 for v in m.groups()]


def lines(path):
    return open(path, errors='ignore')


def match_offsets(path):
    """逐次激光匹配：(数据时间, 匹配结果, 当时 LIO 状态, T_map_lio 的 x/y/yaw)"""
    t = lio = None
    for line in lines(path):
        if (m := RE_END.search(line)):
            t = float(m.group(2))
        elif (m := RE_LIO.search(line)):
            lio = floats(m)
        elif t is not None and lio and (m := RE_CF.search(line)):
            c = floats(m)
            yo = math.radians(c[7] - lio[3])
            ox = c[1] - (math.cos(yo) * lio[0] - math.sin(yo) * lio[1])
            oy = c[2] - (math.sin(yo) * lio[0] + math.cos(yo) * lio[1])
            yield t, c, lio, (ox, oy, math.degrees(yo))


def output_samples(path):
    """(数据时间, 定位输出 x/y/yaw, LIO x/y/yaw)。定位输出即 hf output（发布到 TF 的 T_map_lidar），每个 LIO 帧取其后第一条"""
    t = lio = None; want = False
    for line in lines(path):
        if (m := RE_END.search(line)):
            t = float(m.group(2))
        elif (m := RE_LIO.search(line)):
            lio = floats(m); want = True
        elif want and t is not None and (m := RE_HF.search(line)):
            h = floats(m); want = False
            yield t, (h[0], h[1], h[5]), (lio[0], lio[1], lio[3])


def offset_drift(samples, settle=5.0):
    """定位输出相对"起步时对齐到地图的 LIO 轨迹"的误差，按 1 s 取中位数。返回 [(秒, 位置误差 m, 航向误差 deg)]。
    对齐 T_map_lio 取数据 settle 秒后第一秒（静止）的中位数并固定，位置误差 = |输出 - T_map_lio·LIO|，
    不用逐帧 T_map_lio 的平移：输出与 LIO 相差 0~0.1 s，转身时航向差乘以离原点距离会造成数米假偏差。
    LIO 本身在这两份包上的漂移 < 0.35 m（自建图回放测得），因此该值近似定位误差。"""
    samples = list(samples)
    if not samples:
        return []
    t0 = samples[0][0]
    med = lambda v: sorted(v)[len(v) // 2]
    ref_set = [s for s in samples if settle <= s[0] - t0 < settle + 1] or samples[:10]
    ryaw = med([wrap(h[2] - l[2]) for _, h, l in ref_set])
    c, sn = math.cos(math.radians(ryaw)), math.sin(math.radians(ryaw))
    rx = med([h[0] - (c * l[0] - sn * l[1]) for _, h, l in ref_set])
    ry = med([h[1] - (sn * l[0] + c * l[1]) for _, h, l in ref_set])
    secs = {}
    for t, h, l in samples:
        ex = h[0] - (c * l[0] - sn * l[1] + rx); ey = h[1] - (sn * l[0] + c * l[1] + ry)
        secs.setdefault(int(t - t0), []).append((math.hypot(ex, ey), wrap(h[2] - l[2] - ryaw)))
    return [(s, med([e[0] for e in v]), med([e[1] for e in v])) for s, v in sorted(secs.items())]


def cmd_summary(path, conf_th='1.0'):
    conf_th = float(conf_th)
    confs = [c[0] for _, c, _, _ in match_offsets(path)]
    rows = offset_drift(output_samples(path))
    if not confs or not rows:
        print(f'{path}: no matches/output'); return
    n = len(confs); low = sum(c < conf_th for c in confs); confs.sort()
    peak = max(rows, key=lambda r: r[1]); yaw = max(abs(r[2]) for r in rows)
    over1 = sum(r[1] > 1.0 for r in rows)
    print(f'matches {n} conf median {confs[n // 2]:.2f} min {confs[0]:.2f} <{conf_th:g}: {low} | '
          f'output drift peak {peak[1]:.2f} m ({peak[2]:+.1f} deg) at t+{peak[0]}s, max yaw {yaw:.1f} deg, '
          f'>1m {over1}s of {len(rows)}s | end {rows[-1][1]:.2f} m {rows[-1][2]:+.1f} deg')


def load_tf(path):
    """run_loc_offline --tf_out 或 tfrec.py 记录的逐条定位输出: stamp x y z qx qy qz qw -> [(t, x, y, yaw_deg)]"""
    out = []
    for line in open(path):
        v = line.split()
        if len(v) < 8:
            continue
        t, x, y, _, qx, qy, qz, qw = map(float, v[:8])
        out.append((t, x, y, math.degrees(math.atan2(2 * (qw * qz + qx * qy), 1 - 2 * (qy * qy + qz * qz)))))
    return sorted(out)


def load_lio(path):
    """日志中每帧 LIO 位姿（时间取该帧 lidar_end_time）: [(t, x, y, yaw_deg)]"""
    t = None; out = []
    for line in lines(path):
        if (m := RE_END.search(line)):
            t = float(m.group(2))
        elif t is not None and (m := RE_LIO.search(line)):
            v = floats(m); out.append((t, v[0], v[1], v[3])); t = None
    return sorted(out)


def tf_errors(tf_path, log_path, settle=5.0):
    """每条定位输出相对"起步对齐到地图的 LIO 轨迹"（在该输出时刻插值）的误差。
    返回 [(相对时间 s, 位置误差 m, 航向误差 deg, 相对上一条输出的跳变 m)]"""
    import bisect
    tf, lio = load_tf(tf_path), load_lio(log_path)
    if not tf or len(lio) < 2:
        return []
    lt = [p[0] for p in lio]

    def lio_at(t):
        i = bisect.bisect_left(lt, t)
        if i == 0 or i >= len(lio) or lt[i] - lt[i - 1] > 0.3:
            return None
        a, b = lio[i - 1], lio[i]; r = (t - a[0]) / (b[0] - a[0])
        return (a[1] + r * (b[1] - a[1]), a[2] + r * (b[2] - a[2]), a[3] + r * wrap(b[3] - a[3]))

    pairs = [(h, l) for h in tf if (l := lio_at(h[0])) is not None]
    t0 = pairs[0][0][0]
    med = lambda v: sorted(v)[len(v) // 2]
    ref = [(h, l) for h, l in pairs if settle <= h[0] - t0 < settle + 1] or pairs[:50]
    ryaw = med([wrap(h[3] - l[2]) for h, l in ref])
    c, sn = math.cos(math.radians(ryaw)), math.sin(math.radians(ryaw))
    rx = med([h[1] - (c * l[0] - sn * l[1]) for h, l in ref])
    ry = med([h[2] - (sn * l[0] + c * l[1]) for h, l in ref])
    rows = []; prev = None
    for h, l in pairs:
        px, py = c * l[0] - sn * l[1] + rx, sn * l[0] + c * l[1] + ry  # LIO 预测的地图位置
        ex, ey = h[1] - px, h[2] - py
        jump = math.hypot(ex - prev[0], ey - prev[1]) if prev else 0.0
        prev = (ex, ey)
        rows.append((h[0] - t0, math.hypot(ex, ey), wrap(h[3] - l[2] - ryaw), jump))
    return rows


def cmd_tfdrift(tf_path, log_path, step='0'):
    """逐条定位输出的漂移。step>0 时每 step 秒打印该时段最大值"""
    rows = tf_errors(tf_path, log_path)
    if not rows:
        print('no tf/lio data'); return
    step = float(step)
    if step > 0:
        k0 = -1; buf = []
        for r in rows + [(1e18, 0, 0, 0)]:
            k = int(r[0] // step)
            if k != k0 and buf:
                print(f't+{k0 * step:5.0f}s err max {max(b[1] for b in buf):5.2f} m, yaw max {max(abs(b[2]) for b in buf):5.1f} deg, '
                      f'jump max {max(b[3] for b in buf):.3f} m ({len(buf)} outputs)')
                buf = []
            k0 = k; buf.append(r)
    errs = sorted(r[1] for r in rows); n = len(rows)
    over = sum(rows[i + 1][0] - rows[i][0] for i in range(n - 1) if rows[i][1] > 1.0)
    over05 = sum(rows[i + 1][0] - rows[i][0] for i in range(n - 1) if rows[i][1] > 0.5)
    peak = max(rows, key=lambda r: r[1]); jump = max(rows, key=lambda r: r[3])
    print(f'outputs {n} over {rows[-1][0]:.0f}s | err peak {peak[1]:.2f} m at t+{peak[0]:.0f}s, p95 {errs[int(n * 0.95)]:.2f} m, '
          f'median {errs[n // 2]:.2f} m | >0.5m {over05:.0f}s, >1m {over:.0f}s | yaw max {max(abs(r[2]) for r in rows):.1f} deg | '
          f'max jump {jump[3]:.3f} m at t+{jump[0]:.0f}s | end {rows[-1][1]:.2f} m {rows[-1][2]:+.1f} deg')


def _quat_mul(a, b):
    ax, ay, az, aw = a; bx, by, bz, bw = b
    return (aw * bx + ax * bw + ay * bz - az * by, aw * by - ax * bz + ay * bw + az * bx,
            aw * bz + ax * by - ay * bx + az * bw, aw * bw - ax * bx - ay * by - az * bz)


def _rot(q, v):
    x, y, z, w = q
    r = _quat_mul(_quat_mul(q, (v[0], v[1], v[2], 0.0)), (-x, -y, -z, w))
    return r[:3]


def _compose(A, B):
    """A、B 为 (t, q)，返回 A*B"""
    ta, qa = A; tb, qb = B
    rt = _rot(qa, tb)
    return (ta[0] + rt[0], ta[1] + rt[1], ta[2] + rt[2]), _quat_mul(qa, qb)


def _rpy(q):
    x, y, z, w = q
    roll = math.atan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y))
    pitch = math.asin(max(-1.0, min(1.0, 2 * (w * y - z * x))))
    yaw = math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))
    return [math.degrees(v) for v in (roll, pitch, yaw)]


def cmd_online(d):
    """online_replay.sh 输出目录：由实际发布的 map->base_link、base_link->body_link 与 /tf_static 还原 T_map_lidar，
    逐条统计漂移；检查平面 TF；检查 /base_link_pose 与 /tf 是否一致、TF 停发后是否仍在发旧值"""
    import bisect
    import os
    load = lambda n: [list(map(float, l.split())) for l in open(os.path.join(d, n)) if l.strip()]
    mb, bb, blp = load('map_base.txt'), load('base_body.txt'), load('blp.txt')
    static = {}
    for l in open(os.path.join(d, 'static.txt')):
        v = l.split()
        static[v[1]] = (v[0], (tuple(map(float, v[2:5])), tuple(map(float, v[5:9]))))
    # body_link -> mid360_link（可能经过中间帧）
    chain, f = [], 'mid360_link'
    while f != 'body_link':
        if f not in static:
            print(f'static tf chain body_link->mid360_link broken at {f}'); return
        chain.append(static[f][1]); f = static[f][0]
    T_body_lidar = ((0.0, 0.0, 0.0), (0.0, 0.0, 0.0, 1.0))
    for T in reversed(chain):
        T_body_lidar = _compose(T_body_lidar, T)
    bbs = {round(r[0], 6): r for r in bb}
    n_pair = 0; zmax = tiltmax = body_yaw = 0.0
    with open(os.path.join(d, 'tf.txt'), 'w') as out:
        for r in mb:
            b = bbs.get(round(r[0], 6))
            zmax = max(zmax, abs(r[3])); rp = _rpy(r[4:8]); tiltmax = max(tiltmax, abs(rp[0]), abs(rp[1]))
            if b is None:
                continue
            body_yaw = max(body_yaw, abs(_rpy(b[4:8])[2]))
            T = _compose(_compose((tuple(r[1:4]), tuple(r[4:8])), (tuple(b[1:4]), tuple(b[4:8]))), T_body_lidar)
            out.write(f'{r[0]:.6f} {T[0][0]:.6f} {T[0][1]:.6f} {T[0][2]:.6f} {T[1][0]:.8f} {T[1][1]:.8f} {T[1][2]:.8f} {T[1][3]:.8f}\n')
            n_pair += 1
    dur = mb[-1][8] - mb[0][8] if mb else 0
    print(f'TF: map->base_link {len(mb)} ({len(mb) / max(dur, 1e-9):.0f} Hz over {dur:.0f}s), paired with base->body {n_pair} | '
          f'planar check: |z| max {zmax:.4f} m, base_link roll/pitch max {tiltmax:.3f} deg, body_link yaw max {body_yaw:.3f} deg')
    gaps = [mb[i + 1][8] - mb[i][8] for i in range(len(mb) - 1)]
    if gaps:
        print(f'TF publish gaps (receive time): max {max(gaps):.3f}s, >0.2s: {sum(g > 0.2 for g in gaps)}')
    logp = next(p for p in (os.path.join(d, n) for n in ('run_loc_online.INFO', 'run_slam_online.INFO')) if os.path.exists(p))
    cmd_tfdrift(os.path.join(d, 'tf.txt'), logp)
    # /base_link_pose 对照：stamp 与某条 map->base_link 相同则按 stamp 配对（新版 robot_pose_pub 用 TF 时间戳）；
    # 否则（旧版 stamp=now()）取接收时刻之前最近一条，负载高时 tfrec 接收滞后会出现假差值
    recv = [r[8] for r in mb]
    by_stamp = {round(r[0], 6): r for r in mb}
    diffs, ages, n_stamp = [], [], 0
    for p in blp:
        r = by_stamp.get(round(p[0], 6))
        if r is not None:
            n_stamp += 1
        else:
            i = bisect.bisect_right(recv, p[8]) - 1
            if i < 0:
                continue
            r = mb[i]
        diffs.append((math.hypot(p[1] - r[1], p[2] - r[2]), abs(wrap(_rpy(p[4:8])[2] - _rpy(r[4:8])[2]))))
        ages.append(p[8] - mb[-1][8] if p[8] > mb[-1][8] else 0.0)
    if diffs:
        print(f'/base_link_pose {len(blp)} msgs, stamp == TF stamp: {n_stamp} | max diff vs paired TF '
              f'{max(x[0] for x in diffs):.3f} m / {max(x[1] for x in diffs):.2f} deg | '
              f'published after last TF: {sum(a > 0 for a in ages)} msgs, last {max(ages):.2f}s later | '
              f'last header.stamp - last TF stamp: {blp[-1][0] - mb[-1][0]:.3f}s')


def cmd_tfref(tf_path, ref_path, x, y, yaw, step='0'):
    """定位输出与参考轨迹逐条对比。参考 = 同一录包在自己地图上的定位输出（ref tf.txt），经 T_A_B（gridreg.py 求出的
    两图配准，x y yaw_deg）换到本图坐标，在每条输出的时刻插值。与 LIO 参考不同，不受起步对齐和 LIO 漂移影响。"""
    import bisect
    tf, ref = load_tf(tf_path), load_tf(ref_path)
    c, sn, ya = math.cos(math.radians(float(yaw))), math.sin(math.radians(float(yaw))), float(yaw)
    rt = [r[0] for r in ref]
    rows = []
    for h in tf:
        i = bisect.bisect_left(rt, h[0])
        if i == 0 or i >= len(ref) or rt[i] - rt[i - 1] > 0.3:
            continue
        a, b = ref[i - 1], ref[i]; k = (h[0] - a[0]) / (b[0] - a[0])
        px, py = a[1] + k * (b[1] - a[1]), a[2] + k * (b[2] - a[2]); pyaw = a[3] + k * wrap(b[3] - a[3])
        gx, gy = c * px - sn * py + float(x), sn * px + c * py + float(y)
        rows.append((h[0], math.hypot(h[1] - gx, h[2] - gy), wrap(h[3] - pyaw - ya)))
    if not rows:
        print('no overlap'); return
    t0 = rows[0][0]; n = len(rows); errs = sorted(r[1] for r in rows)
    step = float(step)
    if step > 0:
        k0 = None; buf = []
        for r in rows + [(1e18, 0, 0)]:
            k = int((r[0] - t0) // step)
            if k != k0 and buf:
                print(f't+{k0 * step:5.0f}s err max {max(b[1] for b in buf):5.2f} m, yaw max {max(abs(b[2]) for b in buf):5.1f} deg')
                buf = []
            k0 = k; buf.append(r)
    dt = lambda th: sum(rows[i + 1][0] - rows[i][0] for i in range(n - 1) if rows[i][1] > th)
    peak = max(rows, key=lambda r: r[1])
    print(f'outputs {n} | err vs ref peak {peak[1]:.2f} m at t+{peak[0] - t0:.0f}s, p95 {errs[int(n * 0.95)]:.2f} m, '
          f'median {errs[n // 2]:.2f} m | >0.5m {dt(0.5):.0f}s, >1m {dt(1.0):.0f}s | '
          f'yaw max {max(abs(r[2]) for r in rows):.1f} deg | end {rows[-1][1]:.2f} m {rows[-1][2]:+.1f} deg')


def cmd_outdrift(path, step='10'):
    """定位输出漂移随时间（1 s 中位数，每 step 秒打印一行）"""
    for s, dxy, dyaw in offset_drift(output_samples(path)):
        if s % int(step) == 0:
            print(f't+{s:4d}s output drift {dxy:5.2f} m {dyaw:+6.1f} deg')


def cmd_offset(path, step='5'):
    step = float(step)
    t0 = ref = None; last = -1e9; minc = 9; nfail = 0
    for t, c, lio, off in match_offsets(path):
        if ref is None:
            ref, t0 = off, t
        minc = min(minc, c[0]); nfail += c[4] == 0
        if t - last >= step:
            print(f't+{t - t0:5.0f}s offset dxy {math.hypot(off[0] - ref[0], off[1] - ref[1]):5.2f}m '
                  f'dyaw {wrap(off[2] - ref[2]):6.1f} | conf min {minc:.2f} fail {nfail} | '
                  f'lio {lio[0]:6.1f},{lio[1]:6.1f} y{lio[3]:7.1f} v {math.hypot(lio[4], lio[5]):.2f}')
            last = t; minc = 9; nfail = 0


def cmd_drift(path, win='10'):
    W = float(win)
    t = t0 = lio = off0 = b = prev_hf = None
    tot = dict(maxdrift=0, maxdiff=0, maxdang=0, maxjump=0, minconf=9)

    def flush(b):
        if not b or not b['n']:
            return
        print(f"t+{b['t']:6.0f}s lio-drift {b['drift']:6.1f}deg | out-vs-match max {b['diff']:4.2f}m {b['dang']:5.1f}deg | "
              f"out jump max {b['jump']:4.2f}m | conf min {b['conf']:.2f} | match {b['cx']:7.2f},{b['cy']:7.2f} y{b['cyaw']:7.1f}")

    for line in lines(path):
        if (m := RE_END.search(line)):
            t = float(m.group(2)); t0 = t0 or t
        elif (m := RE_LIO.search(line)):
            lio = floats(m)
        elif t is not None and lio and (m := RE_CF.search(line)):
            c = floats(m)
            off = wrap(c[7] - lio[3])
            if off0 is None:
                off0 = off
            if b is None or t - b['t0abs'] >= W:
                flush(b); b = dict(t0abs=t, t=t - t0, drift=0, diff=0, dang=0, jump=0, conf=9, n=0, cx=0, cy=0, cyaw=0)
            b['n'] += 1; d = wrap(off - off0)
            if abs(d) > abs(b['drift']):
                b['drift'] = d
            b['conf'] = min(b['conf'], c[0]); b['cx'], b['cy'], b['cyaw'] = c[1], c[2], c[7]; b['match'] = c
            tot['maxdrift'] = max(tot['maxdrift'], abs(d)); tot['minconf'] = min(tot['minconf'], c[0])
        elif b is not None and 'match' in b and (m := RE_HF.search(line)):
            h = floats(m); c = b['match']
            dxy = math.hypot(h[0] - c[1], h[1] - c[2]); da = abs(wrap(h[5] - c[7]))
            b['diff'] = max(b['diff'], dxy); b['dang'] = max(b['dang'], da)
            tot['maxdiff'] = max(tot['maxdiff'], dxy); tot['maxdang'] = max(tot['maxdang'], da)
            if prev_hf:
                j = math.hypot(h[0] - prev_hf[0], h[1] - prev_hf[1])
                b['jump'] = max(b['jump'], j); tot['maxjump'] = max(tot['maxjump'], j)
            prev_hf = h
    flush(b)
    print('TOTAL', ' '.join(f'{k}={v:.2f}' for k, v in tot.items()))


def cmd_ndtlag(path):
    last = guess = None; along = []; cross = []
    for line in lines(path):
        if (m := RE_LAST.search(line)):
            last = floats(m)
        elif (m := RE_GUESS.search(line)):
            guess = floats(m)
        elif (m := RE_CF.search(line)) and last and guess:
            c = floats(m)
            mx, my = guess[0] - last[0], guess[1] - last[1]; step = math.hypot(mx, my)
            if step > 0.05 and c[0] > 1.0:  # 行走帧（每帧 >5 cm）且匹配正常
                ux, uy = mx / step, my / step; dx, dy = c[1] - guess[0], c[2] - guess[1]
                along.append(dx * ux + dy * uy); cross.append(-dx * uy + dy * ux)
            last = guess = None
    if along:
        s = sorted(along)
        print(f'walking frames {len(along)}: NDT-guess along-track mean {sum(along) / len(along):+.3f} m, '
              f'median {s[len(s) // 2]:+.3f} m, cross-track mean {sum(cross) / len(cross):+.3f} m')


def cmd_wall(path, win='30'):
    W = float(win); b = None

    def wall(m):
        d = datetime.datetime.strptime(m.group(1) + ' ' + m.group(2), '%Y%m%d %H:%M:%S.%f')
        return time.mktime(d.timetuple()) + d.microsecond * 1e-6

    def flush(b):
        if not b:
            return
        hf, cf = b['hf'], b['cf']
        s = f"{b['hms']} lag {max(b['lag'] or [0]):5.2f}s"
        if hf and cf:
            h, c = hf[-1], cf[-1]
            dxy = math.hypot(h[0] - c[1], h[1] - c[2]); dy = wrap(h[5] - c[7])
            s += (f" | loc {c[1]:6.2f},{c[2]:6.2f} y{c[7]:6.1f} conf {min(x[0] for x in cf):.2f}-{max(x[0] for x in cf):.2f}"
                  f" fail {sum(1 for x in cf if not x[4])}/{len(cf)} | hf {h[0]:6.2f},{h[1]:6.2f} y{h[5]:6.1f}"
                  f" | diff {dxy:4.2f}m {dy:5.1f}deg")
        s += f" | pgoDRback {b['pgo']} abn {b['abn']} lidarbreak {b['brk']}"
        if b['lio']:
            s += f" | lio {b['lio'][-1][0]:6.2f},{b['lio'][-1][1]:6.2f} y{b['lio'][-1][3]:6.1f}"
        print(s)

    for line in lines(path):
        m = RE_WALL.match(line)
        if not m:
            continue
        t = wall(m)
        if b is None or t - b['t0'] >= W:
            flush(b); b = dict(t0=t, hms=m.group(2)[:8], lag=[], hf=[], cf=[], pgo=0, abn=0, brk=0, lio=[])
        if (x := RE_END.search(line)):
            b['lag'].append(t - float(x.group(2)))
        elif (x := RE_HF.search(line)):
            b['hf'].append(floats(x))
        elif (x := RE_CF.search(line)):
            b['cf'].append(floats(x))
        elif (x := RE_LIO.search(line)):
            b['lio'].append(floats(x))
        elif 'pgo.cc:151' in line:
            b['pgo'] += 1
        elif 'abnormal dt' in line:
            b['abn'] += 1
        elif '雷达断流' in line:
            b['brk'] += 1
    flush(b)


def cmd_liocmp(path_a, path_b, step='20'):
    def load(p):
        d, t = {}, None
        for line in lines(p):
            if (m := RE_END.search(line)):
                t = round(float(m.group(2)), 1)
            elif t is not None and (m := RE_LIO.search(line)):
                v = floats(m); d[t] = (v[0], v[1], v[3])
        return d

    A, B = load(path_a), load(path_b); step = float(step)
    ts = sorted(set(A) & set(B)); t0 = ts[0]; last = -1e9; mx = (0, 0)
    for t in ts:
        a, b = A[t], B[t]; dp = math.hypot(a[0] - b[0], a[1] - b[1]); dy = wrap(a[2] - b[2])
        mx = (max(mx[0], dp), max(mx[1], abs(dy)))
        if t - last >= step:
            print(f't+{t - t0:5.0f}s A {a[0]:7.2f},{a[1]:7.2f} y{a[2]:7.1f} | B {b[0]:7.2f},{b[1]:7.2f} y{b[2]:7.1f} | '
                  f'd {dp:5.2f}m {dy:6.1f}deg')
            last = t
    print(f'common frames {len(ts)}  max diff {mx[0]:.2f} m {mx[1]:.1f} deg')


def kf_poses(path):
    """(LIO 创建时位姿, 回环优化后位姿)，按关键帧 id"""
    lio, opt, create_time = {}, {}, {}
    for line in lines(path):
        if (m := re.search(r'^I\d+ (\S+) .*create kf (\d+), state:\s+(\S+)\s+(\S+)\s+(\S+)', line)):
            i = int(m.group(2))
            lio[i] = tuple(float(v.rstrip(',')) for v in m.group(3, 4, 5)); create_time[i] = m.group(1)[:8]
        elif (m := re.search(r'laser_mapping.cc:\d+\] kf (\d+), pose:\s+(\S+)\s+(\S+)\s+(\S+)', line)):
            opt[int(m.group(1))] = tuple(float(v.rstrip(',')) for v in m.group(2, 3, 4))
    return lio, opt, create_time


def cmd_kfdelta(path, a, b, step='5'):
    lio, opt, _ = kf_poses(path)
    for i in range(int(a), int(b) + 1, int(step)):
        if i in lio and i in opt:
            d = [opt[i][k] - lio[i][k] for k in range(3)]
            print(f'kf {i:4d} lio {lio[i][0]:7.2f},{lio[i][1]:7.2f} opt {opt[i][0]:7.2f},{opt[i][1]:7.2f},{opt[i][2]:5.2f} '
                  f'delta {d[0]:6.2f},{d[1]:6.2f},{d[2]:6.2f} |{math.hypot(d[0], d[1]):.2f}|')


def cmd_roomkf(path, x0, y0, x1, y1, th='1.3'):
    x0, y0, x1, y1, th = map(float, (x0, y0, x1, y1, th))
    _, kf, create = kf_poses(path)
    ids = sorted(i for i, (x, y, _) in kf.items() if x0 <= x <= x1 and y0 <= y <= y1)
    segs, cur = [], []
    for i in ids:
        if cur and i - cur[-1] > 3:
            segs.append(cur); cur = []
        cur.append(i)
    if cur:
        segs.append(cur)
    print(f'{len(kf)} kf total, {len(ids)} in region')
    for s in segs:
        print(f"  visit kf {s[0]}-{s[-1]} ({len(s)}) time {create.get(s[0], '?')}-{create.get(s[-1], '?')}")
    room = set(ids)
    print('loop candidates touching region (A-B score corr):')
    for line in lines(path):
        if (m := re.search(r'lc cand (\d+)-(\d+) score (\S+) corr (\S+) m (\S+) deg', line)):
            a, b, sc, c, d = int(m.group(1)), int(m.group(2)), float(m.group(3)), float(m.group(4)), float(m.group(5))
            if a in room or b in room:
                print(f'  {a:4d}-{b:4d} score {sc:.2f} {"ACCEPT" if sc > th else "reject"} corr {c:.2f} m {d:.1f} deg')


COMMANDS = dict(tfref=cmd_tfref, online=cmd_online, tfdrift=cmd_tfdrift, summary=cmd_summary, outdrift=cmd_outdrift, offset=cmd_offset, drift=cmd_drift, ndtlag=cmd_ndtlag, wall=cmd_wall,
                liocmp=cmd_liocmp, kfdelta=cmd_kfdelta, roomkf=cmd_roomkf)

if __name__ == '__main__':
    if len(sys.argv) < 3 or sys.argv[1] not in COMMANDS:
        print(__doc__); sys.exit(1)
    COMMANDS[sys.argv[1]](*sys.argv[2:])
