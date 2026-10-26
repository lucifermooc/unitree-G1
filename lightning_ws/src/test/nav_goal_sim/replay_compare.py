#!/usr/bin/env python3
"""实机 vs 仿真同口径对比（run_replay_test.py 的结果 + 实机 vel_jump_extract.py 导出的 CSV）。

  python3 replay_compare.py --real ~/G1_bags/veljump_1026_1612_csv --sim results/replay_<时间>

每份数据算同一组指标：到点成功率、单程用时、collision ahead / 卡住、速度跳变（按场景分类，见 classify）、
原地转时 wz 换号频率、原地转漂移、到点误差。实机的"位姿"是定位位姿（/base_link_pose），仿真用骨盆真实位姿（truth.csv）。
"""
import argparse
import csv
import glob
import math
import os
import re
import sys

import numpy as np

sys.path.insert(0, os.path.join(os.path.dirname(os.path.realpath(__file__)), "../../../tools/nav2_debug"))
import vel_jump_analyze as va  # noqa: E402

GOAL_RE = re.compile(r"Begin navigating from current location \(([-\d.]+), ([-\d.]+)\) to \(([-\d.]+), ([-\d.]+)\)")


def load_logs(d):
    out = []
    for r in csv.reader(open(os.path.join(d, "rosout.csv"))):
        try:
            out.append((float(r[0]), r[1], r[3]))
        except (ValueError, IndexError):
            pass
    return out


def classify(sp, cmd, plan, logs):
    def near(t, keys, a=-1.0, b=0.3):
        return any(a <= lt - t <= b and any(k in m for k in keys) for lt, _, m in logs)

    def at(arr, t):
        return arr[max(np.searchsorted(arr[:, 0], t) - 1, 0)]
    cats = {}
    for i, _ in va.jumps(sp, 0.15, 0.30):
        t = sp[i, 0]
        pv, cv = sp[i - 1], sp[i]
        c, c0 = at(cmd, t), at(cmd, t - 0.1)
        if near(t, ("Running spin", "Running backup", "Running wait", "spin failed", "Turning", "backup failed")):
            k = "恢复行为"
        elif near(t, ("Reached the goal",), -0.6, 0.4):
            k = "到点停车"
        elif near(t, ("collision ahead",), -0.6, 0.3):
            k = "collision ahead 急停/再起步"
        elif near(t, ("Received a goal",), -1.2, 0.0):
            k = "起步"
        elif abs(pv[1]) < 0.05 and abs(cv[1]) < 0.05:
            k = "原地转 wz 跳/换号"
        elif abs(pv[1] - cv[1]) >= 0.15 and 0.05 <= c[1] < 0.3 and ((abs(c[2]) >= 0.15) != (abs(c0[2]) >= 0.15)):
            k = "桥：|wz| 跨 0.15 切换抬速"
        elif abs(pv[1] - cv[1]) >= 0.15 and min(abs(pv[1]), abs(cv[1])) < 0.05:
            k = "走/停切换"
        else:
            k = "行驶中变化"
        cats[k] = cats.get(k, 0) + 1
    return cats


def spin_segments(sp, min_len=1.0):
    """发给 G1 的指令里连续原地转（|vx|<0.05、|wz|>=0.3）的段：[(t0, t1), ...]"""
    segs, start = [], None
    for t, vx, wz in sp:
        spin = abs(vx) < 0.05 and abs(wz) >= 0.3
        if spin and start is None:
            start = t
        elif not spin and start is not None:
            if t - start >= min_len:
                segs.append((start, t))
            start = None
    return segs


def metrics(d, pose_name):
    nav = va.load(d, "cmd_vel_nav", [0, 1, 2])
    cmd = va.load(d, "cmd_vel", [0, 1, 2])
    s = va.load(d, "sport", [0, 1, 2, 3, 4])
    s = s[np.isfinite(s[:, 2])]
    sp = s[:, [0, 2, 4]]
    pose = va.load(d, pose_name, [0, 1, 2, 3])
    plan = va.load(d, "plan", [0, 1])
    logs = load_logs(d)
    dur = sp[-1, 0] - sp[0, 0]
    m = dict(dur_min=dur / 60)
    begins = [(t, (float(g.group(3)), float(g.group(4)))) for t, _, msg in logs for g in [GOAL_RE.search(msg)] if g]
    ok = [t for t, _, msg in logs if "Goal succeeded" in msg]
    m["goals"], m["ok"] = len(begins), len(ok)
    # 单程用时 / 到点误差：每个 Begin 之后第一条 Goal succeeded
    times, errs = [], []
    for i, (t, g) in enumerate(begins):
        nxt = begins[i + 1][0] if i + 1 < len(begins) else 1e18
        o = [x for x in ok if t < x < nxt]
        if o:
            times.append(o[0] - t)
            j = min(np.searchsorted(pose[:, 0], o[0]), len(pose) - 1)
            errs.append(math.hypot(pose[j, 1] - g[0], pose[j, 2] - g[1]))
    m["leg_s"] = float(np.median(times)) if times else float("nan")
    m["goal_err"] = float(np.mean(errs)) if errs else float("nan")
    ca = sorted(t for t, _, msg in logs if "collision ahead" in msg)
    m["collision_ahead"] = len(ca)
    eps = []
    for t in ca:
        if eps and t - eps[-1][1] < 2.0:
            eps[-1][1] = t
        else:
            eps.append([t, t])
    m["ca_episodes"] = len(eps)
    m["recoveries"] = sum(any(k in msg for k in ("Running spin", "Running backup", "Running wait")) for _, _, msg in logs)
    m["backup_failed"] = sum("backup failed" in msg for _, _, msg in logs)
    m["aborts"] = sum("Aborting" in msg and "follow_path" in msg for _, _, msg in logs)
    # 卡住：10 s 窗口内位移 < 0.15 m 且没在到点后等待（有导航在跑），累计时长
    stuck = 0.0
    if len(pose) > 10:
        active = np.zeros(len(pose), bool)
        for i, (t, _) in enumerate(begins):
            nxt = begins[i + 1][0] if i + 1 < len(begins) else pose[-1, 0]
            o = [x for x in ok if t < x < nxt]
            end = o[0] if o else nxt
            active |= (pose[:, 0] >= t) & (pose[:, 0] < end)
        grid = np.arange(pose[0, 0], pose[-1, 0], 1.0)
        px, py = np.interp(grid, pose[:, 0], pose[:, 1]), np.interp(grid, pose[:, 0], pose[:, 2])
        act = np.interp(grid, pose[:, 0], active.astype(float)) > 0.5
        for k in range(10, len(grid)):
            if act[k] and act[k - 10] and math.hypot(px[k] - px[k - 10], py[k] - py[k - 10]) < 0.15:
                stuck += 1.0
    m["stuck_s"] = stuck
    J = va.jumps(sp, 0.15, 0.30)
    m["jumps_per_min"] = len(J) / (dur / 60)
    m["nav_jumps_per_min"] = len(va.jumps(nav, 0.15, 0.30)) / (dur / 60)
    m["cats"] = {k: v / (dur / 60) for k, v in classify(sp, cmd, plan, logs).items()}
    # 原地转：控制器输出 wz 换号次数 / 原地转秒数；漂移 = 每段起止位置差
    segs = spin_segments(sp)
    spin_s = sum(b - a for a, b in segs)
    flips = 0
    for a, b in segs:
        w = nav[(nav[:, 0] >= a) & (nav[:, 0] <= b), 2]
        w = w[np.abs(w) > 0.05]
        flips += int(np.sum(np.sign(w[1:]) != np.sign(w[:-1])))
    m["spin_s"] = spin_s
    m["spin_flip_per_s"] = flips / spin_s if spin_s else float("nan")
    drift = []
    for a, b in segs:
        i0, i1 = np.searchsorted(pose[:, 0], a), min(np.searchsorted(pose[:, 0], b + 0.5), len(pose) - 1)
        dyaw = abs(math.degrees(math.atan2(math.sin(pose[i1, 3] - pose[i0, 3]), math.cos(pose[i1, 3] - pose[i0, 3]))))
        if dyaw > 45:
            drift.append(math.hypot(pose[i1, 1] - pose[i0, 1], pose[i1, 2] - pose[i0, 2]) / dyaw * 90)
    m["spin_drift_per90"] = float(np.median(drift)) if drift else float("nan")
    # 走路时发给 G1 的 vx 在 (0.05, 0.3) 的比例（桥不抬速的转弯段）
    walk = sp[sp[:, 1] >= 0.05]
    m["walk_vx_below_0.3"] = float(np.mean(walk[:, 1] < 0.299)) if len(walk) else float("nan")
    return m


def fmt_row(name, m):
    return (f"{name:16s} {m['dur_min']:5.1f} | {m['ok']:3d}/{m['goals']:<3d} {m['leg_s']:5.1f} {m['goal_err']:5.2f} | "
            f"{m['collision_ahead']:4d} {m['ca_episodes']:3d} {m['stuck_s']:5.0f} {m['recoveries']:3d} {m['backup_failed']:3d} | "
            f"{m['jumps_per_min']:5.1f} {m['nav_jumps_per_min']:5.1f} | {m['spin_s']:5.0f} {m['spin_flip_per_s']:5.2f} "
            f"{m['spin_drift_per90']:5.2f} | {m['walk_vx_below_0.3']:5.2f}")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--real", required=True)
    ap.add_argument("--sim", required=True)
    a = ap.parse_args()
    rows = [("实机", metrics(os.path.expanduser(a.real), "pose"))]
    for d in sorted(glob.glob(os.path.join(a.sim, "*_p*_s*"))):
        if os.path.exists(os.path.join(d, "sport.csv")) and os.path.getsize(os.path.join(d, "sport.csv")) > 0:
            rows.append(("仿真 " + os.path.basename(d), metrics(d, "truth")))
    print("数据              时长  | 成功/目标 单程s 到点m | CA条 CA段 卡住s 恢复 backup失败 | 跳变/分 控制器跳变/分 | "
          "原地转s wz换号/s 漂移m/90° | 走路vx<0.3比例")
    for n, m in rows:
        print(fmt_row(n, m))
    keys = sorted({k for _, m in rows for k in m["cats"]})
    print("\n跳变按场景（次/分钟）")
    print(f"{'场景':28s}" + "".join(f"{n[-12:]:>14s}" for n, _ in rows))
    for k in keys:
        print(f"{k:28s}" + "".join(f"{m['cats'].get(k, 0):14.2f}" for _, m in rows))


if __name__ == "__main__":
    main()
