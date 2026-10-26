#!/usr/bin/env python3
"""速度跳变分析，第 2 步（本机跑）：读 vel_jump_extract.py 导出的 CSV，找出各级速度的跳变并判断是哪一级引入的。

  python3 vel_jump_analyze.py <csv_dir> [--dvx 0.15] [--dwz 0.30] [--plot out.png]

速度链：控制器 /cmd_vel_nav -> velocity_smoother /cmd_vel -> 桥（抬速 / 死区改写）-> /api/sport/request -> G1。
"跳变" = 相邻两条指令 |Δvx| >= dvx 或 |Δwz| >= dwz，或 wz 换号且两边 |wz| >= 0.2。
归因（按"G1 收到的指令"上的跳变往上游找，±0.25 s 内）：
  控制器    /cmd_vel_nav 同时有同向跳变
  平滑器    /cmd_vel 有、/cmd_vel_nav 没有（一般不会：平滑器只会削跳变）
  桥        /cmd_vel 平滑、只有 sport 跳（vx 过 0.05 被抬到 0.3、|wz| 过 0.15 被抬到 min_wz、或掉进死区被清零）
  断流      跳变前 /cmd_vel_nav 停发 > 0.3 s（控制器卡顿 / 失败 / 行为树切恢复）
另外统计 G1 实际速度（/odommodestate）的跳变，以及跳变前后 1 s 内的相关日志（collision ahead、恢复、重规划、lifted）。
"""
import argparse
import csv
import math
import os

import numpy as np


def load(d, name, cols):
    p = os.path.join(d, name + ".csv")
    if not os.path.exists(p):
        return np.zeros((0, len(cols)))
    rows = []
    for r in csv.reader(open(p)):
        try:
            rows.append([float(r[i]) for i in cols])
        except (ValueError, IndexError):
            continue
    return np.array(rows) if rows else np.zeros((0, len(cols)))


def jumps(a, dvx, dwz):
    """a: [t, vx, wz]；返回跳变索引（i 表示 a[i-1] -> a[i]）和原因。"""
    out = []
    for i in range(1, len(a)):
        why = []
        if abs(a[i, 1] - a[i - 1, 1]) >= dvx:
            why.append(f"vx {a[i - 1, 1]:+.2f}->{a[i, 1]:+.2f}")
        if abs(a[i, 2] - a[i - 1, 2]) >= dwz:
            why.append(f"wz {a[i - 1, 2]:+.2f}->{a[i, 2]:+.2f}")
        elif a[i, 2] * a[i - 1, 2] < 0 and min(abs(a[i, 2]), abs(a[i - 1, 2])) >= 0.2:
            why.append(f"wz 换号 {a[i - 1, 2]:+.2f}->{a[i, 2]:+.2f}")
        if why:
            out.append((i, " ".join(why)))
    return out


def has_jump_near(a, t, dvx, dwz, win=0.25):
    if not len(a):
        return False
    m = (a[:, 0] >= t - win) & (a[:, 0] <= t + win)
    idx = np.nonzero(m)[0]
    for i in idx:
        if i == 0:
            continue
        if abs(a[i, 1] - a[i - 1, 1]) >= dvx * 0.6 or abs(a[i, 2] - a[i - 1, 2]) >= dwz * 0.6:
            return True
    return False


def gap_before(a, t, gap=0.3):
    if not len(a):
        return None
    j = np.searchsorted(a[:, 0], t) - 1
    if j < 1:
        return None
    # 跳变时刻之前最近一次发布离再前一次的间隔，或跳变时刻离上一次发布的间隔
    g = max(t - a[j, 0], a[j, 0] - a[j - 1, 0])
    return g if g > gap else None


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("csv_dir")
    ap.add_argument("--dvx", type=float, default=0.15)
    ap.add_argument("--dwz", type=float, default=0.30)
    ap.add_argument("--plot", default="")
    a = ap.parse_args()
    d = a.csv_dir
    nav = load(d, "cmd_vel_nav", [0, 1, 2])
    cmd = load(d, "cmd_vel", [0, 1, 2])
    sport = load(d, "sport", [0, 1, 2, 3, 4])          # t api vx vy wz
    sport = sport[np.isfinite(sport[:, 2])] if len(sport) else sport
    sp = sport[:, [0, 2, 4]] if len(sport) else np.zeros((0, 3))
    odm = load(d, "odommodestate", [0, 1, 2, 3, 4])     # t vxw vyw yawspeed yaw
    logs = []
    p = os.path.join(d, "rosout.csv")
    if os.path.exists(p):
        for r in csv.reader(open(p)):
            try:
                logs.append((float(r[0]), r[1], r[3]))
            except (ValueError, IndexError):
                pass
    t0 = min(x[0, 0] for x in (nav, cmd, sp) if len(x)) if any(len(x) for x in (nav, cmd, sp)) else 0

    def rel(t):
        return t - t0

    print(f"样本：cmd_vel_nav {len(nav)}  cmd_vel {len(cmd)}  sport {len(sp)}  odommodestate {len(odm)}  日志 {len(logs)}")
    if len(sp) > 2:
        dt = np.diff(sp[:, 0])
        print(f"sport 发送间隔：中位 {np.median(dt) * 1000:.0f} ms，p99 {np.percentile(dt, 99) * 1000:.0f} ms，"
              f"最大 {dt.max() * 1000:.0f} ms（> 0.5 s 的 {int((dt > 0.5).sum())} 次 = 指令 duration 过期，G1 会自己停）")

    J = jumps(sp, a.dvx, a.dwz)
    cause = {}
    rows = []
    for i, why in J:
        t = sp[i, 0]
        g = gap_before(nav, t)
        if g:
            c = f"断流（/cmd_vel_nav 停发 {g:.2f} s）"
        elif has_jump_near(nav, t, a.dvx, a.dwz):
            c = "控制器"
        elif has_jump_near(cmd, t, a.dvx, a.dwz):
            c = "平滑器"
        else:
            c = "桥（抬速 / 死区）"
        key = c.split("（")[0]
        cause[key] = cause.get(key, 0) + 1
        near = [f"{rel(lt):.1f}s {n.split('.')[-1]}: {m[:110]}" for lt, n, m in logs if t - 1.0 <= lt <= t + 0.3
                and any(k in m.lower() for k in ("collision", "recover", "running", "lifted", "abort", "fail",
                                                  "navshim", "goal", "replan", "spin", "backup", "wait", "progress"))]
        jn = np.searchsorted(nav[:, 0], t) if len(nav) else 0
        jc = np.searchsorted(cmd[:, 0], t) if len(cmd) else 0
        ctx = (f"nav {nav[max(jn - 1, 0), 1]:+.2f}/{nav[max(jn - 1, 0), 2]:+.2f}" if len(nav) else "") + \
              (f"  smooth {cmd[max(jc - 1, 0), 1]:+.2f}/{cmd[max(jc - 1, 0), 2]:+.2f}" if len(cmd) else "")
        rows.append((rel(t), why, c, ctx, near[-3:]))

    dur = (sp[-1, 0] - sp[0, 0]) if len(sp) > 1 else 0
    print(f"\nG1 收到的指令（sport）跳变 {len(J)} 次 / {dur:.0f} s（阈值 |Δvx|≥{a.dvx}、|Δwz|≥{a.dwz}）")
    for k, v in sorted(cause.items(), key=lambda x: -x[1]):
        print(f"  {k}: {v}")
    print(f"控制器输出（cmd_vel_nav）本身的跳变 {len(jumps(nav, a.dvx, a.dwz))} 次；平滑后（cmd_vel）{len(jumps(cmd, a.dvx, a.dwz))} 次")

    if len(odm) > 10:
        # G1 实际：世界系速度转成速率。原始值随每一步起伏（步频 ~1 Hz 量级），先做 0.4 s 滑动平均再按 0.2 s 看跳变
        sp_act = np.hypot(odm[:, 1], odm[:, 2])
        tt = odm[:, 0]
        grid = np.arange(tt[0], tt[-1], 0.02)
        k = 20   # 0.4 s
        ker = np.ones(k) / k
        v = np.convolve(np.interp(grid, tt, sp_act), ker, "same")[::10]
        w = np.convolve(np.interp(grid, tt, odm[:, 3]), ker, "same")[::10]
        dv, dw = np.abs(np.diff(v)), np.abs(np.diff(w))
        print(f"G1 实际（odommodestate，0.4 s 平均、0.2 s 间隔）：|Δv|≥0.10 m/s {int((dv >= 0.10).sum())} 次，"
              f"|Δwz|≥0.30 rad/s {int((dw >= 0.30).sum())} 次；Δv p99 {np.percentile(dv, 99):.3f}，Δwz p99 {np.percentile(dw, 99):.3f}")

    print("\n逐次（t 从第一条指令起算；nav / smooth = 跳变前一刻控制器 / 平滑器的 vx/wz）：")
    for t, why, c, ctx, near in rows[:80]:
        print(f"{t:7.1f}s  {why:34s} <- {c:22s} {ctx}")
        for s in near:
            print(f"            · {s}")
    if len(rows) > 80:
        print(f"... 另有 {len(rows) - 80} 次")

    if a.plot:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
        fig, ax = plt.subplots(2, 1, figsize=(16, 7), sharex=True)
        for arr, lab, st in ((nav, "controller cmd_vel_nav", "-"), (cmd, "smoother cmd_vel", "-"),
                             (sp, "to G1 (sport)", "-")):
            if len(arr):
                ax[0].step(rel(arr[:, 0]), arr[:, 1], st, where="post", label=lab, lw=1)
                ax[1].step(rel(arr[:, 0]), arr[:, 2], st, where="post", label=lab, lw=1)
        if len(odm):
            ax[0].plot(rel(odm[:, 0]), np.hypot(odm[:, 1], odm[:, 2]), lw=0.6, alpha=0.6, label="G1 actual |v|")
            ax[1].plot(rel(odm[:, 0]), odm[:, 3], lw=0.6, alpha=0.6, label="G1 actual yaw_speed")
        for t, *_ in rows:
            for x in ax:
                x.axvline(t, color="r", lw=0.4, alpha=0.4)
        ax[0].set_ylabel("vx (m/s)")
        ax[1].set_ylabel("wz (rad/s)")
        ax[1].set_xlabel("t (s)")
        ax[0].legend(loc="upper right", fontsize=8)
        ax[1].legend(loc="upper right", fontsize=8)
        fig.tight_layout()
        fig.savefig(a.plot, dpi=110)
        print(f"图：{a.plot}")


if __name__ == "__main__":
    main()
