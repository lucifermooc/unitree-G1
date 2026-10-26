#!/usr/bin/env python3
"""离线快速版：简化 RPP（纯追踪 + 曲率降速）+ velocity_smoother + 速度桥 + G1 模型，沿直线走，统计跟线指标。
只用来标定 g1_orin_model.py 的扰动参数、快速比较桥的方案；结论以 run_track_test.py（真 Nav2 RPP）为准。

python3 -B track_offline.py                     # 默认几组方案 × 多个随机种子
python3 -B track_offline.py --sweep             # 扫扰动参数，看哪组最像实测"现状"
"""
import argparse
import math

import numpy as np

from g1_orin_model import G1, Bridge

DT = 0.1        # controller / smoother / 桥 10 Hz
SUB = 5         # 模型积分 50 Hz


class Smoother:
    """nav2 velocity_smoother（OPEN_LOOP）：max_accel [0.35, -, 0.60]，max_decel [-2.0, -, -5.0]，max_velocity [0.45, -, 0.9]。"""

    def __init__(self):
        self.v = [0.0, 0.0]
        self.acc, self.dec, self.vmax = (0.35, 0.60), (2.0, 5.0), (0.45, 0.9)

    def __call__(self, vx, wz):
        out = []
        for i, c in enumerate((vx, wz)):
            c = max(-self.vmax[i], min(self.vmax[i], c))
            cur = self.v[i]
            lim = (self.acc[i] if abs(c) > abs(cur) and c * cur >= 0 else self.dec[i]) * DT
            cur += max(-lim, min(lim, c - cur))
            self.v[i] = cur
            out.append(cur)
        return out


def rpp(pose, L=0.6, vdes=0.4, min_r=0.9, min_v=0.25):
    """沿 y=0（+x 方向）的直线路径做纯追踪；返回 (vx, wz)。"""
    x, y, yaw = pose
    gx = x + math.sqrt(max(L * L - y * y, 1e-6))
    dx, dy = gx - x, -y
    c, s = math.cos(-yaw), math.sin(-yaw)
    lx, ly = c * dx - s * dy, s * dx + c * dy
    d2 = lx * lx + ly * ly
    k = 2 * ly / d2
    v = vdes
    if abs(k) > 1e-6 and 1 / abs(k) < min_r:
        v = max(min_v, vdes * (1 / abs(k)) / min_r)
    return v, v * k


def run(bridge_kw, model_kw, seed, dur=60.0, rpp_kw=None):
    g = G1(seed=seed, **model_kw)
    br = Bridge(**bridge_kw)
    sm = Smoother()
    rec = []
    t = 0.0
    while t < dur:
        pub = g.published()
        vx, wz = rpp(pub, **(rpp_kw or {}))
        vx, wz = sm(vx, wz)
        sx, sy, sw = br(vx, wz, cte=pub[1])
        g.command(t, sx, sy, sw)
        for _ in range(SUB):
            g.step(DT / SUB)
        t += DT
        rec.append((t, pub[1], g.y, wz, sw, vx))
    return np.array(rec)


def metrics(r, wz_stop=0.15, min_wz=0.32, skip=8.0):
    r = r[r[:, 0] > skip]                      # 去掉起步
    cte, cte_true, wz, sw = r[:, 1], r[:, 2], r[:, 3], r[:, 4]
    n = max(1, int(1.0 / DT))
    sm = np.convolve(cte, np.ones(n) / n, mode="same")
    a = np.abs(cte)
    w = np.abs(wz)
    lifted = np.mean((w >= wz_stop) & (w < min_wz))
    s = np.sign(np.where(np.abs(wz) >= wz_stop, wz, 0))       # 与实机 track_err.py 同口径：桥前 wz 超过 wz_stop 的换向
    nz = s[s != 0]
    flips = int(np.sum(nz[1:] != nz[:-1])) if len(nz) > 1 else 0
    span = r[-1, 0] - r[0, 0]
    return dict(p50=np.median(a), p95=np.percentile(a, 95), mx=a.max(),
                sm_p50=np.median(np.abs(sm)), sm_p95=np.percentile(np.abs(sm), 95),
                true_p50=np.median(np.abs(cte_true)), true_p95=np.percentile(np.abs(cte_true), 95),
                sway_p50=np.median(np.abs(cte - sm)), lifted=lifted, nonzero=np.mean(np.abs(sw) >= 0.295),
                wz_std=np.std(wz),
                flip_s=span / max(flips, 1))


def summarize(rows):
    keys = rows[0].keys()
    return {k: float(np.mean([r[k] for r in rows])) for k in keys}


def fmt(m):
    return (f"p50 {m['p50']:.3f} p95 {m['p95']:.3f} max {m['mx']:.3f} | 1s平均 p50 {m['sm_p50']:.3f} p95 {m['sm_p95']:.3f} "
            f"| 骨盆 p50 {m['true_p50']:.3f} p95 {m['true_p95']:.3f} | 晃动 p50 {m['sway_p50']:.3f} "
            f"| wz std {m['wz_std']:.3f} 抬速 {m['lifted']*100:.0f}% 实际能转的指令 {m['nonzero']*100:.0f}% 换向每 {m['flip_s']:.1f} s")


TAN = math.tan(math.radians(9.5))
# 名字: (桥参数, G1 模型参数, RPP 参数)。[] 里是对 G1 未实测特性的假设。
A = dict(mode="walkmin", walk_min_wz=0.0, walk_wz_stop=0.0, smooth_s=0.0)   # 方案 A：走路时 wz 原样发出
VARIANTS = {
    "现状 wz_stop0.15": ({}, {}, {}),
    "wz_stop0.05": (dict(wz_stop=0.05), {}, {}),
    "A 走路不抬速": (A, {}, {}),
    "A + 对照: 没有蟹行": (A, dict(crab_deg=0.0), {}),
    "A + 航向补偿9.5°": (A, dict(yaw_comp_deg=9.5), {}),
    "A + 前视0.4": (A, {}, dict(L=0.4)),
    "A + vy前馈 [vy无死区]": (dict(A, vy_ff=TAN), dict(vy_dead=0.0), {}),
}


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--seeds", type=int, default=12)
    ap.add_argument("--sweep", action="store_true")
    ap.add_argument("--sigma", type=float, default=None)
    ap.add_argument("--tau", type=float, default=None)
    a = ap.parse_args()
    if a.sweep:
        print("实测现状：p50 0.10 p95 0.20~0.21 | 1s平均 p50 0.10 p95 0.18 | 晃动 p50 0.04 | 抬速 27~30% | 换向每 3.3~3.7 s")
        print("实测 wz_stop0.05：p50 0.095 p95 0.25 max 0.59 | 抬速(0.05~0.32) 61% | 换向每 ~1 s")
        for sw_yaw in (2.5,):
          for sig in (0.04, 0.08, 0.12, 0.16):
            for tau in (3.0, 8.0):
                mk = dict(dist_sigma=sig, dist_tau=tau, sway_yaw_deg=sw_yaw)
                m0 = summarize([metrics(run(dict(mode="current", wz_stop=0.15), mk, s)) for s in range(a.seeds)])
                m1 = summarize([metrics(run(dict(mode="current", wz_stop=0.05), mk, s), wz_stop=0.05) for s in range(a.seeds)])
                print(f"摆{sw_yaw}° σ {sig:.2f} τ {tau:4.1f} | 0.15: {fmt(m0)}")
                print(f"           | 0.05: {fmt(m1)}")
        return
    extra = {}
    if a.sigma is not None:
        extra["dist_sigma"] = a.sigma
    if a.tau is not None:
        extra["dist_tau"] = a.tau
    for name, (bk, mk, rk) in VARIANTS.items():
        rows = [metrics(run(bk, dict(mk, **extra), s, rpp_kw=rk), wz_stop=bk.get("wz_stop", 0.15) or 0.15) for s in range(a.seeds)]
        print(f"{name:34s} {fmt(summarize(rows))}")


if __name__ == "__main__":
    main()
