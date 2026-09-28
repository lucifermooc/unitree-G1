#!/usr/bin/env python3
"""两张 2D 栅格地图（map.pgm + map.yaml）的刚体配准：求 T_A_B（B 图坐标 -> A 图坐标），用于把一张图上的定位轨迹换到另一张图。
用法: gridreg.py <地图A目录> <地图B目录> [初值 x y yaw_deg]（不给初值时全局搜索：逐 1° 航向做 FFT 互相关求平移）
输出配准结果与残差（占据栅格最近邻距离）。残差中位数接近栅格分辨率说明两张图整体一致。"""
import math
import sys

import numpy as np
import yaml
from scipy.spatial import cKDTree


def occupied_points(d):
    meta = yaml.safe_load(open(d + '/map.yaml'))
    with open(d + '/map.pgm', 'rb') as f:
        assert f.readline().strip() == b'P5'
        line = f.readline()
        while line.startswith(b'#'):
            line = f.readline()
        w, h = map(int, line.split()); maxv = int(f.readline())
        img = np.frombuffer(f.read(w * h), dtype=np.uint8).reshape(h, w)
    r = meta['resolution']; ox, oy = meta['origin'][:2]
    rows, cols = np.nonzero(img < 50)
    return np.stack([ox + (cols + 0.5) * r, oy + (h - rows - 0.5) * r], axis=1)


def apply(T, p):
    x, y, a = T; c, s = math.cos(a), math.sin(a)
    return p @ np.array([[c, s], [-s, c]]) + np.array([x, y])


def icp(A, B, T, iters=60):
    tree = cKDTree(A)
    for th in [1.0] * 15 + [0.5] * 15 + [0.25] * 15 + [0.15] * 15:
        Bt = apply(T, B)
        d, idx = tree.query(Bt)
        m = d < th
        if m.sum() < 50:
            break
        P, Q = Bt[m], A[idx[m]]
        pc, qc = P.mean(0), Q.mean(0)
        H = (P - pc).T @ (Q - qc)
        da = math.atan2(H[0, 1] - H[1, 0], H[0, 0] + H[1, 1])
        c, s = math.cos(da), math.sin(da)
        R = np.array([[c, -s], [s, c]])
        t = qc - R @ pc
        # 复合：新 T = (R,t) ∘ T
        x, y, a = T
        nx, ny = R @ np.array([x, y]) + t
        T = (nx, ny, a + da)
    return T


def global_search(A, B, res=0.1):
    """逐 1° 航向，把 B 旋转后与 A 做栅格互相关（FFT），取重合栅格最多的 (x, y, yaw)"""
    lo = np.minimum(A.min(0), -np.abs(B).max() - 1)
    hi = np.maximum(A.max(0), np.abs(B).max() + 1)
    n = (np.ceil((hi - lo) / res).astype(int) + 1) * 2
    def raster(P, dil):
        g = np.zeros(n, np.float32)
        ij = ((P - lo) / res).astype(int)
        ok = (ij >= 0).all(1) & (ij < n).all(1)
        g[ij[ok, 0], ij[ok, 1]] = 1
        if dil:
            g = np.maximum.reduce([np.roll(np.roll(g, dx, 0), dy, 1) for dx in (-1, 0, 1) for dy in (-1, 0, 1)])
        return g
    FA = np.fft.rfft2(raster(A, True))
    best = (-1, None)
    for deg in range(0, 360):
        a = math.radians(deg); c, s = math.cos(a), math.sin(a)
        Bt = B @ np.array([[c, s], [-s, c]])
        gb = raster(Bt - Bt.mean(0) + (lo + hi) / 2, False)
        corr = np.fft.irfft2(FA * np.conj(np.fft.rfft2(gb)), s=n)
        k = np.unravel_index(np.argmax(corr), corr.shape)
        if corr[k] > best[0]:
            sh = np.array([k[0] if k[0] < n[0] // 2 else k[0] - n[0], k[1] if k[1] < n[1] // 2 else k[1] - n[1]]) * res
            t = sh + (lo + hi) / 2 - Bt.mean(0)
            best = (corr[k], (t[0], t[1], a), deg)
    print(f'global search best yaw {best[2]} deg, overlap cells {best[0]:.0f} / {len(B)}')
    return best[1]


if __name__ == '__main__':
    A, B = occupied_points(sys.argv[1]), occupied_points(sys.argv[2])
    if len(sys.argv) > 5:
        T0 = (float(sys.argv[3]), float(sys.argv[4]), math.radians(float(sys.argv[5])))
    else:
        T0 = global_search(A, B)
    T = icp(A, B, T0)
    d, _ = cKDTree(A).query(apply(T, B))
    dA, _ = cKDTree(apply(T, B)).query(A)
    print(f'A occupied {len(A)} covered by B: <0.10m {np.mean(dA < 0.1) * 100:.0f}%, <0.25m {np.mean(dA < 0.25) * 100:.0f}%')
    print(f'T_A_B x {T[0]:.3f} y {T[1]:.3f} yaw {math.degrees(T[2]):.2f} deg | init {T0[0]:.2f} {T0[1]:.2f} '
          f'{math.degrees(T0[2]):.1f} | B occupied {len(B)}: nn dist median {np.median(d):.3f} m, '
          f'<0.10m {np.mean(d < 0.1) * 100:.0f}%, <0.25m {np.mean(d < 0.25) * 100:.0f}%')
