"""全图俯视（BEV）位姿搜索：不需要初值，给出 (x, y, yaw) 和是否可信。

做法（2026-10-27 Orin 30524 实测：原点初始化错 2.4 m / 78°，本方法找回真值）：
1. 地图与扫描都按重力摆平，各自估地面高度，只留离地 band_lo~band_hi 的点（地面在哪都对得上，留着只会稀释差别）。
2. 粗搜：地图压成 coarse_res 俯视占据栅格（膨胀 1 格）。每个航向把扫描画成以传感器为中心的小栅格，
   和地图做一次 FFT 互相关 = 一次算出整张图每个平移的命中格数。只打分、不做梯度下降，所以不会被局部最优带走
   （lightning 的粗 NDT 会把候选移走，10-24 实测把准初值带到 1 m 外）。
3. 每个航向取局部极大，跨航向做非极大抑制，留前 n_candidates 个互不相同的候选。
4. 精搜：fine_res 的 3D 体素（离地高度 band 内，膨胀 1 格）上对每个候选在 ±refine_xy、±refine_yaw 内网格搜索，按精分排序。
   精分必须用 3D：俯视 2D 把不同高度的东西压到一起，10-27 数据上错误位姿 2D 精分 0.43 / 真值 0.59，3D 是 0.2 / 0.7 量级。
5. 判定：最高分 >= min_score，且第二名（离第一名 > nms_dist 或航向差 > nms_yaw_deg）<= max_second_ratio × 第一名。
   对称场景（长走廊）两个解分数接近时宁可不给，交给人工重定位。
"""
from dataclasses import dataclass, field

import numpy as np
from scipy import fft as sfft
from scipy.ndimage import binary_dilation, maximum_filter


@dataclass
class SearchParams:
    band_lo: float = 0.3          # 离地高度下限（m）
    band_hi: float = 2.0          # 离地高度上限（m）
    min_range: float = 0.6        # 扫描点离传感器的水平距离下限（去机身 / 手臂）
    max_range: float = 25.0       # 扫描点水平距离上限，决定粗搜核大小
    coarse_res: float = 0.2
    yaw_step_deg: float = 3.0
    peaks_per_yaw: int = 5
    n_candidates: int = 8
    nms_dist: float = 0.75
    nms_yaw_deg: float = 10.0
    fine_res: float = 0.1
    refine_xy: float = 0.3
    refine_xy_step: float = 0.05
    refine_yaw_deg: float = 4.0
    refine_yaw_step_deg: float = 1.0
    fine_max_points: int = 4000
    min_score: float = 0.35
    max_second_ratio: float = 0.85


@dataclass
class Candidate:
    x: float
    y: float
    yaw: float                    # rad
    score: float                  # 精分：非地面扫描点落在地图 3D 占据体素（膨胀 1 格）的比例
    coarse: float = 0.0

    def dist(self, o):
        return float(np.hypot(self.x - o.x, self.y - o.y))

    def dyaw_deg(self, o):
        return float(abs((np.degrees(self.yaw - o.yaw) + 180.0) % 360.0 - 180.0))


@dataclass
class SearchResult:
    accepted: bool
    reason: str
    best: Candidate = None
    second: Candidate = None
    candidates: list = field(default_factory=list)
    n_scan: int = 0
    scan_ground: float = 0.0

    def summary(self):
        def c(v):
            return None if v is None else dict(x=round(v.x, 3), y=round(v.y, 3), yaw_deg=round(np.degrees(v.yaw), 2),
                                               score=round(v.score, 3), coarse=round(v.coarse, 3))
        return dict(accepted=self.accepted, reason=self.reason, best=c(self.best), second=c(self.second),
                    n_scan=self.n_scan, scan_ground=round(self.scan_ground, 3))


def rot_to_up(up):
    """最小旋转 R，使 R @ up = +z"""
    u = np.asarray(up, float)
    u = u / np.linalg.norm(u)
    z = np.array([0.0, 0.0, 1.0])
    v = np.cross(u, z)
    s = np.linalg.norm(v)
    c = float(u @ z)
    if s < 1e-9:
        return np.eye(3) if c > 0 else np.diag([1.0, -1.0, -1.0])
    vx = np.array([[0, -v[2], v[1]], [v[2], 0, -v[0]], [-v[1], v[0], 0]])
    return np.eye(3) + vx + vx @ vx * ((1 - c) / s ** 2)


def estimate_ground(z, bin_size=0.05):
    """取下半部分点的高度直方图峰值，再在峰附近求均值"""
    z = np.asarray(z)
    low = z[z <= np.median(z)]
    if len(low) < 50:
        return float(np.min(z))
    edges = np.arange(low.min(), low.max() + bin_size, bin_size)
    if len(edges) < 2:
        return float(low.mean())
    h, edges = np.histogram(low, edges)
    k = int(np.argmax(h))
    zc = 0.5 * (edges[k] + edges[k + 1])
    near = low[np.abs(low - zc) < 0.08]
    return float(near.mean()) if len(near) else float(zc)


def band_xyh(points, ground, p):
    """离地 band 内的点，返回 (x, y, 离地高度)"""
    h = points[:, 2] - ground
    m = (h > p.band_lo) & (h < p.band_hi)
    return np.column_stack([points[m, 0], points[m, 1], h[m]])


class _Grid:
    def __init__(self, xy, lo, res, shape):
        self.lo, self.res, self.shape = lo, res, shape
        g = np.zeros(shape, bool)
        ij = np.floor((xy - lo) / res).astype(int)
        ok = np.all((ij >= 0) & (ij < shape), 1)
        g[ij[ok, 0], ij[ok, 1]] = True
        self.raw = g
        self.dil = binary_dilation(g, iterations=1)

    def hit(self, q):
        """q: (..., 2) 世界坐标 → 是否命中膨胀后的占据格"""
        ij = np.floor((q - self.lo) / self.res).astype(np.int64)
        ok = (ij[..., 0] >= 0) & (ij[..., 0] < self.shape[0]) & (ij[..., 1] >= 0) & (ij[..., 1] < self.shape[1])
        out = np.zeros(q.shape[:-1], bool)
        out[ok] = self.dil[ij[..., 0][ok], ij[..., 1][ok]]
        return out


class _Grid3:
    """离地高度 band 内的 3D 占据体素；z 用离地高度 h"""

    def __init__(self, xyh, lo, res, shape):
        self.lo, self.res, self.shape = np.asarray(lo, float), res, shape
        g = np.zeros(shape, bool)
        ijk = np.floor((xyh - self.lo) / res).astype(int)
        ok = np.all((ijk >= 0) & (ijk < shape), 1)
        g[ijk[ok, 0], ijk[ok, 1], ijk[ok, 2]] = True
        self.dil = binary_dilation(g, iterations=1)

    def hit(self, q):
        ijk = np.floor((q - self.lo) / self.res).astype(np.int64)
        ok = np.all((ijk >= 0) & (ijk < np.asarray(self.shape)), -1)
        out = np.zeros(q.shape[:-1], bool)
        out[ok] = self.dil[ijk[..., 0][ok], ijk[..., 1][ok], ijk[..., 2][ok]]
        return out


class MapIndex:
    """地图预处理（只做一次）：摆平、地面、非地面俯视栅格、粗栅格的 FFT"""

    def __init__(self, map_xyz, params=None, up=(0.0, 0.0, 1.0)):
        self.p = p = params or SearchParams()
        pts = np.asarray(map_xyz, float)
        self.R_level = rot_to_up(up)
        pts = pts @ self.R_level.T
        self.ground = estimate_ground(pts[:, 2])
        xyh = band_xyh(pts, self.ground, p)
        xy = xyh[:, :2]
        if len(xy) < 100:
            raise ValueError(f'map has only {len(xy)} non-ground points in band')
        self.n_points = len(xy)
        self.xy_min = xy.min(0)
        self.xy_max = xy.max(0)
        lo = self.xy_min - 1.0
        self.coarse = _Grid(xy, lo, p.coarse_res, tuple(np.ceil((self.xy_max + 1.0 - lo) / p.coarse_res).astype(int) + 1))
        lo3 = np.array([lo[0], lo[1], p.band_lo - p.fine_res])
        hi3 = np.array([self.xy_max[0] + 1.0, self.xy_max[1] + 1.0, p.band_hi + p.fine_res])
        self.fine = _Grid3(xyh, lo3, p.fine_res, tuple(np.ceil((hi3 - lo3) / p.fine_res).astype(int) + 1))
        self.kc = int(np.ceil(p.max_range / p.coarse_res))
        k = 2 * self.kc + 1
        H, W = self.coarse.shape
        self.fft_shape = (sfft.next_fast_len(H + k - 1, real=True), sfft.next_fast_len(W + k - 1, real=True))
        self.F_map = sfft.rfft2(self.coarse.dil.astype(np.float32), self.fft_shape)


def _rot2(yaw):
    c, s = np.cos(yaw), np.sin(yaw)
    return np.array([[c, -s], [s, c]])


def _coarse_peaks(mi, scan_xy, p):
    """逐航向 FFT 互相关，返回 [(score, x, y, yaw)]"""
    kc = mi.kc
    k = 2 * kc + 1
    H, W = mi.coarse.shape
    res = p.coarse_res
    lo = mi.coarse.lo
    out = []
    nms_cells = max(3, int(round(p.nms_dist / res)) * 2 + 1)
    for yd in np.arange(-180.0, 180.0, p.yaw_step_deg):
        yaw = np.radians(yd)
        q = scan_xy @ _rot2(yaw).T
        uv = np.round(q / res).astype(int) + kc
        ker = np.zeros((k, k), np.float32)
        ker[uv[:, 0], uv[:, 1]] = 1.0
        n = float(ker.sum())
        if n <= 0:
            continue
        full = sfft.irfft2(mi.F_map * sfft.rfft2(ker[::-1, ::-1], mi.fft_shape), mi.fft_shape)
        S = full[kc:kc + H, kc:kc + W] / n
        mx = maximum_filter(S, size=nms_cells, mode='constant', cval=0.0)
        a, b = np.nonzero((S >= mx - 1e-6) & (S > 0.05))
        if len(a) == 0:
            continue
        sc = S[a, b]
        top = np.argsort(-sc)[:p.peaks_per_yaw]
        for t in top:
            out.append((float(sc[t]), lo[0] + (a[t] + 0.5) * res, lo[1] + (b[t] + 0.5) * res, yaw))
    return out


def _nms(cands, p, limit):
    kept = []
    for c in sorted(cands, key=lambda c: -c.score):
        if all(c.dist(o) > p.nms_dist or c.dyaw_deg(o) > p.nms_yaw_deg for o in kept):
            kept.append(c)
        if len(kept) >= limit:
            break
    return kept


def _fine_score_grid(mi, pts, x0, y0, yaw0, half_xy, step_xy, half_yaw_deg, step_yaw_deg):
    offs = np.arange(-half_xy, half_xy + 1e-9, step_xy)
    T = np.stack(np.meshgrid(offs, offs, indexing='ij'), -1).reshape(-1, 2) + [x0, y0]
    best = (-1.0, x0, y0, yaw0)
    T3 = np.column_stack([T, np.zeros(len(T))])
    for dy in np.arange(-half_yaw_deg, half_yaw_deg + 1e-9, step_yaw_deg):
        yaw = yaw0 + np.radians(dy)
        q = _rot_xyh(pts, yaw)
        hits = mi.fine.hit(q[None, :, :] + T3[:, None, :]).mean(1)
        k = int(np.argmax(hits))
        if hits[k] > best[0]:
            best = (float(hits[k]), float(T[k, 0]), float(T[k, 1]), float(yaw))
    return best


def _rot_xyh(pts, yaw):
    out = pts.copy()
    out[:, :2] = pts[:, :2] @ _rot2(yaw).T
    return out


def fine_score(mi, scan_xyh, x, y, yaw):
    return float(mi.fine.hit(_rot_xyh(scan_xyh, yaw) + [x, y, 0.0]).mean())


def voxel_down(pts, res):
    """体素下采样（每格取均值），抹平近处点密度"""
    ijk = np.floor(pts / res).astype(np.int64)
    ijk -= ijk.min(0)
    dims = ijk.max(0) + 1
    key = np.ravel_multi_index(ijk.T, dims)
    _, inv, cnt = np.unique(key, return_inverse=True, return_counts=True)
    out = np.zeros((len(cnt), pts.shape[1]))
    np.add.at(out, inv, pts)
    return out / cnt[:, None]


def prepare_scan(points_level, p):
    """points_level: 传感器系、已按重力摆平的 (N,3)。返回 (非地面 (x, y, 离地高度), 地面高度)"""
    pts = np.asarray(points_level, float)
    r = np.hypot(pts[:, 0], pts[:, 1])
    pts = pts[(r > p.min_range) & (r < p.max_range)]
    g = estimate_ground(pts[:, 2])
    return band_xyh(pts, g, p), g


def search(mi, points_level, p=None, rng=None):
    p = p or mi.p
    xyh, g = prepare_scan(points_level, p)
    res = SearchResult(False, '', n_scan=len(xyh), scan_ground=g)
    if len(xyh) < 200:
        res.reason = f'too few non-ground scan points ({len(xyh)})'
        return res
    peaks = _coarse_peaks(mi, xyh[:, :2], p)
    if not peaks:
        res.reason = 'no coarse peak'
        return res
    coarse = _nms([Candidate(x, y, yaw, s, s) for s, x, y, yaw in peaks], p, p.n_candidates)
    pts = voxel_down(xyh, p.fine_res)
    if len(pts) > p.fine_max_points:
        rng = rng or np.random.default_rng(0)
        pts = pts[rng.choice(len(pts), p.fine_max_points, replace=False)]
    refined = []
    for c in coarse:
        s, x, y, yaw = _fine_score_grid(mi, pts, c.x, c.y, c.yaw, p.refine_xy, p.refine_xy_step,
                                        p.refine_yaw_deg, p.refine_yaw_step_deg)
        s, x, y, yaw = _fine_score_grid(mi, pts, x, y, yaw, p.refine_xy_step, p.refine_xy_step / 2,
                                        p.refine_yaw_step_deg, p.refine_yaw_step_deg / 2)
        refined.append(Candidate(x, y, yaw, s, c.coarse))
    ranked = _nms(refined, p, len(refined))
    res.candidates = ranked
    res.best = ranked[0]
    res.second = ranked[1] if len(ranked) > 1 else None
    if res.best.score < p.min_score:
        res.reason = f'best score {res.best.score:.3f} < min_score {p.min_score}'
    elif res.second is not None and res.second.score > p.max_second_ratio * res.best.score:
        res.reason = (f'ambiguous: second {res.second.score:.3f} > {p.max_second_ratio} x best {res.best.score:.3f}')
    else:
        res.accepted = True
        res.reason = 'ok'
    return res
