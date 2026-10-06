"""LightNav-0 航点 → 速度指令（纯 Python，不依赖 ROS，单元测试直接导入）。

航点约定（LightNav-0 docs/PROTOCOL.md）：每行 ``[forward_m, lateral_m, yaw_rad]``，是相对当前帧机器人位姿的
**累计**位姿，+lateral = 左，+yaw = 逆时针；行本身没有时间基准。和 ROS Twist 的符号一致（linear.y 左、angular.z 逆时针）。

两种换算：

- ``normalized``（默认，与官方参考客户端 lightnav-ws-client / EVT-Bench、同事的 vln_ros2_bridge 一致）：
  第一个航点按"单步最大位移"归一化到 [-1, 1] 再乘限速：
  ``vx = clip(fwd / 0.375) * max_vx``，``vy = clip(lat / 0.25) * max_vy``，``wz = clip(yaw / (pi/20)) * max_wz``。
- ``lookahead``：取第 k 个航点（0 起），按 ``waypoint_dt`` 秒/行折算成速度：``v = wp[k] / ((k + 1) * dt)``，再限幅。

不论哪种换算，最后都经过 :func:`sanitize_twist`：非有限值归零、**前进速度不得为负**（G1 收到负的前进速度会摔倒，
见 lightning_ws/CLAUDE.md《绝对禁止后退》），各轴限幅。下游 g1_cmdvel_to_sport 还会再钳一次 vx>=0，并把小速度抬到
G1 的最小有效值（vx>=0.05 → >=0.3；|wz|>=0.15 → |wz|>=0.6 且 vx>=0.2），这里不重复实现那套抬升。
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Iterable, Sequence, Tuple

# 官方参考客户端用的"单步最大位移"（src/lightnav/cli/ws_client.py）。
WP_FWD_MAX = 0.375
WP_LAT_MAX = 0.25
WP_YAW_MAX = math.pi / 20.0

# 硬上限：任何配置都不能超过。取 G1 Nav2 velocity_smoother 的 max_velocity [0.45, 0.0, 0.9]，
# 横移给一点余量（默认 0，Nav2 也是 0）。
HARD_MAX_VX = 0.45
HARD_MAX_VY = 0.20
HARD_MAX_WZ = 0.90

MAPPINGS = ("normalized", "lookahead")

Twist3 = Tuple[float, float, float]
ZERO: Twist3 = (0.0, 0.0, 0.0)


@dataclass(frozen=True)
class VelocityLimits:
    """VLN 输出限速（m/s、rad/s）。构造时会被钳到硬上限以内。"""

    max_vx: float = 0.35
    max_vy: float = 0.0
    max_wz: float = 0.6

    def clamped(self) -> "VelocityLimits":
        return VelocityLimits(
            max_vx=_clamp_limit(self.max_vx, HARD_MAX_VX),
            max_vy=_clamp_limit(self.max_vy, HARD_MAX_VY),
            max_wz=_clamp_limit(self.max_wz, HARD_MAX_WZ),
        )


@dataclass(frozen=True)
class MappingConfig:
    mapping: str = "normalized"
    lookahead_index: int = 3
    waypoint_dt: float = 0.1
    yaw_sign: float = 1.0
    lat_sign: float = 1.0

    def validate(self) -> None:
        if self.mapping not in MAPPINGS:
            raise ValueError(f"mapping 必须是 {MAPPINGS} 之一，收到 {self.mapping!r}")
        if not (isinstance(self.lookahead_index, int) and self.lookahead_index >= 0):
            raise ValueError("lookahead_index 必须是非负整数")
        if not (math.isfinite(self.waypoint_dt) and self.waypoint_dt > 0):
            raise ValueError("waypoint_dt 必须是正数")
        for name in ("yaw_sign", "lat_sign"):
            if getattr(self, name) not in (1.0, -1.0):
                raise ValueError(f"{name} 只能是 1 或 -1")


def _clamp_limit(value: float, hard: float) -> float:
    try:
        v = float(value)
    except (TypeError, ValueError):
        return 0.0
    if not math.isfinite(v) or v <= 0.0:
        return 0.0
    return min(v, hard)


def _clip(v: float, lo: float, hi: float) -> float:
    return lo if v < lo else hi if v > hi else v


def _finite3(row: Sequence[float]) -> bool:
    return len(row) == 3 and all(isinstance(x, (int, float)) and math.isfinite(float(x)) for x in row)


def sanitize_twist(twist: Iterable[float], limits: VelocityLimits) -> Twist3:
    """最后一道闸：非有限值整条归零；vx 钳到 [0, max_vx]（绝不后退）；vy、wz 对称限幅。"""
    lim = limits.clamped()
    try:
        vx, vy, wz = (float(v) for v in twist)
    except (TypeError, ValueError):
        return ZERO
    if not (math.isfinite(vx) and math.isfinite(vy) and math.isfinite(wz)):
        return ZERO
    vx = _clip(vx, 0.0, lim.max_vx)
    vy = _clip(vy, -lim.max_vy, lim.max_vy)
    wz = _clip(wz, -lim.max_wz, lim.max_wz)
    # 避免 -0.0 出现在日志/界面上
    return (vx + 0.0, vy + 0.0, wz + 0.0)


def waypoints_to_twist(waypoints: Sequence[Sequence[float]], limits: VelocityLimits,
                       cfg: MappingConfig = MappingConfig()) -> Twist3:
    """把一段航点换算成 (vx, vy, wz)。航点为空 / 含非有限值时返回零速。"""
    cfg.validate()
    lim = limits.clamped()
    rows = list(waypoints or [])
    if not rows or not all(_finite3(r) for r in rows):
        return ZERO
    if cfg.mapping == "normalized":
        fwd, lat, yaw = (float(x) for x in rows[0])
        vx = _clip(fwd / WP_FWD_MAX, -1.0, 1.0) * lim.max_vx
        vy = _clip(lat / WP_LAT_MAX, -1.0, 1.0) * lim.max_vy
        wz = _clip(yaw / WP_YAW_MAX, -1.0, 1.0) * lim.max_wz
    else:  # lookahead
        k = min(cfg.lookahead_index, len(rows) - 1)
        t = (k + 1) * cfg.waypoint_dt
        fwd, lat, yaw = (float(x) for x in rows[k])
        vx, vy, wz = fwd / t, lat / t, yaw / t
    return sanitize_twist((vx, vy * cfg.lat_sign, wz * cfg.yaw_sign), lim)


def is_zero(twist: Twist3, eps: float = 1e-9) -> bool:
    return all(abs(v) <= eps for v in twist)
