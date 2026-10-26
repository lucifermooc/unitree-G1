#!/usr/bin/env python3
"""Orin 30524（无 Thor 背包的 G1）的速度桥 + 运动模型。纯 Python，ROS 仿真节点 g1_orin_sim.py 和离线标定 track_offline.py 共用。

模型里每一项的来源（Orin 30524 实测）：
- 走路转向（10-25 motion_test，vx 0.4，直接调 SetVelocity）：**没有死区**，基本线性，实际/指令 0.55~0.78（指令越大比值越高）：
  0.10->0.060、0.15->0.093、0.20->0.132、0.25->0.171、0.30->0.215、0.35->0.262。直走（wz=0）自带左偏 +0.014 rad/s。
  （之前从导航包里得出"走路时 wz<0.15 不转"是测量方法错了：指令每 0.1 s 正负切换，0.5 s 窗口量出来接近 0。）
- 原地转（vx=0）：|wz| <= 0.29 完全不转（10-24），0.30->0.21、0.35->0.28；10-25 从静止转 120°：0.6->0.42~0.46、0.8->0.58~0.65、1.0->0.75~0.78。
- 原地转漂移（里程计，从静止转 120°）：逆时针 0.6 -> 0.22~0.33 m、0.8 -> 0.15~0.20、1.0 -> 0.08~0.10；顺时针 0.6 -> 0.11~0.15、
  0.8 -> 0.07~0.13、1.0 -> 0.06~0.13；按转动用时折成机体系向后的平移速度。身体绕右侧一点画圈。
- 指令延迟 0.3~0.5 s（取 0.35 s 纯延迟 + 一阶响应）。SetVelocity 每条有效 0.5 s。
- 步态晃动：雷达在上身，定位位姿每步左右晃（横向 p50 ~4 cm、p95 7~9 cm），航向 ~1 Hz 摆（标准差 1.4~2.0°）。
  只加在"发布的位姿"上（= RPP / costmap 看到的），不影响真实骨盆轨迹。
- 走路时航向慢漂：用 OU 过程的航向角速度扰动模拟（幅度按实机"现状"的横向偏差标定，见 track_offline.py --sweep）。
- **走路蟹行**：只发 vx 时 G1 实际行进方向比机体航向偏左 ~9.5°（10-25 两个导航包 23 段直走全部偏左；定位位姿 +9.2°，
  G1 自己的 /odommodestate 位置差分对骨盆 IMU 航向 +9.8°、velocity 机体系 +9.7°，所以不是雷达外参）。
  纯追踪的平衡点就是 前视 × tan(9.5°) ≈ 0.10 m，正是实测"一直偏左约 9 cm"。
- 横移 vy：G1 能侧走，但死区和响应都没测过；vy 方案里用"|vy| < vy_dead 不动，之上 1:1"的假设。
"""
import math
import random
from collections import deque


def interp(table, x):
    if x <= table[0][0]:
        return table[0][1]
    for (x0, y0), (x1, y1) in zip(table, table[1:]):
        if x <= x1:
            return y0 + (y1 - y0) * (x - x0) / (x1 - x0)
    return table[-1][1]


# 原地转：指令 |wz| -> 实际 |wz|（0.40->0.42 那档与 0.5 倒挂，按单调平滑处理；1.0 外推）
SPIN_TABLE = [(0.30, 0.21), (0.35, 0.28), (0.40, 0.33), (0.50, 0.39), (0.60, 0.44), (0.80, 0.62), (1.00, 0.77)]
# 走路时（vx 0.4）：指令 |wz| -> 实际 |wz|，无死区（10-25 实测，左右平均）；0.6 以上按原地转外推
WALK_TABLE = [(0.0, 0.0), (0.10, 0.060), (0.15, 0.093), (0.20, 0.132), (0.25, 0.171), (0.30, 0.215), (0.35, 0.262),
              (0.60, 0.46), (1.00, 0.77)]
# 原地转时机体系向后的漂移速度（m/s）= 120° 位移中值 / 转 120° 用时：逆时针 / 顺时针
CCW_DRIFT = [(0.30, 0.07), (0.35, 0.07), (0.60, 0.058), (0.80, 0.051), (1.00, 0.033)]
CW_DRIFT = [(0.30, 0.027), (0.60, 0.027), (0.80, 0.031), (1.00, 0.035)]


def spin_rate(cmd):
    a = abs(cmd)
    return 0.0 if a < 0.295 else math.copysign(interp(SPIN_TABLE, a), cmd)


def walk_rate(cmd, hyp):
    """hyp: measured = 10-25 实测（无死区、增益 0.6~0.78）；ideal = 1:1；jump = 旧假设（和原地转一样 <0.30 不转）。"""
    a = abs(cmd)
    if hyp == "ideal":
        return cmd
    if hyp == "jump":
        return spin_rate(cmd)
    return math.copysign(interp(WALK_TABLE, a), cmd)


class MovingAverage:
    def __init__(self, window_s, dt):
        self.n = max(1, int(round(window_s / dt)))
        self.buf = deque(maxlen=self.n)

    def __call__(self, x):
        self.buf.append(x)
        return sum(self.buf) / len(self.buf)

    def reset(self):
        self.buf.clear()


class Bridge:
    """g1_cmdvel_to_sport 的改写逻辑（与 cmdvel_to_sport.cpp ApplyMinimumVelocity 一致）+ 待评估的跟线修复。

    mode:
      current   现状：|wz|>=wz_stop -> 至少 min_wz；否则 vx 抬到 min_vx、wz 原样
      walkmin   走路时：wz 先做 smooth_s 滑动平均，|wz|>=walk_wz_stop 就抬到 walk_min_wz（原地转仍按 current）。
                walk_min_wz=0、walk_wz_stop=0、smooth_s=0 = 方案 A（走路时 wz 原样发出）
      pulse     走路时：wz 先做 smooth_s 滑动平均，再用一阶 sigma-delta 变成 0 / ±min_wz 的脉冲，平均值 = 平滑后的 wz
      vy        current 的 wz 规则 + 按横向偏差（1 s 平均）发 vy = -vy_gain * cte
    vy_ff > 0 时（任何 mode）：走路时另加前馈 vy = -vy_ff * vx，抵消蟹行（vy_ff = tan(蟹行角)）
    """

    def __init__(self, mode="current", dt=0.1, wz_stop=0.15, min_wz=0.32, min_vx=0.3, vx_stop=0.05, turn_min_vx=0.0,
                 walk_min_wz=0.15, walk_wz_stop=0.03, smooth_s=1.0, vy_gain=0.8, vy_max=0.2, vy_ff=0.0):
        self.mode, self.dt = mode, dt
        self.wz_stop, self.min_wz, self.min_vx, self.vx_stop, self.turn_min_vx = wz_stop, min_wz, min_vx, vx_stop, turn_min_vx
        self.walk_min_wz, self.walk_wz_stop = walk_min_wz, walk_wz_stop
        self.ma = MovingAverage(smooth_s, dt) if smooth_s > 0 else None
        self.cte_ma = MovingAverage(1.0, dt)
        self.vy_gain, self.vy_max, self.vy_ff = vy_gain, vy_max, vy_ff
        self.acc = 0.0

    def __call__(self, vx, wz, cte=None):
        out = self._call(vx, wz, cte)
        if self.vy_ff > 0 and out[0] > 0:
            out = (out[0], out[1] - self.vy_ff * out[0], out[2])
        return out

    @staticmethod
    def lift(v, stop, minimum):
        return 0.0 if abs(v) < stop else math.copysign(max(abs(v), minimum), v)

    def current(self, vx, wz):
        vx = max(vx, 0.0)
        if abs(wz) >= self.wz_stop:
            return max(vx, self.turn_min_vx), math.copysign(max(abs(wz), self.min_wz), wz)
        vx = self.lift(vx, self.vx_stop, self.min_vx)
        return vx, (wz if vx > 0 else 0.0)

    def _call(self, vx, wz, cte=None):
        """返回发给 G1 的 (vx, vy, wz)。cte = 当前横向偏差（左正，m），只有 vy 模式用。"""
        walking = vx >= self.vx_stop
        if self.mode in ("walkmin", "pulse") and walking:
            ws = self.ma(wz) if self.ma else wz
            vx = self.lift(vx, self.vx_stop, self.min_vx)
            if abs(ws) >= self.min_wz:         # 大转向（弯道）照常
                self.acc = 0.0
                return vx, 0.0, ws
            if self.mode == "walkmin":
                return vx, 0.0, (0.0 if abs(ws) < self.walk_wz_stop else math.copysign(max(abs(ws), self.walk_min_wz), ws))
            self.acc += ws * self.dt
            out = math.copysign(self.min_wz, self.acc) if abs(self.acc) >= 0.5 * self.min_wz * self.dt else 0.0
            self.acc -= out * self.dt
            return vx, 0.0, out
        if not walking:
            if self.ma is not None:
                self.ma.reset()
            self.acc = 0.0
        vx2, wz2 = self.current(vx, wz)
        vy = 0.0
        if self.mode == "vy" and cte is not None and vx2 > 0:
            c = self.cte_ma(cte)
            vy = max(-self.vy_max, min(self.vy_max, -self.vy_gain * c))
        return vx2, vy, wz2


class G1:
    """无背包 G1 的运动模型。truth = 骨盆真实位姿；published() = 叠加步态晃动后的"定位位姿"。"""

    def __init__(self, x=0.0, y=0.0, yaw=0.0, walk_resp="measured", walk_yaw_bias=0.014, latency=0.35, tau_v=0.3, tau_w=0.2,
                 dist_sigma=0.10, dist_tau=5.0, sway_y=0.06, sway_yaw_deg=2.5, sway_f=1.0,
                 spin_drift=True, vy_dead=0.10, crab_deg=9.5, yaw_comp_deg=0.0, seed=0):
        self.x, self.y, self.yaw = x, y, yaw
        self.walk_yaw_bias = walk_yaw_bias
        self.walk_resp, self.latency, self.tau_v, self.tau_w = walk_resp, latency, tau_v, tau_w
        self.dist_sigma, self.dist_tau = dist_sigma, dist_tau
        self.sway_y, self.sway_yaw, self.sway_f = sway_y, math.radians(sway_yaw_deg), sway_f
        self.spin_drift, self.vy_dead = spin_drift, vy_dead
        self.crab = math.tan(math.radians(crab_deg))       # 走路时侧向速度 = crab × 前进速度（左正）
        self.yaw_comp = math.radians(yaw_comp_deg)         # 方案：发布的航向按走路速度加上蟹行角（=实际行进方向）
        self.rng = random.Random(seed)
        self.queue = deque()          # (t_apply, t_expire, vx, vy, wz)
        self.cmd = (0.0, 0.0, 0.0)
        self.cmd_expire = -1.0
        self.v = self.vy = self.w = 0.0
        self.b = 0.0                   # 走路时航向角速度扰动（OU）
        self.phase = 0.0
        self.gait = 0.0                # 步态幅度 0~1（随走路速度）
        self.t = 0.0

    def command(self, t, vx, vy, wz, duration=0.5):
        ta = t + self.latency
        self.queue.append((ta, ta + duration, vx, vy, wz))

    def step(self, dt):
        self.t += dt
        while self.queue and self.queue[0][0] <= self.t:
            _, te, vx, vy, wz = self.queue.popleft()
            self.cmd, self.cmd_expire = (vx, vy, wz), te
        vx, vy, wz = self.cmd if self.t <= self.cmd_expire else (0.0, 0.0, 0.0)
        walking = vx >= 0.05
        tw = walk_rate(wz, self.walk_resp) if walking else spin_rate(wz)
        tvy = 0.0 if abs(vy) < self.vy_dead else vy
        a_v = min(1.0, dt / self.tau_v)
        a_w = min(1.0, dt / self.tau_w)
        self.v += (vx - self.v) * a_v
        self.vy += (tvy - self.vy) * a_v
        self.w += (tw - self.w) * a_w
        # 走路时航向慢漂（OU），幅度随速度
        s = min(1.0, max(0.0, self.v / 0.35))
        self.b += -self.b / self.dist_tau * dt + self.dist_sigma * math.sqrt(2 * dt / self.dist_tau) * self.rng.gauss(0, 1)
        w = self.w + (self.b + self.walk_yaw_bias) * s
        # 原地转漂移：机体系向后
        back = 0.0
        if self.spin_drift and not walking and abs(self.w) > 0.1:
            back = interp(CCW_DRIFT if wz > 0 else CW_DRIFT, abs(wz))
        vxb = self.v - back
        vyb = self.vy + (self.crab * self.v if self.v > 0.05 else 0.0)
        self.yaw += w * dt
        c, sn = math.cos(self.yaw), math.sin(self.yaw)
        self.x += (vxb * c - vyb * sn) * dt
        self.y += (vxb * sn + vyb * c) * dt
        # 步态相位与幅度
        self.gait += ((1.0 if self.v > 0.15 else 0.0) - self.gait) * min(1.0, dt / 0.5)
        self.phase += 2 * math.pi * self.sway_f * (1 + 0.04 * self.rng.gauss(0, 1)) * dt
        return self.truth()

    def truth(self):
        return self.x, self.y, self.yaw

    def published(self):
        """定位输出的 base_link（上身晃动 + 航向摆动）。"""
        g = self.gait
        dy = g * self.sway_y * math.sin(self.phase)
        dyaw = g * self.sway_yaw * math.sin(self.phase + math.pi / 2)
        comp = self.yaw_comp * min(1.0, max(0.0, self.v / 0.35))
        c, s = math.cos(self.yaw), math.sin(self.yaw)
        return self.x - dy * s, self.y + dy * c, self.yaw + dyaw + comp
