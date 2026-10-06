"""VLN 与传统导航的仲裁、真机运动的安全闸门、VLN 会话状态机（纯 Python，不依赖 ROS）。

ROS 节点（bridge_node.py）只负责把话题/服务/图结构"喂"进来、把 tick() 的输出发出去；
所有"能不能启动、要不要停、发到哪个话题、发什么速度"的判断都在这里，单元测试直接覆盖。

规则（对应用户的硬性要求）：

1. 导航忙（/task_status 执行中或暂停、Nav2 动作在执行、/cmd_vel_nav 有输出、刚收到 /nav_to_pose 或 /patrol_path）
   → 拒绝启动 VLN，并给出原因。
2. VLN 运行中（含连接中）一旦导航忙 → 立即停止 VLN；如果正在真机输出，给真机话题补发 **一帧** 零速后静默，
   把速度控制让给 Nav2。
3. 默认只发观察话题（/vln/cmd_vel）。真机运动必须由用户在 VLN 运行中显式切换，并且同时满足：导航空闲且状态可知、
   真机话题经过与 Nav2 相同的安全链路（订阅者里有 collision_monitor；不能是 /cmd_vel_safe 之类会绕过它的话题）、
   网页心跳新鲜、相机不是头部 D435、已经有新鲜的推理结果。任何一条在运行中失效 → 自动退回观察并补发零速。
4. 每次停止（任何原因）都回到观察模式，下次启动不会直接驱动机器人。
5. 所有发出去的速度都经过 sanitize_twist：前进速度不得为负（G1 倒走会摔倒）、限幅、非有限值归零。
"""

from __future__ import annotations

import threading
import time
from collections import deque
from dataclasses import dataclass, field
from typing import Any, Deque, Dict, Iterable, List, Optional, Sequence, Tuple

from .velocity import (MappingConfig, Twist3, VelocityLimits, ZERO, sanitize_twist,
                       waypoints_to_twist)

# 会话状态
IDLE = "idle"
STARTING = "starting"
RUNNING = "running"
STATE_TEXT = {IDLE: "空闲", STARTING: "连接推理服务中", RUNNING: "运行中"}

# 输出模式 / 输出目标
OBSERVE = "observe"
REAL = "real"
NONE = "none"
MODE_TEXT = {OBSERVE: "观察（不驱动机器人）", REAL: "真机运动"}

# /task_status（aid_robot_msgs/AidTaskStatus，waypoint_manage 每秒发一次）
TASK_STATUS_TEXT = {0: "空闲", 1: "执行中", 2: "成功", 3: "失败", 4: "暂停", 5: "已取消"}
TASK_TYPE_TEXT = {0: "单点导航", 1: "巡逻"}
# action_msgs/GoalStatus：1 ACCEPTED、2 EXECUTING、3 CANCELING 视为动作还在占用机器人
ACTIVE_GOAL_CODES = (1, 2, 3)

# 停止原因代码
STOP_USER = "user"
STOP_MODEL = "model_stop"
STOP_NAV = "nav_preempt"
STOP_CAMERA = "camera"
STOP_SERVER = "server_error"
STOP_ERROR = "error"
STOP_TIMEOUT = "max_run"
STOP_SHUTDOWN = "shutdown"
_STOP_LEVEL = {STOP_USER: "info", STOP_MODEL: "info", STOP_NAV: "warn", STOP_TIMEOUT: "warn",
               STOP_SHUTDOWN: "warn"}


# =============================================================================
# 导航状态监视
# =============================================================================
@dataclass(frozen=True)
class NavConfig:
    task_status_timeout: float = 3.0   # /task_status 1 Hz，超过这么久没收到 = 状态未知
    nav_cmd_window: float = 0.5        # /cmd_vel_nav 在这个窗口内有消息 = Nav2 正在发速度
    goal_latch_s: float = 3.0          # 收到 /nav_to_pose、/patrol_path 后这么久内视为导航忙（等 /task_status 跟上）
    busy_task_codes: Tuple[int, ...] = (1, 4)   # 执行中、暂停


@dataclass
class NavState:
    known: bool
    busy: bool
    reasons: List[str]
    task_status: Optional[int] = None
    task_type: Optional[int] = None
    task_age: Optional[float] = None
    active_actions: List[str] = field(default_factory=list)
    nav_cmd_age: Optional[float] = None
    goal_age: Optional[float] = None
    goal_source: Optional[str] = None

    def to_status(self) -> Dict[str, Any]:
        return {
            "known": self.known,
            "busy": self.busy,
            "reasons": list(self.reasons),
            "task_status": self.task_status,
            "task_status_text": TASK_STATUS_TEXT.get(self.task_status, "--")
            if self.task_status is not None else "--",
            "task_type": self.task_type,
            "task_type_text": TASK_TYPE_TEXT.get(self.task_type, "--") if self.task_type is not None else "--",
            "task_age_s": _r(self.task_age),
            "active_actions": list(self.active_actions),
            "nav_cmd_age_s": _r(self.nav_cmd_age),
            "goal_age_s": _r(self.goal_age),
            "goal_source": self.goal_source,
        }


class NavMonitor:
    """把几路导航信号合成"导航忙 / 空闲 / 未知"。不加锁，由 VlnCore 的锁保护。"""

    def __init__(self, cfg: NavConfig = NavConfig()):
        self.cfg = cfg
        self.task_status: Optional[int] = None
        self.task_type: Optional[int] = None
        self.task_t: Optional[float] = None
        self.actions: Dict[str, Tuple[Tuple[int, ...], float]] = {}
        self.nav_cmd_t: Optional[float] = None
        self.goal_t: Optional[float] = None
        self.goal_source: Optional[str] = None

    def on_task_status(self, status: int, task_type: int, now: float) -> None:
        self.task_status = int(status)
        self.task_type = int(task_type)
        self.task_t = now

    def on_action_status(self, action: str, codes: Iterable[int], now: float) -> None:
        self.actions[action] = (tuple(int(c) for c in codes), now)

    def clear_action(self, action: str) -> None:
        """动作服务端已经不在了（状态话题没有发布者）：丢掉它最后留下的状态。"""
        self.actions.pop(action, None)

    def on_nav_cmd(self, now: float) -> None:
        self.nav_cmd_t = now

    def on_nav_goal(self, source: str, now: float) -> None:
        self.goal_t = now
        self.goal_source = source

    def state(self, now: float) -> NavState:
        reasons: List[str] = []
        task_age = None if self.task_t is None else max(0.0, now - self.task_t)
        known = task_age is not None and task_age <= self.cfg.task_status_timeout
        if known and self.task_status in self.cfg.busy_task_codes:
            st = TASK_STATUS_TEXT.get(self.task_status, str(self.task_status))
            tt = TASK_TYPE_TEXT.get(self.task_type, str(self.task_type))
            hint = "，请先在 G1 控制台取消该任务" if self.task_status == 4 else ""
            reasons.append(f"导航任务{st}（/task_status={self.task_status}，{tt}）{hint}")
        active = sorted(name for name, (codes, _t) in self.actions.items()
                        if any(c in ACTIVE_GOAL_CODES for c in codes))
        for name in active:
            reasons.append(f"Nav2 动作 {name} 正在执行")
        nav_cmd_age = None if self.nav_cmd_t is None else max(0.0, now - self.nav_cmd_t)
        if nav_cmd_age is not None and nav_cmd_age <= self.cfg.nav_cmd_window:
            reasons.append(f"Nav2 正在发速度（/cmd_vel_nav，{nav_cmd_age:.1f} s 前）")
        goal_age = None if self.goal_t is None else max(0.0, now - self.goal_t)
        if goal_age is not None and goal_age <= self.cfg.goal_latch_s:
            reasons.append(f"收到新的导航目标（{self.goal_source}，{goal_age:.1f} s 前）")
        return NavState(known=known, busy=bool(reasons), reasons=reasons,
                        task_status=self.task_status, task_type=self.task_type, task_age=task_age,
                        active_actions=active, nav_cmd_age=nav_cmd_age, goal_age=goal_age,
                        goal_source=self.goal_source)


# =============================================================================
# 速度链路检查（真机话题后面接的是不是和 Nav2 一样的安全链路）
# =============================================================================
@dataclass(frozen=True)
class Endpoint:
    node: str
    type: str


@dataclass(frozen=True)
class ChainConfig:
    real_topic: str = "/cmd_vel"
    observe_topic: str = "/vln/cmd_vel"
    expected_type: str = "geometry_msgs/msg/Twist"
    require_collision_monitor: bool = True
    collision_monitor_node: str = "collision_monitor"
    # 名字里含这些子串的节点会把速度送到机器人本体
    driver_patterns: Tuple[str, ...] = ("cmdvel_to_sport", "collision_monitor")
    motion_bridge_pattern: str = "cmdvel_to_sport"
    # 发到这些话题会绕过 collision_monitor（它的输出 / 运动桥的直接输入）
    forbidden_real_topics: Tuple[str, ...] = ("/cmd_vel_safe",)
    own_node: str = "vln_bridge"


@dataclass
class ChainStatus:
    checked: bool
    real_ok: bool
    real_reasons: List[str]
    real_subscribers: List[str]
    collision_monitor_in_chain: bool
    collision_monitor_running: bool
    observe_safe: bool
    observe_reasons: List[str]
    observe_subscribers: List[str]

    @staticmethod
    def unchecked() -> "ChainStatus":
        return ChainStatus(checked=False, real_ok=False, real_reasons=["还没检查过速度链路"],
                           real_subscribers=[], collision_monitor_in_chain=False,
                           collision_monitor_running=False, observe_safe=True, observe_reasons=[],
                           observe_subscribers=[])

    def to_status(self) -> Dict[str, Any]:
        return {
            "checked": self.checked,
            "real_ok": self.real_ok,
            "real_reasons": list(self.real_reasons),
            "real_subscribers": list(self.real_subscribers),
            "collision_monitor_in_chain": self.collision_monitor_in_chain,
            "collision_monitor_running": self.collision_monitor_running,
            "observe_safe": self.observe_safe,
            "observe_reasons": list(self.observe_reasons),
            "observe_subscribers": list(self.observe_subscribers),
        }


def _base_name(node: str) -> str:
    return node.rstrip("/").rsplit("/", 1)[-1]


def evaluate_chain(cfg: ChainConfig, real_subs: Sequence[Endpoint], observe_subs: Sequence[Endpoint],
                   node_names: Sequence[str]) -> ChainStatus:
    """根据 ROS 图（话题订阅者 + 在线节点名）判断真机话题是否安全、观察话题是否真的"只观察"。"""
    real_reasons: List[str] = []
    t = cfg.real_topic
    if t in cfg.forbidden_real_topics:
        real_reasons.append(f"真机话题 {t} 在禁止列表里：发到这里会绕过 collision_monitor，必须发 /cmd_vel")
    if t == cfg.observe_topic:
        real_reasons.append("真机话题和观察话题不能相同")
    subs = [s for s in real_subs if _base_name(s.node) != cfg.own_node]
    names = sorted({_base_name(s.node) for s in subs})
    cm_in_chain = any(_base_name(s.node) == cfg.collision_monitor_node for s in subs)
    cm_running = any(_base_name(n) == cfg.collision_monitor_node for n in node_names)
    if not subs:
        real_reasons.append(f"真机话题 {t} 没有订阅者：Nav2 运动桥没在运行（需要定位模式且 Nav2 已启动）")
    bad_types = sorted({s.type for s in subs if s.type != cfg.expected_type})
    if bad_types:
        real_reasons.append(f"{t} 的订阅者类型是 {', '.join(bad_types)}，与 {cfg.expected_type} 不符")
    if cm_running and not cm_in_chain:
        real_reasons.append(f"collision_monitor 在运行但没有订阅 {t}：发到这里会绕过它")
    if cm_in_chain and any(cfg.motion_bridge_pattern in _base_name(s.node) for s in subs):
        real_reasons.append(f"运动桥也直接订阅了 {t}，会绕过 collision_monitor")
    if cfg.require_collision_monitor and subs and not cm_in_chain:
        real_reasons.append(
            f"{t} 的订阅者（{', '.join(names)}）里没有 collision_monitor（栈是 use_collision_monitor:=false 启动的？）。"
            "VLN 自身不避障，默认不允许没有碰撞监测的真机运动；确需测试，用 require_collision_monitor:=false 重启桥接")
    obs = [s for s in observe_subs if _base_name(s.node) != cfg.own_node]
    obs_names = sorted({_base_name(s.node) for s in obs})
    drivers = sorted({_base_name(s.node) for s in obs
                      if any(p in _base_name(s.node) for p in cfg.driver_patterns)})
    observe_reasons: List[str] = []
    if drivers:
        observe_reasons.append(f"观察话题 {cfg.observe_topic} 被 {', '.join(drivers)} 订阅，会驱动机器人：已停止向它发布")
    if cfg.observe_topic in (t, *cfg.forbidden_real_topics):
        observe_reasons.append(f"观察话题 {cfg.observe_topic} 是会驱动机器人的话题")
    return ChainStatus(checked=True, real_ok=not real_reasons, real_reasons=real_reasons,
                       real_subscribers=names, collision_monitor_in_chain=cm_in_chain,
                       collision_monitor_running=cm_running, observe_safe=not observe_reasons,
                       observe_reasons=observe_reasons, observe_subscribers=obs_names)


# =============================================================================
# VLN 会话状态机
# =============================================================================
@dataclass(frozen=True)
class CoreConfig:
    limits: VelocityLimits = VelocityLimits()
    mapping: MappingConfig = MappingConfig()
    nav: NavConfig = NavConfig()
    cmd_timeout: float = 0.6            # 推理结果最多沿用这么久，过期发零速
    image_timeout: float = 3.0          # 运行中相机这么久没新图 → 停止
    start_image_max_age: float = 1.0    # 启动时相机最近一帧不能比这更老
    heartbeat_timeout: float = 3.0      # 真机模式下网页心跳超时 → 退回观察
    require_heartbeat_for_real: bool = True
    require_nav_status_for_real: bool = True
    allow_real_with_head_camera: bool = False
    camera_is_head: bool = False
    camera_topic: str = ""
    stop_confirm_count: int = 1         # 连续几次 stop=true 才结束（官方建议 1）
    max_consecutive_errors: int = 5
    zero_burst: int = 3                 # 退出真机时补发几帧零速（让给导航时只补 1 帧）
    max_run_s: float = 0.0              # >0 时运行这么久自动停止
    max_instruction_len: int = 500


@dataclass
class CameraInfo:
    last_rx: Optional[float] = None
    width: int = 0
    height: int = 0
    encoding: str = ""
    error: str = ""
    error_t: Optional[float] = None
    rx_times: Deque[float] = field(default_factory=lambda: deque(maxlen=90))

    def age(self, now: float) -> Optional[float]:
        return None if self.last_rx is None else max(0.0, now - self.last_rx)

    def fps(self, now: float, window: float = 3.0) -> float:
        n = sum(1 for t in self.rx_times if now - t <= window)
        return n / window


def _r(v: Optional[float], n: int = 2) -> Optional[float]:
    return None if v is None else round(float(v), n)


class VlnCore:
    """线程安全：所有公开方法都在同一把锁里执行。now 一律是 time.monotonic() 秒。"""

    def __init__(self, cfg: CoreConfig = CoreConfig(), chain_cfg: ChainConfig = ChainConfig(),
                 wall_clock=time.time):
        self.cfg = cfg
        self.chain_cfg = chain_cfg
        self._wall = wall_clock
        self._lock = threading.RLock()
        self.nav = NavMonitor(cfg.nav)
        self.state = IDLE
        self.mode = OBSERVE
        self.output = NONE
        self._sid = 0
        self._active_sid: Optional[int] = None
        self.instruction = ""
        self.episode = 0
        self.started_at: Optional[float] = None
        self.running_since: Optional[float] = None
        self.last_pred = None
        self.last_pred_t: Optional[float] = None
        self.pred_valid = False
        self.pred_cmd: Twist3 = ZERO
        self.pred_times: Deque[float] = deque(maxlen=40)
        self.n_pred = 0
        self.consecutive_stops = 0
        self.consecutive_errors = 0
        self.n_errors = 0
        self.last_error_text = ""
        self.cmd: Twist3 = ZERO
        self.cmd_note = "未运行"
        self.pending_real_zeros = 0
        self.pending_observe_zeros = 0
        self.last_stop: Optional[Dict[str, Any]] = None
        self.error = ""
        self.heartbeat_t: Optional[float] = None
        self.heartbeat_src = ""
        self.chain = ChainStatus.unchecked()
        self.camera = CameraInfo()
        self.server: Dict[str, Any] = {"connected": False, "text": "未连接"}
        self.robot_status: Optional[str] = None
        self.robot_status_t: Optional[float] = None
        self.events: Deque[Dict[str, Any]] = deque(maxlen=40)
        self._new_events: List[Dict[str, Any]] = []

    # ------------------------------------------------------------ helpers ----
    def _event(self, level: str, text: str) -> None:
        ev = {"t": round(self._wall(), 3), "level": level, "text": text}
        self.events.append(ev)
        self._new_events.append(ev)

    def drain_events(self) -> List[Dict[str, Any]]:
        """取出上次之后新产生的事件（节点用它写 ROS 日志）。"""
        with self._lock:
            out, self._new_events = self._new_events, []
            return out

    def is_active(self, sid: int) -> bool:
        with self._lock:
            return sid == self._active_sid and self.state in (STARTING, RUNNING)

    def instruction_for(self, sid: int) -> Optional[str]:
        with self._lock:
            return self.instruction if self.is_active(sid) else None

    @property
    def active_session(self) -> Optional[int]:
        with self._lock:
            return self._active_sid

    # ------------------------------------------------------------- inputs ----
    def on_image(self, now: float, width: int, height: int, encoding: str) -> None:
        with self._lock:
            c = self.camera
            c.last_rx = now
            c.rx_times.append(now)
            c.width, c.height, c.encoding = int(width), int(height), str(encoding)

    def on_camera_error(self, text: str, now: float) -> None:
        with self._lock:
            if text != self.camera.error:
                self._event("warn", f"相机图像处理失败：{text}")
            self.camera.error = text
            self.camera.error_t = now

    def on_heartbeat(self, now: float, source: str = "") -> None:
        with self._lock:
            self.heartbeat_t = now
            self.heartbeat_src = str(source)[:64]

    def on_robot_status(self, text: str, now: float) -> None:
        with self._lock:
            self.robot_status = str(text)[:64]
            self.robot_status_t = now

    def on_chain(self, chain: ChainStatus, now: float) -> None:
        with self._lock:
            if self.chain.checked and chain.observe_safe != self.chain.observe_safe and not chain.observe_safe:
                self._event("error", "；".join(chain.observe_reasons))
            self.chain = chain

    def on_task_status(self, status: int, task_type: int, now: float) -> None:
        with self._lock:
            self.nav.on_task_status(status, task_type, now)
            self._check_preempt(now)

    def on_action_status(self, action: str, codes: Iterable[int], now: float) -> None:
        with self._lock:
            self.nav.on_action_status(action, codes, now)
            self._check_preempt(now)

    def clear_action(self, action: str) -> None:
        with self._lock:
            self.nav.clear_action(action)

    def on_nav_cmd(self, now: float) -> None:
        with self._lock:
            self.nav.on_nav_cmd(now)
            self._check_preempt(now)

    def on_nav_goal(self, source: str, now: float) -> None:
        with self._lock:
            self.nav.on_nav_goal(source, now)
            self._check_preempt(now)

    # ----------------------------------------------------------- requests ----
    def validate_instruction(self, text: Any) -> Tuple[bool, str]:
        if not isinstance(text, str):
            return False, "指令必须是字符串"
        s = text.strip()
        if not s:
            return False, "指令为空"
        if len(s) > self.cfg.max_instruction_len:
            return False, f"指令太长（{len(s)} > {self.cfg.max_instruction_len} 字）"
        return True, s

    def set_instruction(self, text: Any, now: float) -> Tuple[bool, str]:
        """更新指令。运行中也可以改：下一帧起生效（LightNav 的指令随每帧发送，不需要 reset）。"""
        with self._lock:
            if isinstance(text, str) and not text.strip() and self.state == IDLE:
                self.instruction = ""
                return True, ""
            ok, s = self.validate_instruction(text)
            if not ok:
                return False, s
            if s != self.instruction and self.state != IDLE:
                self._event("info", f"运行中更新指令：{s}")
            self.instruction = s
            return True, ""

    def request_start(self, now: float, instruction: Optional[str] = None) -> Tuple[bool, str, Optional[int]]:
        with self._lock:
            if self.state != IDLE:
                return False, f"VLN 已经在{STATE_TEXT[self.state]}，先停止再启动", None
            ok, s = self.validate_instruction(self.instruction if instruction is None else instruction)
            if not ok:
                return False, f"不能启动：{s}", None
            nav = self.nav.state(now)
            if nav.busy:
                msg = "拒绝启动：" + "；".join(nav.reasons) + "。导航结束（或在 G1 控制台取消任务）后再启动 VLN"
                self._event("warn", msg)
                return False, msg, None
            cam_age = self.camera.age(now)
            topic = self.cfg.camera_topic or "相机话题"
            if cam_age is None:
                msg = f"拒绝启动：还没收到相机图像（{topic}）。检查相机是否在发布、话题名是否正确"
                self._event("warn", msg)
                return False, msg, None
            if cam_age > self.cfg.start_image_max_age:
                msg = f"拒绝启动：相机最近一帧是 {cam_age:.1f} s 前（{topic}），相机可能停了"
                self._event("warn", msg)
                return False, msg, None
            self._sid += 1
            self._active_sid = self._sid
            self.instruction = s
            self.state = STARTING
            self.mode = OBSERVE
            self.episode += 1
            self.started_at = now
            self.running_since = None
            self.last_pred = None
            self.last_pred_t = None
            self.pred_valid = False
            self.pred_cmd = ZERO
            self.pred_times.clear()
            self.n_pred = 0
            self.consecutive_stops = 0
            self.consecutive_errors = 0
            self.n_errors = 0
            self.last_error_text = ""
            self.error = ""
            self.cmd = ZERO
            self.cmd_note = "等待第一次推理结果"
            msg = f"已启动（观察模式，第 {self.episode} 轮）：{s}"
            notes = []
            if not nav.known:
                notes.append("注意：收不到 /task_status，无法确认导航状态（观察模式不驱动机器人，可以继续）")
            if self.cfg.camera_is_head:
                notes.append("注意：当前相机是头部 D435（下俯 47.6°，看不到前方），只能验证链路")
            self._event("info", msg)
            for n in notes:
                self._event("warn", n)
            return True, "；".join([msg] + notes), self._active_sid

    def request_stop(self, now: float, text: str = "用户停止", code: str = STOP_USER) -> Tuple[bool, str]:
        with self._lock:
            if self.state == IDLE:
                if self.mode == REAL:
                    self.mode = OBSERVE
                return True, "VLN 本来就没有在运行"
            self._stop(code, text, now)
            return True, "已停止" + ("" if code == STOP_USER else f"：{text}")

    def request_mode(self, real: bool, now: float) -> Tuple[bool, str]:
        with self._lock:
            if not real:
                if self.mode == REAL:
                    self._drop_real(now, "用户切回观察模式", level="info")
                return True, "已切到观察模式（只发 " + self.chain_cfg.observe_topic + "，不驱动机器人）"
            if self.mode == REAL:
                return True, "已经是真机运动"
            reasons = self._real_gate(now, for_switch=True)
            if reasons:
                msg = "不能切到真机运动：" + "；".join(reasons)
                self._event("warn", msg)
                return False, msg
            self.mode = REAL
            self.pending_real_zeros = 0
            msg = (f"已切到真机运动：速度发到 {self.chain_cfg.real_topic}（→ "
                   f"{' / '.join(self.chain.real_subscribers) or '?'} → 机器人）")
            self._event("warn", msg)
            return True, msg

    # ------------------------------------------------------- worker inputs ----
    def on_server_state(self, sid: int, connected: bool, text: str) -> None:
        with self._lock:
            if sid == self._active_sid:
                self.server = {"connected": bool(connected), "text": str(text)}

    def on_worker_connected(self, sid: int, now: float) -> None:
        with self._lock:
            if sid != self._active_sid or self.state != STARTING:
                return
            self.state = RUNNING
            self.running_since = now
            self._event("info", "已连上推理服务，开始推理")

    def on_prediction(self, sid: int, pred: Any, now: float) -> None:
        with self._lock:
            if sid != self._active_sid or self.state != RUNNING:
                return
            self.last_pred = pred
            self.last_pred_t = now
            self.pred_valid = True
            self.n_pred += 1
            self.pred_times.append(now)
            self.consecutive_errors = 0
            if pred.stop:
                self.pred_cmd = ZERO
                self.consecutive_stops += 1
                if self.consecutive_stops >= max(1, self.cfg.stop_confirm_count):
                    self._stop(STOP_MODEL, "模型输出 stop（认为已到达目标），本轮结束", now)
            else:
                self.consecutive_stops = 0
                self.pred_cmd = waypoints_to_twist(pred.waypoints, self.cfg.limits, self.cfg.mapping)

    def on_prediction_error(self, sid: int, text: str, now: float) -> None:
        with self._lock:
            if sid != self._active_sid or self.state not in (STARTING, RUNNING):
                return
            self.pred_valid = False
            self.consecutive_errors += 1
            self.n_errors += 1
            self.last_error_text = str(text)
            if self.consecutive_errors == 1 or self.consecutive_errors % 10 == 0:
                self._event("warn", f"推理出错（连续 {self.consecutive_errors} 次）：{text}")
            if self.consecutive_errors >= max(1, self.cfg.max_consecutive_errors):
                self._stop(STOP_SERVER, f"推理连续出错 {self.consecutive_errors} 次，已停止：{text}", now)

    def on_worker_failed(self, sid: int, text: str, now: float) -> None:
        with self._lock:
            if sid != self._active_sid or self.state == IDLE:
                return
            self._stop(STOP_ERROR, str(text), now)

    # --------------------------------------------------------------- tick ----
    def tick(self, now: float) -> List[Tuple[str, Twist3]]:
        """控制周期（10 Hz）。返回本周期要发布的 [(目标, (vx, vy, wz))]，目标是 OBSERVE 或 REAL。"""
        with self._lock:
            out: List[Tuple[str, Twist3]] = []
            self._check_preempt(now)
            if self.state == RUNNING:
                self._check_running_health(now)
            publish_real = False
            if self.state == RUNNING:
                self.cmd, self.cmd_note = self._current_cmd(now)
                if self.chain.observe_safe:
                    out.append((OBSERVE, self.cmd))
                if self.mode == REAL:
                    reasons = self._real_gate(now, for_switch=False)
                    if reasons:
                        self._drop_real(now, "真机运动已自动关闭：" + "；".join(reasons))
                    else:
                        out.append((REAL, self.cmd))
                        publish_real = True
            if not publish_real and self.pending_real_zeros > 0:
                out.append((REAL, ZERO))
                self.pending_real_zeros -= 1
            if self.state != RUNNING and self.pending_observe_zeros > 0:
                if self.chain.observe_safe:
                    out.append((OBSERVE, ZERO))
                self.pending_observe_zeros -= 1
            if publish_real:
                self.output = REAL
            elif self.state == RUNNING and self.chain.observe_safe:
                self.output = OBSERVE
            else:
                self.output = NONE
            return [(target, sanitize_twist(v, self.cfg.limits)) for target, v in out]

    def shutdown(self, now: float) -> List[Tuple[str, Twist3]]:
        """节点退出前调用：停止会话，返回需要立刻发出的零速（真机在用时补满 zero_burst 帧）。"""
        with self._lock:
            was_real = self.mode == REAL
            if self.state != IDLE:
                self._stop(STOP_SHUTDOWN, "桥接节点退出", now)
            n = max(self.pending_real_zeros, self.cfg.zero_burst if was_real else 0)
            self.pending_real_zeros = 0
            self.pending_observe_zeros = 0
            return [(REAL, ZERO)] * n

    # ------------------------------------------------------------ internals ----
    def _current_cmd(self, now: float) -> Tuple[Twist3, str]:
        if self.last_pred is None:
            return ZERO, "等待第一次推理结果（零速）"
        if not self.pred_valid:
            return ZERO, "上一次推理出错（零速）"
        age = now - (self.last_pred_t or now)
        if age > self.cfg.cmd_timeout:
            return ZERO, f"推理结果已过期 {age:.1f} s（>{self.cfg.cmd_timeout:.1f} s，零速）"
        return self.pred_cmd, "按最新推理结果"

    def _check_preempt(self, now: float) -> None:
        if self.state not in (STARTING, RUNNING):
            return
        nav = self.nav.state(now)
        if nav.busy:
            self._stop(STOP_NAV, "导航开始新任务（" + "；".join(nav.reasons) + "），VLN 已停止并让出速度控制",
                       now, yield_to_nav=True)

    def _check_running_health(self, now: float) -> None:
        age = self.camera.age(now)
        if age is None or age > self.cfg.image_timeout:
            shown = "从未收到" if age is None else f"已 {age:.1f} s 没有新图像"
            self._stop(STOP_CAMERA, f"相机 {self.cfg.camera_topic} {shown}，已停止", now)
            return
        if self.cfg.max_run_s > 0 and self.running_since is not None \
                and now - self.running_since > self.cfg.max_run_s:
            self._stop(STOP_TIMEOUT, f"已运行 {self.cfg.max_run_s:.0f} s（max_run_s），自动停止", now)

    def _real_gate(self, now: float, for_switch: bool) -> List[str]:
        reasons: List[str] = []
        if self.state != RUNNING:
            reasons.append("VLN 没有在运行：先启动，在观察模式下确认输出正常后再切真机")
        nav = self.nav.state(now)
        if nav.busy:
            reasons.extend(nav.reasons)
        elif not nav.known and self.cfg.require_nav_status_for_real:
            age = "从未收到" if nav.task_age is None else f"已 {nav.task_age:.1f} s 没收到"
            reasons.append(f"/task_status {age}，无法确认导航空闲")
        if not self.chain.real_ok:
            reasons.extend(self.chain.real_reasons)
        if self.cfg.require_heartbeat_for_real:
            if self.heartbeat_t is None:
                reasons.append("没有收到网页心跳（/vln/heartbeat）：真机运动必须开着控制网页")
            elif now - self.heartbeat_t > self.cfg.heartbeat_timeout:
                reasons.append(f"网页心跳已 {now - self.heartbeat_t:.1f} s 没更新（网页关了、断网或页面在后台）")
        if self.cfg.camera_is_head and not self.cfg.allow_real_with_head_camera:
            reasons.append("当前相机是头部 D435（下俯 47.6°，看不到前方），不允许真机运动；"
                           "接上水平前视相机后用 image_topic:=<话题> 启动桥接")
        if for_switch:
            if self.last_pred is None or not self.pred_valid:
                reasons.append("还没有有效的推理结果")
            elif self.last_pred_t is not None and now - self.last_pred_t > self.cfg.cmd_timeout:
                reasons.append("推理结果已过期")
        return reasons

    def _drop_real(self, now: float, text: str, level: str = "warn") -> None:
        if self.mode != REAL:
            return
        self.mode = OBSERVE
        self.pending_real_zeros = max(self.pending_real_zeros, self.cfg.zero_burst)
        self._event(level, text)

    def _stop(self, code: str, text: str, now: float, yield_to_nav: bool = False) -> None:
        if self.state == IDLE:
            return
        was_real = self.mode == REAL
        was_running = self.state == RUNNING
        self.state = IDLE
        self._active_sid = None
        self.mode = OBSERVE
        if was_real:
            # 让给导航：只补 1 帧零速就静默，避免和 Nav2 的速度交替；其它原因补满 zero_burst 帧
            self.pending_real_zeros = 1 if yield_to_nav else max(1, self.cfg.zero_burst)
        if was_running:
            self.pending_observe_zeros = 1
        self.cmd = ZERO
        self.cmd_note = "已停止"
        self.pred_valid = False
        self.server = {"connected": False, "text": "未连接"}
        level = _STOP_LEVEL.get(code, "error")
        self.last_stop = {"code": code, "text": text, "t": round(self._wall(), 3), "level": level}
        if level == "error":
            self.error = text
        self._event(level, text)

    # ------------------------------------------------------------- status ----
    def snapshot(self, now: float) -> Dict[str, Any]:
        with self._lock:
            nav = self.nav.state(now)
            gate = self._real_gate(now, for_switch=self.mode != REAL)
            pred = self.last_pred.to_status() if self.last_pred is not None else None
            if pred is not None:
                pred["age_s"] = _r(now - self.last_pred_t) if self.last_pred_t is not None else None
            recent = [t for t in self.pred_times if now - t <= 5.0]
            pred_hz = (len(recent) - 1) / (recent[-1] - recent[0]) if len(recent) >= 2 and \
                recent[-1] > recent[0] else 0.0
            cam_age = self.camera.age(now)
            hb_age = None if self.heartbeat_t is None else now - self.heartbeat_t
            return {
                "state": self.state,
                "state_text": STATE_TEXT[self.state],
                "mode": self.mode,
                "mode_text": MODE_TEXT[self.mode],
                "output": self.output,
                "instruction": self.instruction,
                "episode": self.episode,
                "run_s": _r(now - self.running_since, 1) if self.running_since is not None
                and self.state == RUNNING else None,
                "error": self.error,
                "last_stop": self.last_stop,
                "camera": {
                    "topic": self.cfg.camera_topic,
                    "is_head_d435": self.cfg.camera_is_head,
                    "ok": cam_age is not None and cam_age <= self.cfg.start_image_max_age,
                    "age_s": _r(cam_age),
                    "fps": round(self.camera.fps(now), 1),
                    "width": self.camera.width,
                    "height": self.camera.height,
                    "encoding": self.camera.encoding,
                    "error": self.camera.error if self.camera.error_t is not None
                    and now - self.camera.error_t < 10.0 else "",
                },
                "server": {
                    **self.server,
                    "pred_hz": round(pred_hz, 2),
                    "n_pred": self.n_pred,
                    "n_errors": self.n_errors,
                    "consecutive_errors": self.consecutive_errors,
                    "last_error": self.last_error_text,
                },
                "pred": pred,
                "cmd": {"vx": round(self.cmd[0], 3), "vy": round(self.cmd[1], 3), "wz": round(self.cmd[2], 3),
                        "note": self.cmd_note},
                "nav": nav.to_status(),
                "robot_status": self.robot_status,
                "chain": self.chain.to_status(),
                "real_gate": {"ok": not gate, "reasons": gate},
                "heartbeat": {"age_s": _r(hb_age), "source": self.heartbeat_src,
                              "required": self.cfg.require_heartbeat_for_real,
                              "timeout_s": self.cfg.heartbeat_timeout},
                "events": list(self.events),
            }
