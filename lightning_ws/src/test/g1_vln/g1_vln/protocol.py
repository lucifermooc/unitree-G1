"""lightnav-serve 的 WebSocket JSON 协议（纯 Python）：拼请求、严格解析回包。

协议见 LightNav-0 docs/PROTOCOL.md（与 src/test/lightnav_server/bench_ws.py 用的同一套）：

    {"action": "login", "data": {"clientId": "..."}}       -> {"action": "login", "data": {"rc": 0, "msg": "ok"}}
    {"action": "reset", "data": {}}                         -> {"action": "reset", "data": {"rc": 0, "msg": "ok"}}
    {"action": "next",  "data": {"seq": N, "image": "<base64 JPEG>", "instruction": "..."}}
        -> {"action": "next", "data": {"rc": 0, "seq": N, "actions": {"step": .., "actions": [[fwd, lat, yaw] x H]},
                                       "latency_ms": .., "stop": bool, "visible": bool|null, "timings_ms": {..},
                                       "raw_text": "..", "pointing": {..}}}
    出错：{"data": {"rc": 400|500, "msg": "...", "seq": N}}，连接保持。
"""

from __future__ import annotations

import base64
import json
import math
from dataclasses import dataclass, field
from typing import Any, Dict, List, Optional, Tuple

Waypoint = Tuple[float, float, float]
Pixel = Tuple[float, float]

MAX_WAYPOINTS = 64          # 正常是 10 行（H = horizon），多了说明回包不对
MAX_RAW_TEXT = 256
POINTING_STATES = {"none", "point", "not_visible", "rot_left", "rot_right", "stop"}


class ProtocolError(RuntimeError):
    """回包不是协议规定的格式（JSON 坏了、字段类型不对等）。"""


class ServerError(RuntimeError):
    """服务端明确返回 rc != 0（400 请求不合法 / 500 推理或解码失败），连接仍可用。"""

    def __init__(self, action: str, rc: int, msg: str):
        super().__init__(f"{action} rc={rc}: {msg}")
        self.action = action
        self.rc = rc
        self.msg = msg


@dataclass
class Prediction:
    seq: int
    step: Optional[int]
    waypoints: List[Waypoint]
    stop: bool
    visible: Optional[bool]
    latency_ms: Optional[float]
    timings_ms: Dict[str, float] = field(default_factory=dict)
    raw_text: str = ""
    pointing: Optional[Dict[str, Any]] = None
    # 客户端补充：往返耗时、送图尺寸、这帧离开相机多久了
    rtt_ms: Optional[float] = None
    frame_size: Optional[Tuple[int, int]] = None
    frame_age_ms: Optional[float] = None

    def to_status(self) -> Dict[str, Any]:
        """给 /vln/status 用的精简字典（数字保留 3 位小数）。"""
        return {
            "seq": self.seq,
            "step": self.step,
            "stop": self.stop,
            "visible": self.visible,
            "latency_ms": _r(self.latency_ms, 1),
            "rtt_ms": _r(self.rtt_ms, 1),
            "frame_age_ms": _r(self.frame_age_ms, 1),
            "timings_ms": {k: _r(v, 1) for k, v in self.timings_ms.items()},
            "raw_text": self.raw_text,
            "waypoints": [[_r(x, 3) for x in row] for row in self.waypoints],
            "pointing": self.pointing,
            "frame_size": list(self.frame_size) if self.frame_size else None,
        }


def _r(v: Optional[float], n: int) -> Optional[float]:
    if v is None or not isinstance(v, (int, float)) or not math.isfinite(v):
        return None
    return round(float(v), n)


# ----------------------------------------------------------------- requests ----
def build_login(client_id: str) -> str:
    return json.dumps({"action": "login", "data": {"clientId": str(client_id)}})


def build_reset() -> str:
    return json.dumps({"action": "reset", "data": {}})


def build_next(seq: int, jpeg: bytes, instruction: Optional[str]) -> str:
    if isinstance(seq, bool) or not isinstance(seq, int) or seq < 0:
        raise ValueError("seq 必须是非负整数")
    if not jpeg:
        raise ValueError("图像为空")
    return json.dumps({"action": "next", "data": {
        "seq": seq,
        "image": base64.b64encode(jpeg).decode("ascii"),
        "instruction": instruction if instruction else None,
    }})


# ---------------------------------------------------------------- responses ----
def decode_response(raw: Any, expected_action: str) -> Dict[str, Any]:
    """解析一条回包，返回 data；rc != 0 抛 ServerError，格式不对抛 ProtocolError。"""
    if isinstance(raw, (bytes, bytearray)):
        try:
            raw = raw.decode("utf-8")
        except UnicodeDecodeError as e:
            raise ProtocolError(f"回包不是 UTF-8：{e}") from e
    try:
        obj = json.loads(raw)
    except (TypeError, json.JSONDecodeError) as e:
        raise ProtocolError(f"回包不是 JSON：{e}") from e
    if not isinstance(obj, dict):
        raise ProtocolError("回包必须是 JSON 对象")
    data = obj.get("data")
    if not isinstance(data, dict):
        raise ProtocolError("回包缺少 data 对象")
    rc = data.get("rc")
    if isinstance(rc, bool) or not isinstance(rc, int):
        raise ProtocolError("回包缺少整数 data.rc")
    action = obj.get("action")
    if rc != 0:
        raise ServerError(str(action or expected_action), rc, str(data.get("msg", "")))
    if action != expected_action:
        raise ProtocolError(f"期望 {expected_action!r} 的回包，收到 {action!r}")
    return data


def parse_prediction(data: Dict[str, Any], expected_seq: Optional[int] = None) -> Optional[Prediction]:
    """把 next 的 data 解析成 Prediction。只收图不推理（instruction 为空）时返回 None。"""
    seq = data.get("seq")
    if isinstance(seq, bool) or not isinstance(seq, int):
        raise ProtocolError("next 回包缺少整数 seq")
    if expected_seq is not None and seq != expected_seq:
        raise ProtocolError(f"seq 对不上：发 {expected_seq}，回 {seq}")
    if "actions" not in data:
        return None  # buffer-only: {"rc":0,"seq":N,"msg":"image received"}
    actions = data.get("actions")
    if not isinstance(actions, dict):
        raise ProtocolError("actions 必须是对象")
    waypoints = _parse_waypoints(actions.get("actions"))
    step = actions.get("step")
    if step is not None and (isinstance(step, bool) or not isinstance(step, int)):
        raise ProtocolError("actions.step 必须是整数")
    stop = data.get("stop", False)
    if not isinstance(stop, bool):
        raise ProtocolError("stop 必须是布尔值")
    visible = data.get("visible")
    if visible is not None and not isinstance(visible, bool):
        raise ProtocolError("visible 必须是布尔值或 null")
    latency = data.get("latency_ms")
    if latency is not None and (isinstance(latency, bool) or not isinstance(latency, (int, float))):
        raise ProtocolError("latency_ms 必须是数字")
    timings = {}
    raw_timings = data.get("timings_ms") or {}
    if isinstance(raw_timings, dict):
        for k, v in raw_timings.items():
            if isinstance(k, str) and isinstance(v, (int, float)) and not isinstance(v, bool) \
                    and math.isfinite(v):
                timings[k] = float(v)
    raw_text = data.get("raw_text") or ""
    if not isinstance(raw_text, str):
        raw_text = str(raw_text)
    return Prediction(
        seq=seq,
        step=step,
        waypoints=waypoints,
        stop=stop,
        visible=visible,
        latency_ms=float(latency) if latency is not None else None,
        timings_ms=timings,
        raw_text=raw_text[:MAX_RAW_TEXT],
        pointing=_parse_pointing(data.get("pointing")),
    )


def _parse_waypoints(value: Any) -> List[Waypoint]:
    if not isinstance(value, list):
        raise ProtocolError("actions.actions 必须是数组")
    if len(value) > MAX_WAYPOINTS:
        raise ProtocolError(f"航点太多（{len(value)} 行）")
    out: List[Waypoint] = []
    for row in value:
        if not isinstance(row, list) or len(row) != 3:
            raise ProtocolError("每个航点必须是 [forward, lateral, yaw]")
        vals = []
        for x in row:
            if isinstance(x, bool) or not isinstance(x, (int, float)) or not math.isfinite(x):
                raise ProtocolError("航点里有非数字或非有限值")
            vals.append(float(x))
        out.append((vals[0], vals[1], vals[2]))
    return out


def _parse_pixel(value: Any) -> Optional[Pixel]:
    if value is None:
        return None
    if (isinstance(value, list) and len(value) == 2
            and all(isinstance(v, (int, float)) and not isinstance(v, bool) and math.isfinite(v)
                    for v in value)):
        return (round(float(value[0]), 2), round(float(value[1]), 2))
    raise ProtocolError("pointing 像素必须是 [u, v] 或 null")


def _parse_pointing(value: Any) -> Optional[Dict[str, Any]]:
    """pointing 只在 pointing 类检查点出现；字段不认识也不报错，只保留已知字段。"""
    if value is None:
        return None
    if not isinstance(value, dict):
        raise ProtocolError("pointing 必须是对象")
    out: Dict[str, Any] = {"mode": value.get("mode") if isinstance(value.get("mode"), str) else None}
    fs = value.get("frame_size")
    if (isinstance(fs, list) and len(fs) == 2
            and all(isinstance(v, int) and not isinstance(v, bool) and v > 0 for v in fs)):
        out["frame_size"] = [fs[0], fs[1]]
    else:
        out["frame_size"] = None
    for ch in ("apos", "opos"):
        out[f"{ch}_px"] = _parse_pixel(value.get(f"{ch}_px"))
        st = value.get(f"{ch}_state")
        out[f"{ch}_state"] = st if isinstance(st, str) and st in POINTING_STATES else (
            None if st is None else str(st)[:32])
        out[f"{ch}_clamped"] = bool(value.get(f"{ch}_clamped", False))
    for ch in ("apos", "opos"):
        if out[f"{ch}_px"] is not None:
            out[f"{ch}_px"] = list(out[f"{ch}_px"])
    return out
