#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
robot_api.py v3 — 完全对齐 aidrobo_client 前端实现逻辑
Author: Delisa

基于 rosbridge v2.0 协议，每个模块的实现逻辑完全采用 aidrobo_client 前端的方案：
  - 地图管理：newMap.vue / seeMap.vue / map/index.vue
  - 导航管理：navigation.vue / goPoint.vue
  - 机器人管理：telecontrol.vue / relocation.vue / home.vue
  - 位置点管理：patrol.vue (waypoint_node)
  - 巡逻管理：patrol.vue (waypoint)
  - 回充管理：charge.vue

WS_URL 可设置环境变量 ROBOT_WS_URL 或修改 _default_ws_url
"""

import json
import os
import threading
import time
from typing import Any, Callable, Dict, Iterable, List, Optional

import websocket

# ---------- 默认 WebSocket 地址 ----------
_default_ws_url = "ws://127.0.0.1:9090"


def _get_ws_url() -> str:
    return os.environ.get("ROBOT_WS_URL", _default_ws_url)


# ---------- 工具函数 ----------

def _pretty_json(data: Any) -> str:
    try:
        return json.dumps(data, ensure_ascii=False, indent=2)
    except TypeError:
        return str(data)


def _print_json_block(title: str, payload: Any) -> None:
    print(f"\n{title}")
    print(_pretty_json(payload))


def _try_parse_json(raw: str) -> Any:
    try:
        return json.loads(raw)
    except json.JSONDecodeError:
        return raw


def _input_float(prompt: str, default: float) -> float:
    while True:
        raw = input(f"  {prompt} (default {default}): ").strip()
        if not raw:
            return default
        try:
            return float(raw)
        except ValueError:
            print("  Invalid number, retry.")


def _input_int(prompt: str, default: Optional[int] = None) -> int:
    while True:
        suffix = f" (default {default})" if default is not None else ""
        raw = input(f"  {prompt}{suffix}: ").strip()
        if not raw:
            if default is not None:
                return default
            print("  Please enter a number.")
            continue
        try:
            return int(raw)
        except ValueError:
            print("  Integer only, retry.")


def _input_str(prompt: str, default: Optional[str] = None) -> str:
    suffix = f" (default {default})" if default else ""
    raw = input(f"  {prompt}{suffix}: ").strip()
    return raw or (default or "")


# =============================================================================
#   RobotWebSocketClient — persistent WebSocket connection (unchanged core)
# =============================================================================

class RobotWebSocketClient:
    """Persistent WebSocket client for rosbridge v2.0 communication."""

    def __init__(self, ws_url: str = "ws://127.0.0.1:9090", verbose: bool = True):
        self.ws_url = ws_url
        self.verbose = verbose
        self._ws: Optional[websocket.WebSocket] = None
        self._lock = threading.Lock()
        self._subscriptions: Dict[str, threading.Thread] = {}
        self._sub_stop_events: Dict[str, threading.Event] = {}
        self._request_counter = 0

    def connect(self) -> None:
        if self._ws is not None:
            return
        self._ws = websocket.create_connection(self.ws_url)
        if self.verbose:
            print(f"Connected to {self.ws_url}")

    def disconnect(self) -> None:
        for event in self._sub_stop_events.values():
            event.set()
        for thread in self._subscriptions.values():
            thread.join(timeout=2.0)
        self._subscriptions.clear()
        self._sub_stop_events.clear()
        if self._ws is not None:
            try:
                self._ws.close()
            except Exception:
                pass
            self._ws = None
            if self.verbose:
                print("Disconnected")

    def _ensure_connected(self) -> websocket.WebSocket:
        if self._ws is None:
            self.connect()
        return self._ws  # type: ignore[return-value]

    def _next_id(self) -> str:
        self._request_counter += 1
        return f"robot_api_{self._request_counter}"

    # --- Service call ---
    def call_service(self, service: str, args: Optional[Dict[str, Any]] = None) -> Dict[str, Any]:
        ws = self._ensure_connected()
        msg = {"op": "call_service", "service": service, "args": args or {}, "id": self._next_id()}
        if self.verbose:
            _print_json_block(">> REQUEST", msg)
        with self._lock:
            ws.send(json.dumps(msg))
            resp_raw = ws.recv()
        resp = _try_parse_json(resp_raw)
        if self.verbose:
            printable = resp if isinstance(resp, dict) else {"result": resp, "raw": resp_raw}
            _print_json_block("<< RESPONSE", printable)
        return resp

    def call_service_silent(self, service: str, args: Optional[Dict[str, Any]] = None) -> Dict[str, Any]:
        ws = self._ensure_connected()
        msg = {"op": "call_service", "service": service, "args": args or {}, "id": self._next_id()}
        with self._lock:
            ws.send(json.dumps(msg))
            resp_raw = ws.recv()
        return _try_parse_json(resp_raw)

    # --- Topic publish ---
    def publish(self, topic: str, msg: Dict[str, Any], msg_type: Optional[str] = None) -> None:
        ws = self._ensure_connected()
        try:
            with self._lock:
                if msg_type:
                    ws.send(json.dumps({"op": "advertise", "topic": topic, "type": msg_type, "id": self._next_id()}))
                ws.send(json.dumps({"op": "publish", "topic": topic, "msg": msg, "id": self._next_id()}))
                if msg_type:
                    ws.send(json.dumps({"op": "unadvertise", "topic": topic, "id": self._next_id()}))
        except Exception:
            raise
        if self.verbose:
            print(f"Published to {topic}")

    # --- Topic subscribe ---
    def subscribe(self, topic: str, msg_type: str, callback: Callable[..., Any],
                  compression: Optional[str] = None) -> None:
        ws = self._ensure_connected()
        sub_msg: Dict[str, Any] = {"op": "subscribe", "topic": topic, "type": msg_type, "id": self._next_id()}
        if compression:
            sub_msg["compression"] = compression
        with self._lock:
            ws.send(json.dumps(sub_msg))
        if self.verbose:
            print(f"Subscribed to {topic}")

        stop_event = threading.Event()
        self._sub_stop_events[topic] = stop_event

        def _recv_loop() -> None:
            while not stop_event.is_set():
                try:
                    raw = ws.recv()
                except Exception:
                    if stop_event.is_set():
                        break
                    continue
                if not raw:
                    break
                try:
                    msg = json.loads(raw)
                except json.JSONDecodeError:
                    continue
                if msg.get("op") == "publish":
                    try:
                        callback(msg.get("msg", {}))
                    except Exception as exc:
                        if self.verbose:
                            print(f"Callback error ({topic}): {exc}")

        thread = threading.Thread(target=_recv_loop, daemon=True)
        self._subscriptions[topic] = thread
        thread.start()

    def unsubscribe(self, topic: str) -> None:
        ws = self._ensure_connected()
        with self._lock:
            ws.send(json.dumps({"op": "unsubscribe", "topic": topic, "id": self._next_id()}))
        event = self._sub_stop_events.pop(topic, None)
        if event:
            event.set()
        thread = self._subscriptions.pop(topic, None)
        if thread:
            thread.join(timeout=1.0)
        if self.verbose:
            print(f"Unsubscribed from {topic}")

    # --- Helpers ---
    def _safe_values(self, response: Dict[str, Any]) -> Dict[str, Any]:
        if "values" in response and isinstance(response["values"], dict):
            return response["values"]
        if "result" in response and isinstance(response["result"], dict):
            return response["result"]
        return {}

    def _call_with_fallback(self, services: Iterable[str], args: Optional[Dict[str, Any]] = None) -> Dict[str, Any]:
        last_resp: Optional[Dict[str, Any]] = None
        for service in services:
            try:
                resp = self.call_service(service, args)
            except Exception as exc:
                if self.verbose:
                    print(f"Service {service} failed: {exc}")
                continue
            if resp.get("result") is False:
                values = resp.get("values")
                if isinstance(values, str) and "does not exist" in values:
                    last_resp = resp
                    continue
            return resp
        if last_resp is not None:
            return last_resp
        raise RuntimeError(f"All fallback services failed: {list(services)}")


# =============================================================================
#   1. MapManager — aligned with newMap.vue / seeMap.vue / map/index.vue
# =============================================================================

class MapManager:
    """Maps: start/cancel mapping, save, switch, delete, update, live preview.

    Frontend reference:
      - newMap.vue: onOver() -> saveMap(/aid_save_map) -> saveMapDb(/add_map) -> push map view
      - newMap.vue: onOut() -> mode_set idle -> push map view
      - seeMap.vue: onUse() -> setCurrentMapId -> mode_set localization -> start_init_pose(0,0,0)
      - map/index.vue: start mapping flow -> mode_set mapping, cancel -> mode_set idle
    """

    def __init__(self, client: RobotWebSocketClient):
        self._client = client

    # --- Mapping control (newMap.vue) ---
    def start_mapping(self) -> Dict[str, Any]:
        """Start mapping: mode_set mapping.
        Frontend: map/index.vue -> robotMode(mapping) then push to newMap view.
        """
        return self._client.call_service("/mode_set", {"action": "mapping"})

    def cancel_mapping(self) -> Dict[str, Any]:
        """Cancel mapping without saving: mode_set idle.
        Frontend: newMap.vue onOut() -> robotMode(idle).
        """
        return self._client.call_service("/mode_set", {"action": "idle"})

    # --- Save map (newMap.vue onOver) ---
    def save_map_integrated(self, map_file_name: str) -> Dict[str, Any]:
        """Integrated save: /aid_save_map (stop -> pbstream -> occupancy grid).
        Frontend: newMap.vue -> saveMap({map_file_name: '/maps/<timestamp>'}).
        """
        return self._client.call_service("/aid_save_map", {"map_file_name": map_file_name})

    def save_map_to_db(self, map_name: str, map_file: str) -> Dict[str, Any]:
        """Save map record to database: /add_map.
        Frontend: newMap.vue -> saveMapDb({map_name, map_file: '/maps/<timestamp>'}).
        """
        return self._client._call_with_fallback(
            ("/map_management/add_map", "/add_map"),
            {"map_name": map_name, "map_file": map_file},
        )

    def save_map_full_flow(self, map_name: str, map_file_prefix: str) -> Dict[str, Any]:
        """Full save flow matching newMap.vue onOver():
        1. Integrated save (/aid_save_map)
        2. Save to DB (/add_map)
        Returns the DB save result.
        """
        timestamp = str(int(time.time() * 1000))
        prefix = map_file_prefix.rstrip("/") + "/" + timestamp
        result1 = self.save_map_integrated(prefix)
        if not self._client._safe_values(result1).get("success"):
            return result1
        return self.save_map_to_db(map_name, prefix)

    def save_map_state(self, filename: str = "/maps/map.pbstream") -> Dict[str, Any]:
        """Write pbstream state: /write_state."""
        return self._client.call_service("/write_state", {"filename": filename})

    def finish_trajectory(self, trajectory_id: int = 0) -> Dict[str, Any]:
        """Finish cartographer trajectory: /finish_trajectory."""
        return self._client.call_service("/finish_trajectory", {"trajectory_id": trajectory_id})

    # --- Map list / current map ---
    def get_map_list(self) -> Dict[str, Any]:
        """Get map list: /get_map_list.
        Frontend: map/index.vue -> getMapList().
        """
        return self._client._call_with_fallback(
            ("/get_map_list", "/map_management/get_map_list")
        )

    def get_current_map(self) -> Dict[str, Any]:
        """Get current map ID: /get_current_map_id.
        Frontend: home.vue -> getCurrentMapId().
        """
        return self._client._call_with_fallback(
            ("/get_current_map_id", "/map_management/get_current_map")
        )

    # --- Switch map (seeMap.vue onUse) ---
    def switch_map(self, map_id: int) -> Dict[str, Any]:
        """Switch current map: /set_current_map_id.
        Frontend: seeMap.vue -> setCurrentMapId({id}).
        """
        return self._client._call_with_fallback(
            ("/set_current_map_id", "/map_management/set_current_map"),
            {"id": map_id},
        )

    def switch_map_and_activate(self, map_id: int) -> Dict[str, Any]:
        """Full switch flow matching seeMap.vue onUse():
        1. setCurrentMapId({id})
        2. mode_set localization
        3. start_init_pose(0,0,0) — publish PoseStamped to /start_init_pose
        Returns switch result.
        """
        result = self.switch_map(map_id)
        if not self._client._safe_values(result).get("success"):
            return result
        # Frontend: mode_set localization
        self._client.call_service_silent("/mode_set", {"action": "localization"})
        # Frontend: publish PoseStamped(0,0,0)
        init_pose_msg = {
            "header": {"stamp": {"sec": 0, "nanosec": 0}, "frame_id": "map"},
            "pose": {
                "position": {"x": 0.0, "y": 0.0, "z": 0.0},
                "orientation": {"x": 0.0, "y": 0.0, "z": 0.0, "w": 1.0},
            },
        }
        self._client.publish("/start_init_pose", init_pose_msg, "geometry_msgs/msg/PoseStamped")
        return result

    def switch_map_with_localization(self, map_id: int) -> None:
        """Just call switch + set localization mode (simpler variant).
        Frontend: seeMap.vue onUse() sequence.
        """
        self.switch_map(map_id)
        self._client.call_service("/mode_set", {"action": "localization"})
        # Also publish init pose as frontend does
        pose = {
            "header": {"stamp": {"sec": 0, "nanosec": 0}, "frame_id": "map"},
            "pose": {
                "position": {"x": 0.0, "y": 0.0, "z": 0.0},
                "orientation": {"x": 0.0, "y": 0.0, "z": 0.0, "w": 1.0},
            },
        }
        self._client.publish("/start_init_pose", pose, "geometry_msgs/msg/PoseStamped")

    # --- Map CRUD ---
    def add_map_record(self, map_name: str, map_file: str = "/maps/map_latest") -> Dict[str, Any]:
        """Alias for save_map_to_db."""
        return self.save_map_to_db(map_name, map_file)

    def delete_map(self, map_id: int) -> Dict[str, Any]:
        """Delete map: /delete_map with data_type=map.
        Frontend: seeMap.vue onDel() -> deleteMap({id, data_type: "map"}).
        """
        return self._client._call_with_fallback(
            ("/delete_map", "/map_management/delete_map"),
            {"id": map_id, "data_type": "map"},
        )

    def update_map(self, map_id: int, data: Dict[str, Any]) -> Dict[str, Any]:
        """Update map record: /update_map."""
        return self._client._call_with_fallback(
            ("/update_map", "/map_management/update_map"),
            {"id": map_id, "data": json.dumps(data, ensure_ascii=False), "data_type": "map"},
        )

    def get_map_image(self, map_id: int) -> Dict[str, Any]:
        """Get map image: /get_map_image."""
        return self._client.call_service("/get_map_image", {"id": map_id})

    # --- Live mapping preview (newMap.vue / map_base64 subscription) ---
    def subscribe_map_updates(self, callback: Callable[..., Any]) -> None:
        """Subscribe to /map_base64 (PNG compressed OccupancyGrid).
        Frontend: newMap.vue subscribes /map_base64 for live preview.
        """
        self._client.subscribe("/map_base64", "nav_msgs/msg/OccupancyGrid", callback, compression="png")

    def unsubscribe_map_updates(self) -> None:
        self._client.unsubscribe("/map_base64")

    # --- Forbidden lines / eraser (editMap.vue) ---
    def set_forbidden_lines(self, map_id: int, frame_id: str, lines: List[Dict[str, Any]]) -> Dict[str, Any]:
        """Set forbidden lines: /set_forbidden.
        Frontend: editMap.vue saves forbidden lines via /set_forbidden.
        """
        return self._client._call_with_fallback(
            ("/set_forbidden",), {"map_id": map_id, "frame_id": frame_id, "lines": lines}
        )

    def erase_forbidden_area(self, map_id: int, rectangles: List[Dict[str, Any]]) -> Dict[str, Any]:
        """Eraser: /map_editor with type=point.
        Frontend: editMap.vue -> /map_editor with rectangle_array.
        """
        return self._client._call_with_fallback(
            ("/map_editor",),
            {"frame_id": "map", "type": "point", "map_id": map_id, "data": [], "rectangle_array": rectangles},
        )

    def ensure_current_map_selected(self) -> Optional[int]:
        """Ensure current_map is set; if not, guide user selection.
        Used by map/index.vue before entering mapping mode.
        """
        resp = self._client._call_with_fallback(
            ("/get_current_map_id", "/map_management/get_current_map")
        )
        values = self._client._safe_values(resp)
        if values.get("success") and values.get("map_id", 0):
            return values["map_id"]

        print("No current map set. Need to specify map ID.")
        list_resp = self._client._call_with_fallback(
            ("/get_map_list", "/map_management/get_map_list")
        )
        list_values = self._client._safe_values(list_resp)
        map_list_raw = list_values.get("map_list", "[]")
        try:
            map_list = json.loads(map_list_raw)
        except json.JSONDecodeError:
            print("Failed to parse map list.")
            return None
        if not map_list:
            print("No maps in database.")
            return None

        print("\nAvailable maps:")
        for item in map_list:
            print(f"  id={item.get('id')} name={item.get('name')}")
        while True:
            raw_id = input("\nEnter map ID to set as current (or Enter to skip): ").strip()
            if not raw_id:
                return None
            if raw_id.isdigit():
                map_id = int(raw_id)
                set_resp = self.switch_map(map_id)
                if self._client._safe_values(set_resp).get("success"):
                    print(f"Current map set to id={map_id}")
                    return map_id
                print(f"Failed to set current map: {set_resp}")
            else:
                print("Please enter a valid number.")


# =============================================================================
#   2. NavigationManager — aligned with navigation.vue / goPoint.vue
# =============================================================================

class NavigationManager:
    """Navigation: single-point nav, pause/resume/cancel, task/position subscriptions.

    Frontend reference:
      - navigation.vue: onStart() -> if actionNeedCancel -> cancel first, then nav_to_pose publish
      - navigation.vue: onClose() -> patrol_control cancel
      - goPoint.vue: onBegin() -> patrol_path publish (Path message)
      - headArea.vue: subscribes /task_status to update actionStatus in Vuex store
      - home.vue: subscribes /base_link_pose for robot position
    """

    # Task status constants matching frontend store mutation changeRobotTaskStatus
    STATUS_MAP = {0: "idle", 1: "working", 2: "success", 3: "failed", 4: "suspend", 5: "cancel"}
    TASK_TYPE_MAP = {0: "single_nav", 1: "patrol"}

    def __init__(self, client: RobotWebSocketClient):
        self._client = client
        # Track current action status like frontend Vuex actionStatus
        self._action_status: str = ""
        # Track task status from /task_status subscription
        self._task_status: Optional[int] = None
        self._task_type: Optional[int] = None

    @property
    def action_status(self) -> str:
        return self._action_status

    @property
    def is_action_active(self) -> bool:
        """Check if a navigation or patrol is currently active.
        Frontend: store getter actionNeedCancel.
        """
        return self._action_status in ("navigateStart", "navigatePause", "patrolStart", "patrolPause")

    def _update_action_from_task(self, task_type: int, status: int) -> None:
        """Update action status from /task_status, matching frontend changeRobotTaskStatus.
        Frontend: {task_type}-{status} -> actionStatus mapping.
        """
        mapping = {
            (0, 1): "navigateStart",
            (0, 4): "navigatePause",
            (1, 1): "patrolStart",
            (1, 4): "patrolPause",
        }
        self._task_status = status
        self._task_type = task_type
        self._action_status = mapping.get((task_type, status), "")

    # --- Single-point navigation (navigation.vue onStartNavigation) ---
    def nav_to_pose(self, x: float, y: float, z: float = 0.0,
                    orientation_z: float = 0.0, orientation_w: float = 1.0,
                    frame_id: str = "map") -> None:
        """Publish single-point nav goal: /nav_to_pose (PoseStamped).
        Frontend: navigation.vue -> StartNavigation.publish(PoseStamped).
        NOTE: Does NOT auto-cancel. Use nav_to_pose_safe() for that.
        """
        pose = {
            "header": {"stamp": {"sec": 0, "nanosec": 0}, "frame_id": frame_id},
            "pose": {
                "position": {"x": x, "y": y, "z": z},
                "orientation": {"x": 0.0, "y": 0.0, "z": orientation_z, "w": orientation_w},
            },
        }
        self._client.publish("/nav_to_pose", pose, "geometry_msgs/msg/PoseStamped")

    def nav_to_pose_safe(self, x: float, y: float, z: float = 0.0,
                         orientation_z: float = 0.0, orientation_w: float = 1.0,
                         frame_id: str = "map") -> None:
        """Safe navigation: cancel existing task first, then navigate.
        Frontend: navigation.vue onStart() -> if actionNeedCancel -> cancel, then nav.
        """
        if self.is_action_active:
            print("Active task detected, canceling first...")
            self.cancel()
            time.sleep(0.3)
        self.nav_to_pose(x, y, z, orientation_z, orientation_w, frame_id)

    # --- Navigation control (patrol_control service) ---
    def pause(self) -> Dict[str, Any]:
        """Pause navigation/patrol: /patrol_control cmd=pause.
        Frontend: navigation.vue onClose() or universal pause button.
        """
        return self._client._call_with_fallback(("/patrol_control",), {"cmd": "pause"})

    def resume(self) -> Dict[str, Any]:
        """Resume navigation/patrol: /patrol_control cmd=resume."""
        return self._client._call_with_fallback(("/patrol_control",), {"cmd": "resume"})

    def cancel(self) -> Dict[str, Any]:
        """Cancel navigation/patrol: /patrol_control cmd=cancel.
        Frontend: navigation.vue onClose(), goPoint.vue cancel button.
        """
        return self._client._call_with_fallback(("/patrol_control",), {"cmd": "cancel"})

    # --- Task status subscription (headArea.vue) ---
    def subscribe_task_status(self, callback: Callable[..., Any]) -> None:
        """Subscribe /task_status (AidTaskStatus). Also updates internal action_status.
        Frontend: headArea.vue subscribes and dispatches to Vuex changeRobotTaskStatus.

        callback receives msg with: status(0..5), task_type(0=nav,1=patrol)
        """
        def _wrapper(msg: Dict[str, Any]) -> None:
            self._update_action_from_task(msg.get("task_type", 0), msg.get("status", 0))
            callback(msg)

        self._client.subscribe("/task_status", "aid_robot_msgs/msg/AidTaskStatus", _wrapper)

    def unsubscribe_task_status(self) -> None:
        self._client.unsubscribe("/task_status")

    # --- Robot position subscription (home.vue) ---
    def subscribe_robot_position(self, callback: Callable[..., Any]) -> None:
        """Subscribe /base_link_pose (PoseStamped).
        Frontend: home.vue -> robotPosition.subscribe().
        """
        self._client.subscribe("/base_link_pose", "geometry_msgs/msg/PoseStamped", callback)

    def unsubscribe_robot_position(self) -> None:
        self._client.unsubscribe("/base_link_pose")


# =============================================================================
#   3. RobotManager — aligned with telecontrol.vue / relocation.vue / home.vue
# =============================================================================

class RobotManager:
    """Robot: mode switch, init pose, remote control, battery/status subscriptions, IP.

    Frontend reference:
      - telecontrol.vue: mode_set remote_control / idle
      - relocation.vue: /initialpose with PoseWithCovarianceStamped + covariance matrix
      - home.vue: BatteryState.subscribe, GetStrings(/get_ip_addresses)
      - headArea.vue: /robot_status subscription
    """

    MODES = ("mapping", "localization", "patrol", "remote_control", "idle")

    def __init__(self, client: RobotWebSocketClient):
        self._client = client

    # --- Mode switch (telecontrol.vue btnFun) ---
    def set_mode(self, mode: str) -> Dict[str, Any]:
        """Set robot mode: /mode_set.
        Frontend: telecontrol.vue -> robotMode({action: mode}).
        """
        if mode not in self.MODES:
            raise ValueError(f"Invalid mode '{mode}', options: {self.MODES}")
        return self._client.call_service("/mode_set", {"action": mode})

    def enter_remote_control(self) -> Dict[str, Any]:
        """Enter remote control mode.
        Frontend: telecontrol.vue -> robotMode(remote_control) + actionStatus='remote'.
        """
        return self.set_mode("remote_control")

    def enter_localization(self) -> Dict[str, Any]:
        """Enter localization mode.
        Frontend: relocation.vue -> robotMode(localization).
        """
        return self.set_mode("localization")

    def enter_idle(self) -> Dict[str, Any]:
        """Enter idle mode.
        Frontend: telecontrol.vue close -> robotMode(idle).
        """
        return self.set_mode("idle")

    # --- Init pose / relocation (relocation.vue onRelocationSelected) ---
    def init_pose(self, x: float = 0.0, y: float = 0.0, z: float = 0.0,
                  orientation_x: float = 0.0, orientation_y: float = 0.0,
                  orientation_z: float = 0.0, orientation_w: float = 1.0) -> None:
        """Initialize robot pose with covariance: /initialpose (PoseWithCovarianceStamped).
        Frontend: relocation.vue -> InitialPose.publish(PoseWithCovarianceStamped).
        Includes the exact covariance matrix from relocation.vue.
        Also publishes to /aid_init_pose + /start_init_pose for compatibility (H1 doc).
        """
        # Primary: /initialpose with covariance (exactly matching relocation.vue)
        cov_msg = {
            "header": {"stamp": {"sec": 0, "nanosec": 0}, "frame_id": "map"},
            "pose": {
                "pose": {
                    "position": {"x": x, "y": y, "z": z},
                    "orientation": {"x": orientation_x, "y": orientation_y, "z": orientation_z, "w": orientation_w},
                },
                "covariance": [
                    0.25, 0, 0, 0, 0, 0,
                    0, 0.25, 0, 0, 0, 0,
                    0, 0, 0, 0, 0, 0,
                    0, 0, 0, 0, 0, 0,
                    0, 0, 0, 0, 0, 0,
                    0, 0, 0, 0, 0, 0.06853891945200942,
                ],
            },
        }
        self._client.publish("/initialpose", cov_msg, "geometry_msgs/msg/PoseWithCovarianceStamped")

        # Compatibility: also publish to /aid_init_pose + /start_init_pose (H1 doc 1.1)
        pose_msg = {
            "header": {"stamp": {"sec": 0, "nanosec": 0}, "frame_id": "map"},
            "pose": {
                "position": {"x": x, "y": y, "z": z},
                "orientation": {"x": orientation_x, "y": orientation_y, "z": orientation_z, "w": orientation_w},
            },
        }
        self._client.publish("/aid_init_pose", pose_msg, "geometry_msgs/msg/PoseStamped")
        self._client.publish("/start_init_pose", pose_msg, "geometry_msgs/msg/PoseStamped")

    def init_pose_simple(self, x: float = 0.0, y: float = 0.0, z: float = 0.0) -> None:
        """Simple init pose at origin (used after map switch).
        Frontend: seeMap.vue onUse() publishes PoseStamped(0,0,0) to /start_init_pose.
        """
        self.init_pose(x, y, z, 0, 0, 0, 1)

    # --- Remote control (telecontrol.vue) ---
    def remote_control(self, linear_x: float = 0.0, angular_z: float = 0.0,
                       auto_stop: bool = False, auto_stop_delay: float = 3.0) -> None:
        """Remote velocity control: /cmd_vel_remote_ctrl (Twist).
        Frontend: telecontrol.vue publishes linear.x / angular.z,
        auto-stops after 3 seconds of no input.
        If auto_stop=True, a background thread will publish zero velocity after delay.
        """
        msg = {
            "linear": {"x": linear_x, "y": 0.0, "z": 0.0},
            "angular": {"x": 0.0, "y": 0.0, "z": angular_z},
        }
        self._client.publish("/cmd_vel_remote_ctrl", msg, "geometry_msgs/msg/Twist")

        if auto_stop and (linear_x != 0.0 or angular_z != 0.0):
            def _auto_stop() -> None:
                time.sleep(auto_stop_delay)
                stop_msg = {
                    "linear": {"x": 0.0, "y": 0.0, "z": 0.0},
                    "angular": {"x": 0.0, "y": 0.0, "z": 0.0},
                }
                self._client.publish("/cmd_vel_remote_ctrl", stop_msg, "geometry_msgs/msg/Twist")
                print("Auto-stop: velocity set to zero")
            threading.Thread(target=_auto_stop, daemon=True).start()

    def stop_remote_control(self) -> None:
        """Stop remote control: publish zero velocity."""
        self.remote_control(0.0, 0.0)

    # --- Head control ---
    def control_head(self, position_rad: float) -> None:
        """Control head motor (2.94~3.29 rad): /target_head_position."""
        self._client.publish("/target_head_position", {"data": position_rad}, "std_msgs/msg/Float32")

    # --- Get IP (home.vue) ---
    def get_ip(self) -> Dict[str, Any]:
        """Get robot IP: /get_ip_addresses.
        Frontend: home.vue -> GetStrings.callService().
        """
        return self._client.call_service("/get_ip_addresses")

    # --- Battery subscription (home.vue) ---
    def subscribe_battery(self, callback: Callable[..., Any]) -> None:
        """Subscribe /battery_state (BatteryState).
        Frontend: home.vue -> BatteryState.subscribe().
        """
        self._client.subscribe("/battery_state", "sensor_msgs/msg/BatteryState", callback)

    def unsubscribe_battery(self) -> None:
        self._client.unsubscribe("/battery_state")

    # --- Robot status subscription (headArea.vue) ---
    def subscribe_status(self, callback: Callable[..., Any]) -> None:
        """Subscribe /robot_status (String).
        Frontend: headArea.vue subscribes /robot_status.
        msg.data format: "<slam_status>+<control_model>".
        """
        self._client.subscribe("/robot_status", "std_msgs/msg/String", callback)

    def unsubscribe_status(self) -> None:
        self._client.unsubscribe("/robot_status")


# =============================================================================
#   4. WaypointManager — aligned with patrol.vue (waypoint_node CRUD)
# =============================================================================

class WaypointManager:
    """Waypoint nodes: add/delete/update/get points on map (data_type="waypoint_node").

    Frontend reference:
      - Waypoints on map use data_type="waypoint_node".
      - data field is JSON with: {position, orientation, name}.
      - Services: /add_point, /delete_point, /update_point, /get_point, /get_map_point_list.
    """

    def __init__(self, client: RobotWebSocketClient):
        self._client = client

    @staticmethod
    def _make_point_data(position: Dict[str, float], orientation: Dict[str, float],
                         name: str = "") -> Dict[str, Any]:
        return {"position": position, "orientation": orientation, "name": name}

    def add_point(self, map_id: int, position: Dict[str, float],
                  orientation: Dict[str, float], name: str = "",
                  frame_id: str = "map") -> Dict[str, Any]:
        """Add waypoint node: /add_point (data_type=waypoint_node).
        Frontend: data is JSON-stringified {position, orientation, name}.
        """
        data = self._make_point_data(position, orientation, name)
        return self._client._call_with_fallback(
            ("/add_point",),
            {
                "map_id": map_id,
                "frame_id": frame_id,
                "data": json.dumps(data, ensure_ascii=False),
                "data_type": "waypoint_node",
            },
        )

    def delete_point(self, point_id: int) -> Dict[str, Any]:
        """Delete waypoint node: /delete_point."""
        return self._client._call_with_fallback(
            ("/delete_point",), {"id": point_id, "data_type": "waypoint_node"}
        )

    def update_point(self, point_id: int, position: Dict[str, float],
                     orientation: Dict[str, float], name: str = "") -> Dict[str, Any]:
        """Update waypoint node: /update_point."""
        data = self._make_point_data(position, orientation, name)
        return self._client._call_with_fallback(
            ("/update_point",),
            {"id": point_id, "data": json.dumps(data, ensure_ascii=False), "data_type": "waypoint_node"},
        )

    def get_point(self, point_id: int) -> Dict[str, Any]:
        """Get single waypoint node: /get_point."""
        return self._client._call_with_fallback(
            ("/get_point",), {"id": point_id, "data_type": "waypoint_node"}
        )

    def get_map_point_list(self, map_id: int) -> Dict[str, Any]:
        """Get map's waypoint node list: /get_map_point_list."""
        return self._client._call_with_fallback(
            ("/get_map_point_list",), {"map_id": map_id, "data_type": "waypoint_node"}
        )


# =============================================================================
#   5. PatrolManager — aligned with patrol.vue (waypoint CRUD + patrol control)
# =============================================================================

class PatrolManager:
    """Patrol routes: add/delete/update/get waypoints, start/pause/resume/cancel patrol.

    Frontend reference (patrol.vue):
      - addPoint: data = JSON.stringify([{x, y, z}, ...])  (plain coord array!)
      - updatePoint: data = JSON.stringify({map_id, frame_id, point_list: JSON.stringify([{x,y,z},...])})
      - getPoint: get_map_waypoint_list -> parse result.message -> [0].point_list
      - onBegin: if actionNeedCancel -> cancel first, then startPatrol
      - startPatrol: builds nav_msgs/Path with poses from patrol_arr (x,y,z plain coords)
      - onStop: patrol_control cancel
    """

    def __init__(self, client: RobotWebSocketClient):
        self._client = client

    # --- Patrol route CRUD (patrol.vue addPoint/updatePoint/getPoint) ---

    def add_waypoint(self, map_id: int, points: List[Dict[str, Any]],
                     frame_id: str = "map") -> Dict[str, Any]:
        """Add patrol route: /add_waypoint (data_type=waypoint).
        Frontend: data = JSON.stringify([{x, y, z}, ...]) — plain coordinate array.
        """
        # Convert to plain {x, y, z} array matching frontend patrol.vue addPoint()
        plain_points = []
        for p in points:
            pos = p.get("position", p)
            plain_points.append({
                "x": pos.get("x", p.get("x", 0)),
                "y": pos.get("y", p.get("y", 0)),
                "z": pos.get("z", p.get("z", 0)),
            })
        return self._client._call_with_fallback(
            ("/add_waypoint",),
            {
                "map_id": map_id,
                "frame_id": frame_id,
                "data": json.dumps(plain_points, ensure_ascii=False),
                "data_type": "waypoint",
            },
        )

    def delete_waypoint(self, waypoint_id: int) -> Dict[str, Any]:
        """Delete patrol route: /delete_waypoint.
        Frontend: patrol.vue onDelList() -> OperationDelete({id, data_type: "waypoint"}).
        """
        return self._client._call_with_fallback(
            ("/delete_waypoint",), {"id": waypoint_id, "data_type": "waypoint"}
        )

    def update_waypoint(self, waypoint_id: int, map_id: int, frame_id: str,
                        points: List[Dict[str, Any]]) -> Dict[str, Any]:
        """Update patrol route: /update_waypoint.
        Frontend: data = JSON.stringify({map_id, frame_id, point_list: JSON.stringify([{x,y,z},...])}).
        """
        plain_points = []
        for p in points:
            pos = p.get("position", p)
            plain_points.append({
                "x": pos.get("x", p.get("x", 0)),
                "y": pos.get("y", p.get("y", 0)),
                "z": pos.get("z", p.get("z", 0)),
            })
        data = {
            "map_id": map_id,
            "frame_id": frame_id,
            "point_list": json.dumps(plain_points, ensure_ascii=False),
        }
        return self._client._call_with_fallback(
            ("/update_waypoint",),
            {"id": waypoint_id, "data": json.dumps(data, ensure_ascii=False), "data_type": "waypoint"},
        )

    def get_waypoint(self, waypoint_id: int) -> Dict[str, Any]:
        """Get single patrol route: /get_waypoint."""
        return self._client._call_with_fallback(
            ("/get_waypoint",), {"id": waypoint_id, "data_type": "waypoint"}
        )

    def get_map_waypoint_list(self, map_id: int) -> List[Dict[str, Any]]:
        """Get map's patrol routes: /get_map_waypoint_list.
        Frontend: patrol.vue getPoint() -> parses result.message as JSON, returns [0].point_list.
        Returns parsed list of route objects.
        """
        resp = self._client._call_with_fallback(
            ("/get_map_waypoint_list",), {"map_id": map_id, "data_type": "waypoint"}
        )
        values = self._client._safe_values(resp)
        raw = values.get("message", "[]")
        try:
            return json.loads(raw)
        except (json.JSONDecodeError, TypeError):
            return []

    def get_first_route_points(self, map_id: int) -> Optional[List[Dict[str, Any]]]:
        """Get the first patrol route's point_list for a map.
        Frontend: patrol.vue getPoint() -> msg[0].point_list parsed as JSON.
        """
        routes = self.get_map_waypoint_list(map_id)
        if routes and len(routes) > 0:
            raw = routes[0].get("point_list", "[]")
            try:
                return json.loads(raw)
            except (json.JSONDecodeError, TypeError):
                pass
        return None

    # --- Patrol control (patrol.vue onBegin/startPatrol/onStop) ---

    def _cancel_if_needed(self) -> None:
        """Cancel active task before starting new patrol.
        Frontend: patrol.vue onBegin() -> if actionNeedCancel -> cancel first.
        """
        # Check task status subscription if available
        # For simplicity, always attempt a cancel before starting
        try:
            self._client.call_service_silent("/patrol_control", {"cmd": "cancel"})
        except Exception:
            pass

    def start_patrol(self, points: List[Dict[str, Any]], frame_id: str = "map",
                     auto_cancel: bool = True) -> None:
        """Start multi-point patrol: publish /patrol_path (nav_msgs/Path).
        Frontend: patrol.vue startPatrol() -> builds Path from patrol_arr, publishes to TalkerPoint.

        If auto_cancel=True (default), cancels any active task first (matching frontend behavior).
        """
        if not points:
            print("No patrol points, cannot start.")
            return

        if auto_cancel:
            self._cancel_if_needed()
            time.sleep(0.2)

        poses = []
        for p in points:
            x = p.get("x", p.get("position", {}).get("x", 0))
            y = p.get("y", p.get("position", {}).get("y", 0))
            z = p.get("z", p.get("position", {}).get("z", 0))
            poses.append({
                "header": {"stamp": {"sec": 0, "nanosec": 0}, "frame_id": frame_id},
                "pose": {
                    "position": {"x": x, "y": y, "z": z},
                    "orientation": {"x": 0.0, "y": 0.0, "z": 0.0, "w": 1.0},
                },
            })
        self._client.publish("/patrol_path", {"poses": poses}, "nav_msgs/msg/Path")

    def pause(self) -> Dict[str, Any]:
        """Pause patrol: /patrol_control cmd=pause.
        Frontend: patrol.vue pause control.
        """
        return self._client._call_with_fallback(("/patrol_control",), {"cmd": "pause"})

    def resume(self) -> Dict[str, Any]:
        """Resume patrol: /patrol_control cmd=resume."""
        return self._client._call_with_fallback(("/patrol_control",), {"cmd": "resume"})

    def cancel(self) -> Dict[str, Any]:
        """Cancel patrol: /patrol_control cmd=cancel.
        Frontend: patrol.vue onStop().
        """
        return self._client._call_with_fallback(("/patrol_control",), {"cmd": "cancel"})


# =============================================================================
#   6. DockManager — aligned with charge.vue (dock state machine)
# =============================================================================

class DockManager:
    """Dock: set/get dock pose, dock/undock/cancel, dock state subscription.

    Frontend reference (charge.vue):
      - setDock(action): calls dockService(/cmd_dock) with data=<action>.
        If success && action != "undock": waitResult(action).
      - waitResult: subscribes /dock_result for final status (dock_succeeded, etc).
        After set_dock_pose result: calls getDockPose() to refresh.
      - loadingDock state machine: "" | "dock" | "set_dock_pose" | ... | "dock-result" | ...
      - isSetDockPose: tracked from getDockPose response.
      - disable logic: disable dock if charging/goto_dock_pose; disable dock if !isSetDockPose.
      - dockState topic: /dock_state (undock / goto_dock_pose / charging / error).
    """

    DOCK_RESULT_LABELS = {
        "dock_succeeded": "Dock succeeded",
        "dock_failed": "Dock failed",
        "detect_dock_failed": "Dock pose not detected",
        "set_dock_pose_succeeded": "Dock pose set successfully",
        "set_dock_pose_failed": "Failed to set dock pose",
    }

    DOCK_STATE_LABELS = {
        "undock": "Not charging",
        "goto_dock_pose": "Going to dock",
        "charging": "Charging",
        "error": "Error",
    }

    def __init__(self, client: RobotWebSocketClient):
        self._client = client
        # State tracking (matching charge.vue data)
        self._loading_dock: str = ""  # "" | "dock" | "set_dock_pose" | "undock" | "cancel_dock" | "<action>-result"
        self._is_dock_pose_set: bool = False
        self._dock_status: str = ""
        self._dock_result_callback: Optional[Callable[..., Any]] = None

    @property
    def is_dock_pose_set(self) -> bool:
        return self._is_dock_pose_set

    @property
    def dock_status(self) -> str:
        return self._dock_status

    @property
    def is_busy(self) -> bool:
        return bool(self._loading_dock)

    @property
    def is_dock_disabled(self) -> bool:
        """Frontend: disable dock button if charging/goto_dock_pose or !isSetDockPose."""
        if not self._is_dock_pose_set:
            return True
        if self._dock_status in ("goto_dock_pose", "charging"):
            return True
        return False

    @property
    def is_set_dock_pose_disabled(self) -> bool:
        """Frontend: disable set_dock_pose if charging/goto_dock_pose."""
        return self._dock_status in ("goto_dock_pose", "charging")

    # --- Dock pose get/set ---
    def set_dock_pose(self, map_id: int, position: Dict[str, float],
                      orientation: Optional[Dict[str, float]] = None,
                      wait_result: bool = True) -> Dict[str, Any]:
        """Set dock pose via /set_dock_pose service (not /cmd_dock).
        Frontend: uses /get_dock_pose and /set_dock_pose via cmd_dock.
        This method uses the direct /set_dock_pose service.
        """
        if orientation is None:
            orientation = {"x": 0.0, "y": 0.0, "z": 0.0, "w": 1.0}
        return self._client.call_service(
            "/set_dock_pose",
            {"map_id": map_id, "point": {"position": position, "orientation": orientation}},
        )

    def get_dock_pose(self, map_id: int) -> Dict[str, Any]:
        """Get dock pose: /get_dock_pose.
        Frontend: charge.vue getDockPose() -> sets isSetDockPose = result.success.
        """
        resp = self._client.call_service("/get_dock_pose", {"map_id": map_id})
        self._is_dock_pose_set = self._client._safe_values(resp).get("success", False)
        return resp

    def refresh_dock_pose_status(self, map_id: int = 1) -> None:
        """Refresh is_dock_pose_set flag."""
        self.get_dock_pose(map_id)

    # --- /cmd_dock commands (charge.vue setDock) ---
    def _cmd_dock(self, action: str) -> Dict[str, Any]:
        """Internal: call /cmd_dock with given action.
        Frontend: charge.vue setDock() -> dockService({data: action}).
        """
        if self._loading_dock:
            print(f"Dock busy: {self._loading_dock}")
            return {"values": {"success": False, "message": "Another dock action is in progress"}}
        self._loading_dock = action
        try:
            resp = self._client.call_service("/cmd_dock", {"data": action})
            success = self._client._safe_values(resp).get("success", False)
            if not success:
                self._loading_dock = ""
            return resp
        except Exception:
            self._loading_dock = ""
            raise

    def dock(self, wait_result: bool = True) -> Dict[str, Any]:
        """Start docking: /cmd_dock data=dock.
        Frontend: charge.vue -> setDock("dock") -> waitResult("dock").
        If wait_result=True, subscribes /dock_result for final status.
        """
        resp = self._cmd_dock("dock")
        if self._client._safe_values(resp).get("success") and wait_result:
            self._wait_dock_result("dock")
        return resp

    def undock(self) -> Dict[str, Any]:
        """Leave dock: /cmd_dock data=undock.
        Frontend: charge.vue -> setDock("undock") -> no waitResult.
        """
        resp = self._cmd_dock("undock")
        self._loading_dock = ""
        return resp

    def cancel_dock(self) -> Dict[str, Any]:
        """Cancel dock: /cmd_dock data=cancel_dock.
        Frontend: charge.vue cancelDock().
        """
        resp = self._cmd_dock("cancel_dock")
        self._loading_dock = ""
        return resp

    def cmd_set_dock_pose(self, wait_result: bool = True) -> Dict[str, Any]:
        """Set dock pose via /cmd_dock: data=set_dock_pose.
        Frontend: charge.vue -> setDock("set_dock_pose") -> waitResult("set_dock_pose").
        """
        resp = self._cmd_dock("set_dock_pose")
        if self._client._safe_values(resp).get("success") and wait_result:
            self._wait_dock_result("set_dock_pose")
        return resp

    # --- Dock result subscription (charge.vue waitResult) ---
    def _wait_dock_result(self, action: str, timeout: float = 30.0) -> None:
        """Wait for /dock_result after a dock/set_dock_pose command.
        Frontend: charge.vue waitResult(action) -> subscribes dockResultTopic,
        shows message, then unsubscribes.
        """
        self._loading_dock = f"{action}-result"
        result_received = threading.Event()
        final_result: Dict[str, Any] = {}

        def _on_result(msg: Dict[str, Any]) -> None:
            data = msg.get("data", "")
            label = self.DOCK_RESULT_LABELS.get(data, data)
            print(f"Dock result ({action}): {label}")
            final_result["data"] = data
            result_received.set()

        self._client.subscribe("/dock_result", "std_msgs/msg/String", _on_result)

        # Wait with timeout
        if not result_received.wait(timeout):
            print(f"Dock result timeout ({timeout}s) for {action}")
        self._client.unsubscribe("/dock_result")
        self._loading_dock = ""

        # After set_dock_pose result, refresh status (matching frontend)
        if action == "set_dock_pose":
            self._is_dock_pose_set = True

    # --- Dock state subscription (charge.vue initSubscribe) ---
    def subscribe_dock_state(self, callback: Callable[..., Any]) -> None:
        """Subscribe /dock_state (String).
        Frontend: charge.vue -> dockStateTopic.subscribe().
        Also updates internal _dock_status.
        """
        def _wrapper(msg: Dict[str, Any]) -> None:
            self._dock_status = msg.get("data", "")
            callback(msg)

        self._client.subscribe("/dock_state", "std_msgs/msg/String", _wrapper)

    def unsubscribe_dock_state(self) -> None:
        self._client.unsubscribe("/dock_state")



# =============================================================================
#                           Interactive Menu System
# =============================================================================

def _make_menu_client() -> RobotWebSocketClient:
    return RobotWebSocketClient(_get_ws_url(), verbose=True)


def _print_section(title: str, items: list) -> None:
    print(f"\n-- {title} --")
    for key, desc in items:
        print(f"  {key}  {desc}")


# ==================== 1. Map Management Menu ====================

def menu_map_manager(client: RobotWebSocketClient) -> None:
    mgr = MapManager(client)
    items = [
        ("1.1", "Start mapping (mode_set mapping)"),
        ("1.2", "Cancel mapping (mode_set idle)"),
        ("1.3", "Integrated save map (/aid_save_map)"),
        ("1.4", "Save map state (/write_state)"),
        ("1.5", "Finish trajectory (/finish_trajectory)"),
        ("1.6", "Save map to DB (/add_map)"),
        ("1.7", "Full save flow (integrated + DB)"),
        ("1.8", "Get map list (/get_map_list)"),
        ("1.9", "Get current map (/get_current_map_id)"),
        ("1.10", "Switch map only (/set_current_map_id)"),
        ("1.11", "Switch map + activate (localization + init pose)"),
        ("1.12", "Delete map (/delete_map)"),
        ("1.13", "Update map (/update_map)"),
        ("1.14", "Subscribe /map_base64 (live mapping)"),
        ("1.15", "Unsubscribe /map_base64"),
    ]

    def _handle(choice: str) -> bool:
        if choice == "1.1":
            mgr.start_mapping()
        elif choice == "1.2":
            mgr.cancel_mapping()
        elif choice == "1.3":
            name = _input_str("Map file prefix", "/maps/map_latest")
            mgr.save_map_integrated(name)
        elif choice == "1.4":
            filename = _input_str("pbstream path", "/maps/map.pbstream")
            mgr.save_map_state(filename)
        elif choice == "1.5":
            tid = _input_int("Trajectory ID", 0)
            mgr.finish_trajectory(tid)
        elif choice == "1.6":
            map_name = _input_str("Map name", "office")
            map_file = _input_str("Map file path", "/maps/map_latest")
            mgr.save_map_to_db(map_name, map_file)
        elif choice == "1.7":
            map_name = _input_str("Map name", "office")
            prefix = _input_str("Map file prefix", "/maps/map_latest")
            mgr.save_map_full_flow(map_name, prefix)
        elif choice == "1.8":
            mgr.get_map_list()
        elif choice == "1.9":
            mgr.get_current_map()
        elif choice == "1.10":
            mid = _input_int("Map ID to switch to")
            mgr.switch_map(mid)
        elif choice == "1.11":
            mid = _input_int("Map ID to switch to")
            mgr.switch_map_and_activate(mid)
        elif choice == "1.12":
            mid = _input_int("Map ID to delete")
            mgr.delete_map(mid)
        elif choice == "1.13":
            mid = _input_int("Map ID to update")
            raw = _input_str("Update data (JSON)", '{"name":"newmap"}')
            mgr.update_map(mid, json.loads(raw))
        elif choice == "1.14":
            def _on_map(msg):
                info = msg.get("info", {})
                dlen = len(msg.get("data", ""))
                print(f"  Map frame: {info.get('width')}x{info.get('height')}, data={dlen} bytes")
            mgr.subscribe_map_updates(_on_map)
            input("\nPress Enter to stop...")
            mgr.unsubscribe_map_updates()
        elif choice == "1.15":
            mgr.unsubscribe_map_updates()
        else:
            return False
        return True

    while True:
        print("\n=== MapManager (newMap.vue / seeMap.vue) ===")
        _print_section("Map Management", items)
        print("  0  Back")
        choice = input("\nSelect: ").strip()
        if choice == "0":
            break
        if not _handle(choice):
            print("  Invalid choice")


# ==================== 2. Navigation Menu ====================

def menu_navigation_manager(client: RobotWebSocketClient) -> None:
    nav = NavigationManager(client)
    items = [
        ("2.1", "Nav to pose (single point)"),
        ("2.2", "Nav to pose SAFE (cancel first)"),
        ("2.3", "Pause navigation"),
        ("2.4", "Resume navigation"),
        ("2.5", "Cancel navigation"),
        ("2.6", "Subscribe /task_status"),
        ("2.7", "Unsubscribe /task_status"),
        ("2.8", "Subscribe /base_link_pose"),
        ("2.9", "Unsubscribe /base_link_pose"),
    ]

    def _handle(choice: str) -> bool:
        if choice == "2.1":
            x = _input_float("Target x", 0.0)
            y = _input_float("Target y", 0.0)
            z = _input_float("Target z", 0.0)
            oz = _input_float("Orientation z", 0.0)
            ow = _input_float("Orientation w", 1.0)
            fid = _input_str("frame_id", "map")
            nav.nav_to_pose(x, y, z, oz, ow, fid)
        elif choice == "2.2":
            x = _input_float("Target x", 0.0)
            y = _input_float("Target y", 0.0)
            z = _input_float("Target z", 0.0)
            oz = _input_float("Orientation z", 0.0)
            ow = _input_float("Orientation w", 1.0)
            fid = _input_str("frame_id", "map")
            nav.nav_to_pose_safe(x, y, z, oz, ow, fid)
        elif choice == "2.3":
            nav.pause()
        elif choice == "2.4":
            nav.resume()
        elif choice == "2.5":
            nav.cancel()
        elif choice == "2.6":
            def _on_task(msg):
                s = NavigationManager.STATUS_MAP.get(msg.get("status"), "?")
                t = NavigationManager.TASK_TYPE_MAP.get(msg.get("task_type"), "?")
                print(f"  Task: {t} -> {s}  [action_status: {nav.action_status}]")
            nav.subscribe_task_status(_on_task)
            input("\nPress Enter to stop...")
            nav.unsubscribe_task_status()
        elif choice == "2.7":
            nav.unsubscribe_task_status()
        elif choice == "2.8":
            def _on_pos(msg):
                pos = msg.get("pose", {}).get("position", {})
                print(f"  Position: x={pos.get('x', 0):.2f} y={pos.get('y', 0):.2f}")
            nav.subscribe_robot_position(_on_pos)
            input("\nPress Enter to stop...")
            nav.unsubscribe_robot_position()
        elif choice == "2.9":
            nav.unsubscribe_robot_position()
        else:
            return False
        return True

    while True:
        print("\n=== NavigationManager (navigation.vue / goPoint.vue) ===")
        _print_section("Navigation", items)
        print("  0  Back")
        choice = input("\nSelect: ").strip()
        if choice == "0":
            break
        if not _handle(choice):
            print("  Invalid choice")


# ==================== 3. Robot Management Menu ====================

def menu_robot_manager(client: RobotWebSocketClient) -> None:
    robot = RobotManager(client)
    items = [
        ("3.1", "Mode switch (mapping/localization/patrol/remote_control/idle)"),
        ("3.2", "Init pose with covariance (/initialpose)"),
        ("3.3", "Init pose simple (origin)"),
        ("3.4", "Remote control (single velocity)"),
        ("3.5", "Remote control (with auto-stop)"),
        ("3.6", "Stop remote control"),
        ("3.7", "Get IP address"),
        ("3.8", "Subscribe /battery_state"),
        ("3.9", "Unsubscribe /battery_state"),
        ("3.10", "Subscribe /robot_status"),
        ("3.11", "Unsubscribe /robot_status"),
    ]

    def _handle(choice: str) -> bool:
        if choice == "3.1":
            print("  Options: mapping / localization / patrol / remote_control / idle")
            mode = _input_str("Mode")
            robot.set_mode(mode)
        elif choice == "3.2":
            x = _input_float("x", 0.0)
            y = _input_float("y", 0.0)
            z = _input_float("z", 0.0)
            oz = _input_float("Orientation z", 0.0)
            ow = _input_float("Orientation w", 1.0)
            robot.init_pose(x, y, z, 0, 0, oz, ow)
        elif choice == "3.3":
            robot.init_pose_simple()
        elif choice == "3.4":
            vx = _input_float("Linear x (m/s)", 0.32)
            vz = _input_float("Angular z (rad/s)", 0.0)
            robot.remote_control(vx, vz)
        elif choice == "3.5":
            vx = _input_float("Linear x (m/s)", 0.32)
            vz = _input_float("Angular z (rad/s)", 0.0)
            delay = _input_float("Auto-stop delay (s)", 3.0)
            robot.remote_control(vx, vz, auto_stop=True, auto_stop_delay=delay)
        elif choice == "3.6":
            robot.stop_remote_control()
        elif choice == "3.7":
            robot.get_ip()
        elif choice == "3.8":
            def _on_battery(msg):
                pct = msg.get("percentage", 0)
                if isinstance(pct, float) and pct <= 1:
                    pct *= 100
                status_map = {0: "?", 1: "Charging", 2: "Discharging", 3: "Not charging", 4: "Full"}
                st = status_map.get(msg.get("power_supply_status"), "?")
                print(f"  Battery: {pct:.0f}% | {msg.get('voltage', '?')}V | {st}")
            robot.subscribe_battery(_on_battery)
            input("\nPress Enter to stop...")
            robot.unsubscribe_battery()
        elif choice == "3.9":
            robot.unsubscribe_battery()
        elif choice == "3.10":
            def _on_status(msg):
                data = msg.get("data", "")
                if "+" in data:
                    slam, ctrl = data.split("+", 1)
                    print(f"  Robot: SLAM={slam} Control={ctrl}")
                else:
                    print(f"  Robot: {data}")
            robot.subscribe_status(_on_status)
            input("\nPress Enter to stop...")
            robot.unsubscribe_status()
        elif choice == "3.11":
            robot.unsubscribe_status()
        else:
            return False
        return True

    while True:
        print("\n=== RobotManager (telecontrol.vue / relocation.vue / home.vue) ===")
        _print_section("Robot Management", items)
        print("  0  Back")
        choice = input("\nSelect: ").strip()
        if choice == "0":
            break
        if not _handle(choice):
            print("  Invalid choice")


# ==================== 4. Waypoint Management Menu ====================

def menu_waypoint_manager(client: RobotWebSocketClient) -> None:
    wp = WaypointManager(client)
    items = [
        ("4.1", "Add waypoint node"),
        ("4.2", "Delete waypoint node"),
        ("4.3", "Update waypoint node"),
        ("4.4", "Get single waypoint node"),
        ("4.5", "Get map waypoint node list"),
    ]

    def _prompt_pose(name_default: str = "wp") -> tuple:
        x = _input_float("x", 0.0)
        y = _input_float("y", 0.0)
        z = _input_float("z", 0.0)
        oz = _input_float("Orientation z", 0.0)
        ow = _input_float("Orientation w", 1.0)
        name = _input_str("Name", name_default)
        return {"x": x, "y": y, "z": z}, {"x": 0.0, "y": 0.0, "z": oz, "w": ow}, name

    def _handle(choice: str) -> bool:
        if choice == "4.1":
            map_id = _input_int("Map ID")
            fid = _input_str("frame_id", "map")
            pos, ori, name = _prompt_pose("waypoint_1")
            wp.add_point(map_id, pos, ori, name, fid)
        elif choice == "4.2":
            pid = _input_int("Point ID")
            wp.delete_point(pid)
        elif choice == "4.3":
            pid = _input_int("Point ID")
            pos, ori, name = _prompt_pose("waypoint_1")
            wp.update_point(pid, pos, ori, name)
        elif choice == "4.4":
            pid = _input_int("Point ID")
            wp.get_point(pid)
        elif choice == "4.5":
            map_id = _input_int("Map ID")
            wp.get_map_point_list(map_id)
        else:
            return False
        return True

    while True:
        print("\n=== WaypointManager (waypoint_node CRUD) ===")
        _print_section("Waypoint Nodes", items)
        print("  0  Back")
        choice = input("\nSelect: ").strip()
        if choice == "0":
            break
        if not _handle(choice):
            print("  Invalid choice")


# ==================== 5. Patrol Management Menu ====================

def menu_patrol_manager(client: RobotWebSocketClient) -> None:
    patrol = PatrolManager(client)
    items = [
        ("5.1", "Add patrol route (plain coord array)"),
        ("5.2", "Delete patrol route"),
        ("5.3", "Update patrol route"),
        ("5.4", "Get single patrol route"),
        ("5.5", "Get map patrol route list"),
        ("5.6", "Get first route points"),
        ("5.7", "Start patrol (auto-cancel first)"),
        ("5.8", "Pause patrol"),
        ("5.9", "Resume patrol"),
        ("5.10", "Cancel patrol"),
    ]

    def _prompt_points() -> list:
        count = _input_int("Number of points", 1)
        points = []
        for i in range(count):
            print(f"\n  --- Point {i + 1} ---")
            x = _input_float("x", 0.0)
            y = _input_float("y", 0.0)
            z = _input_float("z", 0.0)
            points.append({"x": x, "y": y, "z": z})
        return points

    def _handle(choice: str) -> bool:
        if choice == "5.1":
            map_id = _input_int("Map ID")
            fid = _input_str("frame_id", "map")
            points = _prompt_points()
            patrol.add_waypoint(map_id, points, fid)
        elif choice == "5.2":
            wid = _input_int("Route ID")
            patrol.delete_waypoint(wid)
        elif choice == "5.3":
            wid = _input_int("Route ID")
            map_id = _input_int("Map ID")
            fid = _input_str("frame_id", "map")
            points = _prompt_points()
            patrol.update_waypoint(wid, map_id, fid, points)
        elif choice == "5.4":
            wid = _input_int("Route ID")
            patrol.get_waypoint(wid)
        elif choice == "5.5":
            map_id = _input_int("Map ID")
            result = patrol.get_map_waypoint_list(map_id)
            _print_json_block("Patrol routes", result)
        elif choice == "5.6":
            map_id = _input_int("Map ID")
            points = patrol.get_first_route_points(map_id)
            if points:
                print(f"  Got {len(points)} points from first route")
                for i, p in enumerate(points):
                    print(f"    {i+1}. x={p.get('x')} y={p.get('y')} z={p.get('z')}")
            else:
                print("  No patrol routes found")
        elif choice == "5.7":
            fid = _input_str("frame_id", "map")
            points = _prompt_points()
            patrol.start_patrol(points, fid)
        elif choice == "5.8":
            patrol.pause()
        elif choice == "5.9":
            patrol.resume()
        elif choice == "5.10":
            patrol.cancel()
        else:
            return False
        return True

    while True:
        print("\n=== PatrolManager (patrol.vue) ===")
        _print_section("Patrol Routes (data_type=waypoint)", items)
        print("  0  Back")
        choice = input("\nSelect: ").strip()
        if choice == "0":
            break
        if not _handle(choice):
            print("  Invalid choice")


# ==================== 6. Dock Management Menu ====================

def menu_dock_manager(client: RobotWebSocketClient) -> None:
    dock = DockManager(client)
    items = [
        ("6.1", "Set dock pose (/set_dock_pose service)"),
        ("6.2", "Get dock pose (/get_dock_pose)"),
        ("6.3", "Start docking (/cmd_dock dock + wait /dock_result)"),
        ("6.4", "Cancel docking (/cmd_dock cancel_dock)"),
        ("6.5", "Leave dock (/cmd_dock undock)"),
        ("6.6", "Set dock pose via /cmd_dock + wait result"),
        ("6.7", "Subscribe /dock_state"),
        ("6.8", "Unsubscribe /dock_state"),
        ("6.9", "Show dock status"),
    ]

    def _handle(choice: str) -> bool:
        if choice == "6.1":
            map_id = _input_int("Map ID", 1)
            x = _input_float("Dock x", 2.0)
            y = _input_float("Dock y", 3.0)
            z = _input_float("Dock z", 0.0)
            oz = _input_float("Orientation z", 0.0)
            ow = _input_float("Orientation w", 1.0)
            dock.set_dock_pose(map_id, {"x": x, "y": y, "z": z}, {"x": 0.0, "y": 0.0, "z": oz, "w": ow})
        elif choice == "6.2":
            map_id = _input_int("Map ID", 1)
            dock.get_dock_pose(map_id)
        elif choice == "6.3":
            if dock.is_dock_disabled:
                print(f"  Dock disabled: isSet={dock.is_dock_pose_set} status={dock.dock_status}")
            else:
                dock.dock(wait_result=True)
        elif choice == "6.4":
            dock.cancel_dock()
        elif choice == "6.5":
            dock.undock()
        elif choice == "6.6":
            dock.cmd_set_dock_pose(wait_result=True)
        elif choice == "6.7":
            def _on_dock(msg):
                data = msg.get("data", "")
                label = DockManager.DOCK_STATE_LABELS.get(data, data)
                print(f"  Dock state: {label}")
            dock.subscribe_dock_state(_on_dock)
            input("\nPress Enter to stop...")
            dock.unsubscribe_dock_state()
        elif choice == "6.8":
            dock.unsubscribe_dock_state()
        elif choice == "6.9":
            print(f"  is_dock_pose_set: {dock.is_dock_pose_set}")
            print(f"  dock_status: {dock.dock_status} ({DockManager.DOCK_STATE_LABELS.get(dock.dock_status, '?')})")
            print(f"  is_busy: {dock.is_busy}")
            print(f"  is_dock_disabled: {dock.is_dock_disabled}")
        else:
            return False
        return True

    while True:
        print("\n=== DockManager (charge.vue) ===")
        _print_section("Dock Management", items)
        print("  0  Back")
        choice = input("\nSelect: ").strip()
        if choice == "0":
            break
        if not _handle(choice):
            print("  Invalid choice")


# ==================== Main Menu ====================

def main_menu() -> None:
    """Robot WebSocket Control Menu v3 — aligned with aidrobo_client."""
    client = _make_menu_client()
    client.connect()

    sections = [
        ("1. MapManager (newMap.vue / seeMap.vue)", menu_map_manager),
        ("2. NavigationManager (navigation.vue / goPoint.vue)", menu_navigation_manager),
        ("3. RobotManager (telecontrol.vue / relocation.vue / home.vue)", menu_robot_manager),
        ("4. WaypointManager (waypoint_node CRUD)", menu_waypoint_manager),
        ("5. PatrolManager (patrol.vue)", menu_patrol_manager),
        ("6. DockManager (charge.vue)", menu_dock_manager),
    ]

    try:
        while True:
            print("\n" + "=" * 60)
            print("    Robot API v3 — aligned with aidrobo_client")
            print("=" * 60)
            for idx, (title, _) in enumerate(sections, 1):
                print(f"  {idx}. {title}")
            print(f"  0. Exit")
            print(f"  WS: {_get_ws_url()}")
            print("=" * 60)

            try:
                choice = input("\nSelect module: ").strip()
            except KeyboardInterrupt:
                print("\nExited.")
                break

            if choice == "0":
                print("Exited.")
                break

            try:
                idx = int(choice)
                if 1 <= idx <= len(sections):
                    sections[idx - 1][1](client)
                else:
                    print("Invalid choice.")
            except ValueError:
                print("Invalid choice.")
    finally:
        client.disconnect()


if __name__ == "__main__":
    main_menu()
