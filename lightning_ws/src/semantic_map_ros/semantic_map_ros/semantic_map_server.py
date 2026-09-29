"""语义地图节点：一句话 → 当前地图上最相似的位置点 → 发 /nav_to_pose 让机器人过去。

只通过现有接口和后端交互，不改动后端：
  读点位：/get_current_map_id + /get_map_point_list（map_manager_server）
  导航：  发布 /nav_to_pose，由 waypoint_manage 执行并发布 /task_status

提供的服务（前端经 rosbridge 调用）：
  /semantic_map/rebuild  std_srvs/srv/Trigger              重建当前地图的语义库
  /semantic_map/search   aid_robot_msgs/srv/SetString       data=一句话，message=JSON 结果
  /semantic_map/go       aid_robot_msgs/srv/SetString       同上，找到就发导航目标
  /semantic_map/history  std_srvs/srv/Trigger              最近的调试记录（JSON 列表）
data 可以是普通文本，也可以是 {"text": "...", "source": "asr"}（source 只用于调试记录）。
话题输入（给 ASR 等只会发话题的模块）：
  /semantic_map/text_in  std_msgs/msg/String   收到一句就搜索（text_in_action=go 时找到就导航）
调试输出：
  /semantic_map/debug    std_msgs/msg/String   每次请求一条 JSON：来源、原始文本、发现的问题、得分、结果、耗时
  同样的记录追加写进 log_file（默认 ~/maps/semantic_map_log.jsonl）
点位有增删改时，下次搜索会自动重建（按点位内容指纹判断），也可以手动调 rebuild。
"""
import json
import threading
import time
from datetime import datetime

import rclpy
from aid_robot_msgs.msg import AidTaskStatus
from aid_robot_msgs.srv import GetCurrentMap, MapLinkedDataList, PatrolControl, SetString
from geometry_msgs.msg import PoseStamped
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from std_msgs.msg import String
from std_srvs.srv import Trigger

from .core import SemanticIndex, inspect_text, parse_point_rows, parse_request
from .debug_log import DebugLog, visible
from .embedder import Embedder

TASK_WORKING, TASK_SUSPEND = 1, 4


class SemanticMapServer(Node):
    def __init__(self):
        super().__init__("semantic_map_server")
        p = self.declare_parameter
        self.qdrant_host = p("qdrant_host", "localhost").value
        self.qdrant_port = p("qdrant_port", 6333).value
        self.threshold = p("score_threshold", 0.52).value
        self.top_k = p("top_k", 3).value
        prefix = p("collection_prefix", "semantic_map").value
        self.embedder = Embedder(p("model_name", "BAAI/bge-m3").value, p("use_fp16", False).value)
        text_in_topic = p("text_in_topic", "/semantic_map/text_in").value
        self.text_in_action = p("text_in_action", "search").value
        if self.text_in_action not in ("search", "go"):
            raise ValueError("text_in_action must be 'search' or 'go'")
        self.debug_log = DebugLog(p("log_file", "~/maps/semantic_map_log.jsonl").value,
                                  p("history_size", 200).value)

        from qdrant_client import QdrantClient
        self.index = SemanticIndex(QdrantClient(host=self.qdrant_host, port=self.qdrant_port),
                                   self.embedder.encode, prefix)
        self.lock = threading.Lock()  # 重建和搜索串行，避免同时写同一个 collection
        self.task_status = None

        clients = MutuallyExclusiveCallbackGroup()
        self.current_map_cli = self.create_client(GetCurrentMap, "/get_current_map_id", callback_group=clients)
        self.point_list_cli = self.create_client(MapLinkedDataList, "/get_map_point_list", callback_group=clients)
        self.patrol_cli = self.create_client(PatrolControl, "/patrol_control", callback_group=clients)
        self.nav_pub = self.create_publisher(PoseStamped, "/nav_to_pose", 10)
        self.create_subscription(AidTaskStatus, "/task_status", self._on_task_status, 10,
                                 callback_group=clients)

        services = ReentrantCallbackGroup()
        self.create_service(Trigger, "/semantic_map/rebuild", self._on_rebuild, callback_group=services)
        self.create_service(SetString, "/semantic_map/search", self._on_search, callback_group=services)
        self.create_service(SetString, "/semantic_map/go", self._on_go, callback_group=services)
        self.create_service(Trigger, "/semantic_map/history", self._on_history, callback_group=services)

        self.debug_pub = self.create_publisher(String, "/semantic_map/debug", 50)
        if text_in_topic:
            self.create_subscription(String, text_in_topic, self._on_text_in, 10,
                                     callback_group=MutuallyExclusiveCallbackGroup())

        # 后台预加载模型，第一次搜索不用等
        threading.Thread(target=self._preload, daemon=True).start()
        self.get_logger().info(f"semantic_map_server started (Qdrant {self.qdrant_host}:{self.qdrant_port}, "
                               f"text_in={text_in_topic or 'off'}:{self.text_in_action}, "
                               f"log={self.debug_log.path or 'off'})")

    def _preload(self):
        try:
            self.embedder.load()
            self.get_logger().info(f"embedding model loaded: {self.embedder.model_name}")
        except Exception as e:  # 模型下载失败等，等到真正使用时再报给前端
            self.get_logger().error(f"failed to load embedding model: {e}")

    def _on_task_status(self, msg):
        self.task_status = msg

    # ---------- 调用后端服务 ----------
    def _call(self, client, request, timeout=10.0):
        if not client.wait_for_service(timeout_sec=timeout):
            raise RuntimeError(f"service {client.srv_name} not available")
        future = client.call_async(request)
        done = threading.Event()
        future.add_done_callback(lambda _: done.set())
        if not done.wait(timeout):
            raise RuntimeError(f"service {client.srv_name} timed out")
        return future.result()

    def _current_places(self):
        cur = self._call(self.current_map_cli, GetCurrentMap.Request())
        if not cur.success or cur.map_id == 0:
            raise RuntimeError("no current map (select a map first)")
        req = MapLinkedDataList.Request(map_id=cur.map_id, data_type="waypoint_node")
        res = self._call(self.point_list_cli, req)
        if not res.success:
            raise RuntimeError(f"get_map_point_list failed: {res.message}")
        return cur.map_id, parse_point_rows(res.message)

    def _ensure_index(self, force=False):
        map_id, places = self._current_places()
        rebuilt = False
        if force or not self.index.is_fresh(map_id, places):
            self.index.rebuild(map_id, places)
            rebuilt = True
        return map_id, places, rebuilt

    # ---------- 服务回调 ----------
    def _on_rebuild(self, request, response):
        try:
            with self.lock:
                map_id, places, _ = self._ensure_index(force=True)
            response.success = True
            response.message = json.dumps({"map_id": map_id, "count": len(places)}, ensure_ascii=False)
        except Exception as e:
            response.success, response.message = False, str(e)
        return response

    def _search(self, text):
        with self.lock:
            map_id, places, rebuilt = self._ensure_index()
            if not places:
                return {"query": text, "found": False, "best": None, "candidates": [],
                        "map_id": map_id, "rebuilt": rebuilt, "threshold": self.threshold}
            result = self.index.search(map_id, text, self.threshold, self.top_k)
        result.update(map_id=map_id, rebuilt=rebuilt, threshold=self.threshold)
        top = ", ".join(f"{c['name']}:{c['score']:.3f}" for c in result["candidates"])
        self.get_logger().info(f"search '{text}' map={map_id} points={len(places)} rebuilt={rebuilt} "
                               f"found={result['found']} [{top}]")
        return result

    def _navigate(self, result):
        """找到就发导航目标；有任务在跑时 waypoint_manage 会忽略新目标，先取消（与原前端 nav_to_pose_safe 一致）。"""
        result["navigating"] = False
        if not result["found"]:
            return
        st = self.task_status
        if st is not None and st.status in (TASK_WORKING, TASK_SUSPEND):
            self._call(self.patrol_cli, PatrolControl.Request(cmd="cancel"))
            time.sleep(0.5)
        self.nav_pub.publish(self._goal(result["best"]))
        result["navigating"] = True

    def _handle(self, entry, data, go):
        """所有入口（search / go 服务、text_in 话题）都走这里：解析 → 检查文本 → 搜索 → 可选导航 → 记录。

        出错时抛异常；无论成功失败都会留下一条调试记录。
        """
        t0 = time.monotonic()
        raw, source = parse_request(data, entry)
        text, issues = inspect_text(raw)
        record = {"time": datetime.now().isoformat(timespec="milliseconds"), "entry": entry,
                  "source": source, "raw": raw, "raw_visible": visible(raw), "length": len(raw),
                  "issues": issues, "text": text, "go": go}
        try:
            if not text:
                raise ValueError("empty query")
            result = self._search(text)
            if go:
                self._navigate(result)
            result.update(source=source, issues=issues)
            best = result["best"]
            record.update(ok=True, map_id=result["map_id"], found=result["found"],
                          best=best["name"] if best else None, threshold=result["threshold"],
                          candidates=[{"id": c["id"], "name": c["name"], "score": c["score"]}
                                      for c in result["candidates"]],
                          rebuilt=result["rebuilt"], navigating=result.get("navigating", False))
            return result
        except Exception as e:
            record.update(ok=False, error=str(e))
            raise
        finally:
            record["elapsed_ms"] = round((time.monotonic() - t0) * 1000)
            self.debug_pub.publish(String(data=self.debug_log.add(record)))
            if issues:
                self.get_logger().warn(f"input from {source} has issues: {issues} raw={visible(raw)}")

    def _on_search(self, request, response):
        return self._respond(response, "search", request.data, go=False)

    def _on_go(self, request, response):
        return self._respond(response, "go", request.data, go=True)

    def _respond(self, response, entry, data, go):
        try:
            result = self._handle(entry, data, go)
            response.success, response.message = True, json.dumps(result, ensure_ascii=False)
        except Exception as e:
            response.success, response.message = False, str(e)
        return response

    def _on_text_in(self, msg):
        try:
            self._handle("topic", msg.data, go=self.text_in_action == "go")
        except Exception as e:  # 话题没有应答，错误只在调试记录和日志里
            self.get_logger().error(f"text_in failed: {e}")

    def _on_history(self, request, response):
        response.success = True
        response.message = json.dumps(self.debug_log.recent(), ensure_ascii=False)
        return response

    def _goal(self, best):
        goal = PoseStamped()
        goal.header.frame_id = best["frame_id"]
        goal.header.stamp = self.get_clock().now().to_msg()
        goal.pose.position.x, goal.pose.position.y = best["x"], best["y"]
        goal.pose.position.z = best.get("z", 0.0)
        o = best["orientation"]
        goal.pose.orientation.x, goal.pose.orientation.y = o["x"], o["y"]
        goal.pose.orientation.z, goal.pose.orientation.w = o["z"], o["w"]
        return goal


def main():
    rclpy.init()
    node = SemanticMapServer()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
