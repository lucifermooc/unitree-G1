"""语义地图节点：一句话 → 当前地图上最相似的位置点 → 发 /nav_to_pose 让机器人过去。

只通过现有接口和后端交互，不改动后端：
  读点位：/get_current_map_id + /get_map_point_list（map_manager_server）
  导航：  发布 /nav_to_pose，由 waypoint_manage 执行并发布 /task_status

提供的服务（前端经 rosbridge 调用）：
  /semantic_map/rebuild  std_srvs/srv/Trigger              重建当前地图的语义库
  /semantic_map/search   aid_robot_msgs/srv/SetString       data=一句话，message=JSON 结果
  /semantic_map/go       aid_robot_msgs/srv/SetString       同上，找到就发导航目标
点位有增删改时，下次搜索会自动重建（按点位内容指纹判断），也可以手动调 rebuild。
"""
import json
import threading
import time

import rclpy
from aid_robot_msgs.msg import AidTaskStatus
from aid_robot_msgs.srv import GetCurrentMap, MapLinkedDataList, PatrolControl, SetString
from geometry_msgs.msg import PoseStamped
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from std_srvs.srv import Trigger

from .core import SemanticIndex, parse_point_rows
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

        # 后台预加载模型，第一次搜索不用等
        threading.Thread(target=self._preload, daemon=True).start()
        self.get_logger().info(f"semantic_map_server started (Qdrant {self.qdrant_host}:{self.qdrant_port})")

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
        text = text.strip()
        if not text:
            raise ValueError("empty query")
        with self.lock:
            map_id, places, rebuilt = self._ensure_index()
            if not places:
                return {"query": text, "found": False, "best": None, "candidates": [],
                        "map_id": map_id, "rebuilt": rebuilt}
            result = self.index.search(map_id, text, self.threshold, self.top_k)
        result.update(map_id=map_id, rebuilt=rebuilt, threshold=self.threshold)
        top = ", ".join(f"{c['name']}:{c['score']:.3f}" for c in result["candidates"])
        self.get_logger().info(f"search '{text}' map={map_id} points={len(places)} rebuilt={rebuilt} "
                               f"found={result['found']} [{top}]")
        return result

    def _on_search(self, request, response):
        try:
            result = self._search(request.data)
            response.success, response.message = True, json.dumps(result, ensure_ascii=False)
        except Exception as e:
            response.success, response.message = False, str(e)
        return response

    def _on_go(self, request, response):
        try:
            result = self._search(request.data)
            result["navigating"] = False
            if result["found"]:
                # 有任务在跑时 waypoint_manage 会忽略新目标，先取消（与原前端 nav_to_pose_safe 一致）
                st = self.task_status
                if st is not None and st.status in (TASK_WORKING, TASK_SUSPEND):
                    self._call(self.patrol_cli, PatrolControl.Request(cmd="cancel"))
                    time.sleep(0.5)
                self.nav_pub.publish(self._goal(result["best"]))
                result["navigating"] = True
            response.success, response.message = True, json.dumps(result, ensure_ascii=False)
        except Exception as e:
            response.success, response.message = False, str(e)
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
