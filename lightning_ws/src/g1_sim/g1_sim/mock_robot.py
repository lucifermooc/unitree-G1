"""模拟机器人：仿真时代替真机上依赖硬件的部分（仅用于仿真，真机不启动）。

真机上由这些程序提供，本节点按相同的接口和行为模拟：
  /mode_set, /aid_save_map, /robot_status   ← robot_status_manager（C++，依赖 lightning）
  /map（原始栅格，transient_local）           ← lightning 建图（前端预览由原有的 map_transform_node 转成 /map_base64）
  /base_link_pose                            ← robot_pose_pub（定位 TF）
  /battery_state                             ← g1_nav_bridge battery_state_bridge（宇树电池）
  /start_init_pose, /aid_init_pose, /initialpose ← 重定位
  navigate_to_pose（Nav2 action）            ← Nav2（这里按 A* 路径匀速走过去）
  /map_editor                                ← robot_bringup editor_map（橡皮擦）
  /aid_draw_forbidden_line                   ← robot_bringup forbidden_map_create（禁行线）
地图、位置点、禁行线的数据库，以及单点导航/巡逻的调度，仍由真实的 map_manager_server
和 waypoint_manage 负责，本节点不碰。
"""
import heapq
import math
import os
import shutil
import sqlite3
import threading
import time
from pathlib import Path

import cv2
import numpy as np
import rclpy
import yaml
from aid_robot_msgs.srv import DrawPicture, GetCurrentMap, MapOperation, StatusChange
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import OccupancyGrid
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import BatteryState
from std_msgs.msg import String

FREE, UNKNOWN, OCC = 254, 205, 0


def yaw_to_quat(yaw):
    return 0.0, 0.0, math.sin(yaw / 2), math.cos(yaw / 2)


def quat_to_yaw(q):
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


def angle_diff(a, b):
    return (a - b + math.pi) % (2 * math.pi) - math.pi


class GridMap:
    """map.yaml + map.pgm；提供坐标换算、A* 规划、射线揭图。"""

    def __init__(self, yaml_path):
        meta = yaml.safe_load(Path(yaml_path).read_text())
        image = Path(yaml_path).parent / meta["image"]
        self.img = cv2.imread(str(image), cv2.IMREAD_UNCHANGED)
        self.res = float(meta["resolution"])
        self.ox, self.oy = float(meta["origin"][0]), float(meta["origin"][1])
        self.h, self.w = self.img.shape
        occupied = self.img < 100
        # 机器人半径 0.3 m 膨胀，A* 在膨胀后的图上找路
        r = int(0.3 / self.res)
        kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (2 * r + 1, 2 * r + 1))
        self.blocked = cv2.dilate(occupied.astype(np.uint8), kernel) > 0
        self.occupied = occupied

    def to_cell(self, x, y):
        return int((x - self.ox) / self.res), self.h - 1 - int((y - self.oy) / self.res)

    def to_world(self, i, j):
        return self.ox + (i + 0.5) * self.res, self.oy + (self.h - j - 0.5) * self.res

    def inside(self, i, j):
        return 0 <= i < self.w and 0 <= j < self.h

    def plan(self, start, goal, step=2):
        """在 step 倍降采样的栅格上做 A*，返回世界坐标路径；到不了返回 None。"""
        s, g = self.to_cell(*start), self.to_cell(*goal)
        if not self.inside(*g) or self.blocked[g[1], g[0]]:
            return None
        s, g = (s[0] // step, s[1] // step), (g[0] // step, g[1] // step)
        blocked = self.blocked[::step, ::step]
        hh, ww = blocked.shape
        moves = [(1, 0, 1), (-1, 0, 1), (0, 1, 1), (0, -1, 1),
                 (1, 1, 1.414), (1, -1, 1.414), (-1, 1, 1.414), (-1, -1, 1.414)]
        openq, came, cost = [(0.0, s)], {s: None}, {s: 0.0}
        while openq:
            _, cur = heapq.heappop(openq)
            if cur == g:
                break
            for dx, dy, c in moves:
                n = (cur[0] + dx, cur[1] + dy)
                if not (0 <= n[0] < ww and 0 <= n[1] < hh) or (blocked[n[1], n[0]] and n != s):
                    continue
                nc = cost[cur] + c
                if nc < cost.get(n, 1e18):
                    cost[n], came[n] = nc, cur
                    heapq.heappush(openq, (nc + math.hypot(g[0] - n[0], g[1] - n[1]), n))
        if g not in came:
            return None
        path, n = [], g
        while n is not None:
            path.append(self.to_world(n[0] * step, n[1] * step))
            n = came[n]
        path.reverse()
        path[-1] = goal
        return path[::3] + [goal]  # 稀疏一点

    def reveal(self, mask, x, y, max_range=6.0, rays=240):
        """模拟激光：从 (x,y) 发射线，打到墙为止，把经过的格子标记为已探索。"""
        ci, cj = self.to_cell(x, y)
        steps = int(max_range / self.res)
        for k in range(rays):
            a = 2 * math.pi * k / rays
            dx, dy = math.cos(a), -math.sin(a)
            for s in range(steps):
                i, j = int(ci + dx * s), int(cj + dy * s)
                if not self.inside(i, j):
                    break
                mask[j, i] = True
                if self.occupied[j, i]:
                    break


class MockRobot(Node):
    def __init__(self):
        super().__init__("mock_robot")
        share = get_package_share_directory("g1_sim")
        world = self.declare_parameter("world_map", os.path.join(share, "maps", "office", "map.yaml")).value
        self.speed = self.declare_parameter("speed", 0.8).value                  # 导航速度 m/s
        self.mapping_speed = self.declare_parameter("mapping_speed", 2.5).value  # 建图探索速度（加快仿真）
        self.world = GridMap(world)
        self.maps_root = Path.home() / "maps"
        self.maps_root.mkdir(parents=True, exist_ok=True)

        self.lock = threading.RLock()
        self.slam = "idle"          # idle / mapping / localization
        self.control = "idle"       # idle / patrol / remote_control
        self.x = self.y = self.yaw = 0.0
        self.revealed = np.zeros_like(self.world.occupied)
        self.explore_path = []
        self.battery = 0.86
        self.forbidden_lines = []

        cb = ReentrantCallbackGroup()
        self.create_service(StatusChange, "/mode_set", self.on_mode_set, callback_group=cb)
        self.create_service(MapOperation, "/aid_save_map", self.on_save_map, callback_group=cb)
        self.create_service(DrawPicture, "/map_editor", self.on_map_editor, callback_group=cb)
        self.create_service(DrawPicture, "/aid_draw_forbidden_line", self.on_forbidden_line, callback_group=cb)
        self.current_map_cli = self.create_client(GetCurrentMap, "/get_current_map_id",
                                                  callback_group=MutuallyExclusiveCallbackGroup())
        for topic in ("/start_init_pose", "/aid_init_pose"):
            self.create_subscription(PoseStamped, topic, self.on_init_pose, 10, callback_group=cb)
        self.create_subscription(PoseWithCovarianceStamped, "/initialpose",
                                 lambda m: self.on_init_pose(m.pose), 10, callback_group=cb)

        self.pose_pub = self.create_publisher(PoseStamped, "/base_link_pose", 10)
        self.status_pub = self.create_publisher(String, "/robot_status", 10)
        self.battery_pub = self.create_publisher(BatteryState, "/battery_state", 10)
        qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                         durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.map_pub = self.create_publisher(OccupancyGrid, "/map", qos)

        self.nav_server = ActionServer(
            self, NavigateToPose, "navigate_to_pose", execute_callback=self.execute_nav,
            goal_callback=lambda g: GoalResponse.ACCEPT,
            cancel_callback=lambda g: CancelResponse.ACCEPT, callback_group=cb)

        self.create_timer(0.1, self.tick_motion, callback_group=cb)
        self.create_timer(0.2, self.publish_pose, callback_group=cb)
        self.create_timer(1.0, self.publish_status, callback_group=cb)
        self.create_timer(0.5, self.reveal_tick, callback_group=cb)
        self.create_timer(1.0, self.publish_mapping_preview, callback_group=cb)
        self.get_logger().info(f"mock robot ready, world map {world}, maps dir {self.maps_root}")

    # ---------------- 模式 ----------------
    def _current_map_file(self):
        if not self.current_map_cli.wait_for_service(timeout_sec=3.0):
            return None
        future = self.current_map_cli.call_async(GetCurrentMap.Request())
        done = threading.Event()
        future.add_done_callback(lambda _: done.set())
        done.wait(5.0)
        res = future.result() if future.done() else None
        return res.map_file if res and res.success and res.map_id else None

    def on_mode_set(self, request, response):
        action = request.action.strip()
        ok = True
        # 查当前地图要调服务，不能拿着锁等（会卡住运动定时器）
        has_map = self._current_map_file() is not None if action == "localization" else False
        with self.lock:
            if action == "mapping":
                self.slam, self.control = "mapping", "idle"
                self.x = self.y = self.yaw = 0.0          # 建图原点 = 机器人当前位置
                self.revealed[:] = False
                self.explore_path = self._exploration_route()
            elif action == "localization":
                self.explore_path = []
                ok = has_map  # 真机：没有当前地图就起不了定位
                if ok:
                    self.slam, self.control = "localization", "idle"
            elif action == "patrol":
                ok = self.slam == "localization"
                if ok:
                    self.control = "patrol"
            elif action == "remote_control":
                self.control = "remote_control"
            elif action == "idle":
                self.slam, self.control, self.explore_path = "idle", "idle", []
            else:
                ok = False
        response.message = "ok" if ok else "err"
        self.get_logger().info(f"mode_set {action} -> {response.message} ({self.slam}+{self.control})")
        return response

    def _exploration_route(self):
        """建图时的探索路线：走廊来回，并进每个房间转一圈。"""
        stops = [(3.0, 0.0), (1.0, 4.5), (3.0, 0.0), (7.0, 0.0), (7.0, 4.5), (7.0, 0.0),
                 (12.5, 0.0), (12.5, 4.5), (12.5, 0.0), (18.5, 0.0), (18.5, 4.5), (18.5, 0.0),
                 (20.0, 0.0), (20.0, -4.5), (16.0, 0.0), (16.0, -4.5), (11.0, 0.0), (11.0, -4.5),
                 (5.0, 0.0), (5.0, -4.5), (5.0, 0.0), (0.0, 0.0)]
        route, cur = [], (0.0, 0.0)
        for s in stops:
            seg = self.world.plan(cur, s)
            if seg:
                route += seg
                cur = s
        return route

    # ---------------- 保存地图（lightning save_map 的替身）----------------
    def on_save_map(self, request, response):
        with self.lock:
            if self.slam != "mapping":
                response.success, response.message = False, "must change mode to mapping first"
                return response
            img = np.where(self.revealed, self.world.img, UNKNOWN).astype(np.uint8)
            self.slam, self.explore_path = "idle", []
        target = Path(str(Path.home()) + request.map_file_name.strip())
        target.mkdir(parents=True, exist_ok=True)
        cv2.imwrite(str(target / "map.pgm"), img)
        (target / "map.yaml").write_text(
            f"image: map.pgm\nmode: trinary\nresolution: {self.world.res}\n"
            f"origin: [{self.world.ox}, {self.world.oy}, 0]\nnegate: 0\n"
            "occupied_thresh: 0.65\nfree_thresh: 0.196\n")
        response.success, response.message = True, ""
        self.get_logger().info(f"map saved to {target}")
        return response

    # ---------------- 运动 ----------------
    def _step_towards(self, tx, ty, dt, speed=None):
        dist = math.hypot(tx - self.x, ty - self.y)
        step = (speed or self.speed) * dt
        if dist <= step:
            self.x, self.y = tx, ty
            return True
        self.yaw = math.atan2(ty - self.y, tx - self.x)
        self.x += step * math.cos(self.yaw)
        self.y += step * math.sin(self.yaw)
        return False

    def tick_motion(self):
        with self.lock:
            if self.slam == "mapping" and self.explore_path:
                if self._step_towards(*self.explore_path[0], 0.1, self.mapping_speed):
                    self.explore_path.pop(0)
            self.battery = max(0.05, self.battery - 0.00002)

    def execute_nav(self, goal_handle):
        """Nav2 的替身：定位模式下沿 A* 路径匀速走到目标，再原地转到目标朝向。"""
        pose = goal_handle.request.pose.pose
        goal = (pose.position.x, pose.position.y)
        goal_yaw = quat_to_yaw(pose.orientation)
        result = NavigateToPose.Result()
        with self.lock:
            path = self.world.plan((self.x, self.y), goal) if self.slam == "localization" else None
        if path is None:
            self.get_logger().warn(f"navigate_to_pose {goal} aborted "
                                   f"({'not localized' if self.slam != 'localization' else 'no path'})")
            goal_handle.abort()
            return result
        start = time.time()
        feedback = NavigateToPose.Feedback()
        while True:
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                return result
            with self.lock:
                if self.slam != "localization":
                    goal_handle.abort()
                    return result
                if path:
                    if self._step_towards(*path[0], 0.1):
                        path.pop(0)
                else:
                    turn = angle_diff(goal_yaw, self.yaw)
                    if abs(turn) < 0.05:
                        self.yaw = goal_yaw
                        break
                    self.yaw += max(-0.15, min(0.15, turn))
                remaining = math.hypot(goal[0] - self.x, goal[1] - self.y)
                feedback.current_pose = self._pose_msg()
            feedback.distance_remaining = float(remaining)
            feedback.navigation_time.sec = int(time.time() - start)
            goal_handle.publish_feedback(feedback)
            time.sleep(0.1)
        goal_handle.succeed()
        return result

    # ---------------- 发布 ----------------
    def _pose_msg(self):
        msg = PoseStamped()
        msg.header.frame_id = "map"
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.pose.position.x, msg.pose.position.y = float(self.x), float(self.y)
        q = yaw_to_quat(self.yaw)
        msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z, msg.pose.orientation.w = q
        return msg

    def publish_pose(self):
        with self.lock:
            if self.slam == "idle":   # 真机：没有定位 TF 时 robot_pose_pub 停发
                return
            msg = self._pose_msg()
        self.pose_pub.publish(msg)

    def publish_status(self):
        with self.lock:
            self.status_pub.publish(String(data=f"{self.slam}+{self.control}"))
            b = BatteryState()
            b.header.stamp = self.get_clock().now().to_msg()
            b.percentage = float(round(self.battery, 4))
            b.voltage, b.current, b.temperature = 50.4, -2.1, 31.0
            b.charge = b.capacity = b.design_capacity = float("nan")   # 真机上这三个就是 NaN
            b.power_supply_status = BatteryState.POWER_SUPPLY_STATUS_DISCHARGING
            b.present = True
        self.battery_pub.publish(b)

    def reveal_tick(self):
        with self.lock:
            if self.slam == "mapping":
                self.world.reveal(self.revealed, self.x, self.y)

    def publish_mapping_preview(self):
        with self.lock:
            if self.slam != "mapping":
                return
            img = np.where(self.revealed, self.world.img, UNKNOWN).astype(np.uint8)
        msg = OccupancyGrid()
        msg.header.frame_id = "map"
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.info.resolution = self.world.res
        msg.info.width, msg.info.height = self.world.w, self.world.h
        msg.info.origin.position.x, msg.info.origin.position.y = self.world.ox, self.world.oy
        msg.info.origin.orientation.w = 1.0
        occ = np.full(img.shape, -1, np.int8)          # 灰度图 → 占用值：254 空闲 0，0 障碍 100，其余未知 -1
        occ[img >= 250] = 0
        occ[img < 100] = 100
        msg.data = np.flipud(occ).flatten().tolist()   # OccupancyGrid 第 0 行是地图最下面一行
        self.map_pub.publish(msg)

    # ---------------- 重定位 ----------------
    def on_init_pose(self, msg):
        with self.lock:
            if self.slam != "localization":
                return
            self.x, self.y = msg.pose.position.x, msg.pose.position.y
            self.yaw = quat_to_yaw(msg.pose.orientation)
        self.get_logger().info(f"relocated to ({self.x:.2f}, {self.y:.2f}, {self.yaw:.2f})")

    # ---------------- 地图编辑 ----------------
    def _map_yaml_of(self, map_id):
        db = self.maps_root / "db.sqlite"
        with sqlite3.connect(db) as conn:
            row = conn.execute("select file_path from map where id=?", (map_id,)).fetchone()
        if not row:
            return None
        d = Path(row[0])
        return d / "map.yaml" if (d / "map.yaml").is_file() else d / "lightning" / "map.yaml"

    def on_map_editor(self, request, response):
        """橡皮擦：把矩形区域写成指定灰度（0=空闲，100=障碍），存回地图文件（先备份一份）。"""
        yaml_path = self._map_yaml_of(request.map_id)
        if yaml_path is None or not yaml_path.is_file():
            response.success, response.message = False, "Get occupancy grid map false"
            return response
        if not request.rectangle_array and request.type != "point":
            response.success, response.message = False, "Empty point array received for editing"
            return response
        grid = GridMap(yaml_path)
        pgm = yaml_path.parent / yaml.safe_load(yaml_path.read_text())["image"]
        backup = pgm.with_name(pgm.stem + "_back" + pgm.suffix)
        if not backup.exists():
            shutil.copy(pgm, backup)
        img = grid.img.copy()
        for rect in request.rectangle_array:
            ci, cj = grid.to_cell(rect.center_point.x, rect.center_point.y)
            r = int(rect.side_length / grid.res) // 2
            g = rect.grayscale
            value = UNKNOWN if g > 100 else int(round(FREE - FREE * g / 100))
            img[max(cj - r, 0):cj + r + 1, max(ci - r, 0):ci + r + 1] = value
        cv2.imwrite(str(pgm), img)
        response.success, response.message = True, "Map updated successfully"
        return response

    def on_forbidden_line(self, request, response):
        with self.lock:
            if self.slam != "localization":   # 真机：forbidden_map_create 还没收到 /map
                response.success, response.message = False, "No map get."
                return response
            self.forbidden_lines = [((l.start.x, l.start.y), (l.end.x, l.end.y)) for l in request.data]
        response.success, response.message = True, "success"
        return response


def main():
    rclpy.init()
    node = MockRobot()
    executor = MultiThreadedExecutor(num_threads=6)
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
