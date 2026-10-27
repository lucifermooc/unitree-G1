"""开机全局定位初始化：全图搜索位姿，lightning 当前输出不对时发 /initialpose 重定位。

不改 lightning：只读 /livox/lidar、/livox/imu、TF，只写 /initialpose（lightning 已支持，10-24 Orin 实机验证）。
状态发在 ~/status（std_msgs/String JSON，transient local）。~/trigger（std_srvs/Trigger）重新跑一次。
"""
import json
import math
import os
import threading
import time
from collections import deque

import numpy as np
import rclpy
from geometry_msgs.msg import PoseWithCovarianceStamped
from livox_ros_driver2.msg import CustomMsg
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.duration import Duration
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rclpy.time import Time
from sensor_msgs.msg import Imu
from std_msgs.msg import String
from std_srvs.srv import Trigger
import tf2_ros

from . import bev_search
from .geom import base_pose_for_lightning, lidar_pose_to_base, livox_raw_to_xyz, quat_to_rot, yaw_diff_deg
from .pcd_io import load_pcd_xyz


class GlobalInitNode(Node):
    def __init__(self):
        super().__init__('loc_global_init')
        d = self.declare_parameter
        self.map_dir = d('map_dir', '').value
        self.lidar_topic = d('lidar_topic', '/livox/lidar').value
        self.imu_topic = d('imu_topic', '/livox/imu').value
        self.map_frame = d('map_frame', 'map').value
        self.base_frame = d('base_frame', 'base_link').value
        self.body_frame = d('body_frame', 'body_link').value
        self.lidar_frame = d('lidar_frame', 'mid360_link').value
        self.n_frames = d('n_frames', 10).value
        self.gyro_max = d('gyro_max', 0.15).value            # 采集窗口内 |gyro| 最大值（rad/s），超过视为在动
        self.auto_start = d('auto_start', True).value
        self.dry_run = d('dry_run', False).value
        self.wait_loc_timeout = d('wait_loc_timeout', 30.0).value
        self.agree_xy = d('agree_xy', 0.3).value
        self.agree_yaw_deg = d('agree_yaw_deg', 5.0).value
        self.verify_timeout = d('verify_timeout', 20.0).value
        self.verify_xy = d('verify_xy', 0.3).value
        self.verify_yaw_deg = d('verify_yaw_deg', 5.0).value
        self.max_attempts = d('max_attempts', 2).value
        self.retry_wait = d('retry_wait', 5.0).value
        self.sp = bev_search.SearchParams()
        for k, v in vars(bev_search.SearchParams()).items():
            setattr(self.sp, k, type(v)(d('search.' + k, v).value))

        self.lock = threading.Lock()
        self.scans = deque(maxlen=max(self.n_frames * 3, 30))   # (stamp, Nx3 点)
        self.imu_group = MutuallyExclusiveCallbackGroup()      # IMU 单独一组，不和点云回调互相排队
        self.imu = deque(maxlen=4000)                         # (stamp, acc(3), gyro(3))
        # 雷达 / IMU / TF 都只在一次初始化期间订阅：rclpy 反序列化 CustomMsg（每帧 ~2 万个点对象）很吃 CPU 且占 GIL，
        # 常订阅时搜索从 3 s 拖到 19 s（本地回放实测），Orin 负载本来就高
        self.tf_buf = None
        self.tf_listener = None
        self.sensor_subs = []
        self.pose_pub = self.create_publisher(PoseWithCovarianceStamped, '/initialpose', 10)
        self.status_pub = self.create_publisher(
            String, '~/status', QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.create_service(Trigger, '~/trigger', self._on_trigger)
        self.map_index = None
        self.run_event = threading.Event()
        self.busy = False
        self.stop = False
        if self.auto_start:
            self.run_event.set()
        self.worker = threading.Thread(target=self._worker, daemon=True)
        self.worker.start()

    # ---------- 回调 ----------
    def _on_scan(self, raw):
        stamp, xyz = livox_raw_to_xyz(raw)   # raw=True 订阅：直接解析 CDR，不经 rclpy 反序列化
        with self.lock:
            self.scans.append((stamp, xyz))

    def _on_imu(self, msg):
        a, g = msg.linear_acceleration, msg.angular_velocity
        with self.lock:
            self.imu.append((Time.from_msg(msg.header.stamp).nanoseconds * 1e-9,
                             (a.x, a.y, a.z), (g.x, g.y, g.z)))

    def _on_trigger(self, _req, resp):
        if self.busy:
            resp.success, resp.message = False, 'busy'
        else:
            self.run_event.set()
            resp.success, resp.message = True, 'started'
        return resp

    # ---------- 工具 ----------
    def _status(self, state, **kw):
        kw.update(state=state, stamp=round(time.time(), 3), dry_run=self.dry_run)
        txt = json.dumps(kw, ensure_ascii=False)
        self.status_pub.publish(String(data=txt))
        self.get_logger().info(txt)

    def _tf_on(self):
        if self.tf_listener is None:
            self.tf_buf = tf2_ros.Buffer()
            self.tf_listener = tf2_ros.TransformListener(self.tf_buf, self)

    def _tf_off(self):
        if self.tf_listener is not None:
            self.tf_listener.unregister()
            self.tf_listener = None

    def _sensors_on(self):
        with self.lock:
            self.scans.clear()
            self.imu.clear()
        be = ReliabilityPolicy.BEST_EFFORT
        self.sensor_subs = [
            self.create_subscription(CustomMsg, self.lidar_topic, self._on_scan, QoSProfile(depth=5, reliability=be),
                                     raw=True),
            self.create_subscription(Imu, self.imu_topic, self._on_imu, QoSProfile(depth=200, reliability=be),
                                     callback_group=self.imu_group)]

    def _sensors_off(self):
        for sub in self.sensor_subs:
            self.destroy_subscription(sub)
        self.sensor_subs = []

    def _lookup(self, parent, child, timeout=0.0):
        try:
            t = self.tf_buf.lookup_transform(parent, child, Time(), timeout=Duration(seconds=timeout))
        except Exception:
            return None
        tr, q = t.transform.translation, t.transform.rotation
        return np.array([tr.x, tr.y, tr.z]), quat_to_rot(q)

    def _current_base(self):
        r = self._lookup(self.map_frame, self.base_frame)
        if r is None:
            return None
        t, R = r
        return float(t[0]), float(t[1]), math.atan2(R[1, 0], R[0, 0])

    def _load_map(self):
        pcd = os.path.join(self.map_dir, 'global.pcd')
        up = (0.0, 0.0, 1.0)
        gu = os.path.join(self.map_dir, 'gravity_up.txt')
        if os.path.isfile(gu):
            vals = [float(v) for v in open(gu).read().split()]
            if len(vals) >= 3 and np.linalg.norm(vals[:3]) > 0.5:
                up = tuple(vals[:3])
        t0 = time.time()
        self.map_index = bev_search.MapIndex(load_pcd_xyz(pcd), self.sp, up)
        mi = self.map_index
        self.get_logger().info(
            f'map {pcd}: ground {mi.ground:.3f}, {mi.n_points} pts in band, coarse {mi.coarse.shape}, '
            f'fine {mi.fine.shape}, {time.time() - t0:.2f} s')

    def _collect(self, timeout=30.0):
        """等到最近 n_frames 帧期间 IMU 静止，返回 (摆平后的雷达系点, R_level, 窗口内最大角速度) 或 None"""
        self._sensors_on()
        try:
            return self._collect_static(timeout)
        finally:
            self._sensors_off()

    def _collect_static(self, timeout):
        t_end = time.time() + timeout
        self.collect_diag = {}
        while time.time() < t_end and not self.stop:
            with self.lock:
                scans = list(self.scans)[-self.n_frames:]
                imu = list(self.imu)
            # 失败时报出来：收到多少帧、IMU 与雷达时间戳差、窗口内最大角速度
            d = self.collect_diag
            d['scans'], d['imu'] = len(scans), len(imu)
            if scans and imu:
                d['imu_minus_scan_s'] = round(imu[-1][0] - scans[-1][0], 3)
            if len(scans) >= self.n_frames and imu:
                t0, t1 = scans[0][0] - 0.1, scans[-1][0]
                win = [m for m in imu if t0 <= m[0] <= t1 + 0.05]
                d['imu_in_window'] = len(win)
                if win:
                    d['gyro_max'] = round(float(np.max(np.linalg.norm([m[2] for m in win], axis=1))), 3)
                if len(win) >= 20:
                    gyro = np.array([m[2] for m in win])
                    gmax = float(np.max(np.linalg.norm(gyro, axis=1)))
                    if gmax <= self.gyro_max:
                        acc = np.array([m[1] for m in win]).mean(0)
                        R_level = bev_search.rot_to_up(acc)   # 静止时比力指向上
                        pts = np.concatenate([xyz for _, xyz in scans])
                        return pts @ R_level.T, R_level, gmax
            time.sleep(0.2)
        return None

    def _extrinsic(self):
        r = self._lookup(self.body_frame, self.lidar_frame, timeout=1.0)
        if r is None:
            return None
        t, R = r
        return R, t

    # ---------- 主流程 ----------
    def _worker(self):
        try:
            self._status('loading_map', map_dir=self.map_dir)
            self._load_map()
        except Exception as e:  # noqa: BLE001
            self._status('error', reason=f'map load failed: {e}')
            return
        while not self.stop:
            if not self.run_event.wait(timeout=0.5):
                continue
            self.run_event.clear()
            self.busy = True
            try:
                self._tf_on()
                self._run_once()
            except Exception as e:  # noqa: BLE001
                self._status('error', reason=repr(e))
            finally:
                self._tf_off()
                self.busy = False

    def _run_once(self):
        attempt = 0
        moved = 0
        while attempt < self.max_attempts and moved <= 3 and not self.stop:
            attempt += 1
            self._status('collecting', attempt=attempt)
            ext = None
            t_end = time.time() + 30.0
            while ext is None and time.time() < t_end and not self.stop:
                ext = self._extrinsic()
            if ext is None:
                self._status('error', reason=f'no TF {self.body_frame}->{self.lidar_frame}')
                return
            got = self._collect()
            if got is None:
                self._status('error', reason='no static scan window (robot moving or no lidar / imu)',
                             **getattr(self, 'collect_diag', {}))
                return
            pts, R_level, gmax = got
            self._status('searching', n_points=len(pts), gyro_max=round(gmax, 3))
            t0 = time.time()
            res = bev_search.search(self.map_index, pts, self.sp)
            dt = time.time() - t0
            info = res.summary()
            info['search_s'] = round(dt, 2)
            if not res.accepted:
                self._status('rejected', **info)
                return
            b = res.best
            # 地图若不是 +z 朝上，结果在摆平后的地图系，转回地图系（只转平面位置，倾角小时航向差可忽略）
            p = self.map_index.R_level.T @ np.array([b.x, b.y, 0.0])
            R_bl, t_bl = ext
            bx, by, byaw = lidar_pose_to_base(p[0], p[1], b.yaw, R_level, R_bl, t_bl)
            target = dict(x=round(bx, 3), y=round(by, 3), yaw_deg=round(math.degrees(byaw), 2))
            info['target_base'] = target

            # 等 lightning 自己的初始化有输出，对比
            cur = None
            t_end = time.time() + self.wait_loc_timeout
            while cur is None and time.time() < t_end and not self.stop:
                cur = self._current_base()
                if cur is None:
                    time.sleep(0.5)
            if cur is not None:
                dxy = math.hypot(cur[0] - bx, cur[1] - by)
                dyaw = yaw_diff_deg(cur[2], byaw)
                info['current_base'] = dict(x=round(cur[0], 3), y=round(cur[1], 3), yaw_deg=round(math.degrees(cur[2]), 2))
                info['current_err'] = dict(xy=round(dxy, 3), yaw_deg=round(dyaw, 2))
                if dxy <= self.agree_xy and dyaw <= self.agree_yaw_deg:
                    self._status('ok_already', **info)
                    return
            if self.dry_run:
                self._status('dry_run_would_publish', **info)
                return
            # 发的是"lightning 换算后雷达落在搜索结果上"的位姿，核对仍按真实 base 位姿
            px, py, pyaw = base_pose_for_lightning(p[0], p[1], byaw, t_bl)
            info['published_base'] = dict(x=round(px, 3), y=round(py, 3), yaw_deg=round(math.degrees(pyaw), 2))
            self._publish_initialpose(px, py, pyaw)
            self._status('published', attempt=attempt, **info)
            result, last = self._verify(bx, by, byaw)
            if result == 'ok':
                self._status('relocalized', attempt=attempt, final=last, **info)
                return
            if result == 'moved':
                # 核对期间机器人动了，静止目标没法比；重新采集一次（不计入失败次数），lightning 跟对了会判 ok_already
                self._status('verify_interrupted_moving', attempt=attempt, final=last, **info)
                attempt -= 1
                moved += 1
                continue
            self._status('verify_failed', attempt=attempt, final=last, **info)
            time.sleep(self.retry_wait)
        self._status('failed', reason=f'not converged after {attempt} attempts ({moved} interrupted by motion)')

    def _publish_initialpose(self, x, y, yaw):
        m = PoseWithCovarianceStamped()
        m.header.frame_id = self.map_frame
        m.header.stamp = self.get_clock().now().to_msg()
        m.pose.pose.position.x = float(x)
        m.pose.pose.position.y = float(y)
        m.pose.pose.orientation.z = math.sin(yaw / 2)
        m.pose.pose.orientation.w = math.cos(yaw / 2)
        cov = [0.0] * 36
        cov[0] = cov[7] = 0.1 ** 2
        cov[35] = math.radians(3) ** 2
        m.pose.covariance = cov
        self.pose_pub.publish(m)

    def _verify(self, x, y, yaw):
        """lightning 重定位后输出进 verify_xy / verify_yaw_deg 并稳住 1 s → 'ok'；期间 IMU 显示在动 → 'moved'；超时 → 'timeout'。
        lightning 收到 /initialpose 后约 0.6 s 完成重定位（本地联调 glog）。"""
        t_start = time.time()
        t_end = t_start + self.verify_timeout
        ok_since = None
        last = None
        with self.lock:
            self.imu.clear()
        sub = self.create_subscription(Imu, self.imu_topic, self._on_imu,
                                       QoSProfile(depth=200, reliability=ReliabilityPolicy.BEST_EFFORT),
                                       callback_group=self.imu_group)
        try:
            time.sleep(0.3)
            while time.time() < t_end and not self.stop:
                with self.lock:
                    gyro = [m[2] for m in self.imu]
                if gyro and float(np.max(np.linalg.norm(np.array(gyro), axis=1))) > self.gyro_max:
                    return 'moved', last
                cur = self._current_base()
                if cur is not None:
                    dxy = math.hypot(cur[0] - x, cur[1] - y)
                    dyaw = yaw_diff_deg(cur[2], yaw)
                    last = dict(x=round(cur[0], 3), y=round(cur[1], 3), yaw_deg=round(math.degrees(cur[2]), 2),
                                err_xy=round(dxy, 3), err_yaw_deg=round(dyaw, 2), after_s=round(time.time() - t_start, 1))
                    if dxy <= self.verify_xy and dyaw <= self.verify_yaw_deg:
                        ok_since = ok_since or time.time()
                        if time.time() - ok_since >= 1.0:
                            return 'ok', last
                    else:
                        ok_since = None
                time.sleep(0.1)
            return 'timeout', last
        finally:
            self.destroy_subscription(sub)


def main():
    rclpy.init()
    node = GlobalInitNode()
    ex = MultiThreadedExecutor(num_threads=2)
    ex.add_node(node)
    try:
        ex.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.stop = True
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
