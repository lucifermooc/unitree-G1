#!/usr/bin/env python3
"""重规划仿真：规划后才看到的障碍，旧行为树（只在路径失效时重规划）vs 新行为树（每 2 s 或失效时重规划）。

障碍走 obstacle_layer_sim.yaml 的点云层，只在离机器人 2.5 m 内标进 costmap（= 实机 STVL 标记距离）；
"static" 场景的障碍直接画在地图里，规划一开始就看得到，用来单独检验规划器本身会不会贴障碍。
机器人用 g1_orin_sim.py（Orin 30524 实测模型，当前部署的 A + 原地转 0.9 + 蟹行航向补偿）。

python3 -B run_replan_test.py [--scenes ...] [--bts old new] [--seeds 0 1] [--jobs 5]
结果：results/replan_<时间>/summary.csv、每次的 rec_*.npz、地图和 launch 日志。
"""
import argparse
import csv
import math
import os
import signal
import subprocess
import time
from concurrent.futures import ThreadPoolExecutor

import numpy as np
import rclpy
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped, Twist
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import Path
from rcl_interfaces.msg import Log
from rclpy.action import ActionClient
from rclpy.executors import SingleThreadedExecutor
from rclpy.qos import QoSDurabilityPolicy, QoSProfile
from scipy import ndimage
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Header

HERE = os.path.dirname(os.path.realpath(__file__))
SRC = os.path.realpath(os.path.join(HERE, "..", ".."))
BT_REL = "aid_navigation2/behavior_trees/navigate_w_recovery_and_replanning_only_if_path_becomes_invalid.xml"
OLD_BT_REV = "51acdfe"   # 加定时重规划之前的最后一个提交，"旧树"从这里取
os.environ["ROS_LOCALHOST_ONLY"] = "1"
RES = 0.05
ORIGIN = (-1.0, -5.0)          # 地图左下角
SIZE = (18.0, 10.5)            # 宽 x 高（m）
SIM_ARGS = ("-p bridge.mode:=walkmin -p bridge.walk_min_wz:=0.0 -p bridge.walk_wz_stop:=0.0 -p bridge.smooth_s:=0.0 "
            "-p model.yaw_comp_deg:=9.5")
CTRL = os.path.join(HERE, "ctrl_rot09.yaml")
STATUS = {GoalStatus.STATUS_SUCCEEDED: "OK", GoalStatus.STATUS_ABORTED: "ABORTED", GoalStatus.STATUS_CANCELED: "CANCELED"}


def box(x0, x1, y0, y1):
    return (x0, x1, y0, y1)


def corridor_walls():
    """3.0 m 宽、16 m 长的走廊：除走廊以外全是墙。"""
    return dict(free=[box(0.0, 16.0, -1.5, 1.5)], walls=[])


def room_walls():
    return dict(free=[box(0.0, 16.0, -4.0, 4.5)], walls=[])


GAP_BLOCKS = [box(6.0, 9.0, 1.0, 4.5), box(6.0, 9.0, -4.0, -1.0)]   # 两块大障碍夹出 2.0 m 宽的通道（截图里的形状）


def side_box(d):
    """路径（走廊中线 y=0）左侧一个 0.5 x 0.5 箱子，近边离中线 d。"""
    return [box(7.0, 7.5, d, d + 0.5)]


# 名字: (地图, 画进地图的障碍, 只有传感器能看到的障碍, 起点, 终点, robot_radius)
SCENES = {
    "侧箱0.55": (corridor_walls(), [], side_box(0.55), (1.5, 0.0, 0.0), (14.5, 0.0, 0.0), 0.5),
    "侧箱0.70": (corridor_walls(), [], side_box(0.70), (1.5, 0.0, 0.0), (14.5, 0.0, 0.0), 0.5),
    "侧箱0.45_r0.4": (corridor_walls(), [], side_box(0.45), (1.5, 0.0, 0.0), (14.5, 0.0, 0.0), 0.4),
    "通道_地图里": (room_walls(), GAP_BLOCKS, [], (1.5, 2.5, 0.0), (14.5, 2.5, 0.0), 0.5),
    "通道_传感器": (room_walls(), [], GAP_BLOCKS, (1.5, 2.5, 0.0), (14.5, 2.5, 0.0), 0.5),
    "通道_传感器_r0.4": (room_walls(), [], GAP_BLOCKS, (1.5, 2.5, 0.0), (14.5, 2.5, 0.0), 0.4),
}


def grid_shape():
    return int(round(SIZE[1] / RES)), int(round(SIZE[0] / RES))


def rasterize(boxes, occ, value):
    for x0, x1, y0, y1 in boxes:
        c0, c1 = int(round((x0 - ORIGIN[0]) / RES)), int(round((x1 - ORIGIN[0]) / RES))
        r0, r1 = int(round((y0 - ORIGIN[1]) / RES)), int(round((y1 - ORIGIN[1]) / RES))
        occ[max(r0, 0):r1, max(c0, 0):c1] = value


def static_occ(scene):
    walls, drawn = SCENES[scene][0], SCENES[scene][1]
    occ = np.ones(grid_shape(), bool)            # 行 0 = y 最小
    rasterize(walls["free"], occ, False)
    rasterize(drawn, occ, True)
    return occ


def write_map(scene, out_dir):
    occ = static_occ(scene)
    img = np.where(occ, 0, 254).astype(np.uint8)[::-1]
    name = f"map_{scene}"
    with open(os.path.join(out_dir, name + ".pgm"), "wb") as f:
        f.write(b"P5\n%d %d\n255\n" % (img.shape[1], img.shape[0]))
        f.write(img.tobytes())
    with open(os.path.join(out_dir, name + ".yaml"), "w") as f:
        f.write(f"image: {name}.pgm\nmode: trinary\nresolution: {RES}\norigin: [{ORIGIN[0]}, {ORIGIN[1]}, 0]\n"
                "negate: 0\noccupied_thresh: 0.65\nfree_thresh: 0.196\n")
    return os.path.join(out_dir, name + ".yaml")


def clearance_grid(scene):
    """全部障碍（地图 + 传感器）到每格的距离（m）。"""
    occ = static_occ(scene)
    rasterize(SCENES[scene][2], occ, True)
    return ndimage.distance_transform_edt(~occ) * RES


def clear_at(dist, pts):
    c = np.clip(((pts[:, 0] - ORIGIN[0]) / RES).astype(int), 0, dist.shape[1] - 1)
    r = np.clip(((pts[:, 1] - ORIGIN[1]) / RES).astype(int), 0, dist.shape[0] - 1)
    return dist[r, c]


def obstacle_cloud(boxes):
    pts = []
    for x0, x1, y0, y1 in boxes:
        xs = np.arange(x0 + RES / 2, x1, RES)
        ys = np.arange(y0 + RES / 2, y1, RES)
        X, Y = np.meshgrid(xs, ys)
        pts.append(np.stack([X.ravel(), Y.ravel(), np.full(X.size, 0.5)], 1))
    return np.concatenate(pts).astype(np.float32) if pts else np.zeros((0, 3), np.float32)


def yaw_of(q):
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


def run_one(scene, bt, seed, domain, out_dir, bt_files, timeout):
    walls, drawn, sensed, start, goal_xy, radius = SCENES[scene]
    map_yaml = write_map(scene, out_dir)
    tag = f"{scene}_{bt}_s{seed}"
    log = open(os.path.join(out_dir, f"launch_{tag}.log"), "w")
    cmd = ["ros2", "launch", os.path.join(HERE, "sim_orin_launch.py"), f"map:={map_yaml}",
           f"x:={start[0]}", f"y:={start[1]}", f"yaw_deg:={start[2]}", f"sim_args:={SIM_ARGS} -p model.seed:={seed}",
           f"controller_extra:={CTRL}", f"bt_xml:={bt_files[bt]}",
           "costmap_extra:=" + (combined_extra(radius, out_dir) if radius != 0.5
                                else os.path.join(HERE, "obstacle_layer_sim.yaml"))]
    env = dict(os.environ, ROS_DOMAIN_ID=str(domain), ROS_LOCALHOST_ONLY="1")
    proc = subprocess.Popen(cmd, env=env, stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
    ctx = rclpy.context.Context()
    rclpy.init(context=ctx, domain_id=domain)
    node = rclpy.create_node(f"replan_tester_{domain}", context=ctx)
    ex = SingleThreadedExecutor(context=ctx)
    ex.add_node(node)
    rec = dict(truth=[], cmd=[], plans=[], logs=[])
    now = time.time
    node.create_subscription(PoseStamped, "sim/truth", lambda m: rec["truth"].append(
        (now(), m.pose.position.x, m.pose.position.y, yaw_of(m.pose.orientation))), 50)
    node.create_subscription(Twist, "cmd_vel", lambda m: rec["cmd"].append((now(), m.linear.x, m.angular.z)), 50)
    node.create_subscription(Path, "plan", lambda m: rec["plans"].append(
        (now(), np.array([(p.pose.position.x, p.pose.position.y) for p in m.poses]))), 10)
    node.create_subscription(Log, "/rosout", lambda m: rec["logs"].append((now(), m.name, m.msg)),
                             QoSProfile(depth=200, durability=QoSDurabilityPolicy.VOLATILE))
    cloud_pub = node.create_publisher(PointCloud2, "/sim/obstacles", 10)
    cloud = obstacle_cloud(sensed)

    def pub_cloud():
        if len(cloud):
            cloud_pub.publish(point_cloud2.create_cloud_xyz32(
                Header(frame_id="map", stamp=node.get_clock().now().to_msg()), cloud))
    node.create_timer(0.2, pub_cloud)
    client = ActionClient(node, NavigateToPose, "navigate_to_pose")

    def spin_for(sec, until=None):
        end = time.time() + sec
        while time.time() < end:
            ex.spin_once(timeout_sec=0.02)
            if until and until():
                return True
        return False

    out = dict(scene=scene, bt=bt, seed=seed)
    try:
        if not client.wait_for_server(timeout_sec=60.0):
            out["status"] = "NO_SERVER"
            return out, rec
        spin_for(3.0)
        g = NavigateToPose.Goal()
        g.pose.header.frame_id = "map"
        g.pose.pose.position.x, g.pose.pose.position.y = goal_xy[0], goal_xy[1]
        g.pose.pose.orientation.z = math.sin(math.radians(goal_xy[2]) / 2)
        g.pose.pose.orientation.w = math.cos(math.radians(goal_xy[2]) / 2)
        handle = None
        for _ in range(30):
            fut = client.send_goal_async(g)
            spin_for(5.0, until=fut.done)
            if fut.done() and fut.result().accepted:
                handle = fut.result()
                break
            spin_for(1.0)
        if handle is None:
            out["status"] = "REJECTED"
            return out, rec
        t0 = time.time()
        res = handle.get_result_async()
        spin_for(timeout, until=res.done)
        out["status"] = STATUS.get(res.result().status, str(res.result().status)) if res.done() else "TIMEOUT"
        if not res.done():
            handle.cancel_goal_async()
        out["time_s"] = round(time.time() - t0, 1)
        out["t0"] = t0
        spin_for(1.0)
        return out, rec
    finally:
        node.destroy_node()
        rclpy.shutdown(context=ctx)
        os.killpg(proc.pid, signal.SIGINT)
        try:
            proc.wait(timeout=20)
        except subprocess.TimeoutExpired:
            os.killpg(proc.pid, signal.SIGKILL)
        log.close()


def combined_extra(radius, out_dir):
    """obstacle_layer_sim.yaml + 两张图的 robot_radius（网页"避障半径"只改这一项，膨胀仍 0.85）。"""
    import yaml
    d = yaml.safe_load(open(os.path.join(HERE, "obstacle_layer_sim.yaml")))
    for cm in ("local_costmap", "global_costmap"):
        d[cm][cm]["ros__parameters"]["robot_radius"] = radius
    f = os.path.join(out_dir, f"obstacle_layer_r{radius}.yaml")
    yaml.safe_dump(d, open(f, "w"), allow_unicode=True)
    return f


def analyze(out, rec, dist):
    tru = np.array(rec["truth"]) if rec["truth"] else np.zeros((0, 4))
    t0 = out.get("t0", 0)
    plans = [(t, P) for t, P in rec["plans"] if len(P) > 1 and t >= t0 - 1]
    out["plans"] = len(plans)
    # 机器人实际离障碍
    if len(tru):
        c = clear_at(dist, tru[:, 1:3])
        out["robot_min"] = float(c.min())
        dt = np.diff(tru[:, 0], prepend=tru[0, 0])
        out["robot_lt0.6_s"] = float(dt[c < 0.6].sum())
        out["robot_lt0.7_s"] = float(dt[c < 0.7].sum())
    # 机器人正在跟的路径，在机器人附近 2 m 内离障碍最近多少（= 截图里看到的"路径贴障碍"）
    pt = [t for t, _ in plans]
    follow = []
    for t, x, y, _ in tru[::5]:
        j = np.searchsorted(pt, t) - 1
        if j < 0:
            continue
        P = plans[j][1]
        near = P[np.hypot(P[:, 0] - x, P[:, 1] - y) < 2.0]
        if len(near):
            follow.append(clear_at(dist, near).min())
    if follow:
        out["path_ahead_min"] = float(np.min(follow))
    if plans:
        out["first_plan_min"] = float(clear_at(dist, plans[0][1]).min())
        out["last_plan_min"] = float(clear_at(dist, plans[-1][1]).min())
    cmd = np.array(rec["cmd"]) if rec["cmd"] else np.zeros((0, 3))
    spin = (np.abs(cmd[:, 1]) < 0.05) & (np.abs(cmd[:, 2]) > 0.3)
    out["spins"] = int(np.sum(spin[1:] & ~spin[:-1])) if len(spin) > 1 else 0
    msgs = [m for _, _, m in rec["logs"]]
    out["collision_ahead"] = sum("ollision" in m and "ahead" in m.lower() for m in msgs)
    out["recoveries"] = sum(("Running spin" in m) or ("Running wait" in m) or ("Running backup" in m) for m in msgs)
    return out


def fmt(r):
    g = lambda k, f=".2f": format(r[k], f) if k in r and r[k] == r[k] else "-"  # noqa: E731
    return (f"{r['scene']:16s} {r['bt']:3s} s{r['seed']} | {r.get('status', '-'):8s} {g('time_s', '.0f')} s | "
            f"路径条数 {r.get('plans', '-')} | 首条路径离障碍 {g('first_plan_min')} 跟随中路径最近 {g('path_ahead_min')} | "
            f"机器人最近 {g('robot_min')} <0.6m {g('robot_lt0.6_s', '.1f')} s <0.7m {g('robot_lt0.7_s', '.1f')} s | "
            f"原地转 {r.get('spins', '-')} collision ahead {r.get('collision_ahead', '-')} 恢复 {r.get('recoveries', '-')}")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--scenes", nargs="+", default=list(SCENES), choices=list(SCENES))
    ap.add_argument("--bts", nargs="+", default=["old", "new"], choices=["old", "new"])
    ap.add_argument("--seeds", nargs="+", type=int, default=[0])
    ap.add_argument("--jobs", type=int, default=5)
    ap.add_argument("--domain", type=int, default=110)
    ap.add_argument("--timeout", type=float, default=150.0)
    a = ap.parse_args()
    out_dir = os.path.join(HERE, "results", time.strftime("replan_%Y%m%d_%H%M%S"))
    os.makedirs(out_dir)
    bt_files = {"new": os.path.join(SRC, BT_REL), "old": os.path.join(out_dir, "bt_old.xml")}
    old = subprocess.run(["git", "show", f"{OLD_BT_REV}:lightning_ws/src/{BT_REL}"], cwd=SRC, capture_output=True, text=True,
                         check=True)
    assert "PathExpiringTimer" not in old.stdout and "PathExpiringTimer" in open(bt_files["new"]).read()
    open(bt_files["old"], "w").write(old.stdout)
    jobs = [(s, b, sd) for s in a.scenes for sd in a.seeds for b in a.bts]

    def work(i_job):
        i, (scene, bt, seed) = i_job
        out, rec = run_one(scene, bt, seed, a.domain + i, out_dir, bt_files, a.timeout)
        out = analyze(out, rec, clearance_grid(scene))
        np.savez_compressed(os.path.join(out_dir, f"rec_{scene}_{bt}_s{seed}.npz"), truth=np.array(rec["truth"]),
                            cmd=np.array(rec["cmd"]), plan_t=np.array([t for t, _ in rec["plans"]]),
                            plans=np.array([P for _, P in rec["plans"]], dtype=object), allow_pickle=True)
        print(fmt(out), flush=True)
        return out

    with ThreadPoolExecutor(max_workers=a.jobs) as pool:
        rows = list(pool.map(work, enumerate(jobs)))
    keys = ["scene", "bt", "seed", "status", "time_s", "plans", "first_plan_min", "path_ahead_min", "last_plan_min",
            "robot_min", "robot_lt0.6_s", "robot_lt0.7_s", "spins", "collision_ahead", "recoveries"]
    with open(os.path.join(out_dir, "summary.csv"), "w", newline="") as f:
        w = csv.DictWriter(f, fieldnames=keys, extrasaction="ignore")
        w.writeheader()
        for r in rows:
            w.writerow({k: (round(v, 3) if isinstance(v, float) else v) for k, v in r.items()})
    print("\n汇总（离障碍 = 到最近障碍格中心的距离，m；障碍含只有传感器看得到的）")
    for r in rows:
        print(fmt(r))
    print(f"results: {out_dir}")


if __name__ == "__main__":
    main()
