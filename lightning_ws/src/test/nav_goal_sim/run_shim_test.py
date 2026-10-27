#!/usr/bin/env python3
"""控制器对比仿真：现状 RPP（ctrl_rot09.yaml）vs shim（ctrl_shim.yaml：远处 MPPI、近目标 RPP，参考轮式 128.23）。

障碍走 obstacle_scan_sim.yaml 的"激光"层：本脚本按定位位姿对 地图墙 + 场景障碍（含按时间移动的动态障碍）做 360° 射线求交，
发 /sim/scan（base_link 系），标记 2.5 m、射线清除 3.0 m，障碍走开后格子会被清掉。
机器人用 g1_orin_sim.py（Orin 30524 实测模型，当前部署的 A + 原地转 0.9 + 蟹行航向补偿），行为树用仓库当前的（每 2 s 或失效时重规划）。

python3 -B run_shim_test.py [--scenes ...] [--ctrls rpp shim] [--seeds 0 1] [--jobs 6] [--set 名称:参数=值,...]
--set 同 run_scurve_test.py：名称 = 基础控制器+后缀（如 shim_settle+BD），在 FollowPath 下改参数作为一个新变体。
需要先编译 aid_nav_shim_controller（humble），并用 --shim-install 指向它的 install 目录。
结果：results/shim_<时间>/summary.csv、每次的 rec_*.npz 和 launch 日志。
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
import yaml
from geometry_msgs.msg import PoseStamped, Twist
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import Path
from rcl_interfaces.msg import Log
from rclpy.action import ActionClient
from rclpy.executors import SingleThreadedExecutor
from rclpy.qos import QoSDurabilityPolicy, QoSProfile
from scipy import ndimage
from sensor_msgs.msg import LaserScan

import run_replan_test as rr

HERE = rr.HERE
RES, ORIGIN = rr.RES, rr.ORIGIN
CTRLS = {"rpp": os.path.join(HERE, "ctrl_rot09.yaml"), "shim": os.path.join(HERE, "ctrl_shim.yaml"),
         "shim_settle": os.path.join(HERE, "ctrl_shim_settle.yaml")}
COSTMAP_EXTRA = os.path.join(HERE, "obstacle_scan_sim.yaml")
SCAN_N, SCAN_MAX, SCAN_STEP = 360, 3.5, 0.025
BODY_R = 0.25        # G1 机身实际半宽（肩宽约 0.45~0.5 m）；骨盆离障碍 < 0.25 m 记为"碰上"


def corridor(w):
    return dict(free=[rr.box(0.0, 16.0, -w / 2, w / 2)], walls=[])


def mover(size, track):
    """动态障碍：size=(宽 x, 高 y)，track=[(t, cx, cy), ...]（t 从目标被接受算起），线性插值，两端保持。"""
    return dict(size=size, track=np.array(track, float))


def mover_box(m, t):
    tr = m["track"]
    cx = np.interp(t, tr[:, 0], tr[:, 1])
    cy = np.interp(t, tr[:, 0], tr[:, 2])
    w, h = m["size"]
    return (cx - w / 2, cx + w / 2, cy - h / 2, cy + h / 2)


# 名字: (地图墙, 画进地图的障碍, 只有传感器看得到的静止障碍, 动态障碍, 起点, 终点)
SCENES = {
    # 规划后才看到的侧箱（= run_replan_test 的 侧箱0.55）
    "侧箱0.55": (corridor(3.0), [], rr.side_box(0.55), [], (1.5, 0.0, 0.0), (14.5, 0.0, 0.0)),
    # 箱子从左墙边横推进走廊，近边停在离中线 0.2 m（6~12 s 推进；机器人约 15 s 到 x=7）
    "侧推0.2": (corridor(3.0), [], [], [mover((0.5, 0.5), [(6, 7.25, 1.25), (12, 7.25, 0.45)])],
               (1.5, 0.0, 0.0), (14.5, 0.0, 0.0)),
    # 迎面来人（0.4 m 方块）沿 y=0.15 以 0.5 m/s 走过来，一直走出走廊
    "迎面来人": (corridor(3.0), [], [], [mover((0.4, 0.4), [(2, 14.0, 0.15), (32, -1.0, 0.15)])],
              (1.5, 0.0, 0.0), (14.5, 0.0, 0.0)),
    # 人从下方横穿走廊（0.5 m/s），大约在机器人前方 1 m 处穿过
    "横穿": (corridor(3.0), [], [], [mover((0.4, 0.4), [(10, 7.0, -1.3), (16, 7.0, 1.3)])],
            (1.5, 0.0, 0.0), (14.5, 0.0, 0.0)),
    # 两块大障碍夹 2.0 m 通道，只有传感器看得到（= run_replan_test 的 通道_传感器）
    "通道_传感器": (rr.room_walls(), [], rr.GAP_BLOCKS, [], (1.5, 2.5, 0.0), (14.5, 2.5, 0.0)),
    # 1.6 m 窄走廊，侧箱把通道收窄到 1.15 m（机器人中心可走的带只剩 0.15 m）
    "窄道收窄": (corridor(1.6), [], [rr.box(7.0, 7.5, 0.35, 0.8)], [], (1.5, 0.0, 0.0), (14.5, 0.0, 0.0)),
    # 目标在侧箱旁 0.7 m（近目标阶段 RPP 接手时障碍就在身边）
    "目标贴箱": (corridor(3.0), [], [rr.box(9.0, 9.5, 0.3, 0.8)], [], (1.5, 0.0, 0.0), (9.25, -0.45, 0.0)),
}


def static_occ(scene):
    walls, drawn = SCENES[scene][0], SCENES[scene][1]
    occ = np.ones(rr.grid_shape(), bool)
    rr.rasterize(walls["free"], occ, False)
    rr.rasterize(drawn, occ, True)
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


def world_occ(scene, t):
    """t 时刻全部障碍（墙 + 地图障碍 + 传感器静止障碍 + 动态障碍）。"""
    occ = static_occ(scene)
    rr.rasterize(SCENES[scene][2], occ, True)
    rr.rasterize([mover_box(m, t) for m in SCENES[scene][3]], occ, True)
    return occ


def raycast(occ, x, y, yaw):
    ang = yaw + np.linspace(-math.pi, math.pi, SCAN_N, endpoint=False)
    s = np.arange(SCAN_STEP, SCAN_MAX, SCAN_STEP)
    px = x + np.cos(ang)[:, None] * s[None, :]
    py = y + np.sin(ang)[:, None] * s[None, :]
    c = ((px - ORIGIN[0]) / RES).astype(int)
    r = ((py - ORIGIN[1]) / RES).astype(int)
    inside = (c >= 0) & (c < occ.shape[1]) & (r >= 0) & (r < occ.shape[0])
    hit = np.zeros_like(inside)
    hit[inside] = occ[r[inside], c[inside]]
    first = np.where(hit.any(1), hit.argmax(1), -1)
    return np.where(first >= 0, s[np.maximum(first, 0)], np.inf)   # 没打到 = inf（inf_is_valid: true 时用来清除）


def box_dist(b, pts):
    x0, x1, y0, y1 = b
    dx = np.maximum(np.maximum(x0 - pts[:, 0], 0), pts[:, 0] - x1)
    dy = np.maximum(np.maximum(y0 - pts[:, 1], 0), pts[:, 1] - y1)
    return np.hypot(dx, dy)


def clearance(scene, tru, t0):
    """每个真实位姿到最近障碍（边界）的距离：静止障碍用距离变换，动态障碍按当时位置解析算。"""
    occ = static_occ(scene)
    rr.rasterize(SCENES[scene][2], occ, True)
    dist = ndimage.distance_transform_edt(~occ) * RES
    pts = tru[:, 1:3]
    d = rr.clear_at(dist, pts)
    for m in SCENES[scene][3]:
        tt = tru[:, 0] - t0
        tr = m["track"]
        cx, cy = np.interp(tt, tr[:, 0], tr[:, 1]), np.interp(tt, tr[:, 0], tr[:, 2])
        w, h = m["size"]
        dx = np.maximum(np.abs(pts[:, 0] - cx) - w / 2, 0)
        dy = np.maximum(np.abs(pts[:, 1] - cy) - h / 2, 0)
        d = np.minimum(d, np.hypot(dx, dy))
    return d


def ctrl_file(name, sets, out_dir):
    src = CTRLS[name.split("+")[0]]
    if name not in sets:
        return src
    doc = yaml.safe_load(open(src))
    doc["controller_server"]["ros__parameters"]["FollowPath"].update(sets[name])
    path = os.path.join(out_dir, f"ctrl_{name}.yaml")
    with open(path, "w") as f:
        yaml.safe_dump(doc, f, allow_unicode=True)
    return path


def run_one(scene, ctrl, seed, domain, out_dir, shim_install, timeout, ctrl_yaml):
    walls, drawn, sensed, movers, start, goal = SCENES[scene]
    map_yaml = write_map(scene, out_dir)
    tag = f"{scene}_{ctrl}_s{seed}"
    log = open(os.path.join(out_dir, f"launch_{tag}.log"), "w")
    cmd = ["ros2", "launch", os.path.join(HERE, "sim_orin_launch.py"), f"map:={map_yaml}",
           f"x:={start[0]}", f"y:={start[1]}", f"yaw_deg:={start[2]}", f"sim_args:={rr.SIM_ARGS} -p model.seed:={seed}",
           f"controller_extra:={ctrl_yaml}", f"bt_xml:={os.path.join(rr.SRC, rr.BT_REL)}",
           f"costmap_extra:={COSTMAP_EXTRA}"]
    env = dict(os.environ, ROS_DOMAIN_ID=str(domain), ROS_LOCALHOST_ONLY="1")
    if shim_install:
        env["AMENT_PREFIX_PATH"] = shim_install + ":" + env.get("AMENT_PREFIX_PATH", "")
        env["LD_LIBRARY_PATH"] = os.path.join(shim_install, "lib") + ":" + env.get("LD_LIBRARY_PATH", "")
    proc = subprocess.Popen(cmd, env=env, stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
    ctx = rclpy.context.Context()
    rclpy.init(context=ctx, domain_id=domain)
    node = rclpy.create_node(f"shim_tester_{domain}", context=ctx)
    ex = SingleThreadedExecutor(context=ctx)
    ex.add_node(node)
    rec = dict(truth=[], cmd=[], plans=[], logs=[])
    st = dict(t0=None, pose=None)
    now = time.time

    def on_pub(m):
        st["pose"] = m
    node.create_subscription(PoseStamped, "sim/truth", lambda m: rec["truth"].append(
        (now(), m.pose.position.x, m.pose.position.y, rr.yaw_of(m.pose.orientation))), 50)
    node.create_subscription(PoseStamped, "sim/pub_pose", on_pub, 10)
    node.create_subscription(Twist, "cmd_vel", lambda m: rec["cmd"].append((now(), m.linear.x, m.angular.z)), 50)
    node.create_subscription(Path, "plan", lambda m: rec["plans"].append(
        (now(), np.array([(p.pose.position.x, p.pose.position.y) for p in m.poses]))), 10)
    node.create_subscription(Log, "/rosout", lambda m: rec["logs"].append((now(), m.name, m.msg)),
                             QoSProfile(depth=200, durability=QoSDurabilityPolicy.VOLATILE))
    scan_pub = node.create_publisher(LaserScan, "/sim/scan", 10)

    def pub_scan():
        p = st["pose"]
        if p is None:
            return
        t = 0.0 if st["t0"] is None else now() - st["t0"]
        rng = raycast(world_occ(scene, t), p.pose.position.x, p.pose.position.y, rr.yaw_of(p.pose.orientation))
        s = LaserScan()
        s.header.stamp, s.header.frame_id = p.header.stamp, "base_link"
        s.angle_min, s.angle_increment = -math.pi, 2 * math.pi / SCAN_N
        s.angle_max = s.angle_min + s.angle_increment * (SCAN_N - 1)
        s.range_min, s.range_max = 0.05, SCAN_MAX
        s.ranges = rng.astype(np.float32).tolist()
        scan_pub.publish(s)
    node.create_timer(0.1, pub_scan)
    client = ActionClient(node, NavigateToPose, "navigate_to_pose")

    def spin_for(sec, until=None):
        end = time.time() + sec
        while time.time() < end:
            ex.spin_once(timeout_sec=0.02)
            if until and until():
                return True
        return False

    out = dict(scene=scene, ctrl=ctrl, seed=seed)
    try:
        if not client.wait_for_server(timeout_sec=60.0):
            out["status"] = "NO_SERVER"
            return out, rec
        spin_for(3.0)
        g = NavigateToPose.Goal()
        g.pose.header.frame_id = "map"
        g.pose.pose.position.x, g.pose.pose.position.y = goal[0], goal[1]
        g.pose.pose.orientation.z = math.sin(math.radians(goal[2]) / 2)
        g.pose.pose.orientation.w = math.cos(math.radians(goal[2]) / 2)
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
        t0 = st["t0"] = time.time()
        res = handle.get_result_async()
        spin_for(timeout, until=res.done)
        out["status"] = rr.STATUS.get(res.result().status, str(res.result().status)) if res.done() else "TIMEOUT"
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


def analyze(out, rec):
    scene = out["scene"]
    goal = SCENES[scene][5]
    tru = np.array(rec["truth"]) if rec["truth"] else np.zeros((0, 4))
    t0 = out.get("t0")
    if t0 is None or not len(tru):
        return out
    tru = tru[tru[:, 0] >= t0]
    c = clearance(scene, tru, t0)
    dt = np.diff(tru[:, 0], prepend=tru[0, 0])
    out["robot_min"] = float(c.min())
    out["lt0.5_s"] = float(dt[c < 0.5].sum())
    out["contact_s"] = float(dt[c < BODY_R].sum())
    out["goal_err"] = float(math.hypot(tru[-1, 1] - goal[0], tru[-1, 2] - goal[1]))
    out["yaw_err_deg"] = float(abs(math.degrees(math.atan2(math.sin(tru[-1, 3] - math.radians(goal[2])),
                                                           math.cos(tru[-1, 3] - math.radians(goal[2]))))))
    out["path_len"] = float(np.hypot(*np.diff(tru[:, 1:3], axis=0).T).sum())
    out["plans"] = sum(1 for t, P in rec["plans"] if len(P) > 1 and t >= t0 - 1)
    cmd = np.array(rec["cmd"]) if rec["cmd"] else np.zeros((0, 3))
    cmd = cmd[cmd[:, 0] >= t0] if len(cmd) else cmd
    stop = (np.abs(cmd[:, 1]) < 0.01) & (np.abs(cmd[:, 2]) < 0.01) if len(cmd) else np.zeros(0, bool)
    out["stopped_s"] = float(np.sum(stop) / 10.0)   # cmd_vel ~10 Hz
    msgs = [m for t, _, m in rec["logs"] if t >= t0]
    out["collision_ahead"] = sum("ollision" in m and "ahead" in m.lower() for m in msgs)
    out["recoveries"] = sum(("Running spin" in m) or ("Running wait" in m) or ("Running backup" in m) for m in msgs)
    out["to_near"] = sum("-> near" in m for m in msgs)
    out["rotates"] = sum("-> rotate" in m for m in msgs)
    out["near_fallback"] = sum("near controller failed" in m for m in msgs)
    out["mppi_fail"] = sum("Optimizer fail" in m or "fail to compute" in m.lower() for m in msgs)
    out["stalls"] = sum("no progress along path" in m for m in msgs)
    out["settle"] = sum("NavShim settle:" in m for m in msgs)
    out["settle_skip"] = sum("NavShim settle skipped" in m for m in msgs)
    out["debounce"] = sum("NavShim debounce" in m for m in msgs)
    # 顿挫（同 run_scurve_test.py）：走 <-> 转 直接换 + 停不到 1 s 又动（按 /cmd_vel，离起点 / 终点 0.8 m 内不算）
    stomp = 0
    if len(cmd):
        c = np.where(cmd[:, 1] >= 0.05, 1, np.where(cmd[:, 1] <= -0.05, 2, np.where(
            np.abs(cmd[:, 2]) >= 0.15, np.where(cmd[:, 2] > 0, 3, 4), 0)))
        segs = []
        i = 0
        while i < len(c):
            j = i
            while j + 1 < len(c) and c[j + 1] == c[i]:
                j += 1
            segs.append((cmd[i, 0], cmd[j, 0] + 0.1, c[i]))
            i = j + 1
        start = SCENES[scene][4]
        for k in range(1, len(segs)):
            t, a, b = segs[k][0], segs[k - 1][2], segs[k][2]
            p = tru[min(np.searchsorted(tru[:, 0], t), len(tru) - 1), 1:3]
            if min(math.hypot(p[0] - goal[0], p[1] - goal[1]), math.hypot(p[0] - start[0], p[1] - start[1])) < 0.8:
                continue
            if b != 0 and a != 0 and a != b:
                stomp += 1
            elif b != 0 and a == 0 and segs[k - 1][1] - segs[k - 1][0] < 1.0 and k >= 2 and segs[k - 2][2] != 0:
                stomp += 1
    out["stomp"] = stomp
    return out


def fmt(r):
    g = lambda k, f=".2f": format(r[k], f) if k in r and r[k] == r[k] else "-"  # noqa: E731
    return (f"{r['scene']:8s} {r['ctrl']:4s} s{r['seed']} | {r.get('status', '-'):8s} {g('time_s', '.0f')} s | "
            f"最近 {g('robot_min')} <0.5m {g('lt0.5_s', '.1f')} s 碰上 {g('contact_s', '.1f')} s | "
            f"到点 {g('goal_err')} m / {g('yaw_err_deg', '.0f')}° | 停 {g('stopped_s', '.1f')} s | "
            f"collision ahead {r.get('collision_ahead', '-')} 恢复 {r.get('recoveries', '-')} | "
            f"near {r.get('to_near', '-')} rotate {r.get('rotates', '-')} 回退 {r.get('near_fallback', '-')} 卡住 {r.get('stalls', '-')} | "
            f"顿挫 {r.get('stomp', '-')} 站稳 {r.get('settle', '-')} 跳过 {r.get('settle_skip', '-')} 防抖 {r.get('debounce', '-')}")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--scenes", nargs="+", default=list(SCENES), choices=list(SCENES))
    ap.add_argument("--ctrls", nargs="+", default=["rpp", "shim"])
    ap.add_argument("--set", nargs="*", default=[], help="名称:参数=值[,参数=值]")
    ap.add_argument("--seeds", nargs="+", type=int, default=[0])
    ap.add_argument("--jobs", type=int, default=6)
    ap.add_argument("--domain", type=int, default=140)
    ap.add_argument("--timeout", type=float, default=150.0)
    ap.add_argument("--shim-install", default="", help="aid_nav_shim_controller 的 install 目录（humble 编译）")
    a = ap.parse_args()
    out_dir = os.path.join(HERE, "results", time.strftime("shim_%Y%m%d_%H%M%S"))
    os.makedirs(out_dir)
    sets = {}
    for s in a.set:
        name, kv = s.split(":", 1)
        sets[name] = {k: yaml.safe_load(v) for k, v in (p.split("=") for p in kv.split(","))}
    files = {c: ctrl_file(c, sets, out_dir) for c in a.ctrls}
    jobs = [(s, c, sd) for s in a.scenes for sd in a.seeds for c in a.ctrls]

    def work(i_job):
        i, (scene, ctrl, seed) = i_job
        out, rec = run_one(scene, ctrl, seed, a.domain + i, out_dir, a.shim_install, a.timeout, files[ctrl])
        out = analyze(out, rec)
        np.savez_compressed(os.path.join(out_dir, f"rec_{scene}_{ctrl}_s{seed}.npz"), truth=np.array(rec["truth"]),
                            cmd=np.array(rec["cmd"]), t0=out.get("t0", 0.0), allow_pickle=True)
        print(fmt(out), flush=True)
        return out

    with ThreadPoolExecutor(max_workers=a.jobs) as pool:
        rows = list(pool.map(work, enumerate(jobs)))
    keys = ["scene", "ctrl", "seed", "status", "time_s", "robot_min", "lt0.5_s", "contact_s", "goal_err", "yaw_err_deg",
            "path_len", "plans", "stopped_s", "collision_ahead", "recoveries", "to_near", "rotates", "near_fallback",
            "mppi_fail", "stalls", "stomp", "settle", "settle_skip", "debounce"]
    with open(os.path.join(out_dir, "summary.csv"), "w", newline="") as f:
        w = csv.DictWriter(f, fieldnames=keys, extrasaction="ignore")
        w.writeheader()
        for r in rows:
            w.writerow({k: (round(v, 3) if isinstance(v, float) else v) for k, v in r.items()})
    print("\n汇总（最近 = 骨盆真实位置到最近障碍边界，m；碰上 = 最近 < 0.25 m 的累计时间）")
    for r in rows:
        print(fmt(r))
    print(f"results: {out_dir}")


if __name__ == "__main__":
    main()
