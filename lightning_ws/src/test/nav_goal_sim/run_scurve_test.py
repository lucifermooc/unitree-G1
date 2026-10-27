#!/usr/bin/env python3
"""展厅 S 弯仿真（2026-10-27）：实机地图 1792839199393 + 只有传感器看得到的障碍（展品 / 站人 / 过人），按上午实机的
D/B/C 循环发目标，比较 shim 控制器各版本在 S 弯里"走走停停 / 换动作"的次数。

起因：10-27 上午实机（settle_time 0.6）出 (10.69, 0.92) 经 S 弯去 (5.02, −0.48) 时 MPPI 连续算不出轨迹、规划 "no valid path"、
"Start occupied"，(10.39, 0.22)、(10.30, 0.81)、(7.80, −1.22) 跟线中止；用户反映 S 弯靠近竖墙"噔噔噔"差点摔。
干净地图上（run_replay_test.py）S 弯中段没有一次换动作，所以障碍是地图里没有的东西：这里按上午出事的位置摆。

  python3 -B run_scurve_test.py --map ~/G1_bags/maps/1792839199393/map.yaml --ctrls shim shim_settle \
      [--set shim_settle:settle_time=0.6] [--scenes ...] [--seeds 0 1] [--jobs 6] [--shim-install <install>]

配置同 run_replay_test.py（10-26 实机：半径 0.40、膨胀 0.85、桥 deployed、蟹行补偿 base_link_nav、BackUp 行为树），
另加 obstacle_scan_sim.yaml 的"激光"层（本脚本按定位位姿对 地图占据格 + 场景障碍 做 360° 射线求交，发 /sim/scan）。
--set 名称:参数=值[,参数=值]：在该控制器的 FollowPath 下改参数，作为一个新变体（名称 = 原名+后缀），如 shim_settle:settle_time=0.6。
结果：results/scurve_<时间>/<场景>_<变体>_s<种子>/*.csv（列同 run_replay_test.py）+ summary.csv / summary.txt
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
from nav_msgs.msg import Odometry, Path
from PIL import Image
from rcl_interfaces.msg import Log
from rclpy.action import ActionClient
from rclpy.executors import SingleThreadedExecutor
from rclpy.qos import QoSDurabilityPolicy, QoSProfile
from scipy import ndimage
from sensor_msgs.msg import LaserScan

import run_replay_test as rp

HERE = rp.HERE
os.environ["ROS_LOCALHOST_ONLY"] = "1"
COSTMAP_SCAN = os.path.join(HERE, "obstacle_scan_sim.yaml")
SCAN_N, SCAN_MAX, SCAN_STEP = 360, 3.5, 0.025
BODY_R = 0.25        # 骨盆离障碍 < 0.25 m 记为"碰上"（同 run_shim_test.py）
GOAL_ZONE = 0.8      # 离目标 0.8 m 以内的换动作算到点 / 起步，不算途中

# 上午实机的三个点（朝向 = 实机到点航向）：D 起点 / B 展厅入口 / C 展厅里
D, B, C = rp.D, rp.B, rp.C
LEGS = [B, C, D, B, C, D]          # 从 D 出发两圈
START = (5.02, -0.48, -8.0)


def box(x0, x1, y0, y1):
    return (x0, x1, y0, y1)


def person(track, legs=("CD",), yield_dist=0.0):
    """0.4 m 方块（人），track=[(t, cx, cy)]，t 从 legs 里那种腿的目标发出算起；其他腿时不在场。
    yield_dist > 0：人离机器人（中心）这么近时停下让路（时间不走），机器人走开后再接着走。"""
    return dict(size=(0.4, 0.4), track=np.array(track, float), legs=legs, yield_dist=yield_dist)


# 名字: (只有传感器看得到的静止障碍, 动态障碍)。腿名 "CD" = 从 C 去 D，以此类推。
# 路径参照：C→B / C→D 从 C (10.69, 0.92) 沿 x≈10.3 往南下到 y≈−1.3 再往西进走廊；地图上东墙 x≈11.5（y −1~0.5），
# 弧墙西端 (9.4, −0.35)。
SCENES = {
    "干净": ([], []),
    # 下坡道左边一个展品、右下一个展品，斜对角间隙 1.0 m：路径先靠右再往左拐（S 形），两边都在 0.5~0.8 m
    # （间隙 0.85 m 时 B→C 全部版本都过不去；上午实机慢但都过去了）
    "S弯挤窄": ([box(9.55, 9.95, -0.2, 0.5), box(10.9, 11.3, -1.0, -0.5)], []),
    # 右侧一块竖立展板（0.1 m 厚、1.8 m 长）+ 左下展品，间隙 1.2 m：中心可走的带只剩 ~0.4 m，贴墙走（上午 MPPI 失败 / no valid path 那片）
    "竖墙窄口": ([box(11.1, 11.2, -1.2, 0.6), box(9.5, 9.9, -0.9, -0.5)], []),
    # C 点两边有东西：起步 / 到点都在 0.45~0.5 m 的槽里（上午 "Start occupied"、出 C 就中止）
    "C点挤": ([box(9.95, 10.2, 0.4, 1.3), box(11.15, 11.4, 0.3, 1.2)], []),
    # S 弯中间站着一个人，C→D 腿开始后 15 s 往西走开
    "S弯站人": ([], [person([(0, 10.45, -0.35), (15, 10.45, -0.35), (21, 7.5, -0.35)])]),
    # C→D 腿上一个人从走廊迎面走进 S 弯（0.5 m/s），在 S 弯里和机器人相遇；离机器人 0.9 m 时停下让路
    # （不让路的版本所有控制器都被撞 ~3 s，分不出好坏）
    "S弯迎面": ([], [person([(0, 7.0, -1.2), (6.6, 10.3, -1.2), (10.0, 10.4, 0.5), (14, 10.4, 2.5)], yield_dist=0.9)]),
    # 走廊靠南墙一个临时障碍（10-26 实机 (8.4, −1.2) 1~4 格；10-27 上午 (7.80, −1.22) 跟线中止）；
    # 放在走廊中间时把路堵死（所有版本规划失败 56 次、只到 2/6），实机是慢但能过
    "走廊中障碍": ([box(8.2, 8.5, -1.75, -1.5)], []),
    # 挤窄 + 站人 + 迎面，最差情况
    "S弯全": ([box(9.55, 9.95, -0.2, 0.5), box(10.9, 11.3, -1.0, -0.5)],
             [person([(0, 7.0, -1.2), (6.6, 10.3, -1.2), (10.0, 10.4, 0.5), (14, 10.4, 2.5)], yield_dist=0.9)]),
}
LEGNAME = {B: "B", C: "C", D: "D"}


class World:
    def __init__(self, map_yaml):
        info = yaml.safe_load(open(map_yaml))
        img = np.array(Image.open(os.path.join(os.path.dirname(map_yaml), info["image"])).convert("L"))[::-1]
        self.res = float(info["resolution"])
        self.origin = info["origin"][:2]
        occ_th = (1.0 - float(info["occupied_thresh"])) * 255
        self.map_occ = img <= occ_th

    def rasterize(self, boxes, occ):
        for x0, x1, y0, y1 in boxes:
            c0, c1 = int(round((x0 - self.origin[0]) / self.res)), int(round((x1 - self.origin[0]) / self.res))
            r0, r1 = int(round((y0 - self.origin[1]) / self.res)), int(round((y1 - self.origin[1]) / self.res))
            occ[max(r0, 0):r1, max(c0, 0):c1] = True

    def static_occ(self, scene):
        occ = self.map_occ.copy()
        self.rasterize(SCENES[scene][0], occ)
        return occ


def mover_box(m, t):
    tr = m["track"]
    cx, cy = np.interp(t, tr[:, 0], tr[:, 1]), np.interp(t, tr[:, 0], tr[:, 2])
    w, h = m["size"]
    return (cx - w / 2, cx + w / 2, cy - h / 2, cy + h / 2)


def raycast(world, occ, x, y, yaw):
    ang = yaw + np.linspace(-math.pi, math.pi, SCAN_N, endpoint=False)
    s = np.arange(SCAN_STEP, SCAN_MAX, SCAN_STEP)
    px = x + np.cos(ang)[:, None] * s[None, :]
    py = y + np.sin(ang)[:, None] * s[None, :]
    c = ((px - world.origin[0]) / world.res).astype(int)
    r = ((py - world.origin[1]) / world.res).astype(int)
    inside = (c >= 0) & (c < occ.shape[1]) & (r >= 0) & (r < occ.shape[0])
    hit = np.zeros_like(inside)
    hit[inside] = occ[r[inside], c[inside]]
    first = np.where(hit.any(1), hit.argmax(1), -1)
    return np.where(first >= 0, s[np.maximum(first, 0)], np.inf)


def ctrl_file(name, sets, out_dir):
    """名称 → controller_extra 文件；有 --set 时在 out_dir 写一份改过参数的副本。"""
    base = name.split("+")[0]
    src = rp.CTRL[base]
    if name not in sets:
        return src
    doc = yaml.safe_load(open(src))
    fp = doc["controller_server"]["ros__parameters"]["FollowPath"]
    fp.update(sets[name])
    path = os.path.join(out_dir, f"ctrl_{name}.yaml")
    with open(path, "w") as f:
        yaml.safe_dump(doc, f, allow_unicode=True)
    return path


def costmap_extra(out_dir):
    doc = yaml.safe_load(open(COSTMAP_SCAN))
    for k in ("local_costmap", "global_costmap"):
        doc[k][k]["ros__parameters"]["robot_radius"] = 0.40
    path = os.path.join(out_dir, "scan_r040.yaml")
    with open(path, "w") as f:
        yaml.safe_dump(doc, f, allow_unicode=True)
    return path


def run_one(world, map_yaml, scene, ctrl, seed, domain, out_dir, ctrl_yaml, extra, smoother, leg_timeout, shim_install):
    tag = f"{scene}_{ctrl}_s{seed}"
    d = os.path.join(out_dir, tag)
    os.makedirs(d)
    sx, sy, syaw = START
    cmd = ["ros2", "launch", os.path.join(HERE, "sim_orin_launch.py"), f"map:={map_yaml}",
           f"x:={sx}", f"y:={sy}", f"yaw_deg:={syaw}", f"sim_args:={rp.SIM_ARGS} -p model.seed:={seed}",
           f"controller_extra:={ctrl_yaml}", f"bt_xml:={rp.BT}", f"costmap_extra:={extra}", f"smoother_extra:={smoother}",
           "crab_frame_deg:=10.0", f"local_extra:={os.path.join(HERE, 'local_base_link_nav.yaml')}"]
    env = dict(os.environ, ROS_DOMAIN_ID=str(domain), ROS_LOCALHOST_ONLY="1")
    if shim_install:
        env["AMENT_PREFIX_PATH"] = shim_install + ":" + env.get("AMENT_PREFIX_PATH", "")
        env["LD_LIBRARY_PATH"] = os.path.join(shim_install, "lib") + ":" + env.get("LD_LIBRARY_PATH", "")
    log = open(os.path.join(d, "launch.log"), "w")
    proc = subprocess.Popen(cmd, env=env, stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
    ctx = rclpy.context.Context()
    rclpy.init(context=ctx, domain_id=domain)
    node = rclpy.create_node(f"scurve_tester_{domain}", context=ctx)
    ex = SingleThreadedExecutor(context=ctx)
    ex.add_node(node)
    files = {k: open(os.path.join(d, k + ".csv"), "w", newline="") for k in
             ("cmd_vel_nav", "cmd_vel", "sport", "odommodestate", "odom", "pose", "rosout", "plan", "truth", "legs")}
    w = {k: csv.writer(f) for k, f in files.items()}
    now = time.time
    st = dict(pose=None, leg=None, leg_t0=None, mt={}, mlast=None)
    static = world.static_occ(scene)
    movers = SCENES[scene][1]

    node.create_subscription(Twist, "cmd_vel_nav", lambda m: w["cmd_vel_nav"].writerow(
        [f"{now():.4f}", f"{m.linear.x:.4f}", f"{m.angular.z:.4f}"]), 50)
    node.create_subscription(Twist, "cmd_vel", lambda m: w["cmd_vel"].writerow(
        [f"{now():.4f}", f"{m.linear.x:.4f}", f"{m.angular.z:.4f}"]), 50)
    node.create_subscription(Twist, "sim/sent", lambda m: w["sport"].writerow(
        [f"{now():.4f}", 7105, f"{m.linear.x:.4f}", f"{m.linear.y:.4f}", f"{m.angular.z:.4f}", 0.5]), 50)
    last = {}

    def on_truth(m):
        t, x, y, yw = now(), m.pose.position.x, m.pose.position.y, rp.yaw_of(m.pose.orientation)
        w["truth"].writerow([f"{t:.4f}", f"{x:.4f}", f"{y:.4f}", f"{yw:.4f}"])
        if last and t - last["t"] > 1e-3:
            dt = t - last["t"]
            dyaw = math.atan2(math.sin(yw - last["yaw"]), math.cos(yw - last["yaw"])) / dt
            w["odommodestate"].writerow([f"{t:.4f}", f"{(x - last['x']) / dt:.4f}", f"{(y - last['y']) / dt:.4f}",
                                         f"{dyaw:.4f}", f"{yw:.4f}"])
        last.update(t=t, x=x, y=y, yaw=yw)
    node.create_subscription(PoseStamped, "sim/truth", on_truth, 50)

    def on_pose(m):
        st["pose"] = m
        w["pose"].writerow([f"{now():.4f}", f"{m.pose.position.x:.4f}", f"{m.pose.position.y:.4f}",
                            f"{rp.yaw_of(m.pose.orientation):.4f}"])
    node.create_subscription(PoseStamped, "sim/pub_pose", on_pose, 50)
    node.create_subscription(Odometry, "odom", lambda m: w["odom"].writerow(
        [f"{now():.4f}", f"{m.twist.twist.linear.x:.4f}", f"{m.twist.twist.linear.y:.4f}",
         f"{m.twist.twist.angular.z:.4f}"]), 50)

    def on_plan(m):
        P = m.poses
        if P:
            a, b = P[0].pose.position, P[-1].pose.position
            w["plan"].writerow([f"{now():.4f}", len(P), f"{a.x:.3f}", f"{a.y:.3f}", f"{b.x:.3f}", f"{b.y:.3f}"])
    node.create_subscription(Path, "plan", on_plan, 10)
    node.create_subscription(Log, "/rosout", lambda m: w["rosout"].writerow(
        [f"{now():.4f}", m.name, m.level, m.msg.replace("\n", " ")[:300]]),
        QoSProfile(depth=500, durability=QoSDurabilityPolicy.VOLATILE))
    scan_pub = node.create_publisher(LaserScan, "/sim/scan", 10)

    def movers_now():
        out = []
        if st["leg"] is None:
            return out
        t = now()
        dt = 0.0 if st["mlast"] is None else min(t - st["mlast"], 0.5)
        st["mlast"] = t
        for i, m in enumerate(movers):
            if st["leg"] not in m["legs"]:
                continue
            mt = st["mt"].get(i, 0.0)
            b = mover_box(m, mt)
            p = st["pose"]
            blocked = False
            if m["yield_dist"] > 0 and p is not None:
                cx, cy = (b[0] + b[1]) / 2, (b[2] + b[3]) / 2
                blocked = math.hypot(cx - p.pose.position.x, cy - p.pose.position.y) < m["yield_dist"]
            st["mt"][i] = mt if blocked else mt + dt
            out.append(b)
        return out

    def pub_scan():
        p = st["pose"]
        if p is None:
            return
        occ = static
        mb = movers_now()
        if mb:
            occ = static.copy()
            world.rasterize(mb, occ)
            w["legs"].writerow([f"{now():.4f}", "mover"] + [f"{v:.3f}" for b in mb for v in b])
        rng = raycast(world, occ, p.pose.position.x, p.pose.position.y, rp.yaw_of(p.pose.orientation))
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

    legs = []
    try:
        if not client.wait_for_server(timeout_sec=90.0):
            return tag, [dict(status="NO_SERVER")]
        spin_for(4.0)
        prev = START[:2]
        prev_name = "D"
        for gx, gy, gyaw in LEGS:
            name = prev_name + LEGNAME[(gx, gy, gyaw)]
            g = NavigateToPose.Goal()
            g.pose.header.frame_id = "map"
            g.pose.pose.position.x, g.pose.pose.position.y = gx, gy
            g.pose.pose.orientation.z, g.pose.pose.orientation.w = math.sin(math.radians(gyaw) / 2), math.cos(math.radians(gyaw) / 2)
            handle = None
            for _ in range(20):
                fut = client.send_goal_async(g)
                spin_for(5.0, until=fut.done)
                if fut.done() and fut.result().accepted:
                    handle = fut.result()
                    break
                spin_for(1.0)
            if handle is None:
                legs.append(dict(leg=name, status="REJECTED"))
                continue
            t0 = time.time()
            st["leg"], st["leg_t0"], st["mt"], st["mlast"] = name, t0, {}, None
            res = handle.get_result_async()
            spin_for(leg_timeout, until=res.done)
            status = rp.STATUS.get(res.result().status, str(res.result().status)) if res.done() else "TIMEOUT"
            if not res.done():
                cf = handle.cancel_goal_async()
                spin_for(3.0, until=cf.done)
            t1 = time.time()
            st["leg"] = None
            w["legs"].writerow([f"{t0:.4f}", "leg", name, status, f"{t1:.4f}", gx, gy, gyaw, prev[0], prev[1]])
            legs.append(dict(leg=name, status=status, time_s=round(t1 - t0, 1), t0=t0, t1=t1, goal=(gx, gy)))
            print(f"  [{tag}] {name} {status} {t1 - t0:.0f} s", flush=True)
            spin_for(1.0)
            prev, prev_name = (gx, gy), LEGNAME[(gx, gy, gyaw)]
        return tag, legs
    finally:
        node.destroy_node()
        rclpy.shutdown(context=ctx)
        for f in files.values():
            f.close()
        os.killpg(proc.pid, signal.SIGINT)
        try:
            proc.wait(timeout=20)
        except subprocess.TimeoutExpired:
            os.killpg(proc.pid, signal.SIGKILL)
        log.close()


# ---------------- 分析 ----------------
def cls(vx, wz, vth=0.05, wth=0.15):
    if vx >= vth:
        return 1
    if vx <= -vth:
        return 2
    if abs(wz) >= wth:
        return 3 if wz > 0 else 4
    return 0


def load_csv(path, cols=None):
    try:
        a = np.loadtxt(path, delimiter=",", usecols=cols, ndmin=2)
    except (ValueError, OSError):
        return np.zeros((0, len(cols) if cols else 1))
    return a


def analyze_run(world, scene, d, legs):
    """途中（离起点 / 目标 > GOAL_ZONE）的动作变化统计 + 离障碍距离 + 日志计数。"""
    sp = load_csv(os.path.join(d, "sport.csv"), (0, 2, 4))
    T = load_csv(os.path.join(d, "truth.csv"))
    O = load_csv(os.path.join(d, "odommodestate.csv"))
    logs = []
    with open(os.path.join(d, "rosout.csv")) as f:
        for row in csv.reader(f):
            if len(row) >= 4:
                logs.append((float(row[0]), row[3]))
    out = dict(legs=len(legs), ok=sum(l["status"] == "OK" for l in legs))
    out["time_s"] = sum(l.get("time_s", 0) for l in legs)
    out["leg_times"] = " ".join(f"{l['leg']}:{l['status'][:2]}{l.get('time_s', 0):.0f}" for l in legs)
    if not len(sp) or not len(T):
        return out
    c = np.array([cls(v, wz) for _, v, wz in sp])
    pos = lambda t: T[min(np.searchsorted(T[:, 0], t), len(T) - 1), 1:3]   # noqa: E731

    def body(t):
        m = (O[:, 0] > t - 0.15) & (O[:, 0] < t + 0.05)
        if not m.any():
            return 0.0, 0.0
        return float(np.hypot(O[m, 1].mean(), O[m, 2].mean())), float(abs(O[m, 3].mean()))

    def leg_of(t):
        for l in legs:
            if "t0" in l and l["t0"] <= t <= l["t1"]:
                return l
        return None
    segs = []
    i = 0
    while i < len(c):
        j = i
        while j + 1 < len(c) and c[j + 1] == c[i]:
            j += 1
        segs.append((sp[i, 0], sp[j, 0] + 0.1, c[i]))
        i = j + 1
    n_change = n_stomp = n_moving = n_flip = 0
    n_zone = n_zone_moving = 0
    for k in range(1, len(segs)):
        t, a, b = segs[k][0], segs[k - 1][2], segs[k][2]
        l = leg_of(t)
        if l is None:
            continue
        p = pos(t)
        start = pos(l["t0"])
        if min(math.hypot(*(p - np.array(l["goal"]))), math.hypot(*(p - start))) < GOAL_ZONE:
            # 到点 / 起步区：开始新动作时身体还在动（10-26 实机"到点急停 0.1 s 就转、转完 0.5 s 就起步"）
            if b != 0:
                v, wz = body(t)
                n_zone += 1
                n_zone_moving += (v > 0.15 or wz > 0.3)
            continue
        n_change += 1
        dur_prev = segs[k - 1][1] - segs[k - 1][0]
        if b != 0:
            v, wz = body(t)
            n_moving += (v > 0.15 or wz > 0.3)
            if a != 0 and a != b:
                n_flip += 1                     # 走 ↔ 转 / 左 ↔ 右 直接换
                n_stomp += 1
            elif a == 0 and dur_prev < 1.0 and k >= 2 and segs[k - 2][2] != 0:
                n_stomp += 1                    # 停不到 1 s 又动（"噔噔噔"）
    out.update(change=n_change, stomp=n_stomp, flip=n_flip, moving_start=n_moving, zone_start=n_zone,
               zone_moving=n_zone_moving)
    # 到点航向误差：成功的腿结束后 1.5 s 的真实航向 vs 目标朝向
    yerr = []
    for l in legs:
        if l.get("status") == "OK":
            g = next(gg for gg in LEGS if (gg[0], gg[1]) == tuple(l["goal"]))
            k = min(np.searchsorted(T[:, 0], l["t1"] + 1.5), len(T) - 1)
            e = T[k, 3] - math.radians(g[2])
            yerr.append(abs(math.degrees(math.atan2(math.sin(e), math.cos(e)))))
    out["yaw_err"] = float(np.median(yerr)) if yerr else float("nan")
    # 离障碍（地图占据 + 静止传感器障碍按距离变换；动态障碍按当时位置）
    occ = world.static_occ(scene)
    dist = ndimage.distance_transform_edt(~occ) * world.res
    in_leg = np.zeros(len(T), bool)
    for l in legs:
        if "t0" in l:
            in_leg |= (T[:, 0] >= l["t0"]) & (T[:, 0] <= l["t1"])
    Tl = T[in_leg]
    if len(Tl):
        cc = np.clip(((Tl[:, 1] - world.origin[0]) / world.res).astype(int), 0, dist.shape[1] - 1)
        rr_ = np.clip(((Tl[:, 2] - world.origin[1]) / world.res).astype(int), 0, dist.shape[0] - 1)
        dd = dist[rr_, cc]
        mv = load_csv(os.path.join(d, "legs.csv"))
        for m in SCENES[scene][1]:
            pass
        # 动态障碍：legs.csv 里 mover 行（t, "mover", x0,x1,y0,y1）
        rows = []
        with open(os.path.join(d, "legs.csv")) as f:
            for row in csv.reader(f):
                if len(row) >= 6 and row[1] == "mover":
                    rows.append([float(v) for v in [row[0]] + row[2:6]])
        if rows:
            M = np.array(rows)
            k = np.clip(np.searchsorted(M[:, 0], Tl[:, 0]), 0, len(M) - 1)
            ok = np.abs(M[k, 0] - Tl[:, 0]) < 0.2
            x0, x1, y0, y1 = M[k, 1], M[k, 2], M[k, 3], M[k, 4]
            dx = np.maximum(np.maximum(x0 - Tl[:, 1], 0), Tl[:, 1] - x1)
            dy = np.maximum(np.maximum(y0 - Tl[:, 2], 0), Tl[:, 2] - y1)
            dd = np.where(ok, np.minimum(dd, np.hypot(dx, dy)), dd)
        dt = np.diff(Tl[:, 0], prepend=Tl[0, 0])
        dt[dt > 0.5] = 0
        out["min_clear"] = float(dd.min())
        out["contact_s"] = float(dt[dd < BODY_R].sum())
    msgs = [m for t, m in logs if any("t0" in l and l["t0"] <= t <= l["t1"] + 0.5 for l in legs)]
    out["settle"] = sum("NavShim settle:" in m for m in msgs)
    out["settle_skip"] = sum("NavShim settle skipped" in m for m in msgs)
    out["debounce"] = sum("NavShim debounce" in m for m in msgs)
    out["mppi_fail"] = sum("Optimizer fail" in m or "fail to compute" in m.lower() for m in msgs)
    out["ctrl_abort"] = sum("[follow_path]" in m and "Aborting" in m for m in msgs)
    out["plan_fail"] = sum(("no valid path" in m.lower()) or ("Start occupied" in m) or ("Failed to create plan" in m)
                           for m in msgs)
    out["recovery"] = sum(("Running spin" in m) or ("Running wait" in m) or ("Running backup" in m) for m in msgs)
    out["stall"] = sum("no progress along path" in m for m in msgs)
    out["ca"] = sum("ollision" in m and "ahead" in m.lower() for m in msgs)
    return out


KEYS = ["scene", "ctrl", "seed", "ok", "legs", "time_s", "stomp", "flip", "moving_start", "change", "zone_start",
        "zone_moving", "yaw_err", "min_clear",
        "contact_s", "settle", "settle_skip", "debounce", "mppi_fail", "ctrl_abort", "plan_fail", "recovery", "stall", "ca", "leg_times"]


def fmt(r):
    g = lambda k, f=".2f": format(r[k], f) if k in r and r[k] == r[k] else "-"  # noqa: E731
    return (f"{r['scene']:6s} {r['ctrl']:16s} s{r['seed']} | 到 {r.get('ok', '-')}/{r.get('legs', '-')} {g('time_s', '.0f')} s | "
            f"顿挫 {r.get('stomp', '-')}（直接换 {r.get('flip', '-')}）带速起动作 {r.get('moving_start', '-')} 途中变化 {r.get('change', '-')} | "
            f"到点/起步 带速起动作 {r.get('zone_moving', '-')}/{r.get('zone_start', '-')} 航向误差 {g('yaw_err', '.1f')}° | "
            f"最近 {g('min_clear')} 碰 {g('contact_s', '.1f')} s | 站稳 {r.get('settle', '-')} 跳过 {r.get('settle_skip', '-')} "
            f"防抖 {r.get('debounce', '-')} "
            f"中止 {r.get('ctrl_abort', '-')} 规划失败 {r.get('plan_fail', '-')} 恢复 {r.get('recovery', '-')} 卡住 {r.get('stall', '-')} | "
            f"{r.get('leg_times', '')}")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--map", required=True)
    ap.add_argument("--scenes", nargs="+", default=list(SCENES), choices=list(SCENES))
    ap.add_argument("--ctrls", nargs="+", default=["shim", "shim_settle"])
    ap.add_argument("--set", nargs="*", default=[], help="名称:参数=值[,参数=值]，名称 = 基础控制器+后缀（如 shim_settle+06）")
    ap.add_argument("--seeds", nargs="+", type=int, default=[0])
    ap.add_argument("--jobs", type=int, default=6)
    ap.add_argument("--domain", type=int, default=60)
    ap.add_argument("--leg_timeout", type=float, default=90.0)
    ap.add_argument("--shim-install", default="")
    a = ap.parse_args()
    sets = {}
    for s in a.set:
        name, kv = s.split(":", 1)
        sets[name] = {k: yaml.safe_load(v) for k, v in (p.split("=") for p in kv.split(","))}
    out_dir = os.path.join(HERE, "results", time.strftime("scurve_%Y%m%d_%H%M%S"))
    os.makedirs(out_dir)
    map_yaml = os.path.expanduser(a.map)
    world = World(map_yaml)
    extra = costmap_extra(out_dir)
    _, smoother = rp.write_extra(out_dir)
    files = {c: ctrl_file(c, sets, out_dir) for c in a.ctrls}
    jobs = [(s, c, sd) for s in a.scenes for sd in a.seeds for c in a.ctrls]

    def work(ij):
        i, (scene, ctrl, seed) = ij
        tag, legs = run_one(world, map_yaml, scene, ctrl, seed, a.domain + i, out_dir, files[ctrl], extra, smoother,
                            a.leg_timeout, a.shim_install)
        r = dict(scene=scene, ctrl=ctrl, seed=seed)
        try:
            r.update(analyze_run(world, scene, os.path.join(out_dir, tag), legs))
        except Exception as e:   # noqa: BLE001
            r["err"] = str(e)
        print(fmt(r), flush=True)
        return r

    with ThreadPoolExecutor(max_workers=a.jobs) as pool:
        rows = list(pool.map(work, enumerate(jobs)))
    with open(os.path.join(out_dir, "summary.csv"), "w", newline="") as f:
        wr = csv.DictWriter(f, fieldnames=KEYS, extrasaction="ignore")
        wr.writeheader()
        for r in rows:
            wr.writerow({k: (round(v, 3) if isinstance(v, float) else v) for k, v in r.items()})
    with open(os.path.join(out_dir, "summary.txt"), "w") as f:
        for r in rows:
            f.write(fmt(r) + "\n")
    print("\n汇总（顿挫 = 途中 走↔转 直接换 + 停不到 1 s 又动；带速起动作 = 开始新动作时身体 > 0.15 m/s 或 > 0.3 rad/s；"
          "最近 = 骨盆到障碍边界 m）")
    for r in rows:
        print(fmt(r))
    print(f"results: {out_dir}")


if __name__ == "__main__":
    main()
