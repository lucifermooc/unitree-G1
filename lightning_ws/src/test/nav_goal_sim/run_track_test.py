#!/usr/bin/env python3
"""Orin 30524 跟线 / 到点仿真：每个变体起一套 sim_orin_launch.py（独立 ROS_DOMAIN_ID），按实机 10-25 的三个目标点来回发，
统计和实机 track_err.py 同口径的横向偏差、到点误差、离障碍距离、collision ahead 次数。

python3 -B run_track_test.py --map <map_sim.yaml> [--variants 现状 航向补偿] [--legs 6] [--jobs 4] [--seed 0]
结果：results/track_<时间>/summary.csv、每个变体的 rec_<变体>.npz 和 launch 日志。
"""
import argparse
import bisect
import csv
import math
import os
import signal
import subprocess
import time

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

HERE = os.path.dirname(os.path.realpath(__file__))
os.environ["ROS_LOCALHOST_ONLY"] = "1"
TAN = math.tan(math.radians(9.5))
# 实机 10-25 地图 1792839199393 上来回发的三个点（朝向是猜的：实机日志没记目标朝向）
START = (0.6, 0.1, -7.0)
GOALS = [(6.00, -0.94, 180.0), (10.39, 1.02, 180.0), (10.37, -1.46, 90.0)]
ROT09 = os.path.join(HERE, "ctrl_rot09.yaml")
ROT09_TOL = os.path.join(HERE, "ctrl_rot09_tol015.yaml")
A = "-p bridge.mode:=walkmin -p bridge.walk_min_wz:=0.0 -p bridge.walk_wz_stop:=0.0 -p bridge.smooth_s:=0.0"
COMP = "-p model.yaw_comp_deg:=9.5"
NAVFRAME = "NAVFRAME"   # 标记：起 crab_comp_sim.py（10°）+ local costmap 用 base_link_nav（= g1_nav_bridge/crab_comp_frame 的接法）
# 名字: (g1_orin_sim 参数, controller_extra 文件)。A = 走路时 wz 原样发出；B = 原地转 0.9；C = wz_stop 0.15（= 默认）
VARIANTS = {
    "现状": ("", None),
    "wz_stop0.05": ("-p bridge.wz_stop:=0.05", None),
    "A": (A, None),
    "A+B": (A, ROT09),
    "A+B+容差0.15": (A, ROT09_TOL),
    "A+B+航向补偿": (f"{A} {COMP}", ROT09),
    "A+B+航向补偿+容差0.15": (f"{A} {COMP}", ROT09_TOL),
    "A+B+base_link_nav": (f"{A} {NAVFRAME}", ROT09),
    "A+B+base_link_nav+容差0.15": (f"{A} {NAVFRAME}", ROT09_TOL),
}
STATUS = {GoalStatus.STATUS_SUCCEEDED: "OK", GoalStatus.STATUS_ABORTED: "ABORTED", GoalStatus.STATUS_CANCELED: "CANCELED"}


def yaw_of(q):
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


def run_variant(name, map_yaml, domain, legs, seed, out_dir, leg_timeout):
    sim_args, ctrl = VARIANTS[name]
    navframe = NAVFRAME in sim_args
    sim_args = f"{sim_args.replace(NAVFRAME, '')} -p model.seed:={seed}".strip()
    env = dict(os.environ, ROS_DOMAIN_ID=str(domain), ROS_LOCALHOST_ONLY="1")
    log = open(os.path.join(out_dir, f"launch_{name}.log"), "w")
    cmd = ["ros2", "launch", os.path.join(HERE, "sim_orin_launch.py"), f"map:={map_yaml}",
           f"x:={START[0]}", f"y:={START[1]}", f"yaw_deg:={START[2]}", f"sim_args:={sim_args}"]
    if ctrl:
        cmd.append(f"controller_extra:={ctrl}")
    if navframe:
        cmd += ["crab_frame_deg:=10.0", f"local_extra:={os.path.join(HERE, 'local_base_link_nav.yaml')}"]
    proc = subprocess.Popen(cmd, env=env, stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
    ctx = rclpy.context.Context()
    rclpy.init(context=ctx, domain_id=domain)
    node = rclpy.create_node(f"track_tester_{domain}", context=ctx)
    ex = SingleThreadedExecutor(context=ctx)
    ex.add_node(node)
    rec = dict(truth=[], pub=[], cmd=[], sent=[], plans=[], logs=[])
    now = lambda: time.time()  # noqa: E731
    node.create_subscription(PoseStamped, "sim/truth", lambda m: rec["truth"].append(
        (now(), m.pose.position.x, m.pose.position.y, yaw_of(m.pose.orientation))), 50)
    node.create_subscription(PoseStamped, "sim/pub_pose", lambda m: rec["pub"].append(
        (now(), m.pose.position.x, m.pose.position.y, yaw_of(m.pose.orientation))), 50)
    node.create_subscription(Twist, "cmd_vel", lambda m: rec["cmd"].append((now(), m.linear.x, m.angular.z)), 50)
    node.create_subscription(Twist, "sim/sent", lambda m: rec["sent"].append(
        (now(), m.linear.x, m.linear.y, m.angular.z)), 50)
    node.create_subscription(Path, "plan", lambda m: rec["plans"].append(
        (now(), np.array([(p.pose.position.x, p.pose.position.y) for p in m.poses]))), 10)
    node.create_subscription(Log, "/rosout", lambda m: rec["logs"].append((now(), m.name, m.msg)),
                             QoSProfile(depth=200, durability=QoSDurabilityPolicy.VOLATILE))
    client = ActionClient(node, NavigateToPose, "navigate_to_pose")
    goals_out = []

    def spin_for(sec, until=None):
        end = time.time() + sec
        while time.time() < end:
            ex.spin_once(timeout_sec=0.02)
            if until and until():
                return True
        return False

    try:
        if not client.wait_for_server(timeout_sec=60.0):
            return dict(variant=name, error="NO_SERVER"), rec, goals_out
        spin_for(3.0)
        for leg in range(legs):
            gx, gy, gyaw = GOALS[leg % len(GOALS)]
            goal = NavigateToPose.Goal()
            goal.pose.header.frame_id = "map"
            goal.pose.pose.position.x, goal.pose.pose.position.y = gx, gy
            goal.pose.pose.orientation.z, goal.pose.pose.orientation.w = math.sin(math.radians(gyaw) / 2), math.cos(math.radians(gyaw) / 2)
            handle = None
            for _ in range(30):
                fut = client.send_goal_async(goal)
                spin_for(5.0, until=fut.done)
                if fut.done() and fut.result().accepted:
                    handle = fut.result()
                    break
                spin_for(1.0)
            if handle is None:
                goals_out.append(dict(leg=leg, goal=(gx, gy, gyaw), status="REJECTED"))
                continue
            t0 = time.time()
            res = handle.get_result_async()
            spin_for(leg_timeout, until=res.done)
            st = STATUS.get(res.result().status, str(res.result().status)) if res.done() else "TIMEOUT"
            if not res.done():
                handle.cancel_goal_async()
            t1 = time.time()
            spin_for(2.5)                          # 停稳（指令 0.5 s 失效 + 延迟 + 一阶响应）
            tx, ty, tyaw = rec["truth"][-1][1:] if rec["truth"] else (np.nan,) * 3
            goals_out.append(dict(leg=leg, goal=(gx, gy, gyaw), status=st, t0=t0, t1=t1, time_s=round(t1 - t0, 1),
                                  err_xy=math.hypot(tx - gx, ty - gy),
                                  err_yaw=math.degrees((tyaw - math.radians(gyaw) + math.pi) % (2 * math.pi) - math.pi)))
            print(f"   [{name}] leg {leg} -> ({gx}, {gy}, {gyaw}) {st} {t1 - t0:.0f} s err {goals_out[-1]['err_xy']:.2f} m", flush=True)
        return dict(variant=name), rec, goals_out
    finally:
        node.destroy_node()
        rclpy.shutdown(context=ctx)
        os.killpg(proc.pid, signal.SIGINT)
        try:
            proc.wait(timeout=20)
        except subprocess.TimeoutExpired:
            os.killpg(proc.pid, signal.SIGKILL)
        log.close()


def seg_dist(P, q):
    a, b = P[:-1], P[1:]
    ab = b - a
    l2 = np.maximum((ab ** 2).sum(1), 1e-9)
    u = np.clip(((q - a) * ab).sum(1) / l2, 0, 1)
    d = np.hypot(*(q - (a + ab * u[:, None])).T)
    k = int(np.argmin(d))
    cr = ab[k, 0] * (q[1] - a[k, 1]) - ab[k, 1] * (q[0] - a[k, 0])
    return math.copysign(d[k], cr), k, len(P)


def occupied_points(map_yaml):
    import yaml
    from PIL import Image
    meta = yaml.safe_load(open(map_yaml))
    im = np.array(Image.open(os.path.join(os.path.dirname(map_yaml), meta["image"])))
    res, (ox, oy, _) = meta["resolution"], meta["origin"]
    r, c = np.nonzero(im < 0.35 * 255)
    return np.stack([ox + (c + 0.5) * res, oy + (im.shape[0] - 1 - r + 0.5) * res], 1)


def analyze(rec, goals, occ, wz_stop=0.15, min_wz=0.32):
    cmd = np.array(rec["cmd"]) if rec["cmd"] else np.zeros((0, 3))
    pub = np.array(rec["pub"])
    tru = np.array(rec["truth"])
    ct = cmd[:, 0]
    pt = [p[0] for p in rec["plans"]]
    rows = []
    for i in range(0, len(pub), 5):                     # 10 Hz
        t, x, y, _ = pub[i]
        k = bisect.bisect_right(ct, t) - 1
        if k < 0 or t - ct[k] > 0.3 or cmd[k, 1] <= 0.05:
            continue
        j = bisect.bisect_right(pt, t) - 1
        if j < 0 or len(rec["plans"][j][1]) < 25:
            continue
        P = rec["plans"][j][1]
        d, kk, n = seg_dist(P, np.array([x, y]))
        if kk < 10 or kk > n - 10:
            continue
        jt = min(i, len(tru) - 1)
        dt_, _, _ = seg_dist(P, tru[jt, 1:3])
        rows.append((t, d, dt_))
    out = {}
    if rows:
        R = np.array(rows)
        a = np.abs(R[:, 1])
        tt, dd = R[:, 0], R[:, 1]
        sm = np.array([dd[(tt >= x - 0.5) & (tt <= x + 0.5)].mean() for x in tt])
        out.update(cte_p50=np.median(a), cte_p95=np.percentile(a, 95), cte_max=a.max(), cte_mean_signed=dd.mean(),
                   cte1s_p50=np.median(np.abs(sm)), cte1s_p95=np.percentile(np.abs(sm), 95),
                   true_p50=np.median(np.abs(R[:, 2])), true_p95=np.percentile(np.abs(R[:, 2]), 95), walk_s=len(R) / 10)
    walk = cmd[:, 1] > 0.05
    w = np.abs(cmd[walk, 2])
    out["lifted"] = float(np.mean((w >= wz_stop) & (w < min_wz))) if len(w) else np.nan
    s = np.sign(np.where(np.abs(cmd[:, 2]) >= wz_stop, cmd[:, 2], 0))
    flips, last = 0, 0
    for i in np.nonzero(walk)[0]:
        if s[i] != 0:
            if last != 0 and s[i] != last:
                flips += 1
            last = s[i]
    out["flip_s"] = (walk.sum() / 10) / max(flips, 1)
    # 离障碍：骨盆真实位置到地图占据格
    if len(tru) and len(occ):
        d = np.array([np.min(np.hypot(*(occ - p).T)) for p in tru[::10, 1:3]])
        out.update(clear_min=d.min(), inscribed_s=float(np.sum(d < 0.5)) * 10 * 0.02)
    ok = [g for g in goals if g.get("status") == "OK"]
    out.update(goals=len(goals), ok=len(ok),
               arrive_mean=np.mean([g["err_xy"] for g in ok]) if ok else np.nan,
               arrive_max=np.max([g["err_xy"] for g in ok]) if ok else np.nan,
               yaw_mean=np.mean([abs(g["err_yaw"]) for g in ok]) if ok else np.nan,
               leg_time=np.mean([g["time_s"] for g in ok]) if ok else np.nan)
    msgs = [m for _, n, m in rec["logs"]]
    out["collision_ahead"] = sum("ollision" in m and "ahead" in m.lower() for m in msgs)
    out["recoveries"] = sum(("Running spin" in m) or ("Running wait" in m) or ("Running backup" in m) for m in msgs)
    return out


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--map", required=True)
    ap.add_argument("--variants", nargs="+", default=list(VARIANTS), choices=list(VARIANTS))
    ap.add_argument("--legs", type=int, default=6)
    ap.add_argument("--seed", type=int, default=0)
    ap.add_argument("--jobs", type=int, default=4)
    ap.add_argument("--domain", type=int, default=80)
    ap.add_argument("--leg_timeout", type=float, default=120.0)
    a = ap.parse_args()
    out_dir = os.path.join(HERE, "results", time.strftime("track_%Y%m%d_%H%M%S"))
    os.makedirs(out_dir)
    occ = occupied_points(a.map)

    def work(i_name):
        i, name = i_name
        print(f"== {name}: {VARIANTS[name][0] or '(默认)'}", flush=True)
        head, rec, goals = run_variant(name, a.map, a.domain + i, a.legs, a.seed, out_dir, a.leg_timeout)
        np.savez_compressed(os.path.join(out_dir, f"rec_{name}.npz"),
                            truth=np.array(rec["truth"]), pub=np.array(rec["pub"]), cmd=np.array(rec["cmd"]),
                            sent=np.array(rec["sent"]), goals=np.array([str(g) for g in goals]))
        res = dict(head, **analyze(rec, goals, occ))
        print(f"   [{name}] {fmt(res)}", flush=True)
        return res

    from concurrent.futures import ThreadPoolExecutor
    with ThreadPoolExecutor(max_workers=a.jobs) as pool:
        rows = list(pool.map(work, enumerate(a.variants)))
    keys = sorted({k for r in rows for k in r}, key=lambda k: (k != "variant", k))
    with open(os.path.join(out_dir, "summary.csv"), "w", newline="") as f:
        w = csv.DictWriter(f, fieldnames=keys)
        w.writeheader()
        for r in rows:
            w.writerow({k: (round(v, 3) if isinstance(v, float) else v) for k, v in r.items()})
    print("\n汇总（横向偏差只算走路段、离路径起终点 10 个点以外；到点误差按骨盆真实位置）")
    for r in rows:
        print(f"{r['variant']:28s} {fmt(r)}")
    print(f"results: {out_dir}")


def fmt(r):
    g = lambda k, f=".3f": format(r[k], f) if k in r and r[k] == r[k] else "-"  # noqa: E731
    return (f"偏差 p50 {g('cte_p50')} p95 {g('cte_p95')} max {g('cte_max')} 均值 {g('cte_mean_signed', '+.3f')} | "
            f"1s平均 p50 {g('cte1s_p50')} p95 {g('cte1s_p95')} | 骨盆 p50 {g('true_p50')} | 抬速 {g('lifted', '.0%')} "
            f"换向每 {g('flip_s', '.1f')} s | 到点 {r.get('ok', 0)}/{r.get('goals', 0)} 均 {g('arrive_mean', '.2f')} 最大 {g('arrive_max', '.2f')} m "
            f"航向 {g('yaw_mean', '.1f')}° 单程 {g('leg_time', '.0f')} s | 离墙最近 {g('clear_min', '.2f')} 进内切带 {g('inscribed_s', '.1f')} s "
            f"| collision ahead {r.get('collision_ahead', '-')} 恢复 {r.get('recoveries', '-')}")


if __name__ == "__main__":
    main()
