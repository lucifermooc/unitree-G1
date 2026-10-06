#!/usr/bin/env python3
"""Nav2 到点精度仿真测试：每个 (规划器, 场景, 重复) 单独起一套 sim_launch.py（独立 ROS_DOMAIN_ID），
发 NavigateToPose，记录结果、用时、最终位置/朝向误差和轨迹。

python3 run_goal_test.py                      # 全部场景 × 全部变体各 1 次
python3 run_goal_test.py --reps 2 --variants hybrid+fast --scenarios s1 s3 --jobs 4
结果：results/<时间>/summary.csv 和每次运行的轨迹 traj_*.csv
"""
import argparse
import csv
import math
import os
import signal
import subprocess
import time

import rclpy
from action_msgs.msg import GoalStatus
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from rclpy.executors import SingleThreadedExecutor
from tf2_ros import Buffer, TransformListener

HERE = os.path.dirname(os.path.realpath(__file__))
# 仿真和测试端都只走本机回环，且必须一致（不一致时 DDS 互相发现不了）
os.environ["ROS_LOCALHOST_ONLY"] = "1"
PLANNERS = {"2d": os.path.join(HERE, "planner_2d.yaml"), "hybrid": os.path.join(HERE, "planner_hybrid.yaml")}
# 变体 = 规划器 [+fast：速度平滑器快速减速 decel_fast.yaml] [+tight：到点位置容差 0.12 m goal_tight.yaml]
VARIANTS = ["2d", "hybrid", "2d+fast", "hybrid+fast", "hybrid+fast+tight"]
# 地图 1790676615248 的坐标（base_link）。s1 = 2026-10-03 16:40 实机出问题的那一次。
SCENARIOS = {
    "s1": dict(desc="去走廊尽头，终点朝向与来向相反（实机事故）", start=(-0.16, -0.21, -14.0), goal=(-4.21, 0.16, 0.0)),
    "s2": dict(desc="去走廊尽头，终点朝向与来向一致（对照）", start=(-0.16, -0.21, -14.0), goal=(-4.21, 0.16, 180.0)),
    "s3": dict(desc="回原点，终点朝向与来向相反", start=(-4.21, 0.16, 0.0), goal=(-0.16, -0.21, 180.0)),
    "s4": dict(desc="终点朝向与来向垂直", start=(-0.16, -0.21, -14.0), goal=(-2.50, 0.20, 90.0)),
}
STATUS = {GoalStatus.STATUS_SUCCEEDED: "SUCCEEDED", GoalStatus.STATUS_ABORTED: "ABORTED",
          GoalStatus.STATUS_CANCELED: "CANCELED"}


def yaw_of(q):
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


def ang_diff_deg(a, b):
    return (a - b + 180.0) % 360.0 - 180.0


def run_once(variant, sc, domain, timeout, out_dir, tag):
    env = dict(os.environ, ROS_DOMAIN_ID=str(domain), ROS_LOCALHOST_ONLY="1")
    sx, sy, syaw = sc["start"]
    gx, gy, gyaw = sc["goal"]
    planner, *flags = variant.split("+")
    smoother = os.path.join(HERE, "decel_fast.yaml" if "fast" in flags else "empty.yaml")
    controller = os.path.join(HERE, "goal_tight.yaml" if "tight" in flags else "empty_controller.yaml")
    log = open(os.path.join(out_dir, f"launch_{tag}.log"), "w")
    proc = subprocess.Popen(
        ["ros2", "launch", os.path.join(HERE, "sim_launch.py"), f"planner:={PLANNERS[planner]}",
         f"smoother_extra:={smoother}", f"controller_extra:={controller}",
         f"x:={float(sx)}", f"y:={float(sy)}", f"yaw_deg:={float(syaw)}"],
        env=env, stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
    ctx = rclpy.context.Context()
    rclpy.init(context=ctx, domain_id=domain)
    node = rclpy.create_node(f"goal_tester_{domain}", context=ctx)
    ex = SingleThreadedExecutor(context=ctx)
    ex.add_node(node)
    buf = Buffer()
    TransformListener(buf, node)
    client = ActionClient(node, NavigateToPose, "navigate_to_pose")
    traj, row = [], dict(tag=tag, variant=variant, scenario=sc["name"])

    def pose():
        try:
            t = buf.lookup_transform("map", "base_link", rclpy.time.Time())
        except Exception:
            return None
        return t.transform.translation.x, t.transform.translation.y, math.degrees(yaw_of(t.transform.rotation))

    def spin_for(sec, until=None):
        end = time.time() + sec
        while time.time() < end:
            ex.spin_once(timeout_sec=0.05)
            p = pose()
            if p and (not traj or time.time() - traj[-1][0] >= 0.1):
                traj.append((time.time(), *p))
            if until and until():
                return True
        return False

    try:
        if not client.wait_for_server(timeout_sec=60.0):
            row["status"] = "NO_SERVER"
            return row
        goal = NavigateToPose.Goal()
        goal.pose.header.frame_id = "map"
        goal.pose.pose.position.x, goal.pose.pose.position.y = gx, gy
        goal.pose.pose.orientation.z = math.sin(math.radians(gyaw) / 2)
        goal.pose.pose.orientation.w = math.cos(math.radians(gyaw) / 2)
        handle = None
        for _ in range(60):  # bt_navigator 激活前会拒绝目标，重试
            spin_for(1.0)
            fut = client.send_goal_async(goal)
            spin_for(5.0, until=fut.done)
            if fut.done() and fut.result().accepted:
                handle = fut.result()
                break
        if handle is None:
            row["status"] = "REJECTED"
            return row
        t0 = time.time()
        res = handle.get_result_async()
        spin_for(timeout, until=res.done)
        row["time_s"] = round(time.time() - t0, 1)
        row["status"] = STATUS.get(res.result().status, str(res.result().status)) if res.done() else "TIMEOUT"
        if not res.done():
            handle.cancel_goal_async()
        spin_for(2.0)  # 等 G1 停稳（指令 0.5 s 超时 + 一阶响应）
        p = pose()
        if p:
            row.update(final_x=round(p[0], 3), final_y=round(p[1], 3), final_yaw=round(p[2], 1),
                       err_xy=round(math.hypot(p[0] - gx, p[1] - gy), 3),
                       err_yaw=round(ang_diff_deg(p[2], gyaw), 1))
        row["path_len"] = round(sum(math.hypot(b[1] - a[1], b[2] - a[2]) for a, b in zip(traj, traj[1:])), 2)
        return row
    finally:
        with open(os.path.join(out_dir, f"traj_{tag}.csv"), "w", newline="") as f:
            csv.writer(f).writerows([("t", "x", "y", "yaw_deg")] + [(round(a - traj[0][0], 2), *map(lambda v: round(v, 3), b))
                                    for a, *b in traj] if traj else [])
        node.destroy_node()
        rclpy.shutdown(context=ctx)
        os.killpg(proc.pid, signal.SIGINT)
        try:
            proc.wait(timeout=15)
        except subprocess.TimeoutExpired:
            os.killpg(proc.pid, signal.SIGKILL)
        log.close()


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--variants", nargs="+", default=VARIANTS, choices=VARIANTS)
    ap.add_argument("--scenarios", nargs="+", default=list(SCENARIOS), choices=list(SCENARIOS))
    ap.add_argument("--reps", type=int, default=1)
    ap.add_argument("--timeout", type=float, default=150.0)
    ap.add_argument("--domain", type=int, default=60)
    ap.add_argument("--jobs", type=int, default=1, help="并行跑几套仿真（各用独立 ROS_DOMAIN_ID）")
    args = ap.parse_args()
    out_dir = os.path.join(HERE, "results", time.strftime("%Y%m%d_%H%M%S"))
    os.makedirs(out_dir)
    jobs = []
    for variant in args.variants:
        for name in args.scenarios:
            for rep in range(args.reps):
                jobs.append((variant, dict(SCENARIOS[name], name=name), f"{variant.replace('+', '-')}_{name}_{rep}"))

    def work(i_job):
        i, (variant, sc, tag) = i_job
        print(f"== {tag}: {sc['desc']}  start {sc['start']} -> goal {sc['goal']}", flush=True)
        row = run_once(variant, sc, args.domain + i, args.timeout, out_dir, tag)
        print(f"   {tag}:", {k: v for k, v in row.items() if k not in ("tag", "variant", "scenario")}, flush=True)
        return row

    from concurrent.futures import ThreadPoolExecutor
    with ThreadPoolExecutor(max_workers=args.jobs) as pool:
        rows = list(pool.map(work, enumerate(jobs)))
    keys = ["tag", "variant", "scenario", "status", "time_s", "err_xy", "err_yaw", "final_x", "final_y", "final_yaw",
            "path_len"]
    with open(os.path.join(out_dir, "summary.csv"), "w", newline="") as f:
        w = csv.DictWriter(f, fieldnames=keys, extrasaction="ignore")
        w.writeheader()
        w.writerows(rows)
    print(f"results: {out_dir}")


if __name__ == "__main__":
    main()
