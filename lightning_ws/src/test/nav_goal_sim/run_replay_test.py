#!/usr/bin/env python3
"""按实机 bag 的条件复现一次导航（Orin 30524，veljump_1026_1612）：同一张地图、同一串目标（含朝向）、同一个起点、同一套配置，
导出与实机 tools/nav2_debug/vel_jump_extract.py 同格式的 CSV，用同一套分析脚本对比"实机有 / 仿真有 / 共有"的问题。

  python3 -B run_replay_test.py --map ~/G1_bags/maps/1792839199393/map.yaml [--ctrl rpp|shim] [--seeds 0 1] [--parts 1 2]

配置（= 录包时机器上实际生效的，见 ~/G1_bags/veljump_1026_1612_ctx）：
  控制器  RPP，原地转 0.9，容差 0.3 / 0.30（nav2_orin_rpp.yaml + ctrl_rot09.yaml）；--ctrl shim 换成 ctrl_shim.yaml
  半径    robot_radius 0.40、inflation 0.85（replay_extra.yaml）
  行为树  navigate_w_recovery_backup_...（含 BackUp 0.25 m @0.3），smoother 下限 -0.3（replay_smoother.yaml）
  桥      bridge.mode deployed（= 机器上的 cmdvel_rewrite.hpp：min_vx 0.3、turn_min_vx 0、walk_min_wz 0、wz_stop 0.15、
          min_wz 0.32、后退 0.25~0.3）
  蟹行    模型蟹行 9.5°；local costmap / controller 用 base_link_nav（crab_comp_sim.py 10°，v0 0.1、v_full 0.2）
  里程计  原地转时 /odom 带踏步速度（odom.spin_step_speed 0.11、sigma 0.06，按实机 bag 标定）
目标序列分两段（--parts）：1 = 实机 0~700 s 的 21 个目标（A/B/C 循环），2 = 702 s 之后的 17 个（D/B/C 循环），起点取实机该段开始时的位姿。
结果：results/replay_<时间>/<ctrl>_p<段>_s<种子>/*.csv（列同实机导出）+ summary.txt
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

HERE = os.path.dirname(os.path.realpath(__file__))
SRC = os.path.realpath(os.path.join(HERE, "..", ".."))
os.environ["ROS_LOCALHOST_ONLY"] = "1"
BT = os.path.join(SRC, "aid_navigation2/behavior_trees/navigate_w_recovery_backup_and_replanning_only_if_path_becomes_invalid.xml")
CTRL = {"rpp": os.path.join(HERE, "ctrl_rot09.yaml"), "shim": os.path.join(HERE, "ctrl_shim.yaml"),
         "shim_settle": os.path.join(HERE, "ctrl_shim_settle.yaml")}
SIM_ARGS = ("-p bridge.mode:=deployed -p bridge.walk_min_wz:=0.0 -p bridge.min_wz:=0.32 -p bridge.wz_stop:=0.15 "
            "-p bridge.min_vx:=0.3 -p bridge.turn_min_vx:=0.0 -p bridge.max_reverse_vx:=0.3 -p bridge.min_reverse_vx:=0.25 "
            "-p odom.spin_step_speed:=0.11 -p odom.spin_step_sigma:=0.06")
# 实机目标（朝向按每次 Goal succeeded 时的机器人航向取圆周均值）
A, B, C, D = (6.00, -0.94, -5.0), (10.37, -1.46, 73.0), (10.69, 0.92, -172.0), (5.02, -0.48, -8.0)
PARTS = {
    1: dict(start=(6.43, -1.24, -96.5), goals=[B, A, B, C, A, B, C, A, B, C, A, B, C, A, B, C, A, B, C, A, B]),
    2: dict(start=(8.08, -1.01, -28.0), goals=[D, D, B, C, D, B, C, D, B, C, D, B, C, D, B, C, D]),
}
STATUS = {GoalStatus.STATUS_SUCCEEDED: "OK", GoalStatus.STATUS_ABORTED: "ABORTED", GoalStatus.STATUS_CANCELED: "CANCELED"}


def yaw_of(q):
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


def write_extra(out_dir):
    costmap = os.path.join(out_dir, "replay_extra.yaml")
    with open(costmap, "w") as f:
        f.write("local_costmap:\n  local_costmap:\n    ros__parameters:\n      robot_radius: 0.40\n"
                "global_costmap:\n  global_costmap:\n    ros__parameters:\n      robot_radius: 0.40\n")
    smoother = os.path.join(out_dir, "replay_smoother.yaml")
    with open(smoother, "w") as f:
        f.write("velocity_smoother:\n  ros__parameters:\n    min_velocity: [-0.3, 0.0, -0.9]\n")
    return costmap, smoother


def run_one(map_yaml, ctrl, part, seed, domain, out_dir, extra, leg_timeout):
    tag = f"{ctrl}_p{part}_s{seed}"
    d = os.path.join(out_dir, tag)
    os.makedirs(d)
    sx, sy, syaw = PARTS[part]["start"]
    cmd = ["ros2", "launch", os.path.join(HERE, "sim_orin_launch.py"), f"map:={map_yaml}",
           f"x:={sx}", f"y:={sy}", f"yaw_deg:={syaw}", f"sim_args:={SIM_ARGS} -p model.seed:={seed}",
           f"controller_extra:={CTRL[ctrl]}", f"bt_xml:={BT}", f"costmap_extra:={extra[0]}", f"smoother_extra:={extra[1]}",
           "crab_frame_deg:=10.0", f"local_extra:={os.path.join(HERE, 'local_base_link_nav.yaml')}"]
    env = dict(os.environ, ROS_DOMAIN_ID=str(domain), ROS_LOCALHOST_ONLY="1")
    log = open(os.path.join(d, "launch.log"), "w")
    proc = subprocess.Popen(cmd, env=env, stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
    ctx = rclpy.context.Context()
    rclpy.init(context=ctx, domain_id=domain)
    node = rclpy.create_node(f"replay_tester_{domain}", context=ctx)
    ex = SingleThreadedExecutor(context=ctx)
    ex.add_node(node)
    files = {k: open(os.path.join(d, k + ".csv"), "w", newline="") for k in
             ("cmd_vel_nav", "cmd_vel", "sport", "odommodestate", "odom", "pose", "rosout", "plan", "truth")}
    w = {k: csv.writer(f) for k, f in files.items()}
    now = time.time
    # 列与实机 vel_jump_extract.py 相同；sport = 桥发给 G1 的（/sim/sent）；odommodestate 用骨盆真实速度近似（世界系）
    node.create_subscription(Twist, "cmd_vel_nav", lambda m: w["cmd_vel_nav"].writerow(
        [f"{now():.4f}", f"{m.linear.x:.4f}", f"{m.angular.z:.4f}"]), 50)
    node.create_subscription(Twist, "cmd_vel", lambda m: w["cmd_vel"].writerow(
        [f"{now():.4f}", f"{m.linear.x:.4f}", f"{m.angular.z:.4f}"]), 50)
    node.create_subscription(Twist, "sim/sent", lambda m: w["sport"].writerow(
        [f"{now():.4f}", 7105, f"{m.linear.x:.4f}", f"{m.linear.y:.4f}", f"{m.angular.z:.4f}", 0.5]), 50)
    last_truth = {}

    def on_truth(m):
        t, x, y, yw = now(), m.pose.position.x, m.pose.position.y, yaw_of(m.pose.orientation)
        w["truth"].writerow([f"{t:.4f}", f"{x:.4f}", f"{y:.4f}", f"{yw:.4f}"])
        if last_truth and t - last_truth["t"] > 1e-3:
            dt = t - last_truth["t"]
            vxw, vyw = (x - last_truth["x"]) / dt, (y - last_truth["y"]) / dt
            dyaw = math.atan2(math.sin(yw - last_truth["yaw"]), math.cos(yw - last_truth["yaw"])) / dt
            w["odommodestate"].writerow([f"{t:.4f}", f"{vxw:.4f}", f"{vyw:.4f}", f"{dyaw:.4f}", f"{yw:.4f}"])
        last_truth.update(t=t, x=x, y=y, yaw=yw)
    node.create_subscription(PoseStamped, "sim/truth", on_truth, 50)
    node.create_subscription(PoseStamped, "sim/pub_pose", lambda m: w["pose"].writerow(
        [f"{now():.4f}", f"{m.pose.position.x:.4f}", f"{m.pose.position.y:.4f}", f"{yaw_of(m.pose.orientation):.4f}"]), 50)
    from nav_msgs.msg import Odometry
    node.create_subscription(Odometry, "odom", lambda m: w["odom"].writerow(
        [f"{now():.4f}", f"{m.twist.twist.linear.x:.4f}", f"{m.twist.twist.linear.y:.4f}", f"{m.twist.twist.angular.z:.4f}"]), 50)

    def on_plan(m):
        P = m.poses
        if P:
            a, b = P[0].pose.position, P[-1].pose.position
            w["plan"].writerow([f"{now():.4f}", len(P), f"{a.x:.3f}", f"{a.y:.3f}", f"{b.x:.3f}", f"{b.y:.3f}"])
    node.create_subscription(Path, "plan", on_plan, 10)
    node.create_subscription(Log, "/rosout", lambda m: w["rosout"].writerow(
        [f"{now():.4f}", m.name, m.level, m.msg.replace("\n", " ")[:300]]),
        QoSProfile(depth=500, durability=QoSDurabilityPolicy.VOLATILE))
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
        spin_for(3.0)
        for gx, gy, gyaw in PARTS[part]["goals"]:
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
                legs.append(dict(goal=(gx, gy, gyaw), status="REJECTED"))
                continue
            t0 = time.time()
            res = handle.get_result_async()
            spin_for(leg_timeout, until=res.done)
            st = STATUS.get(res.result().status, str(res.result().status)) if res.done() else "TIMEOUT"
            if not res.done():
                cf = handle.cancel_goal_async()
                spin_for(3.0, until=cf.done)
            legs.append(dict(goal=(gx, gy, gyaw), status=st, time_s=round(time.time() - t0, 1)))
            print(f"  [{tag}] -> ({gx}, {gy}, {gyaw}) {st} {time.time() - t0:.0f} s", flush=True)
            spin_for(1.0)
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


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--map", required=True)
    ap.add_argument("--ctrl", nargs="+", default=["rpp"], choices=list(CTRL))
    ap.add_argument("--parts", nargs="+", type=int, default=[1, 2])
    ap.add_argument("--seeds", nargs="+", type=int, default=[0])
    ap.add_argument("--jobs", type=int, default=4)
    ap.add_argument("--domain", type=int, default=160)   # DDS domain 上限 232
    ap.add_argument("--leg_timeout", type=float, default=90.0)
    a = ap.parse_args()
    out_dir = os.path.join(HERE, "results", time.strftime("replay_%Y%m%d_%H%M%S"))
    os.makedirs(out_dir)
    extra = write_extra(out_dir)
    jobs = [(c, p, s) for c in a.ctrl for p in a.parts for s in a.seeds]

    def work(ij):
        i, (c, p, s) = ij
        return run_one(os.path.expanduser(a.map), c, p, s, a.domain + i, out_dir, extra, a.leg_timeout)

    with ThreadPoolExecutor(max_workers=a.jobs) as pool:
        res = list(pool.map(work, enumerate(jobs)))
    with open(os.path.join(out_dir, "summary.txt"), "w") as f:
        for tag, legs in res:
            ok = sum(l.get("status") == "OK" for l in legs)
            line = f"{tag}: {ok}/{len(legs)} OK  " + " ".join(f"{l.get('status')}" for l in legs)
            print(line)
            f.write(line + "\n")
    print(f"results: {out_dir}")


if __name__ == "__main__":
    main()
