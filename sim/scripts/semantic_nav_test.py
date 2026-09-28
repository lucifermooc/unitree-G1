"""全流程仿真测试：一句话 → 语义地图 → Nav2 导航 → 检查机器人是否真的到了对应点位。

每句话记录：
- 语义匹配到的地点和分数
- Nav2 导航结果
- 到达误差：Cartographer 定位的位置 与 目标点 的距离
- 真值误差：Gazebo 里机器人的真实位置 与 目标点 的距离（验证定位本身准不准）
"""
import json
import math
import os
import re
import subprocess
import sys
import time

SIM_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(os.path.dirname(SIM_DIR), "semantic_map"))

import rclpy  # noqa: E402
from rclpy.duration import Duration  # noqa: E402
from rclpy.time import Time  # noqa: E402
from tf2_ros import Buffer, TransformListener  # noqa: E402

from semantic_nav import SemanticNavigator  # noqa: E402

TESTS = [
    ("我想喝水", "茶水间"),
    ("带我去开会", "会议室"),
    ("我要上厕所", "卫生间"),
    ("去拿个工具", "仓库"),
    ("我饿了想热一下饭", "厨房"),
    ("回到工位", "办公区"),
    ("有访客来了", "前台"),
    ("今天股票涨了吗", None),   # 无关的话：不应该导航
    ("机器人没电了", "充电桩"),
]
ARRIVE_TOLERANCE = 0.3  # 米：Nav2 默认到达容差 0.25，留一点余量


def gazebo_pose():
    """Gazebo 里 burger 的真实位置（世界坐标 x, y, yaw）。"""
    out = subprocess.run(["gz", "model", "-m", "burger", "-p"], capture_output=True, text=True,
                         timeout=15).stdout
    nums = [list(map(float, m.split())) for m in re.findall(r"\[([-\d.e\s]+)\]", out)]
    (x, y, _), (_, _, yaw) = nums[-2], nums[-1]
    return x, y, yaw


def yaw_of(q):
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


def angle_diff(a, b):
    return abs((a - b + math.pi) % (2 * math.pi) - math.pi)


def main():
    nav = SemanticNavigator(use_sim_time=True)
    node = nav.navigator
    tf_buffer = Buffer()
    TransformListener(tf_buffer, node)

    def map_pose():
        deadline = time.time() + 10
        while time.time() < deadline:
            rclpy.spin_once(node, timeout_sec=0.1)
            try:
                t = tf_buffer.lookup_transform("map", "base_footprint", Time(), Duration(seconds=0.5))
                return t.transform.translation.x, t.transform.translation.y, yaw_of(t.transform.rotation)
            except Exception:
                continue
        raise RuntimeError("拿不到 map -> base_footprint 的变换，定位没起来？")

    # 地图坐标系 = 建图开始时机器人的位置；用启动时的一对（定位位姿, 真实位姿）求出两个坐标系的偏移
    mx, my, _ = map_pose()
    gx, gy, _ = gazebo_pose()
    offset = (mx - gx, my - gy)
    print(f"世界坐标 → 地图坐标 偏移：({offset[0]:.3f}, {offset[1]:.3f})")

    rows = []
    for text, expect in TESTS:
        t0 = time.time()
        r = nav.go(text)
        elapsed = time.time() - t0
        row = {"query": text, "expect": expect, "nav_result": r["nav_result"],
               "matched": r["best"]["title"] if r["found"] else None,
               "score": r["candidates"][0]["score"] if r["candidates"] else None,
               "seconds": round(elapsed, 1)}
        if r["found"]:
            b = r["best"]
            lx, ly, lyaw = map_pose()
            wx, wy, wyaw = gazebo_pose()
            tx, ty = wx + offset[0], wy + offset[1]
            row.update({
                "target": [b["x"], b["y"], b["yaw"]],
                "localized": [round(lx, 3), round(ly, 3), round(lyaw, 3)],
                "arrive_err_m": round(math.hypot(lx - b["x"], ly - b["y"]), 3),
                "truth_err_m": round(math.hypot(tx - b["x"], ty - b["y"]), 3),
                "loc_err_m": round(math.hypot(tx - lx, ty - ly), 3),
                "yaw_err_rad": round(angle_diff(lyaw, b["yaw"]), 3),
            })
        ok_match = row["matched"] == expect
        ok_nav = (expect is None and r["nav_result"] == "NOT_FOUND") or (
            r["nav_result"] == "SUCCEEDED" and row.get("truth_err_m", 99) <= ARRIVE_TOLERANCE)
        row["pass"] = ok_match and ok_nav
        rows.append(row)
        print(("✅ " if row["pass"] else "❌ ") + json.dumps(row, ensure_ascii=False), flush=True)

    passed = sum(r["pass"] for r in rows)
    print(f"\n通过 {passed}/{len(rows)}")
    with open(os.path.join(SIM_DIR, "logs", "semantic_nav_test_result.json"), "w", encoding="utf-8") as f:
        json.dump(rows, f, ensure_ascii=False, indent=2)
    rclpy.try_shutdown()
    sys.exit(0 if passed == len(rows) else 1)


if __name__ == "__main__":
    main()
