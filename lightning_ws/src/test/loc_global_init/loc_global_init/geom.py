"""与 ROS 无关的几何小工具（节点和离线工具共用，单测不需要 ROS 环境）。"""
import math
import struct

import numpy as np


def quat_to_rot(q):
    x, y, z, w = q.x, q.y, q.z, q.w
    return np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
                     [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
                     [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])


_LIVOX_PT = np.dtype([('t', '<u4'), ('x', '<f4'), ('y', '<f4'), ('z', '<f4'), ('r', 'u1'), ('tag', 'u1'),
                      ('line', 'u1'), ('pad', 'u1')])   # CDR 里每个 CustomPoint 19 字节 + 对齐到 4 = 20


def livox_raw_to_xyz(buf):
    """CDR 序列化的 livox_ros_driver2/CustomMsg（小端）→ (stamp 秒, Nx3 点)。
    不经 rclpy 反序列化：Orin 上逐点生成 Python 对象一帧要 1~3 s，还占着执行器把 IMU 回调饿死（10-27 实机）。"""
    b = memoryview(buf)
    if b[1] != 1:
        raise ValueError('only little-endian CDR supported')
    o = 4                                  # 封装头；之后的对齐都相对这里
    sec, nsec, slen = struct.unpack_from('<iII', b, o)
    q = 12 + slen                          # 相对偏移：stamp 8 + 字符串长度 4 + 内容（含 \0）
    q = (q + 7) & ~7                       # uint64 timebase 按 8 对齐
    q += 8                                 # timebase
    q += 4 + 1 + 3                         # point_num、lidar_id、rsvd[3]
    q = (q + 3) & ~3
    n, = struct.unpack_from('<I', b, o + q)
    q += 4
    data = bytes(b[o + q:o + q + 20 * n])
    if len(data) < 20 * n:                 # 最后一个点后面没有对齐填充
        data += b'\0' * (20 * n - len(data))
    a = np.frombuffer(data, dtype=_LIVOX_PT, count=n)
    xyz = np.column_stack([a['x'], a['y'], a['z']]).astype(np.float64)
    return sec + nsec * 1e-9, xyz


def livox_to_xyz(msg):
    n = len(msg.points)
    a = np.empty((n, 3))
    for i, p in enumerate(msg.points):
        a[i, 0] = p.x
        a[i, 1] = p.y
        a[i, 2] = p.z
    return a


def lidar_pose_to_base(x, y, yaw, R_level, R_bl, t_bl):
    """摆平雷达系的平面位姿 → base_link 平面位姿。
    R_bl, t_bl：body_link->雷达外参（p_body = R_bl p_lidar + t_bl）；base_link 与 body_link 同原点（脚下地面），只差躯干倾斜。"""
    body_origin_l = -R_bl.T @ t_bl                 # body 原点在雷达系
    body_x_l = R_bl.T @ np.array([1.0, 0.0, 0.0])  # body x 轴在雷达系
    o = R_level @ body_origin_l
    ax = R_level @ body_x_l
    c, s = math.cos(yaw), math.sin(yaw)
    bx = x + c * o[0] - s * o[1]
    by = y + s * o[0] + c * o[1]
    byaw = yaw + math.atan2(ax[1], ax[0])
    return bx, by, math.atan2(math.sin(byaw), math.cos(byaw))


def base_pose_for_lightning(lx, ly, yaw, t_bl):
    """要发给 lightning /initialpose 的 base 位姿，使它换出来的雷达位置正好是 (lx, ly)。
    lightning BaseTFPublisher::BasePoseToLidar 只用静态外参：雷达 = (x, y) + Rz(yaw) t_bl，不管躯干倾斜；
    机身前倾 ~4° 时雷达实际在 base 前方 ~0.09 m，直接发真实 base 位姿会把雷达放到后面 0.09 m，
    沿走廊 NDT 拉不回来（10-27 Orin 实测停在差 0.12 m 处）。yaw 用 base 航向（lidar_pose_to_base 的结果）。"""
    c, s = math.cos(yaw), math.sin(yaw)
    return lx - (c * t_bl[0] - s * t_bl[1]), ly - (s * t_bl[0] + c * t_bl[1]), yaw


def yaw_diff_deg(a, b):
    return abs((math.degrees(a - b) + 180.0) % 360.0 - 180.0)
