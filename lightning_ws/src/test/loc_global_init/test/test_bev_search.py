"""全图搜索的单测：合成场景，不需要 ROS。

  cd src/test/loc_global_init && python3 -B -m pytest -q test
"""
import math
import os
import struct
import sys

import numpy as np
import pytest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))
from loc_global_init import bev_search as b  # noqa: E402
from loc_global_init.geom import base_pose_for_lightning, lidar_pose_to_base, livox_raw_to_xyz  # noqa: E402
from loc_global_init.pcd_io import load_pcd_xyz  # noqa: E402

GROUND = -1.2   # 地图系地面高度（地图原点 = 建图起点雷达位置，约在地面上 1.2 m）


def _box(rng, x0, x1, y0, y1, z0, z1, n):
    """长方体表面采样（只取四个侧面）"""
    pts = []
    for _ in range(n):
        side = rng.integers(4)
        z = rng.uniform(z0, z1)
        if side < 2:
            pts.append((rng.uniform(x0, x1), (y0, y1)[side], z))
        else:
            pts.append(((x0, x1)[side - 2], rng.uniform(y0, y1), z))
    return np.array(pts)


def _ground(rng, x0, x1, y0, y1, n):
    return np.column_stack([rng.uniform(x0, x1, n), rng.uniform(y0, y1, n), np.full(n, GROUND)])


def make_room(rng, symmetric=False):
    """20 m x 12 m 的房间 + 一些不同高度的箱子 / 柱子；symmetric=True 时房间关于中心点对称（转 180° 完全一样）"""
    h0, h1 = GROUND, GROUND + 3.0
    parts = [_ground(rng, -10, 10, -6, 6, 20000),
             _box(rng, -10, 10, -6, 6, h0, h1, 30000)]
    objs = [(-6, -5, 2, 3.5, 1.0), (3, 4.5, -4, -3, 0.6), (7, 7.4, 1, 1.4, 2.5), (-2, -1.6, -2, -1.6, 2.5),
            (0.5, 2.5, 3, 3.6, 0.9)]
    if symmetric:
        objs = [(-6, -5, 2, 3.5, 1.0), (5, 6, -3.5, -2, 1.0)]
    for x0, x1, y0, y1, hh in objs:
        parts.append(_box(rng, x0, x1, y0, y1, h0, h0 + hh, 3000))
    return np.concatenate(parts)


def make_scan(rng, world, x, y, yaw, sensor_h=0.0, max_range=20.0, noise=0.02):
    """从世界点里取传感器（位于 (x, y, sensor_h)）可见范围内的点，换到传感器系（简单起见不做遮挡）"""
    d = world[:, :2] - [x, y]
    r = np.hypot(d[:, 0], d[:, 1])
    pts = world[(r < max_range) & (r > 0.3)]
    pts = pts[rng.choice(len(pts), min(len(pts), 30000), replace=False)]
    c, s = math.cos(yaw), math.sin(yaw)
    rel = pts - [x, y, sensor_h]
    loc = np.column_stack([c * rel[:, 0] + s * rel[:, 1], -s * rel[:, 0] + c * rel[:, 1], rel[:, 2]])
    return loc + rng.normal(0, noise, loc.shape)


def test_rot_to_up():
    rng = np.random.default_rng(1)
    for _ in range(20):
        u = rng.normal(size=3)
        u[2] = abs(u[2]) + 0.5
        R = b.rot_to_up(u)
        assert np.allclose(R @ (u / np.linalg.norm(u)), [0, 0, 1], atol=1e-9)
        assert np.allclose(R @ R.T, np.eye(3), atol=1e-9)
    assert np.allclose(b.rot_to_up([0, 0, 1]), np.eye(3))


def test_estimate_ground():
    rng = np.random.default_rng(2)
    z = np.concatenate([rng.normal(-1.23, 0.02, 5000), rng.uniform(-1.0, 2.0, 5000)])
    assert abs(b.estimate_ground(z) - (-1.23)) < 0.03


@pytest.mark.parametrize('x,y,yaw_deg', [(0.0, 0.0, 0.0), (3.2, -1.7, 78.0), (-5.5, 2.4, -150.0), (6.0, 3.0, 179.0)])
def test_finds_pose_without_initial_guess(x, y, yaw_deg):
    rng = np.random.default_rng(3)
    world = make_room(rng)
    mi = b.MapIndex(world)
    scan = make_scan(rng, world, x, y, math.radians(yaw_deg))
    res = b.search(mi, scan)
    assert res.accepted, res.reason
    assert math.hypot(res.best.x - x, res.best.y - y) < 0.1
    assert abs((math.degrees(res.best.yaw) - yaw_deg + 180) % 360 - 180) < 2.0


def test_ambiguous_scene_is_rejected():
    """房间关于中心点对称：(x, y, yaw) 与 (-x, -y, yaw+180°) 的扫描完全一样，必须拒绝而不是随便挑一个"""
    rng = np.random.default_rng(4)
    world = make_room(rng, symmetric=True)
    world = np.concatenate([world, world * [-1, -1, 1]])   # 补成严格中心对称
    mi = b.MapIndex(world)
    scan = make_scan(rng, world, 2.0, 1.0, math.radians(30))
    res = b.search(mi, scan)
    assert not res.accepted
    assert 'ambiguous' in res.reason


def test_wrong_place_scores_low():
    """扫描来自另一个房间：分数不够，拒绝"""
    rng = np.random.default_rng(5)
    mi = b.MapIndex(make_room(rng))
    other = np.concatenate([_ground(rng, -4, 4, -4, 4, 8000), _box(rng, -4, 4, -4, 4, GROUND, GROUND + 3, 8000),
                            _box(rng, -1, 1, 0.5, 1.5, GROUND, GROUND + 1.5, 2000)])
    res = b.search(mi, make_scan(rng, other, 0.5, -0.5, 0.3))
    assert not res.accepted


def test_lidar_pose_to_base():
    # 雷达在 body 原点正上方 1.236 m、无倾斜：base 位姿 = 雷达平面位姿
    R_bl, t_bl = np.eye(3), np.array([0.0, 0.0, 1.236])
    assert np.allclose(lidar_pose_to_base(1.0, 2.0, 0.5, np.eye(3), R_bl, t_bl), (1.0, 2.0, 0.5))
    # 雷达装在 body 前方 0.1 m：base 在雷达后方 0.1 m（沿航向）
    bx, by, byaw = lidar_pose_to_base(0.0, 0.0, math.pi / 2, np.eye(3), np.eye(3), np.array([0.1, 0.0, 1.2]))
    assert np.allclose((bx, by, byaw), (0.0, -0.1, math.pi / 2), atol=1e-9)
    # 躯干前倾 3°（雷达系被摆平）：body 原点在雷达正下方偏后，平面偏移 ≈ 1.236 × sin3°
    tilt = math.radians(3)
    R_level = np.array([[math.cos(tilt), 0, math.sin(tilt)], [0, 1, 0], [-math.sin(tilt), 0, math.cos(tilt)]])
    bx, by, _ = lidar_pose_to_base(0.0, 0.0, 0.0, R_level, R_bl, t_bl)
    assert abs(abs(bx) - 1.236 * math.sin(tilt)) < 1e-6 and abs(by) < 1e-9


def test_base_pose_for_lightning():
    t_bl = np.array([0.05, 0.0, 1.236])
    lx, ly, yaw = 3.0, -1.0, math.radians(30)
    # 躯干不倾斜：与真实 base 位姿相同
    assert np.allclose(base_pose_for_lightning(lx, ly, yaw, t_bl), lidar_pose_to_base(lx, ly, yaw, np.eye(3), np.eye(3), t_bl))
    # 前倾 4°：真实 base 在雷达后方 1.236·sin4° + 0.05，lightning 只按外参换算，发的位姿要比真实 base 靠前 1.236·sin4°
    tilt = math.radians(4)
    R_level = np.array([[math.cos(tilt), 0, math.sin(tilt)], [0, 1, 0], [-math.sin(tilt), 0, math.cos(tilt)]])
    bx, by, byaw = lidar_pose_to_base(lx, ly, yaw, R_level, np.eye(3), t_bl)
    px, py, _ = base_pose_for_lightning(lx, ly, byaw, t_bl)
    # 用 lightning 的换算：雷达 = 发的位姿 + Rz(yaw) t_bl → 回到 (lx, ly)
    c, s = math.cos(byaw), math.sin(byaw)
    assert np.allclose((px + c * t_bl[0] - s * t_bl[1], py + s * t_bl[0] + c * t_bl[1]), (lx, ly))
    assert abs(math.hypot(px - bx, py - by) - 1.236 * math.sin(tilt)) < 0.01


def test_pcd_binary_roundtrip(tmp_path):
    pts = np.random.default_rng(6).normal(size=(100, 3)).astype(np.float32)
    hdr = ('VERSION 0.7\nFIELDS x y z intensity\nSIZE 4 4 4 4\nTYPE F F F F\nCOUNT 1 1 1 1\n'
           f'WIDTH {len(pts)}\nHEIGHT 1\nVIEWPOINT 0 0 0 1 0 0 0\nPOINTS {len(pts)}\nDATA binary\n')
    body = np.column_stack([pts, np.zeros(len(pts), np.float32)]).tobytes()
    p = tmp_path / 'a.pcd'
    p.write_bytes(hdr.encode() + body)
    assert np.allclose(load_pcd_xyz(str(p)), pts, atol=1e-6)


def _cdr_custom_msg(frame_id, pts, sec=1793086131, nsec=973000000):
    """按 CDR（小端）手工拼 livox_ros_driver2/CustomMsg，对齐相对封装头之后计算"""
    body = bytearray(struct.pack('<iI', sec, nsec))
    fid = frame_id.encode() + b'\0'
    body += struct.pack('<I', len(fid)) + fid
    body += b'\0' * (-len(body) % 8) + struct.pack('<Q', 123456789)   # timebase
    body += struct.pack('<I', len(pts)) + bytes([1, 0, 0, 0])          # point_num、lidar_id、rsvd[3]
    body += b'\0' * (-len(body) % 4) + struct.pack('<I', len(pts))
    for i, (x, y, z) in enumerate(pts):
        body += b'\0' * (-len(body) % 4) + struct.pack('<IfffBBB', i * 10, x, y, z, 7, 0x10, i % 4)
    return b'\x00\x01\x00\x00' + bytes(body)


@pytest.mark.parametrize('frame_id', ['', 'a', 'mid360_link', 'mid360_lin', 'abcdefghijk'])
def test_livox_raw_to_xyz(frame_id):
    pts = np.random.default_rng(7).normal(size=(57, 3)).astype(np.float32)
    stamp, xyz = livox_raw_to_xyz(_cdr_custom_msg(frame_id, pts))
    assert abs(stamp - 1793086131.973) < 1e-6
    assert np.allclose(xyz, pts, atol=1e-7)
    _, empty = livox_raw_to_xyz(_cdr_custom_msg(frame_id, np.zeros((0, 3))))
    assert empty.shape == (0, 3)
