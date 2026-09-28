"""map_preview_bridge 的转换函数测试（不需要 ROS 运行）。"""
import base64
from types import SimpleNamespace

import cv2
import numpy as np

from g1_web.map_preview_bridge import gray_to_payload, grid_to_gray


def make_grid(w, h, data):
    return SimpleNamespace(info=SimpleNamespace(width=w, height=h), data=data)


def test_grid_to_gray_values_and_flip():
    # 2 行 3 列，第 0 行是地图最下面一行
    grid = make_grid(3, 2, [0, 100, -1,
                            50, 10, 70])
    img = grid_to_gray(grid)
    assert img.shape == (2, 3)
    assert img[1].tolist() == [254, 0, 205]      # 最下面一行在图片最后一行
    assert img[0].tolist() == [205, 254, 0]      # 50 → 未知（阈值之间）


def test_payload_decodes_like_frontend():
    img = np.full((40, 60), 254, np.uint8)
    img[10:20, 10:30] = 0
    payload = gray_to_payload(img)
    assert all(0 <= v < 128 for v in payload)    # base64 字符都是 ASCII，放得进 int8
    jpg = base64.b64decode(bytes(payload).decode())
    decoded = cv2.imdecode(np.frombuffer(jpg, np.uint8), cv2.IMREAD_COLOR)
    assert decoded.shape == (40, 60, 3)
    assert decoded[15, 20].sum() < 100            # 障碍物是黑的
    b, g, r = decoded[35, 50]
    assert abs(int(b) - 200) < 20 and abs(int(r) - 127) < 20   # 空闲区与 get_map_image 同色
