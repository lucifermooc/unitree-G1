"""生成仿真用的办公室地图（map.pgm + map.yaml，和 map_server / lightning 保存的格式一致）。

    python3 make_office_map.py ../maps/office

布局（单位米，地图坐标系；机器人从 (0,0) 出发，站在走廊西头）：
  y  7 ┌──────┬────────────┬──────────┬──────────────┐
       │ 大厅 │   会议室   │  茶水间  │    办公区    │
     1.5│(前台)└──门────────┴───门─────┴──────门──────┤
       │  ●起点        走   廊                        │
    -1.5│      ┌──门──┬────门─────┬───门───┬────门────┤
       │      │卫生间│   仓库    │ 打印室 │   厨房   │
    -7 └──────┴──────┴───────────┴────────┴──────────┘
      x=-2    4      8          14       18         22
"""
import sys
from pathlib import Path

import numpy as np

RES, X0, Y0, X1, Y1 = 0.05, -2.0, -7.0, 22.0, 7.0
W, H = int((X1 - X0) / RES), int((Y1 - Y0) / RES)
FREE, OCC = 254, 0
img = np.full((H, W), FREE, np.uint8)


def px(x, y):
    return int((x - X0) / RES), H - 1 - int((y - Y0) / RES)


def box(x0, y0, x1, y1, value=OCC):
    (i0, j1), (i1, j0) = px(x0, y0), px(x1, y1)
    img[max(j0, 0):j1 + 1, max(i0, 0):i1 + 1] = value


def wall_h(y, x0, x1, doors=()):
    box(x0, y - 0.1, x1, y + 0.1)
    for d in doors:  # 门宽 1.2 m
        box(d - 0.6, y - 0.1, d + 0.6, y + 0.1, FREE)


def wall_v(x, y0, y1):
    box(x - 0.1, y0, x + 0.1, y1)


# 外墙
wall_h(Y0 + 0.1, X0, X1); wall_h(Y1 - 0.1, X0, X1); wall_v(X0 + 0.1, Y0, Y1); wall_v(X1 - 0.1, Y0, Y1)
# 走廊两侧墙（大厅 x<4 北侧敞开）
wall_h(1.5, 4.0, X1, doors=(7.0, 12.5, 18.5))
wall_h(-1.5, 2.0, X1, doors=(5.0, 11.0, 16.0, 20.0))
# 房间隔墙
for x in (10.0, 15.0):
    wall_v(x, 1.5, Y1)
wall_v(4.0, 1.5, Y1)
for x in (2.0, 8.0, 14.0, 18.0):
    wall_v(x, Y0, -1.5)
# 家具：前台桌、会议桌、茶水台、办公桌、货架、打印机、灶台
box(-0.5, 3.5, 1.5, 4.2)                                   # 前台桌
box(5.5, 3.5, 8.5, 5.0)                                    # 会议桌
box(10.4, 6.0, 12.0, 6.6)                                  # 茶水台（靠北墙）
for x in (16.0, 18.5):
    box(x, 3.0, x + 1.6, 3.8); box(x, 5.2, x + 1.6, 6.0)   # 办公桌
for y in (-3.5, -5.5):
    box(9.0, y, 13.0, y + 0.5)                             # 货架
box(14.4, -6.6, 15.4, -6.0)                                # 打印机
box(20.5, -6.6, 21.6, -3.0)                                # 灶台
box(2.4, -6.6, 3.2, -5.6); box(6.8, -6.6, 7.6, -5.6)      # 卫生间隔间

out = Path(sys.argv[1] if len(sys.argv) > 1 else ".")
out.mkdir(parents=True, exist_ok=True)
with open(out / "map.pgm", "wb") as f:
    f.write(f"P5\n{W} {H}\n255\n".encode())
    f.write(img.tobytes())
(out / "map.yaml").write_text(
    f"image: map.pgm\nmode: trinary\nresolution: {RES}\norigin: [{X0}, {Y0}, 0]\n"
    "negate: 0\noccupied_thresh: 0.65\nfree_thresh: 0.196\n")
print(f"wrote {out}/map.pgm ({W}x{H}) and map.yaml")
