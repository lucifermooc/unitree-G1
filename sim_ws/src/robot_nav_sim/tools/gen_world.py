#!/usr/bin/env python3
"""从同一份场景描述生成 Gazebo 世界、每层的 2D 真值地图和楼层配置。

改场景只改这个文件，然后重新运行：
    python3 tools/gen_world.py
会覆盖 worlds/*.sdf、maps/*.pgm|yaml、config/floors.yaml。

坐标约定：建筑内部 x∈[0,16]、y∈[0,10]（米），二楼地面高 FLOOR_H。
楼梯沿北墙从西往东上：碰撞体是一段光滑斜坡（隐藏轮子能爬），
外观是 15 级台阶（雷达看到的是台阶）。
"""
import math
import os

HERE = os.path.dirname(os.path.abspath(__file__))
PKG = os.path.dirname(HERE)

# ---------------- 场景参数 ----------------
W, D = 16.0, 10.0          # 建筑内部尺寸
WALL_T = 0.2               # 外墙厚度
FLOOR_H = 2.6              # 二楼地面高度
SLAB_T = 0.2               # 楼板厚度
TOP_H = 5.0                # 外墙/二楼墙顶高度

N_STEPS = 15
STAIR_X0 = 8.0             # 第一级台阶前沿
STAIR_Y = (8.4, 9.9)       # 楼梯宽度方向
RISE = FLOOR_H / N_STEPS
TREAD = 0.3733
STAIR_X1 = STAIR_X0 + N_STEPS * TREAD          # 最后一级台阶后沿 = 13.6
RAMP_X0 = STAIR_X0 - TREAD                     # 斜坡起点（地面）
RAMP_X1 = STAIR_X0 + (N_STEPS - 1) * TREAD     # 斜坡终点（二楼地面）
OPEN_X = (7.6, RAMP_X1)                        # 二楼楼板上的楼梯开口

SCAN_BAND = (0.1, 1.0)     # 2D 地图只画这个离地高度区间里的障碍（与 /scan 一致）
RES = 0.05
MAP_MARGIN = 1.0

COL = {
    "wall": "0.86 0.86 0.84 1", "inner": "0.80 0.82 0.85 1", "slab": "0.78 0.74 0.68 1",
    "stair": "0.62 0.50 0.38 1", "rail": "0.35 0.38 0.42 1", "pillar": "0.70 0.70 0.72 1",
    "wood": "0.55 0.40 0.28 1", "fabric": "0.30 0.42 0.55 1", "crate": "0.72 0.58 0.36 1",
    "metal": "0.45 0.47 0.50 1",
}


def box(name, x0, x1, y0, y1, z0, z1, color, collide=True, visual=True):
    return dict(name=name, x0=x0, x1=x1, y0=y0, y1=y1, z0=z0, z1=z1, color=color,
                collide=collide, visual=visual, pitch=0.0)


def floor1_items(with_stairs):
    h = FLOOR_H - SLAB_T if with_stairs else 2.6
    it = []
    # 一楼内墙：x=5 的隔墙（门洞 y 3.6~4.8），y=4.5 的隔墙（门洞 x 12.0~13.2）
    it += [box("f1_w1a", 4.925, 5.075, 0.0, 3.6, 0, h, COL["inner"]),
           box("f1_w1b", 4.925, 5.075, 4.8, 8.2, 0, h, COL["inner"]),
           box("f1_w2a", 10.5, 12.0, 4.425, 4.575, 0, h, COL["inner"]),
           box("f1_w2b", 13.2, 16.0, 4.425, 4.575, 0, h, COL["inner"])]
    it += [box("f1_pillar1", 7.8, 8.2, 1.8, 2.2, 0, h, COL["pillar"]),
           box("f1_pillar2", 7.8, 8.2, 5.8, 6.2, 0, h, COL["pillar"]),
           box("f1_crate", 1.6, 2.4, 1.7, 2.3, 0, 0.7, COL["crate"]),
           box("f1_sofa", 1.6, 3.4, 6.6, 7.4, 0, 0.8, COL["fabric"]),
           box("f1_table", 12.4, 13.6, 1.6, 2.4, 0, 0.75, COL["wood"]),
           box("f1_cabinet", 15.4, 15.9, 5.5, 7.5, 0, 1.6, COL["metal"])]
    if with_stairs:
        # 楼梯侧墙：把楼梯和楼梯下面的空间围起来，入口在西端
        it.append(box("stair_side_wall", 7.6, W, 8.2, 8.4, 0, FLOOR_H - SLAB_T, COL["inner"]))
    return it


def floor2_items():
    z0 = FLOOR_H
    it = []
    # 楼板（留出楼梯开口）
    it += [box("slab_a", 0, W, 0, STAIR_Y[0], z0 - SLAB_T, z0, COL["slab"]),
           box("slab_b", 0, OPEN_X[0], STAIR_Y[0], D, z0 - SLAB_T, z0, COL["slab"]),
           box("slab_c", OPEN_X[1], W, STAIR_Y[0], D, z0 - SLAB_T, z0, COL["slab"])]
    # 楼梯口栏杆
    it += [box("rail_s", OPEN_X[0] - 0.1, OPEN_X[1], 8.25, 8.35, z0, z0 + 1.0, COL["rail"]),
           box("rail_w", OPEN_X[0] - 0.15, OPEN_X[0] - 0.05, 8.25, STAIR_Y[1], z0, z0 + 1.0, COL["rail"])]
    top = TOP_H
    # 二楼内墙：y=5 的隔墙（门洞 x 2.5~3.7），x=10.5 的隔墙（门洞 y 3.5~4.7）
    it += [box("f2_w3a", 0.0, 2.5, 4.925, 5.075, z0, top, COL["inner"]),
           box("f2_w3b", 3.7, 7.0, 4.925, 5.075, z0, top, COL["inner"]),
           box("f2_w4a", 10.425, 10.575, 0.0, 3.5, z0, top, COL["inner"]),
           box("f2_w4b", 10.425, 10.575, 4.7, 7.5, z0, top, COL["inner"])]
    it += [box("f2_pillar1", 7.8, 8.2, 1.8, 2.2, z0, top, COL["pillar"]),
           box("f2_pillar2", 7.8, 8.2, 5.8, 6.2, z0, top, COL["pillar"]),
           box("f2_shelf", 0.5, 2.5, 0.8, 1.2, z0, z0 + 1.5, COL["metal"]),
           box("f2_desk", 3.3, 4.7, 7.15, 7.85, z0, z0 + 0.75, COL["wood"]),
           box("f2_boxes", 11.7, 12.3, 1.7, 2.3, z0, z0 + 0.6, COL["crate"]),
           box("f2_bench", 13.0, 15.0, 6.0, 6.5, z0, z0 + 0.45, COL["wood"])]
    return it


def outer_walls(top):
    t = WALL_T
    return [box("wall_s", -t, W + t, -t, 0, 0, top, COL["wall"]),
            box("wall_n", -t, W + t, D, D + t, 0, top, COL["wall"]),
            box("wall_w", -t, 0, 0, D, 0, top, COL["wall"]),
            box("wall_e", W, W + t, 0, D, 0, top, COL["wall"])]


def stairs():
    it = []
    for i in range(N_STEPS):
        x0 = STAIR_X0 + i * TREAD
        z1 = (i + 1) * RISE - (0.002 if i == N_STEPS - 1 else 0.0)
        it.append(box(f"step_{i:02d}", x0, x0 + TREAD, STAIR_Y[0], STAIR_Y[1], 0.0, z1,
                      COL["stair"], collide=False))
    # 隐藏的斜坡碰撞体：表面经过每一级台阶的前沿
    run = RAMP_X1 - RAMP_X0
    ang = math.atan2(FLOOR_H, run)
    # 只在坡脚多伸出一段（埋进地面以下），坡顶必须正好和二楼楼板平齐，
    # 否则多出来的部分会高出楼板几厘米，下楼时会被卡住
    ext = 0.15
    length = math.hypot(run, FLOOR_H) + ext
    thick = 0.2
    ramp = box("stair_ramp", 0, length, STAIR_Y[0], STAIR_Y[1], -thick, 0, COL["stair"], visual=False)
    # 用中心+俯仰角表示：上表面中点沿坡向下挪 ext/2，再沿法向往下挪半个厚度
    cx = (RAMP_X0 + RAMP_X1) / 2 - math.cos(ang) * ext / 2 + math.sin(ang) * thick / 2
    cz = FLOOR_H / 2 - math.sin(ang) * ext / 2 - math.cos(ang) * thick / 2
    ramp.update(center=(cx, sum(STAIR_Y) / 2, cz), size=(length, STAIR_Y[1] - STAIR_Y[0], thick), pitch=-ang)
    it.append(ramp)
    return it


# ---------------- SDF 输出 ----------------
def sdf_box(b):
    if "center" in b:
        cx, cy, cz = b["center"]
        sx, sy, sz = b["size"]
    else:
        cx, cy, cz = (b["x0"] + b["x1"]) / 2, (b["y0"] + b["y1"]) / 2, (b["z0"] + b["z1"]) / 2
        sx, sy, sz = b["x1"] - b["x0"], b["y1"] - b["y0"], b["z1"] - b["z0"]
    pose = f"{cx:.4f} {cy:.4f} {cz:.4f} 0 {b['pitch']:.5f} 0"
    geo = f"<geometry><box><size>{sx:.4f} {sy:.4f} {sz:.4f}</size></box></geometry>"
    out = []
    if b["collide"]:
        out.append(f'      <collision name="{b["name"]}_c"><pose>{pose}</pose>{geo}'
                   f'<surface><friction><ode><mu>1.0</mu><mu2>1.0</mu2></ode></friction></surface></collision>')
    if b["visual"]:
        out.append(f'      <visual name="{b["name"]}_v"><pose>{pose}</pose>{geo}'
                   f'<material><ambient>{b["color"]}</ambient><diffuse>{b["color"]}</diffuse>'
                   f'<specular>0.1 0.1 0.1 1</specular></material></visual>')
    return "\n".join(out)


def world_sdf(name, items):
    body = "\n".join(sdf_box(b) for b in items)
    return f"""<?xml version="1.0"?>
<!-- 由 tools/gen_world.py 生成，请勿手改 -->
<sdf version="1.9">
  <world name="{name}">
    <physics name="1ms" type="ignored">
      <max_step_size>0.002</max_step_size>
      <real_time_factor>1.0</real_time_factor>
    </physics>
    <plugin filename="gz-sim-physics-system" name="gz::sim::systems::Physics"/>
    <plugin filename="gz-sim-user-commands-system" name="gz::sim::systems::UserCommands"/>
    <plugin filename="gz-sim-scene-broadcaster-system" name="gz::sim::systems::SceneBroadcaster"/>
    <plugin filename="gz-sim-sensors-system" name="gz::sim::systems::Sensors">
      <render_engine>ogre2</render_engine>
    </plugin>
    <plugin filename="gz-sim-imu-system" name="gz::sim::systems::Imu"/>

    <gui fullscreen="0">
      <camera name="user_camera"><pose>-6 -8 10 0 0.6 0.9</pose></camera>
    </gui>

    <scene>
      <ambient>0.55 0.55 0.55 1</ambient>
      <background>0.78 0.84 0.90 1</background>
      <shadows>true</shadows>
    </scene>

    <light type="directional" name="sun">
      <cast_shadows>true</cast_shadows>
      <pose>0 0 20 0 0 0</pose>
      <diffuse>0.85 0.85 0.82 1</diffuse>
      <specular>0.2 0.2 0.2 1</specular>
      <direction>-0.4 0.3 -0.9</direction>
    </light>

    <model name="ground_plane">
      <static>true</static>
      <link name="link">
        <collision name="c"><geometry><plane><normal>0 0 1</normal><size>200 200</size></plane></geometry>
          <surface><friction><ode><mu>1.0</mu><mu2>1.0</mu2></ode></friction></surface></collision>
        <visual name="v"><geometry><plane><normal>0 0 1</normal><size>200 200</size></plane></geometry>
          <material><ambient>0.62 0.63 0.60 1</ambient><diffuse>0.62 0.63 0.60 1</diffuse></material></visual>
      </link>
    </model>

    <model name="building">
      <static>true</static>
      <link name="structure">
{body}
      </link>
    </model>
  </world>
</sdf>
"""


# ---------------- 2D 真值地图 ----------------
def rasterize(items, floor_z, extra_occupied=()):
    x_min, y_min = -WALL_T - MAP_MARGIN, -WALL_T - MAP_MARGIN
    x_max, y_max = W + WALL_T + MAP_MARGIN, D + WALL_T + MAP_MARGIN
    nx, ny = int(round((x_max - x_min) / RES)), int(round((y_max - y_min) / RES))
    FREE, OCC, UNK = 254, 0, 205
    grid = [[UNK] * nx for _ in range(ny)]
    for j in range(ny):
        y = y_min + (j + 0.5) * RES
        for i in range(nx):
            x = x_min + (i + 0.5) * RES
            if 0 <= x <= W and 0 <= y <= D:
                grid[j][i] = FREE
    lo, hi = floor_z + SCAN_BAND[0], floor_z + SCAN_BAND[1]
    rects = [(b["x0"], b["x1"], b["y0"], b["y1"]) for b in items
             if b.get("visual") and "center" not in b and b["z1"] > lo and b["z0"] < hi]
    rects += list(extra_occupied)
    for (x0, x1, y0, y1) in rects:
        i0 = max(0, int(math.floor((x0 - x_min) / RES)))
        i1 = min(nx, int(math.ceil((x1 - x_min) / RES)))
        j0 = max(0, int(math.floor((y0 - y_min) / RES)))
        j1 = min(ny, int(math.ceil((y1 - y_min) / RES)))
        for j in range(j0, j1):
            for i in range(i0, i1):
                grid[j][i] = OCC
    return grid, (x_min, y_min)


def write_map(stem, grid, origin):
    ny, nx = len(grid), len(grid[0])
    with open(os.path.join(PKG, "maps", stem + ".pgm"), "wb") as f:
        f.write(f"P5\n# {stem} generated by gen_world.py\n{nx} {ny}\n255\n".encode())
        for row in reversed(grid):   # PGM 第一行是地图最上方（y 最大）
            f.write(bytes(row))
    with open(os.path.join(PKG, "maps", stem + ".yaml"), "w") as f:
        f.write(f"image: {stem}.pgm\nmode: trinary\nresolution: {RES}\n"
                f"origin: [{origin[0]:.3f}, {origin[1]:.3f}, 0.0]\n"
                "negate: 0\noccupied_thresh: 0.65\nfree_thresh: 0.25\n")


def main():
    os.makedirs(os.path.join(PKG, "worlds"), exist_ok=True)
    os.makedirs(os.path.join(PKG, "maps"), exist_ok=True)

    f1_flat = floor1_items(with_stairs=False)
    flat = outer_walls(2.6) + f1_flat
    with open(os.path.join(PKG, "worlds", "flat.sdf"), "w") as f:
        f.write(world_sdf("flat", flat))
    write_map("flat", *rasterize(flat, 0.0))

    f1 = floor1_items(with_stairs=True)
    f2 = floor2_items()
    st = stairs()
    two = outer_walls(TOP_H) + f1 + f2 + st
    with open(os.path.join(PKG, "worlds", "two_floor.sdf"), "w") as f:
        f.write(world_sdf("two_floor", two))
    # 一楼：楼梯和楼梯下方整体不可通行；二楼：楼梯开口不可通行
    stair_block = (STAIR_X0, W, STAIR_Y[0], STAIR_Y[1])
    write_map("floor1", *rasterize(outer_walls(TOP_H) + f1 + st, 0.0, [stair_block]))
    open_block = (OPEN_X[0], OPEN_X[1], STAIR_Y[0], STAIR_Y[1])
    write_map("floor2", *rasterize(outer_walls(TOP_H) + f2, FLOOR_H, [open_block]))

    yc = sum(STAIR_Y) / 2
    with open(os.path.join(PKG, "config", "floors.yaml"), "w") as f:
        f.write(f"""# 由 tools/gen_world.py 生成。楼层 1 在下，楼层 2 在上。
# entry：上/下楼梯前要导航到的位置（朝向楼梯）；landing：走完楼梯后停下的位置。
floors:
  1:
    height: 0.0
    map: floor1.yaml
    stair_entry: {{x: 6.9, y: {yc:.2f}, yaw: 0.0}}
    stair_landing: {{x: 6.9, y: {yc:.2f}, yaw: 3.1416}}
  2:
    height: {FLOOR_H}
    map: floor2.yaml
    stair_entry: {{x: 14.6, y: {yc:.2f}, yaw: 3.1416}}
    stair_landing: {{x: 14.4, y: {yc:.2f}, yaw: 0.0}}
stair:
  slope_deg: {math.degrees(math.atan2(FLOOR_H, RAMP_X1 - RAMP_X0)):.1f}
  length_m: {math.hypot(RAMP_X1 - RAMP_X0, FLOOR_H):.2f}
""")
    print("written: worlds/flat.sdf worlds/two_floor.sdf maps/{flat,floor1,floor2}.* config/floors.yaml")
    print(f"stair slope {math.degrees(math.atan2(FLOOR_H, RAMP_X1 - RAMP_X0)):.1f} deg, ramp x {RAMP_X0:.3f}..{RAMP_X1:.3f}")


if __name__ == "__main__":
    main()
