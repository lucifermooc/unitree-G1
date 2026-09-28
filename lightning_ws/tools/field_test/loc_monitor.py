#!/usr/bin/env python3
"""Thor 上执行：监控定位健康度 + 导航状态，结束时给出结论。只读，不发布任何指令。
用法: python3 loc_monitor.py [秒数，默认 120]   走动/导航时边走边跑，Ctrl-C 提前结束。
定位判定：输出频率、位姿跳变（相邻帧 >0.3 m 或航向 >10°）、姿态是否平面、NDT 置信度与重力修正量，
          外加配准点云与静态地图的匹配率/残差（独立于定位器自评）。
导航判定：目标状态迁移、距目标距离、重规划次数、cmd_vel、局部代价地图中心代价与最近障碍、
          nav2 的 WARN/ERROR（Optimizer fail、清障、Goal failed 等）。"""
import glob, math, os, re, sys, time
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSReliabilityPolicy
from geometry_msgs.msg import PoseStamped, Twist

DUR = float(sys.argv[1]) if len(sys.argv) > 1 else 120.0
STATUS = {0: 'UNKNOWN', 1: 'ACCEPTED', 2: 'EXECUTING', 3: 'CANCELING',
          4: 'SUCCEEDED', 5: 'CANCELED', 6: 'ABORTED'}
WATCH = ('controller_server', 'planner_server', 'bt_navigator', 'behavior_server',
         'local_costmap', 'global_costmap', 'velocity_smoother')


def yaw_rp(q):
    r = math.degrees(math.atan2(2*(q.w*q.x+q.y*q.z), 1-2*(q.x*q.x+q.y*q.y)))
    p = math.degrees(math.asin(max(-1, min(1, 2*(q.w*q.y-q.z*q.x)))))
    y = math.degrees(math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z)))
    return r, p, y


class Mon(Node):
    def __init__(self):
        super().__init__('loc_monitor')
        self.n = 0; self.jumps = []; self.last = None; self.path = 0.0
        self.first = None; self.maxtilt = 0.0
        self.create_subscription(PoseStamped, '/base_link_pose', self.cb, 50)
        # ---------- 导航监控（nav2 没起时这些订阅是空转，不影响定位部分）----------
        self.goal = None; self.goal_events = []; self.plan_count = 0
        self.plan_end = None; self.cmd = (0.0, 0.0); self.cmd_t = 0.0
        self.errors = []; self.center_cost = None; self.near_lethal = None
        self.match = None; self.match_hist = []; self.plans = []; self.gcm = None
        self.goal_pose = None; self.arrivals = []; self.last_arrival_t = 0.0
        self.create_subscription(Twist, '/cmd_vel', self.cb_cmd, 1)
        try:
            from action_msgs.msg import GoalStatusArray
            from nav_msgs.msg import Path
            from rcl_interfaces.msg import Log
            self.create_subscription(GoalStatusArray, '/navigate_to_pose/_action/status',
                                     self.cb_goal, 10)
            self.create_subscription(Path, '/plan', self.cb_plan, 1)
            self.create_subscription(Log, '/rosout', self.cb_log, 50)
        except Exception as e:
            print(f'导航话题订阅失败（nav2 未运行？）: {e}')
        # ---------- 定位精度：配准点云 vs 静态地图 ----------
        self.dist = None; self.minfo = None; self.scan = None; self.tf = None
        try:
            import numpy, tf2_ros
            from nav_msgs.msg import OccupancyGrid
            from sensor_msgs.msg import PointCloud2
            from nav2_msgs.msg import Costmap
            tl = QoSProfile(depth=1)
            tl.durability = QoSDurabilityPolicy.TRANSIENT_LOCAL
            tl.reliability = QoSReliabilityPolicy.RELIABLE
            self.buf = tf2_ros.Buffer(); self.tf = tf2_ros.TransformListener(self.buf, self)
            self.create_subscription(OccupancyGrid, '/map', self.cb_map, tl)
            self.create_subscription(Costmap, '/local_costmap/costmap_raw', self.cb_costmap, tl)
            self.create_subscription(Costmap, '/global_costmap/costmap_raw',
                                     lambda m: setattr(self, 'gcm', m), tl)
            self.create_subscription(PointCloud2, '/lightning/registered_scan', self.cb_scan, 1)
            self.create_timer(1.0, self.tick_match)
        except Exception as e:
            print(f'匹配度监控不可用: {e}')

    # ---------- 定位 ----------
    def cb(self, m):
        p = m.pose.position; r, pi, y = yaw_rp(m.pose.orientation); t = time.time()
        self.n += 1; self.maxtilt = max(self.maxtilt, abs(r), abs(pi))
        if self.first is None: self.first = (p.x, p.y, y)
        if self.last:
            lx, ly, lyaw, lt = self.last
            d = math.hypot(p.x-lx, p.y-ly); dy = abs((y-lyaw+180) % 360-180)
            self.path += d
            if (d > 0.3 or dy > 10) and t-lt < 0.5:
                self.jumps.append((time.strftime('%T'), round(d, 3), round(dy, 1)))
                print(f'!! 跳变 {time.strftime("%T")} d={d:.3f}m dyaw={dy:.1f}deg')
        self.last = (p.x, p.y, y, t)

    # ---------- 导航 ----------
    def cb_cmd(self, m):
        self.cmd = (m.linear.x, m.angular.z); self.cmd_t = time.time()

    def cb_goal(self, m):
        if not m.status_list: return
        s = STATUS.get(m.status_list[-1].status, '?')
        if s != self.goal:
            print(f'>> 导航目标 {self.goal} -> {s}  {time.strftime("%T")}')
            self.goal_events.append((time.strftime('%T'), s))
            # 到点精度：EXECUTING -> SUCCEEDED 的瞬间，实际位姿与目标位姿之差。
            # 参考点取最后一次 /plan 的终点（规划器 tolerance=0.125，可能与真实目标差这么多）。
            # 容差：xy_goal_tolerance=0.20 m, yaw_goal_tolerance=0.20 rad=11.5deg
            if s == 'SUCCEEDED' and self.goal == 'EXECUTING' and self.goal_pose and self.last:
                gx, gy, gyaw = self.goal_pose
                x, y, yaw, _ = self.last
                dxy = math.hypot(x-gx, y-gy)
                dyaw = abs((yaw-gyaw+180) % 360-180)
                flag = '' if (dxy <= 0.20 and dyaw <= 11.5) else '  <== 超容差'
                print(f'>> 到点#{len(self.arrivals)+1} 误差 dxy={dxy*100:.1f}cm '
                      f'dyaw={dyaw:.1f}deg  目标({gx:.2f},{gy:.2f},{gyaw:.0f}deg) '
                      f'实际({x:.2f},{y:.2f},{yaw:.0f}deg){flag}')
                self.arrivals.append((time.strftime('%T'), dxy, dyaw))
                self.last_arrival_t = time.time()
                self.arrival_ref = (gx, gy)
            self.goal = s

    def cb_plan(self, m):
        self.plan_count += 1
        if not m.poses:
            return
        g = m.poses[-1].pose.position
        self.plan_end = (g.x, g.y)
        q = m.poses[-1].pose.orientation
        self.goal_pose = (g.x, g.y, math.degrees(math.atan2(
            2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z))))
        # 记录路径形状：绕行系数 = 路径长 / 首尾直线距离。>1.5 就是明显绕远，
        # 配合直线上的最大代价可判断是"被挡住绕行"还是"规划器自己绕"。
        try:
            import numpy as np
            pts = np.array([[q.pose.position.x, q.pose.position.y] for q in m.poses])
            L = float(np.hypot(*np.diff(pts, axis=0).T).sum()) if len(pts) > 1 else 0.0
            st = float(math.hypot(pts[-1][0]-pts[0][0], pts[-1][1]-pts[0][1]))
            ratio = L / max(st, 1e-6)
            info = (f'>> 重规划#{self.plan_count} ({pts[0][0]:.2f},{pts[0][1]:.2f})->'
                    f'({pts[-1][0]:.2f},{pts[-1][1]:.2f}) 直线{st:.2f}m 路径{L:.2f}m 绕行{ratio:.2f}x'
                    f' 包络x=[{pts[:,0].min():.1f},{pts[:,0].max():.1f}]'
                    f' y=[{pts[:,1].min():.1f},{pts[:,1].max():.1f}]')
            if ratio > 1.4 and self.gcm is not None:
                md = self.gcm.metadata
                g2 = np.array(self.gcm.data, dtype=np.uint8).reshape(md.size_y, md.size_x)
                N = max(2, int(st/0.1)); worst = 0; bad = 0; unk = 0
                for k in range(N+1):
                    t = k/N
                    x = pts[0][0]+(pts[-1][0]-pts[0][0])*t
                    y = pts[0][1]+(pts[-1][1]-pts[0][1])*t
                    ix = int((x-md.origin.position.x)/md.resolution)
                    iy = int((y-md.origin.position.y)/md.resolution)
                    if 0 <= ix < md.size_x and 0 <= iy < md.size_y:
                        c = int(g2[iy, ix]); worst = max(worst, c)
                        if c == 255: unk += 1
                        elif c >= 253: bad += 1
                info += f'  |直线上 最大代价={worst} 不可通行={bad}/{N+1} 未知={unk}/{N+1}'
            print(info)
            self.plans.append(info)
        except Exception as e:
            print(f'>> 重规划#{self.plan_count} 统计失败: {e}')

    def cb_log(self, m):
        if m.level < 30 or not any(w in m.name for w in WATCH): return
        lvl = {30: 'WARN', 40: 'ERROR', 50: 'FATAL'}.get(m.level, str(m.level))
        txt = f'{time.strftime("%T")} [{lvl}] {m.name}: {m.msg.strip()[:120]}'
        self.errors.append(txt); print('!! ' + txt)

    # ---------- 匹配度 ----------
    def cb_map(self, m):
        import numpy as np
        g = np.array(m.data, dtype=np.int16).reshape(m.info.height, m.info.width)
        occ = g >= 65
        self.minfo = m.info; self.occ = occ
        try:
            from scipy import ndimage
            self.dist = ndimage.distance_transform_edt(~occ) * m.info.resolution
        except Exception:
            self.dist = None
        print(f'地图已加载 {m.info.width}x{m.info.height} res={m.info.resolution:.3f} '
              f'占据格={int(occ.sum())} 距离场={"有" if self.dist is not None else "无(装scipy更准)"}')

    def cb_costmap(self, m): self.costmap = m
    def cb_scan(self, m): self.scan = m

    def tick_match(self):
        import numpy as np
        import sensor_msgs_py.point_cloud2 as pc2
        if self.minfo is None or self.scan is None: return
        try:
            t = self.buf.lookup_transform('map', self.scan.header.frame_id,
                                          rclpy.time.Time.from_msg(self.scan.header.stamp))
        except Exception:
            try:
                t = self.buf.lookup_transform('map', self.scan.header.frame_id, rclpy.time.Time())
            except Exception:
                return
        q = t.transform.rotation; w, x, y, z = q.w, q.x, q.y, q.z
        R = np.array([[1-2*(y*y+z*z), 2*(x*y-z*w), 2*(x*z+y*w)],
                      [2*(x*y+z*w), 1-2*(x*x+z*z), 2*(y*z-x*w)],
                      [2*(x*z-y*w), 2*(y*z+x*w), 1-2*(x*x+y*y)]])
        tr = t.transform.translation
        a = pc2.read_points(self.scan, field_names=('x', 'y', 'z'), skip_nans=True)
        p = np.stack([a['x'], a['y'], a['z']], axis=-1).astype(float)
        # 必须先按传感器距离剔除自身机体点：mid360 每帧约 2400 个点打在自己头上
        # (range<0.25 m)，占全帧 40%，它们落在脚下空地上会把残差中位数抬到 1~2 m，
        # 让匹配率指标完全失真。costmap 用 obstacle_min_range=0.25 做同样的事。
        rng = np.linalg.norm(p, axis=1)
        p = p[(rng > 0.5) & (rng < 15.0)]                  # 远处结构也不参与，避免未建图物体主导
        if len(p) < 50:
            return
        m = (R @ p.T).T + np.array([tr.x, tr.y, tr.z])
        m = m[(m[:, 2] > 0.15) & (m[:, 2] < 1.8)]          # 只用墙/家具高度的点
        if len(m) < 50: return
        i = self.minfo
        mx = ((m[:, 0]-i.origin.position.x)/i.resolution).astype(int)
        my = ((m[:, 1]-i.origin.position.y)/i.resolution).astype(int)
        ok = (mx >= 0) & (mx < i.width) & (my >= 0) & (my < i.height)
        mx, my = mx[ok], my[ok]
        if len(mx) < 50: return
        if self.dist is not None:
            d = self.dist[my, mx]
            # p25 才是定位偏移的判据：系统性偏移会把整个分布右移（连贴墙点都对不上）；
            # 未建图的桌椅杂物只抬高中位数和高分位，p25 仍然接近 0。
            self.match = (float((d <= i.resolution*1.5).mean()), float(np.percentile(d, 25)),
                          float(np.median(d)), len(mx))
        else:
            self.match = (float(self.occ[my, mx].mean()), float('nan'), float('nan'), len(mx))
        self.match_hist.append(self.match[0])
        # 顺带刷新局部代价地图指标
        try:
            cm = self.costmap; md = cm.metadata
            g = np.array(cm.data, dtype=np.uint8).reshape(md.size_y, md.size_x)
            bx, by, _, _ = self.last
            cx = int((bx-md.origin.position.x)/md.resolution)
            cy = int((by-md.origin.position.y)/md.resolution)
            if 0 <= cx < md.size_x and 0 <= cy < md.size_y:
                self.center_cost = int(g[cy, cx])
                ys, xs = np.where(g == 254)
                self.near_lethal = (float(np.hypot((xs-cx)*md.resolution,
                                                   (ys-cy)*md.resolution).min())
                                    if len(xs) else None)
        except Exception:
            pass


rclpy.init(); n = Mon(); t0 = time.time(); tp = t0
try:
    while time.time()-t0 < DUR:
        rclpy.spin_once(n, timeout_sec=0.1)
        if time.time()-tp > 5 and n.last:
            tp = time.time(); x, y, yaw, _ = n.last
            line = [f'{time.strftime("%T")} pose=({x:.2f},{y:.2f}) yaw={yaw:.1f}',
                    f'rate={n.n/(time.time()-t0):.1f}Hz', f'path={n.path:.1f}m']
            if n.match:
                r, p25, med, cnt = n.match
                flag = '' if (math.isnan(p25) or p25 < 0.20) else ' <==定位偏移可疑'
                line.append(f'贴合={r*100:.0f}% p25={p25*100:.1f}cm 中位={med*100:.1f}cm n={cnt}{flag}')
            line.append(f'导航={n.goal or "无目标"}')
            if time.time()-n.last_arrival_t < 6.0 and getattr(n, 'arrival_ref', None):
                ax, ay = n.arrival_ref
                line.append(f'到点后残差={math.hypot(x-ax, y-ay)*100:.0f}cm')
            if n.plan_end and n.goal == 'EXECUTING':
                line.append(f'距目标={math.hypot(n.plan_end[0]-x, n.plan_end[1]-y):.2f}m')
                line.append(f'重规划={n.plan_count}')
            if time.time()-n.cmd_t < 1.0:
                line.append(f'cmd=({n.cmd[0]:+.2f},{n.cmd[1]:+.2f})')
            if n.center_cost is not None:
                nl = f'{n.near_lethal:.2f}m' if n.near_lethal else '无'
                line.append(f'中心代价={n.center_cost} 最近障碍={nl}')
            print('  '.join(line))
except KeyboardInterrupt:
    pass
el = time.time()-t0; rate = n.n/el if el > 0 else 0
L = sorted(glob.glob('/tmp/run_loc_online*INFO*'), key=os.path.getmtime)
conf, corr = [], []
if L:
    for line in open(L[-1], errors='ignore'):
        m = re.search(r'\] confidence: ([\d.]+), t:', line)
        if m: conf.append(float(m.group(1)))
        m = re.search(r'gravity constrain\(track\): corrected ([\d.]+) deg', line)
        if m: corr.append(float(m.group(1)))
conf, corr = conf[-100:], corr[-100:]
print('\n===== 定位结论 =====')
print(f'时长 {el:.0f}s  消息 {n.n}  频率 {rate:.1f}Hz  路程 {n.path:.1f}m  跳变 {len(n.jumps)}  最大 roll/pitch {n.maxtilt:.2f}deg')
if n.first and n.last:
    print(f'起点->终点 位移 {math.hypot(n.last[0]-n.first[0], n.last[1]-n.first[1]):.3f}m  (绕圈回原点时即闭环误差)')
if conf: print(f'NDT 置信度(近100次) min {min(conf):.2f} avg {sum(conf)/len(conf):.2f}  (<1.3 需关注)')
if corr: print(f'重力修正量(近100次) max {max(corr):.2f}deg avg {sum(corr)/len(corr):.2f}deg')
if n.match_hist:
    h = n.match_hist
    print(f'点云贴墙率(残差<7.5cm) min {min(h)*100:.0f}% avg {sum(h)/len(h)*100:.0f}%'
          f'   (接近 0 才说明定位偏了；只是偏低通常是未建图的桌椅杂物)')
print('\n===== 导航结论 =====')
print(f'目标状态迁移: {n.goal_events if n.goal_events else "本次没有导航目标"}')
print(f'全局重规划次数 {n.plan_count}')
if n.arrivals:
    import statistics as st
    dxy = [a[1] for a in n.arrivals]; dyaw = [a[2] for a in n.arrivals]
    over = sum(1 for a in n.arrivals if a[1] > 0.20 or a[2] > 11.5)
    print(f'到点 {len(n.arrivals)} 次  位置误差 中位={st.median(dxy)*100:.1f}cm '
          f'最大={max(dxy)*100:.1f}cm   航向误差 中位={st.median(dyaw):.1f}deg 最大={max(dyaw):.1f}deg')
    print(f'   超出容差(20cm/11.5deg) {over}/{len(n.arrivals)} 次')
    for t, a, b in n.arrivals[-8:]:
        print(f'   {t}  dxy={a*100:5.1f}cm  dyaw={b:5.1f}deg')
else:
    print('本次没有成功到点')
if n.plans:
    print('绕行最严重的 5 次重规划：')
    for t in sorted(n.plans, key=lambda x: -float(x.split('绕行')[1].split('x')[0]))[:5]:
        print('   ' + t)
if n.errors:
    print(f'nav2 告警/错误 {len(n.errors)} 条，最后 10 条：')
    for e in n.errors[-10:]: print('   ' + e)
else:
    print('nav2 无 WARN/ERROR')
ok = (rate > 5 and not n.jumps and n.maxtilt < 0.5 and (not conf or min(conf) > 1.3)
      and (not n.match_hist or max(n.match_hist) > 0.05))
print('\n定位 PASS' if ok else '\n定位 FAIL / 需人工看上面各项')
n.destroy_node(); rclpy.shutdown()
