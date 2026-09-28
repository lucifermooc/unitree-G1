#!/usr/bin/env python3
"""robot_radius 扫描：判定 footprint 自清是否真的关闭。
判据2(决定性)：最近 lethal 不随 robot_radius 变化。
判据3：published_footprint 的实际半径确实跟着变了（证明改的是真足迹，不只是参数服务器）。
判据4：足迹内是否出现自身点。只读 costmap + 改 robot_radius，不发任何速度。"""
import rclpy, time, math, subprocess, numpy as np
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSReliabilityPolicy
from nav2_msgs.msg import Costmap
from geometry_msgs.msg import PolygonStamped
import tf2_ros

class P(Node):
    def __init__(self):
        super().__init__('sweep'); self.cm=None; self.fp=None
        q=QoSProfile(depth=1); q.durability=QoSDurabilityPolicy.TRANSIENT_LOCAL; q.reliability=QoSReliabilityPolicy.RELIABLE
        self.create_subscription(Costmap,'/local_costmap/costmap_raw',lambda m:setattr(self,'cm',m),q)
        self.create_subscription(PolygonStamped,'/local_costmap/published_footprint',lambda m:setattr(self,'fp',m),1)
        self.buf=tf2_ros.Buffer(); self.tl=tf2_ros.TransformListener(self.buf,self)

rclpy.init(); n=P()
def spin(s):
    t0=time.time()
    while time.time()-t0<s: rclpy.spin_once(n,timeout_sec=0.1)
spin(3.0)

def meas(tag):
    n.cm=None; n.fp=None; spin(4.0)
    tf=n.buf.lookup_transform('map','base_link',rclpy.time.Time())
    rx,ry=tf.transform.translation.x,tf.transform.translation.y
    q=tf.transform.rotation; yaw=math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))
    # 判据3：实际足迹半径（相对机器人当前位置）
    fr_r=float('nan')
    if n.fp is not None and n.fp.polygon.points:
        fr_r=max(math.hypot(p.x-rx,p.y-ry) for p in n.fp.polygon.points)
    md=n.cm.metadata; res=md.resolution
    g=np.array(n.cm.data,dtype=np.uint8).reshape(md.size_y,md.size_x)
    cx=int((rx-md.origin.position.x)/res); cy=int((ry-md.origin.position.y)/res)
    ys,xs=np.where(g==254)
    near=frnear=float('nan'); infp=0
    if len(xs):
        d=np.hypot((xs-cx)*res,(ys-cy)*res)
        a=(np.degrees(np.arctan2((ys-cy)*res,(xs-cx)*res))-math.degrees(yaw)+180)%360-180
        f=np.abs(a)<45
        near=float(d.min()); infp=int((d<fr_r).sum()) if not math.isnan(fr_r) else -1
        if f.any(): frnear=float(d[f].min())
    print(f'[{tag}] 中心代价={g[cy,cx]:3d}  最近lethal全向={near:.3f}m  正前±45°={frnear:.3f}m  '
          f'实际足迹半径={fr_r:.3f}m  足迹内lethal={infp}', flush=True)
    return near, frnear, fr_r

def setr(v):
    for nd in ('/local_costmap/local_costmap','/global_costmap/global_costmap'):
        subprocess.run(['ros2','param','set',nd,'robot_radius',str(v)],capture_output=True,timeout=10)
    spin(3.0)

res={}
res[0.40]=meas('r=0.40 起始')
for v in (0.20,0.30,0.55):
    setr(v); res[v]=meas(f'r={v:.2f}')
setr(0.40); res['restore']=meas('r=0.40 恢复')

print('\n===== 判定 =====')
fronts=[res[k][1] for k in (0.40,0.20,0.30,0.55) if not math.isnan(res[k][1])]
fps=[res[k][2] for k in (0.40,0.20,0.30,0.55)]
print(f'各档正前最近 lethal: {[f"{x:.3f}" for x in fronts]}')
print(f'各档实际足迹半径:   {[f"{x:.3f}" for x in fps]}')
if len(fronts)>=3:
    spread=max(fronts)-min(fronts)
    print(f'最近 lethal 极差 = {spread*100:.1f} cm')
    print('→ 自清已真正关闭（读数与半径解耦）' if spread<0.06 else '→ 自清仍在起作用（读数跟着半径走）')
rclpy.shutdown()
