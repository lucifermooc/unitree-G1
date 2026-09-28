import math, rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSReliabilityPolicy
from nav2_msgs.msg import Costmap
import tf2_ros

class P(Node):
    def __init__(self):
        super().__init__('cost_probe')
        q = QoSProfile(depth=1); q.durability = QoSDurabilityPolicy.TRANSIENT_LOCAL; q.reliability = QoSReliabilityPolicy.RELIABLE
        self.data = {}
        self.create_subscription(Costmap, '/local_costmap/costmap_raw', lambda m: self.cb('local', m), q)
        self.create_subscription(Costmap, '/global_costmap/costmap_raw', lambda m: self.cb('global', m), q)
        self.buf = tf2_ros.Buffer(); self.tl = tf2_ros.TransformListener(self.buf, self)
    def cb(self, k, m): self.data[k] = m

rclpy.init(); n = P()
import time
t0 = time.time()
while time.time() - t0 < 12.0 and len(n.data) < 2:
    rclpy.spin_once(n, timeout_sec=0.2)
try:
    tf = n.buf.lookup_transform('map', 'base_link', rclpy.time.Time())
    x = tf.transform.translation.x; y = tf.transform.translation.y
    print(f'base_link in map: x={x:.3f} y={y:.3f} z={tf.transform.translation.z:.3f}')
except Exception as e:
    print('TF fail', e); x = y = None
for k, m in n.data.items():
    md = m.metadata
    res = md.resolution; ox = md.origin.position.x; oy = md.origin.position.y
    w = md.size_x; h = md.size_y
    print(f'--- {k}: {w}x{h} res={res} origin=({ox:.2f},{oy:.2f}) frame={m.header.frame_id}')
    if x is None: continue
    mx = int((x-ox)/res); my = int((y-oy)/res)
    if not (0 <= mx < w and 0 <= my < h):
        print('  robot outside costmap'); continue
    c = m.data[my*w+mx]
    print(f'  cost at robot center cell ({mx},{my}) = {c}')
    for r in (0.20, 0.40, 0.85):
        n_cells = int(r/res); mxs = []
        vals = []
        for dy in range(-n_cells, n_cells+1):
            for dx in range(-n_cells, n_cells+1):
                if dx*dx+dy*dy > n_cells*n_cells: continue
                ax, ay = mx+dx, my+dy
                if 0 <= ax < w and 0 <= ay < h: vals.append(m.data[ay*w+ax])
        if vals:
            print(f'  within r={r}: max={max(vals)} n253={sum(1 for v in vals if v==253)} n254={sum(1 for v in vals if v==254)} n255={sum(1 for v in vals if v==255)}')
rclpy.shutdown()
