import rclpy, time, numpy as np
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
import sensor_msgs_py.point_cloud2 as pc2
import tf2_ros

class P(Node):
    def __init__(self):
        super().__init__('scan_probe2'); self.msgs=[]
        self.create_subscription(PointCloud2,'/lightning/registered_scan',lambda m:self.msgs.append(m),10)
        self.buf=tf2_ros.Buffer(); self.tl=tf2_ros.TransformListener(self.buf,self)
rclpy.init(); n=P(); t0=time.time()
while time.time()-t0<4.0: rclpy.spin_once(n,timeout_sec=0.2)
m=n.msgs[-1]
def mat(t):
    q=t.transform.rotation; tr=t.transform.translation; w,x,y,z=q.w,q.x,q.y,q.z
    R=np.array([[1-2*(y*y+z*z),2*(x*y-z*w),2*(x*z+y*w)],[2*(x*y+z*w),1-2*(x*x+z*z),2*(y*z-x*w)],[2*(x*z-y*w),2*(y*z+x*w),1-2*(x*x+y*y)]])
    return R,np.array([tr.x,tr.y,tr.z])
st=rclpy.time.Time.from_msg(m.header.stamp)
Rb,tb=mat(n.buf.lookup_transform('base_link',m.header.frame_id,st))
a=pc2.read_points(m,field_names=('x','y','z'),skip_nans=True)
p=np.stack([a['x'],a['y'],a['z']],axis=-1).astype(float)   # in mid360_link
rng=np.linalg.norm(p,axis=1)                               # range from sensor
b=(Rb@p.T).T+tb                                            # in base_link (= map z, plane mode)
d=np.hypot(b[:,0],b[:,1])
hz=(b[:,2]>0.10)&(b[:,2]<1.8)
print('total pts %d; sensor origin in base_link = %s'%(len(p), np.round(tb,3)))
print('--- 以 base_link 为中心的水平距离分布（仅 0.1<z<1.8 的"障碍高度"点）---')
for lo,hi in [(0,0.2),(0.2,0.4),(0.4,0.6),(0.6,0.85),(0.85,1.5),(1.5,3.0)]:
    s=hz&(d>=lo)&(d<hi)
    if s.sum():
        print(f'  d=[{lo},{hi}) n={int(s.sum()):5d}  z范围=[{b[s,2].min():.2f},{b[s,2].max():.2f}]  传感器距离range=[{rng[s].min():.2f},{rng[s].max():.2f}]  被obstacle_min_range(0.25)过滤={int((rng[s]<0.25).sum())}')
s=hz&(d>=0.4)&(d<0.85)
if s.sum():
    ang=np.degrees(np.arctan2(b[s,1],b[s,0]))
    print('  0.4~0.85m 的点方位角分布(度, 0=正前):', np.round(np.percentile(ang,[0,25,50,75,100]),1))
