#!/usr/bin/env python3
"""obstacle_layer（MID360 层）残影取证 / A/B：无 MID360 支撑的 254 格 vs 有支撑格，按 MID360 前方盲区 ±30° / 其余分开，
再给出清除延迟（最后一次有 MID360 点支撑 → 格子消失）。

    python3 obstacle_residual_ab.py <原始bag> <被测bag1> [被测bag2 ...] [--topic T] [--skip 秒]

--topic 默认自动找 /obstacle_layer_raw（回放）或 /local_costmap/obstacle_layer_raw（线上）；
global 用 --topic /global_costmap/obstacle_layer_raw（回放 --section global_costmap 时仍是 /obstacle_layer_raw）。
--skip 跳过开头的冷启动秒数（按时间，不按帧：local 5 Hz、global 1 Hz）。

被测 bag 可以是回放台输出（话题 /obstacle_layer_raw，根命名空间），也可以是线上实录
（/local_costmap/obstacle_layer_raw）——原始 bag 自己也可以作为被测 bag。
MID360 支撑点取自原始 bag 的 /lightning/registered_scan（z 0.10~1.8，距雷达 >0.25 m，±7.5 cm 容差）。
2026-09-23 walkby_0923：线上实录前方无支撑 89 格/帧、最长挂 174 s；回放 base 70.2 → 加 D435 只清除源 0.0。
"""
import sys, math, numpy as np, rosbag2_py
from collections import deque
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
import tf2_ros, rclpy.time, rclpy.duration, sensor_msgs_py.point_cloud2 as pc2
from scipy.spatial import cKDTree
import argparse
ap = argparse.ArgumentParser()
ap.add_argument('src_bag'); ap.add_argument('reps', nargs='+')
ap.add_argument('--topic', default=None); ap.add_argument('--skip', type=float, default=4.0)
args = ap.parse_args()
src_bag, reps = args.src_bag, args.reps
LAYER_TOPICS = [args.topic] if args.topic else ['/obstacle_layer_raw', '/local_costmap/obstacle_layer_raw']
def after_skip(frames):
    if not frames: return frames
    t0 = rclpy.time.Time.from_msg(frames[0].header.stamp).nanoseconds / 1e9
    return [m for m in frames if rclpy.time.Time.from_msg(m.header.stamp).nanoseconds / 1e9 >= t0 + args.skip]
def mat(tr):
    q=tr.transform.rotation; t=tr.transform.translation; w,x,y,z=q.w,q.x,q.y,q.z
    return np.array([[1-2*(y*y+z*z),2*(x*y-z*w),2*(x*z+y*w)],[2*(x*y+z*w),1-2*(x*x+z*z),2*(y*z-x*w)],[2*(x*z-y*w),2*(y*z+x*w),1-2*(x*x+y*y)]]),np.array([t.x,t.y,t.z])
def read(bag, topics):
    r=rosbag2_py.SequentialReader(); r.open(rosbag2_py.StorageOptions(uri=bag,storage_id='mcap'),rosbag2_py.ConverterOptions('',''))
    types={t.name:t.type for t in r.get_all_topics_and_types()}
    r.set_filter(rosbag2_py.StorageFilter(topics=[t for t in topics if t in types]))
    while r.has_next():
        tp,data,st=r.read_next(); yield tp, deserialize_message(data,get_message(types[tp]))
# 回放输出里的 obstacle_layer + tf
def load_rep(bag):
    buf=tf2_ros.Buffer(cache_time=rclpy.duration.Duration(seconds=3600)); fr=[]
    for tp,m in read(bag,['/tf','/tf_static'] + LAYER_TOPICS):
        if tp=='/tf_static':
            for x in m.transforms: buf.set_transform_static(x,'b')
        elif tp=='/tf':
            for x in m.transforms: buf.set_transform(x,'b')
        else: fr.append(m)
    return buf, fr
buf0, fr0 = load_rep(reps[0])
t_lo=min(rclpy.time.Time.from_msg(m.header.stamp).nanoseconds for m in fr0)/1e9-2
t_hi=max(rclpy.time.Time.from_msg(m.header.stamp).nanoseconds for m in fr0)/1e9+1
# 原始 bag 的 MID360 点（map 系，z 0.10~1.8，距雷达 >0.25）
lid=[]
for tp,m in read(src_bag,['/lightning/registered_scan']):
    t=rclpy.time.Time.from_msg(m.header.stamp).nanoseconds/1e9
    if t<t_lo or t>t_hi: continue
    try: tr=buf0.lookup_transform('map',m.header.frame_id,rclpy.time.Time.from_msg(m.header.stamp))
    except Exception: continue
    R,tv=mat(tr); a=pc2.read_points(m,field_names=('x','y','z'),skip_nans=True); p=np.stack([a['x'],a['y'],a['z']],-1).astype(float)
    p=p[np.linalg.norm(p,axis=1)>0.25]; q=p@R.T+tv; q=q[(q[:,2]>=0.10)&(q[:,2]<=1.8)]
    lid.append((t,q[:,:2]))
print(f'MID360 帧 {len(lid)}  窗口 {t_hi-t_lo:.0f}s')
for bag in reps:
    buf, fr = (buf0, fr0) if bag==reps[0] else load_rep(bag)
    S={'front_stale':[],'front_sup':[],'other_stale':[],'other_sup':[]}
    for m in after_skip(fr):
        t=rclpy.time.Time.from_msg(m.header.stamp).nanoseconds/1e9
        a=np.array(m.data,dtype=np.uint8).reshape(m.metadata.size_y,m.metadata.size_x); ys,xs=np.nonzero(a==254)
        if len(xs)==0: continue
        res=m.metadata.resolution; P=np.c_[m.metadata.origin.position.x+(xs+.5)*res, m.metadata.origin.position.y+(ys+.5)*res]
        rec=[xy for tt,xy in lid if t-1.0<=tt<=t+0.05 and len(xy)]
        sup=cKDTree(np.vstack(rec)).query_ball_point(P,0.075,return_length=True)>0 if rec else np.zeros(len(P),bool)
        try: tr=buf.lookup_transform('map','base_link',rclpy.time.Time.from_msg(m.header.stamp))
        except Exception: continue
        Rb,tb=mat(tr); yaw=math.atan2(Rb[1,0],Rb[0,0]); d=P-tb[:2]; dist=np.hypot(d[:,0],d[:,1])
        brg=(np.degrees(np.arctan2(d[:,1],d[:,0])-yaw)+180)%360-180
        front=(np.abs(brg)<=30)&(dist>0.3)&(dist<2.5)
        S['front_stale'].append((front&~sup).sum()); S['front_sup'].append((front&sup).sum())
        S['other_stale'].append((~front&~sup).sum()); S['other_sup'].append((~front&sup).sum())
    print(f'\n== {bag.split("/")[-1]}  帧 {len(S["front_sup"])}')
    if not S['front_sup']: print('   无可用帧（该层全程无 254 格，或查不到 map->base_link）'); continue
    for k,v in S.items(): v=np.array(v); print(f'   {k:12s} 每帧均值 {v.mean():7.1f}   p90 {np.percentile(v,90):7.1f}')

# ---- 清除延迟：格子最后一次有 MID360 支撑 → 格子消失；结束时仍在的记为"截尾"并给出已存活时长
def latency(bag, frames, buf):
    last_sup={}; present=set(); lat={'front':[],'other':[]}; where={}
    for m in after_skip(frames):
        t=rclpy.time.Time.from_msg(m.header.stamp).nanoseconds/1e9
        a=np.array(m.data,dtype=np.uint8).reshape(m.metadata.size_y,m.metadata.size_x); ys,xs=np.nonzero(a==254)
        res=m.metadata.resolution; P=np.c_[m.metadata.origin.position.x+(xs+.5)*res, m.metadata.origin.position.y+(ys+.5)*res]
        keys=[(round(x,3),round(y,3)) for x,y in P]
        rec=[xy for tt,xy in lid if t-0.3<=tt<=t+0.05 and len(xy)]
        sup=cKDTree(np.vstack(rec)).query_ball_point(P,0.075,return_length=True)>0 if (rec and len(P)) else np.zeros(len(P),bool)
        try:
            tr=buf.lookup_transform('map','base_link',rclpy.time.Time.from_msg(m.header.stamp)); Rb,tb=mat(tr); yaw=math.atan2(Rb[1,0],Rb[0,0])
        except Exception: continue
        for k,s,(x,y) in zip(keys,sup,P):
            if s or k not in last_sup: last_sup[k]=t if s else last_sup.get(k,t)
            if k not in where:
                d=np.hypot(x-tb[0],y-tb[1]); b=(math.degrees(math.atan2(y-tb[1],x-tb[0])-yaw)+180)%360-180
                where[k]='front' if (abs(b)<=30 and 0.3<d<2.5) else 'other'
        now=set(keys)
        for k in present-now: lat[where[k]].append(t-last_sup[k])
        present=now; tend=t
    cens={'front':[],'other':[]}
    for k in present: cens[where[k]].append(tend-last_sup[k])
    for z in ('front','other'):
        v=np.array(lat[z]); c=np.array(cens[z])
        s=f'   {z:5s} 消失的格 {len(v):5d}' + (f'  延迟 p50/p90/max = {np.percentile(v,50):.2f}/{np.percentile(v,90):.2f}/{v.max():.1f}s  >2s 的 {(v>2).sum()}' if len(v) else '')
        s+=f'  | 结束时仍在且无支撑>2s 的 {(c>2).sum()} 格（最长 {c.max() if len(c) else 0:.1f}s）'
        print(s)
for bag in reps:
    buf, fr = (buf0, fr0) if bag==reps[0] else load_rep(bag)
    print(f'-- 清除延迟 {bag.split("/")[-1]}'); latency(bag, fr, buf)
