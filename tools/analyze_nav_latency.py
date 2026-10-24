#!/usr/bin/env python3
"""Offline time/freshness and actuator-lag report; never open a live bag."""
import bisect
import collections
import math
import sqlite3
import sys
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message


def pct(a, p):
    return sorted(a)[min(len(a)-1, round((len(a)-1)*p))] if a else 0


db = sqlite3.connect('file:' + sys.argv[1] + '?mode=ro', uri=True)
topics = {i: (n, t) for i, n, t in db.execute('select id,name,type from topics')}
wanted = {'/tf', '/scan', '/scan_filtered', '/scan_camera_raw', '/scan_camera_safety',
          '/scan_camera', '/odometry/filtered', '/wheel/odom', '/imu/data',
          '/cmd_vel_nav', '/cmd_vel', '/cmd_vel_safe', '/rosout', '/wheel_feedback',
          '/luck_controller/lookahead', '/luck_controller/local_trajectory', '/plan',
          '/local_costmap/costmap', '/global_costmap/costmap'}
classes = {i: get_message(t) for i, (n, t) in topics.items() if n in wanted}
streams = collections.defaultdict(list)
signals = collections.defaultdict(list)
warnings = []
def stamp(s):
    return s.sec + s.nanosec * 1e-9
start, end = db.execute('select min(timestamp),max(timestamp) from messages').fetchone()
print('BAG EPOCH', start/1e9, end/1e9)
for tid, ns, blob in db.execute('select topic_id,timestamp,data from messages order by timestamp'):
    if tid not in classes:
        continue
    n = topics[tid][0]; t = ns/1e9
    m = deserialize_message(blob, classes[tid])
    if n == '/rosout':
        if m.level >= 30:
            warnings.append((t, stamp(m.stamp), m.name, m.msg))
        continue
    if n == '/tf':
        for tr in m.transforms:
            if tr.child_frame_id in ('odom', 'base_footprint'):
                streams[n + ':' + tr.header.frame_id + '->' + tr.child_frame_id].append((t, stamp(tr.header.stamp)))
        continue
    streams[n].append((t, stamp(m.header.stamp) if hasattr(m, 'header') else t))
    if n.startswith('/cmd_vel'):
        signals[n].append((t, m.linear.x, m.angular.z))
    elif n in ('/wheel/odom', '/odometry/filtered'):
        signals[n].append((stamp(m.header.stamp), m.twist.twist.linear.x, m.twist.twist.angular.z))
    elif n == '/imu/data':
        # URDF imu_joint roll=-pi; body angular Z is -sensor angular Z.
        signals[n].append((stamp(m.header.stamp), 0, -m.angular_velocity.z))
for name, samples in streams.items():
    ages = [t-s for t,s in samples]
    gaps = [(b[0]-a[0], a[0], b[0]) for a,b in zip(samples,samples[1:])]
    print(name, 'n', len(samples), 'age p50/p95/max', [round(pct(ages,p),3) for p in (.5,.95,1)],
          'last_stamp',round(samples[-1][1],3), 'last_arrival',round(samples[-1][0],3),
          'largest_gaps', [(round(d,3),round(a,3),round(b,3)) for d,a,b in sorted(gaps,reverse=True)[:3]])
print('LAG fits: commands use bag time (unstamped); feedback uses measurement stamp and body axes')
for source, measured, component in [('/cmd_vel_nav','/cmd_vel_safe',2),('/cmd_vel_safe','/imu/data',2),('/cmd_vel_safe','/wheel/odom',1)]:
    a=signals[source]; b=signals[measured]; times=[x[0] for x in a]
    fits=[]
    for delay in [i*.05 for i in range(41)]:
        errors=[]
        for row in b:
            j=bisect.bisect_right(times,row[0]-delay)-1
            if j<0 or row[0]-delay-times[j]>.15 or abs(a[j][component])<.05:
                continue
            errors.append(abs(row[component]-a[j][component]))
        fits.append((sum(errors)/len(errors) if errors else float('inf'),delay,len(errors)))
    print(source,measured,component, 'best',sorted(fits)[:3], 'zero',fits[0])
print('WARN first/last and frequent:')
counts=collections.Counter((n,msg) for _,_,n,msg in warnings)
for (n,msg),count in counts.most_common(15): print(count,n,msg[:260])
for t,s,n,msg in warnings[-20:]: print('arrival',round(t,3),'source',round(s,3),n,msg[:260])
