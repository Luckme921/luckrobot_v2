#!/usr/bin/env python3
"""Read-only analysis of safety gating, map sizes and recorded performance."""
import collections
import re
import sqlite3
import sys
import numpy as np
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message

db=sqlite3.connect('file:'+sys.argv[1]+'?mode=ro',uri=True)
topics={i:(n,get_message(t)) for i,n,t in db.execute('select id,name,type from topics')}
chosen={'/motion_freshness_guard/status','/cmd_vel_nav','/cmd_vel_safe','/cmd_vel_drive',
        '/scan_camera_safety','/map','/global_costmap/costmap','/rosout'}
data=collections.defaultdict(list)
logs=collections.Counter()
for tid,stamp,blob in db.execute('select topic_id,timestamp,data from messages order by timestamp'):
    name,cls=topics[tid]
    if name not in chosen: continue
    m=deserialize_message(blob,cls); t=stamp/1e9
    if name=='/rosout':
        if m.level>=30: logs[(m.name,m.msg.split('timestamps differ')[0][:150])]+=1
    elif name.endswith('/status'): data[name].append((t,m.data))
    elif name.startswith('/cmd_vel'): data[name].append((t,m.linear.x,m.angular.z))
    elif name.endswith('costmap') or name=='/map':
        data[name].append((t,(m.info.width,m.info.height,m.info.origin.position.x,m.info.origin.position.y)))
    else: data[name].append((t,m.header.stamp.sec+m.header.stamp.nanosec/1e9))
for name,rows in data.items():
    if name.endswith('/status'):
        durations=collections.Counter()
        for (a,s),(b,_) in zip(rows,rows[1:]): durations[s.split(':')[0]]+=b-a
        print('GATE seconds',durations)
    elif name.startswith('/cmd_vel'):
        active=[r for r in rows if abs(r[1])+abs(r[2])>.001]
        print(name,'count',len(rows),'nonzero',len(active),'last nonzero',active[-1] if active else None)
    elif name.endswith('costmap') or name=='/map':
        changes=[r for i,r in enumerate(rows) if i==0 or r[1]!=rows[i-1][1]]
        print(name,'geometry changes',changes)
    else:
        gaps=np.diff([r[0] for r in rows]); ages=[t-s for t,s in rows]
        print(name,'n',len(rows),'gap quantiles',np.quantile(gaps,[.5,.95,.99,1]),
              'gaps>0.35',sum(gaps>.35),'age quantiles',np.quantile(ages,[.5,.95,.99,1]))
print('WARN/ERROR',logs.most_common(10))
if len(sys.argv)>2:
    cpu=[]; mem=[];gpu=[]
    for line in open(sys.argv[2]):
        match=re.search(r'CPU \[([^]]+)\]',line)
        if match: cpu.append(np.mean([int(x) for x in re.findall(r'(\d+)%',match[1])]))
        match=re.search(r'RAM (\d+)/',line)
        if match:mem.append(int(match[1]))
        match=re.search(r'GR3D_FREQ (\d+)%',line)
        if match:gpu.append(int(match[1]))
    print('CPU all-core mean p50/p95/max',np.quantile(cpu,[.5,.95,1]),
          'RAM MB max',max(mem),'GPU% p50/max',np.quantile(gpu,[.5,1]))
