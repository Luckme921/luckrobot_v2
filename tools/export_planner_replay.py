"""Export bounded recorded global costmap/plan snapshots for isolated C++ replay.
OccupancyGrid cost conversion is lossy; these are replay estimates, not ground truth.
"""
import sqlite3,sys,math
import numpy as np
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
db=sqlite3.connect('file:'+sys.argv[1]+'?mode=ro',uri=True)
topics={i:(n,get_message(t)) for i,n,t in db.execute('select id,name,type from topics')
        if n in ('/plan','/global_costmap/costmap','/global_costmap/costmap_updates')}
grid=None; meta=None; cases=[]; seen=set(); count=0
for tid,blob in db.execute('select topic_id,data from messages order by timestamp'):
    if tid not in topics: continue
    name,cls=topics[tid]; m=deserialize_message(blob,cls)
    if name=='/global_costmap/costmap':
        meta=m.info; grid=np.asarray(m.data,dtype=np.int16).reshape(meta.height,meta.width).copy()
    elif name.endswith('costmap_updates') and grid is not None:
        if m.y+m.height<=grid.shape[0] and m.x+m.width<=grid.shape[1]:
            grid[m.y:m.y+m.height,m.x:m.x+m.width]=np.asarray(m.data).reshape(m.height,m.width)
    elif name=='/plan' and grid is not None and len(m.poses)>3:
        a=m.poses[0].pose.position; b=m.poses[-1].pose.position
        if math.hypot(a.x-b.x,a.y-b.y)<.5: continue
        key=tuple(round(v,1) for v in (a.x,a.y,b.x,b.y))
        if key in seen: continue
        seen.add(key);count+=1
        if count%3: continue
        cost=np.where(grid<0,255,np.where(grid==100,254,np.where(grid==99,253,
            np.where(grid==0,0,np.ceil(1+(grid-1)*251/97))))).astype(int)
        cases.append((meta,cost.copy(),a,b))
        if len(cases)>=20: break
with open(sys.argv[2],'w') as out:
    out.write(str(len(cases))+'\n')
    for m,g,a,b in cases:
        out.write(f'{m.width} {m.height} {m.resolution} {m.origin.position.x} {m.origin.position.y}\n')
        out.write(' '.join(map(str,g.flat))+'\n')
        out.write(f'{a.x} {a.y} {b.x} {b.y}\n')
print('Exported',len(cases),'recorded snapshots')
