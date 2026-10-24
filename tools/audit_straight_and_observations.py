"""Read-only diagnostics; no claim that replay changes predict closed-loop motion."""
import sqlite3, sys, json, collections
import numpy as np
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
db=sqlite3.connect('file:'+sys.argv[1]+'?mode=ro',uri=True)
topics={i:(n,t) for i,n,t in db.execute('select id,name,type from topics')}
rows=[]; scans=collections.defaultdict(list); maps=[]; logs=[]
for tid,ns,b in db.execute('select topic_id,timestamp,data from messages order by timestamp'):
    n,t=topics[tid]
    if n not in ('/luck_controller/tracking_error','/scan_camera','/scan_camera_safety','/map','/rosout'): continue
    m=deserialize_message(b,get_message(t))
    if n=='/luck_controller/tracking_error': rows.append([ns/1e9]+list(m.data))
    elif n.startswith('/scan_camera'):
        r=np.asarray(m.ranges); scans[n].append((ns/1e9,float(np.mean(np.isfinite(r)))))
    elif n=='/map':
        a=np.asarray(m.data); maps.append((ns/1e9,int((a>=65).sum()),m.info.width,m.info.height))
    elif m.level>=30: logs.append(m.name+': '+m.msg)
a=np.asarray(rows); straight=(a[:,12]==0)&(a[:,8]==0)&(a[:,5]>.10)&(a[:,23]<.08)&(np.abs(a[:,2])<.20)
ids=np.flatnonzero(straight); pairs=[(i,j) for i,j in zip(ids[:-1],ids[1:]) if j==i+1 and a[j,0]-a[i,0]<.25]
def switches(w,threshold): return int(sum(w[i]*w[j]<0 and min(abs(w[i]),abs(w[j]))>threshold for i,j in pairs))
q=lambda x: np.quantile(np.abs(x),[.5,.9,.99,1]).tolist() if len(x) else []
report={'straight_samples':len(ids),'straight_cross_m':q(a[ids,3]),'straight_bearing_rad':q(a[ids,2]),
 'straight_yaw_command':q(a[ids,6]),'adjacent_sign_changes':{str(t):switches(a[:,6],t) for t in (0,.01,.03)},
 'camera_finite_fraction':{k:q(np.asarray(v)[:,1]) for k,v in scans.items()},
 'map_snapshots':len(maps),'map_occupied_first_last': [maps[0],maps[-1]] if maps else [],
 'warnings':collections.Counter(logs).most_common(12)}
print(json.dumps(report,indent=2))
