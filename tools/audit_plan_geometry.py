"""Recorded planner geometry audit, not a counterfactual motion simulation."""
import sqlite3, sys, json
import numpy as np
from rclpy.serialization import deserialize_message
from nav_msgs.msg import Path
db=sqlite3.connect('file:'+sys.argv[1]+'?mode=ro',uri=True)
tid=db.execute("select id from topics where name='/plan'").fetchone()[0]
seen=set(); rows=[]
for (b,) in db.execute('select data from messages where topic_id=?',(tid,)):
    m=deserialize_message(b,Path)
    xy=np.asarray([(p.pose.position.x,p.pose.position.y) for p in m.poses])
    if len(xy)<3: continue
    key=np.round(xy,3).tobytes()
    if key in seen: continue
    seen.add(key)
    d=np.diff(xy,axis=0); d=d[np.linalg.norm(d,axis=1)>1e-6]
    if len(d)<2: continue
    h=np.arctan2(d[:,1],d[:,0]); a=abs(np.arctan2(np.sin(np.diff(h)),np.cos(np.diff(h))))
    rows.append([float(a.max()*180/np.pi),int((a>np.pi/6).sum()),float(np.linalg.norm(d,axis=1).sum())])
a=np.asarray(rows)
print(json.dumps({'distinct_path_messages':len(rows),
 'max_vertex_turn_deg_p50_p90_max':np.quantile(a[:,0],[.5,.9,1]).tolist() if len(a) else [],
 'messages_with_turn_over_30deg':int((a[:,1]>0).sum()) if len(a) else 0,
 'note':'Truncated cached paths count separately; this is not independent route count.'},indent=2))
