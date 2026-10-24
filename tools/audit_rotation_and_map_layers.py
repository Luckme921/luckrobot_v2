"""Read-only rotation transitions and final map-layer overlap."""
import sys, sqlite3, json
import numpy as np
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
db=sqlite3.connect('file:'+sys.argv[1]+'?mode=ro',uri=True)
topics={i:(n,get_message(t)) for i,n,t in db.execute('select id,name,type from topics')}
grids={}; rows=[]; scan_counts={}
for tid,ns,blob in db.execute('select topic_id,timestamp,data from messages order by timestamp'):
    n,cls=topics[tid]
    if n=='/luck_controller/tracking_error':
        rows.append([ns/1e9]+list(deserialize_message(blob,cls).data))
    elif n in ('/map','/global_costmap/costmap','/local_costmap/costmap'):
        m=deserialize_message(blob,cls)
        grids[n]=(m.info,np.asarray(m.data,dtype=np.int16).reshape(m.info.height,m.info.width).copy(),ns/1e9)
    elif n in ('/global_costmap/costmap_updates','/local_costmap/costmap_updates','/map_updates'):
        key=n.replace('_updates','')
        if key not in grids: continue
        m=deserialize_message(blob,cls); info,a,_=grids[key]
        if m.x+m.width<=a.shape[1] and m.y+m.height<=a.shape[0]:
            a[m.y:m.y+m.height,m.x:m.x+m.width]=np.asarray(m.data).reshape(m.height,m.width)
            grids[key]=(info,a,ns/1e9)
    elif n=='/scan_camera_clear': scan_counts[n]=scan_counts.get(n,0)+1
a=np.asarray(rows); exits=[]
for i in range(1,len(a)):
    if a[i-1,8]==1 and a[i,8]==0 and a[i-1,10]==0 and a[i,0]-a[i-1,0]<.3:
        exits.append({'t':float(a[i,0]-a[0,0]),'bearing_deg':float(np.degrees(a[i,2])),
            'measured_w':float(a[i,25]),'command_v':float(a[i,5]),'command_w':float(a[i,6])})
report={'nonterminal_rotation_exits':exits,'clearing_messages':scan_counts,'final_layers':{}}
if '/map' in grids:
    info,base,t=grids['/map']
    for name,(ci,cost,ct) in grids.items():
        if name=='/map': continue
        ys,xs=np.nonzero(cost>=99)
        wx=ci.origin.position.x+(xs+.5)*ci.resolution
        wy=ci.origin.position.y+(ys+.5)*ci.resolution
        mx=np.floor((wx-info.origin.position.x)/info.resolution).astype(int)
        my=np.floor((wy-info.origin.position.y)/info.resolution).astype(int)
        valid=(mx>=0)&(my>=0)&(mx<base.shape[1])&(my<base.shape[0])
        vals=base[my[valid],mx[valid]]
        report['final_layers'][name]={'lethal_or_inscribed':len(xs),
            'over_static_free':int((vals==0).sum()),'over_static_unknown':int((vals<0).sum()),
            'over_static_occupied':int((vals>=65).sum()),'outside_static_grid':int((~valid).sum()),
            'map_age_difference_s':ct-t}
print(json.dumps(report,indent=2))
