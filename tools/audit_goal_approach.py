import sqlite3,sys,json
import numpy as np
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
db=sqlite3.connect('file:'+sys.argv[1]+'?mode=ro',uri=True)
topics={i:(n,get_message(t)) for i,n,t in db.execute('select id,name,type from topics')}
rows=[]; grids={}; counts={}
for tid,ns,b in db.execute('select topic_id,timestamp,data from messages order by timestamp'):
    n,c=topics[tid]
    if n=='/luck_controller/tracking_error': rows.append(list(deserialize_message(b,c).data))
    if n in ('/map','/diagnostics/global_camera_obstacle_grid'):
        m=deserialize_message(b,c); grids[n]=m; counts[n]=counts.get(n,0)+1
a=np.asarray(rows); report={'grid_messages':counts,'approach_bins':[]}
for lo,hi in ((.08,.15),(.15,.25),(.25,.4),(.4,.6),(.6,.8)):
    sel=(a[:,0]>=lo)&(a[:,0]<hi)&(a[:,7]==0)&(a[:,11]==0)&(np.abs(a[:,1])<.15)&(a[:,6]<100)
    report['approach_bins'].append({'distance':[lo,hi],'n':int(sel.sum()),
        'command_v_quantiles':np.quantile(a[sel,4],[.1,.5,.9]).tolist() if sel.any() else []})
print(json.dumps(report,indent=2))
if len(grids)==2:
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    fig,axes=plt.subplots(1,2,figsize=(12,7))
    for ax,(name,m) in zip(axes,grids.items()):
        a=np.asarray(m.data).reshape(m.info.height,m.info.width)
        x=m.info.origin.position.x;y=m.info.origin.position.y;r=m.info.resolution
        ax.imshow(a,origin='lower',extent=[x,x+a.shape[1]*r,y,y+a.shape[0]*r],vmin=-1,vmax=100,cmap='gray_r',interpolation='nearest')
        ax.set_title(name);ax.set_aspect('equal');ax.grid(alpha=.2)
    fig.tight_layout();fig.savefig(sys.argv[2],dpi=140)
