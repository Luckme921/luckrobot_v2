import sqlite3,sys,json
import numpy as np
from scipy.ndimage import distance_transform_edt
from rclpy.serialization import deserialize_message
from nav_msgs.msg import OccupancyGrid
db=sqlite3.connect('file:'+sys.argv[1]+'?mode=ro',uri=True)
grids={}
for topic in ['/map','/global_costmap/costmap']:
    tid=db.execute('select id from topics where name=?',(topic,)).fetchone()[0]
    b=db.execute('select data from messages where topic_id=? order by timestamp desc limit 1',(tid,)).fetchone()[0]
    grids[topic]=deserialize_message(b,OccupancyGrid)
m=grids['/map'];n=grids['/global_costmap/costmap']
a=np.array(m.data).reshape(m.info.height,m.info.width)
b=np.array(n.data).reshape(n.info.height,n.info.width)
clear=distance_transform_edt(np.pad(a==0,1))[1:-1,1:-1]*m.info.resolution
y,x=np.where((a==0)&(clear>.25))
wx=m.info.origin.position.x+(x+.5)*m.info.resolution
wy=m.info.origin.position.y+(y+.5)*m.info.resolution
nx=np.floor((wx-n.info.origin.position.x)/n.info.resolution).astype(int)
ny=np.floor((wy-n.info.origin.position.y)/n.info.resolution).astype(int)
ok=(nx>=0)&(ny>=0)&(nx<n.info.width)&(ny<n.info.height)
cost=np.full(len(x),-1);cost[ok]=b[ny[ok],nx[ok]]
keep=(cost>=0)&(cost<=20)
print(json.dumps({'free_cells':int((a==0).sum()),'clear_cells':len(x),'safe_low_cost_cells':int(keep.sum()),'safe_xy':list(zip(wx[keep].round(2).tolist(),wy[keep].round(2).tolist()))[:20]},indent=2))
