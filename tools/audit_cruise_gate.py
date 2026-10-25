"""Counterfactual request-only audit; not a closed-loop safety validation."""
import json, sqlite3, sys
import numpy as np
from rclpy.serialization import deserialize_message
from std_msgs.msg import Float64MultiArray

db=sqlite3.connect('file:'+sys.argv[1]+'?mode=ro',uri=True)
tid=db.execute("select id from topics where name='/luck_controller/tracking_error'").fetchone()[0]
rows=[(t,deserialize_message(b,Float64MultiArray).data) for t,b in
      db.execute('select timestamp,data from messages where topic_id=? order by timestamp',(tid,))]
t=np.array([r[0]/1e9 for r in rows]);a=np.array([r[1] for r in rows])
cross=abs(a[:,2]);yaw=abs(a[:,24]-a[:,23]*a[:,3])
def fade(x,lo,hi):
    u=np.clip((x-lo)/(hi-lo),0,1)
    return 1-u*u*(3-2*u)
oldw=np.clip(1-cross/.05,0,1)*np.clip(1-yaw/.20,0,1)
neww=fade(cross,.01,.05)*fade(yaw,.05,.20)
old=a[:,28];cap=a[:,29]
new=np.minimum(cap,old+neww*(np.minimum(cap,old*1.25)-old))
mask=(old>=0)&(a[:,0]>.8)&(a[:,7]==0)&(a[:,11]==0)&(a[:,6]==0)&(a[:,4]>.02)
valid=np.flatnonzero(mask)
out={'samples':int(mask.sum()),'changed':int((mask&(new>a[:,30]+.001)).sum()),
     'request_delta_p50_p90_max':np.quantile((new-a[:,30])[mask],[.5,.9,1]).tolist(),
     'old_request_p50':float(np.median(a[mask,30])),
     'candidate_request_p50':float(np.median(new[mask]))}
# Only compare contiguous, nearly unchanged curvature/envelope, excluding arrival/avoidance.
pair=mask[1:]&mask[:-1]&(np.diff(t)<.2)&(np.diff(t)>.04)&(abs(np.diff(a[:,3]))<.05)&(abs(np.diff(cap))<.005)
out['steady_geometry_pairs']=int(pair.sum())
out['old_request_step_abs_p90']=float(np.quantile(abs(np.diff(a[:,30]))[pair],.9)) if pair.any() else None
out['candidate_request_step_abs_p90']=float(np.quantile(abs(np.diff(new))[pair],.9)) if pair.any() else None
print(json.dumps(out,indent=2))
