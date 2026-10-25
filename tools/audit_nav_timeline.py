"""Read-only incident timeline and speed-envelope diagnostics."""
import sqlite3,sys,json,collections,hashlib
import numpy as np
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
db=sqlite3.connect('file:'+sys.argv[1]+'?mode=ro',uri=True)
t0=db.execute('select min(timestamp) from messages').fetchone()[0]
topics={i:(n,get_message(t)) for i,n,t in db.execute('select id,name,type from topics')
        if n in ('/rosout','/luck_controller/tracking_error','/navigate_to_pose/_action/status','/plan')}
events=[];tracking=[];last_status=None;plan_count=0;plans=[]
for tid,t,b in db.execute('select topic_id,timestamp,data from messages order by timestamp'):
    if tid not in topics:continue
    name,cls=topics[tid];m=deserialize_message(b,cls);s=(t-t0)/1e9
    if name=='/rosout' and s<150 and (m.level>=30 or any(x in m.msg.lower() for x in ('goal','curve optimization','wait','plan'))):
        events.append([round(s,2),m.name,m.msg])
    elif name.endswith('/status'):
        status=[(bytes(x.goal_info.goal_id.uuid).hex()[:8],x.status) for x in m.status_list]
        if status!=last_status and s<150:events.append([round(s,2),'nav_status',status])
        last_status=status
    elif name=='/plan' and s<150:
        plan_count+=1
        points=[(round(p.pose.position.x,4),round(p.pose.position.y,4)) for p in m.poses]
        plans.append([round(s,2),len(points),hashlib.sha256(repr(points).encode()).hexdigest()[:12],points[0] if points else None,points[-1] if points else None])
    elif name.endswith('tracking_error'):
        if len(m.data)>=31:tracking.append([s]+list(m.data))
a=np.array(tracking);summary={'first150_plan_messages':plan_count,'tracking_samples':len(a)}
if len(a):
    valid=a[:,29]>=0 # message field 28, offset by time column
    summary['envelope_samples']=int(valid.sum())
    for field in (28,29,30,4):
        summary[str(field)+'_p10_p50_p90']=np.quantile(a[valid,field+1],[.1,.5,.9]).tolist() if valid.any() else []
    v=a[:,5];dt=np.diff(a[:,0]);dv=np.diff(v)
    good=(dt>.04)&(dt<.2)&(a[:-1,8]==0)&(a[1:,8]==0)&(a[:-1,1]>.5)&(a[1:,1]>.5)
    summary['nonrotate_cruise_cmd_accel_abs_p95']=float(np.quantile(abs(dv[good]/dt[good]),.95)) if good.any() else None
print(json.dumps({'summary':summary,'first150_events':events,'first150_plans':plans},indent=2,ensure_ascii=False))
