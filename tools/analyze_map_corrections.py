import sqlite3,sys,math,collections
import numpy as np
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
db=sqlite3.connect('file:'+sys.argv[1]+'?mode=ro',uri=True)
topics={i:(n,get_message(t)) for i,n,t in db.execute('select id,name,type from topics')}
data=collections.defaultdict(list)
def yaw(q):return math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))
for tid,ns,b in db.execute('select topic_id,timestamp,data from messages order by timestamp'):
 n,c=topics[tid]
 if n not in ['/tf','/cmd_vel_drive','/wheel/odom','/imu/data']:continue
 m=deserialize_message(b,c);t=ns/1e9
 if n=='/tf':
  for tr in m.transforms:
   if tr.header.frame_id=='map' and tr.child_frame_id=='odom':data['map'].append((t,tr.transform.translation.x,tr.transform.translation.y,yaw(tr.transform.rotation)))
 elif n=='/cmd_vel_drive':data[n].append((t,m.linear.x,m.angular.z))
 elif n=='/wheel/odom':data[n].append((t,m.twist.twist.linear.x,m.twist.twist.angular.z))
 else:data[n].append((t,0,-m.angular_velocity.z))
changes=[]
for a,b in zip(data['map'],data['map'][1:]):
 d=math.hypot(b[1]-a[1],b[2]-a[2]);w=math.atan2(math.sin(b[3]-a[3]),math.cos(b[3]-a[3]))
 if d>1e-5 or abs(w)>1e-5:changes.append((b[0],d,math.degrees(w)))
print('Largest map corrections',sorted(changes,key=lambda r:r[1],reverse=True)[:10])
for name,rows in data.items():
 if name=='map':continue
 for start,end in [(1792742260,1792742270),(1792742270,1792742280),(1792742280,1792742286),(1792742286,1792742290),(1792742290,1792742300)]:
  vals=[r[1:] for r in rows if start<=r[0]<end]
  if vals:print(name,start,'mean/maxabs',np.mean(vals,axis=0),np.max(np.abs(vals),axis=0))
