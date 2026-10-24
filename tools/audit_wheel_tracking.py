#!/usr/bin/env python3
"""Read-only per-bag command/encoder/IMU audit. Raw IMU is inverted in URDF."""
import json, sqlite3, sys
import numpy as np
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
db=sqlite3.connect('file:'+sys.argv[1]+'?mode=ro',uri=True)
topics={i:(n,t) for i,n,t in db.execute('select id,name,type from topics')}
data={}
for tid,stamp,blob in db.execute('select topic_id,timestamp,data from messages order by timestamp'):
    name,typ=topics[tid]
    if name not in ['/wheel_response_debug','/wheel/odom','/imu/data','/cmd_vel_drive','/cmd_vel_nav','/wheel_feedback']:
        continue
    m=deserialize_message(blob,get_message(typ)); t=stamp*1e-9
    if name=='/wheel/odom': row=[t,m.twist.twist.linear.x,m.twist.twist.angular.z]
    elif name=='/imu/data': row=[t,0.,-m.angular_velocity.z]
    elif name.startswith('/cmd_vel'): row=[t,m.linear.x,m.angular.z]
    else: row=[t,*m.data]
    data.setdefault(name,[]).append(row)
data={k:np.array(v) for k,v in data.items()}
out={}
for name,a in data.items():
    if name in ['/wheel/odom','/imu/data','/cmd_vel_drive','/cmd_vel_nav']:
        out[name]={'n':len(a),'max_gap_s':float(np.max(np.diff(a[:,0]))),
          'abs_v_p95':float(np.quantile(abs(a[:,1]),.95)),
          'abs_w_p95':float(np.quantile(abs(a[:,2]),.95)),
          'yaw_integral_deg':float(np.trapz(a[:,2],a[:,0])*180/np.pi)}
if '/wheel_response_debug' in data:
    a=data['/wheel_response_debug']; valid=(a[:,10]==1)&(a[:,11]==1)
    moving=valid & (a[:,4]>.015)
    straight=moving & (abs(a[:,5])<.005)
    for label,mask in [('moving',moving),('near_straight',straight)]:
        b=a[mask]
        if len(b):
            # Targets/raw feedback: left positive forward, right negative forward.
            out[label]={'n':len(b),'target_RminusL_rpm_mean':float(np.mean(-b[:,7]-b[:,6])),
              'feedback_RminusL_rpm_mean':float(np.mean(-b[:,9]-b[:,8])),
              'L_error_rpm_mean':float(np.mean(b[:,8]-b[:,6])),
              'R_error_rpm_mean':float(np.mean(-b[:,9]+b[:,7]))}
print(json.dumps(out,indent=2))
