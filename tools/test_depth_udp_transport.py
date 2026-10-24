#!/usr/bin/env python3
"""Domain-191 synthetic depth transport only; no real camera/chassis."""
import os
if os.environ.get('ROS_DOMAIN_ID')!='191': raise SystemExit('Requires isolated domain 191')
import json, subprocess, sys, time
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image

def drops():
    lines=open('/proc/net/snmp').readlines()
    for a,b in zip(lines,lines[1:]):
        if a.startswith('Udp:') and b.startswith('Udp:'):
            return dict(zip(a.split()[1:],map(int,b.split()[1:])))['RcvbufErrors']
rclpy.init(); n=Node('synthetic_depth_'+('tx' if '--tx' in sys.argv else 'rx'))
if '--tx' in sys.argv:
    p=n.create_publisher(Image,'/isolated/depth',qos_profile_sensor_data)
    m=Image(width=640,height=400,encoding='16UC1',step=1280,data=bytes(512000))
    start=time.monotonic()
    while time.monotonic()-start<22:
        m.header.stamp=n.get_clock().now().to_msg(); p.publish(m)
        rclpy.spin_once(n,timeout_sec=.066)
else:
    samples=[]
    n.create_subscription(Image,'/isolated/depth',lambda m:samples.append((time.monotonic(),time.time()-m.header.stamp.sec-m.header.stamp.nanosec/1e9)),qos_profile_sensor_data)
    before=drops(); p=subprocess.Popen([sys.executable,__file__,'--tx'])
    while p.poll() is None:rclpy.spin_once(n,timeout_sec=.05)
    gaps=sorted(b[0]-a[0] for a,b in zip(samples,samples[1:]))
    print(json.dumps({'received':len(samples),'udp_receive_errors_delta':drops()-before,
      'max_gap':max(gaps) if gaps else None,'gaps_over_035':sum(g>.35 for g in gaps),
      'max_age':max(x[1] for x in samples) if samples else None}))
n.destroy_node();rclpy.shutdown()
