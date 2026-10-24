#!/usr/bin/env python3
"""Supervised bounded wheel tests; floor modes require confirmed clear space."""
import json
import time
import argparse
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import Float64MultiArray
from rclpy.qos import qos_profile_sensor_data

parser = argparse.ArgumentParser()
parser.add_argument('--mode', choices=('raised', 'floor_short', 'floor_turn', 'floor_arc'), default='raised')
args = parser.parse_args()
rclpy.init()
n = Node('raised_wheel_response_test')
p = n.create_publisher(Twist, '/cmd_vel_bench', 1)
rows = []
start = time.monotonic()
last_feedback = [None]
motor_fault = [False]
def motor_status(m):
    rows.append(['motor', time.monotonic()-start, *m.data])
    if len(m.data) >= 4 and (m.data[1] != 2 or m.data[3] != 0 or abs(m.data[2]) > 1.25):
        motor_fault[0] = True
def odom(m):
    last_feedback[0] = time.monotonic()
    rows.append(['odom', last_feedback[0]-start, m.twist.twist.linear.x,
                 m.twist.twist.angular.z])
    q = m.pose.pose.orientation
    rows.append(['pose', last_feedback[0]-start, m.pose.pose.position.x,
                 m.pose.pose.position.y, q.x, q.y, q.z, q.w])
def feedback(m):
    rows.append(['wheel', time.monotonic()-start, *m.data])
n.create_subscription(Odometry, '/wheel/odom', odom, 5)
n.create_subscription(Float64MultiArray, '/wheel_feedback', feedback, 5)
n.create_subscription(Float64MultiArray, '/motor_status_debug', motor_status, qos_profile_sensor_data)
n.create_subscription(Float64MultiArray, '/wheel_response_debug',
    lambda m: rows.append(['driver', time.monotonic()-start, *m.data]), qos_profile_sensor_data)
status = 'not_started'
try:
    deadline = time.monotonic()+8
    while time.monotonic() < deadline:
        rclpy.spin_once(n, timeout_sec=.05)
        if p.get_subscription_count() == 1 and last_feedback[0] is not None:
            break
    if p.get_subscription_count() != 1 or last_feedback[0] is None:
        raise RuntimeError('missing unique chassis subscriber/fresh feedback')
    if n.count_publishers('/cmd_vel_bench') != 1:
        raise RuntimeError('another command publisher exists')
    sequence = ([(1.,0.,0.), (2.,.06,0.), (4.,0.,0.)] if args.mode == 'floor_short'
                else [(1.,0.,0.), (2.,0.,.18), (4.,0.,0.)] if args.mode == 'floor_turn'
                else [(1.,0.,0.), (2.,.06,.12), (4.,0.,0.)] if args.mode == 'floor_arc'
                else [(1.,0.,0.), (3.,.06,0.), (3.,0.,0.),
                      (3.,0.,.18), (3.,0.,0.)] * 2)
    for duration, v, w in sequence:
        until = time.monotonic()+duration
        while time.monotonic() < until:
            if motor_fault[0]:
                raise RuntimeError('motor fault/mode/current safety abort')
            if time.monotonic()-last_feedback[0] > .3:
                raise RuntimeError('wheel feedback stale, abort')
            m = Twist()
            m.linear.x, m.angular.z = v, w
            p.publish(m)
            rows.append(['cmd',time.monotonic()-start,v,w])
            end = time.monotonic()+.05
            while time.monotonic() < end:
                rclpy.spin_once(n, timeout_sec=min(.01,max(0.,end-time.monotonic())))
    status = 'completed'
except Exception as exc:
    status = 'aborted: '+str(exc)
finally:
    for _ in range(20):
        p.publish(Twist())
        rclpy.spin_once(n, timeout_sec=.05)
    print(json.dumps({'status':status,'rows':rows}))
    n.destroy_node()
    rclpy.shutdown()
