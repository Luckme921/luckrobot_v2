#!/usr/bin/env python3
"""Isolated ROS integration and recorder-churn test. NEVER uses domain 100.

Run: ROS_DOMAIN_ID=191 python3 tools/test_motion_guard_ros.py
Synthetic data only. Launches no hardware drivers and sends no navigation goals.
"""
import os
if os.environ.get('ROS_DOMAIN_ID') != '191':
    raise SystemExit('Refusing test outside isolated ROS_DOMAIN_ID=191')
import json
import signal
import subprocess
import tempfile
import time
from pathlib import Path

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from geometry_msgs.msg import Twist, TransformStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan, Image, Imu
from tf2_msgs.msg import TFMessage

root = Path(__file__).resolve().parents[1]
out = Path(tempfile.mkdtemp(prefix='luckrobot_guard_test_'))
rclpy.init()
node = Node('isolated_guard_test')
sensor_qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)
publishers = {
    'tf': node.create_publisher(TFMessage, '/tf', 10),
    'odom': node.create_publisher(Odometry, '/odometry/filtered', sensor_qos),
    'wheel': node.create_publisher(Odometry, '/wheel/odom', sensor_qos),
    'imu': node.create_publisher(Imu, '/imu/data', sensor_qos),
    'lidar': node.create_publisher(LaserScan, '/scan_filtered', sensor_qos),
    'camera': node.create_publisher(LaserScan, '/scan_camera_safety', sensor_qos),
    'cmd': node.create_publisher(Twist, '/cmd_vel_safe', 1),
    'image': node.create_publisher(Image, '/isolated/depth_load', sensor_qos),
}
outputs = []
node.create_subscription(Twist, '/cmd_vel_drive',
    lambda m: outputs.append((time.monotonic(), m.linear.x, m.angular.z)), sensor_qos)
# Load payload matches one 640x480 16-bit depth stream, without a real camera.
im = Image(height=480, width=640, encoding='16UC1', step=1280, data=bytes(614400))
node.create_subscription(Image, '/isolated/depth_load', lambda m: None, sensor_qos)
guard_log = (out / 'guard.log').open('w')
guard = subprocess.Popen(['ros2', 'run', 'my_robot_bringup', 'motion_freshness_guard'],
                         stdout=guard_log, stderr=subprocess.STDOUT, start_new_session=True)
records = []
report = []

def run(seconds, fault=None, send_cmd=True, requested=(.1,.05), tail_only=True):
    start = time.monotonic()
    first = len(outputs)
    last_pub = 0.
    while time.monotonic() - start < seconds:
        now = time.monotonic()
        if now - last_pub >= .04:
            stamp = node.get_clock().now().to_msg()
            transforms = []
            for parent, child in [('map','odom'), ('odom','base_footprint')]:
                tr = TransformStamped()
                tr.header.stamp = stamp
                tr.header.frame_id, tr.child_frame_id = parent, child
                tr.transform.rotation.w = 1.
                if fault == 'map_tf' and child == 'odom':
                    tr.header.stamp.sec -= 3
                transforms.append(tr)
            publishers['tf'].publish(TFMessage(transforms=transforms))
            od = Odometry()
            od.header.stamp = stamp
            od.header.frame_id='odom'
            publishers['odom'].publish(od)
            if fault != 'wheel':
                publishers['wheel'].publish(od)
            imu=Imu()
            imu.header.stamp=stamp
            if fault != 'imu':
                publishers['imu'].publish(imu)
            for kind in ('lidar','camera'):
                scan = LaserScan()
                scan.header.stamp = node.get_clock().now().to_msg()
                scan.header.frame_id='base_footprint'
                if fault == kind:
                    scan.header.stamp.sec -= 2
                publishers[kind].publish(scan)
            if send_cmd:
                command = Twist()
                command.linear.x, command.angular.z = requested
                publishers['cmd'].publish(command)
            im.header.stamp = stamp
            publishers['image'].publish(im)
            last_pub=now
        rclpy.spin_once(node, timeout_sec=.005)
        if guard.poll() is not None:
            raise RuntimeError('Guard exited: ' + (out/'guard.log').read_text())
    return [row for row in outputs[first:]
            if not tail_only or row[0] > start + seconds - .5]

def assert_stopped(label, values):
    assert values and all(abs(v)+abs(w) < 1e-9 for _,v,w in values), (label, values)
    report.append(label + ': PASS')

def assert_moving(label, values):
    assert values and all(.09 < v <= .100001 for _,v,w in values), (label, values)
    report.append(label + ': PASS')

try:
    assert_moving('healthy forwarding', run(4.))
    assert_moving('left to right has no injected linear stop',
                  run(1., requested=(.1,-.05), tail_only=False))
    assert_moving('right to left has no injected linear stop',
                  run(1., requested=(.1,.05), tail_only=False))
    assert_stopped('old camera despite continuous arrivals', run(1.5, 'camera'))
    assert_moving('recovery ramps with new commands', run(2.))
    assert_stopped('old map TF', run(1.5, 'map_tf'))
    assert_moving('TF recovery', run(2.))
    assert_stopped('old lidar', run(1.5, 'lidar'))
    assert_moving('lidar recovery', run(2.))
    assert_stopped('wheel source lost while EKF keeps publishing', run(1., 'wheel'))
    assert_moving('wheel recovery', run(2.))
    assert_stopped('IMU lost while EKF keeps publishing', run(1., 'imu'))
    assert_moving('IMU recovery', run(2.))
    assert_stopped('lost command heartbeat', run(1., send_cmd=False))
    assert_stopped('fresh sensors cannot replay cached command', run(1., send_cmd=False))
    assert_moving('new command resumes', run(2.))
    for i in range(3):
        log=(out/('recorder_%d.log'%i)).open('w')
        proc=subprocess.Popen(['ros2','bag','record','-o',str(out/('bag_%d'%i)),
                               '--qos-profile-overrides-path',str(root/'tools/nav_diag_qos.yaml'),
                               '/tf','/cmd_vel_drive','/odometry/filtered'],
                              stdin=subprocess.DEVNULL, stdout=log,stderr=subprocess.STDOUT,
                              start_new_session=True)
        records.append(proc)
        assert_moving('recording cycle %d'%i, run(3.))
        os.killpg(proc.pid, signal.SIGINT)
        assert_moving('after recorder exit %d'%i, run(2.))
        proc.wait(timeout=10)
        assert proc.returncode in (0,130,-2), proc.returncode
        log.close()
    result = json.dumps({'results':report,'output':str(out)}, indent=2)
    (out/'results.json').write_text(result + '\n')
    print(result)
finally:
    for proc in records+[guard]:
        # ros2 run is a wrapper. Signal its own process group, including the
        # child executable, even if the wrapper has already exited.
        try: os.killpg(proc.pid, signal.SIGINT)
        except ProcessLookupError: pass
        try: proc.wait(timeout=5)
        except subprocess.TimeoutExpired:
            os.killpg(proc.pid, signal.SIGKILL)
            proc.wait()
    guard_log.close()
    node.destroy_node()
    rclpy.shutdown()
