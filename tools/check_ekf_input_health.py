#!/usr/bin/env python3
"""Fail-closed finite-value check for LuckRobot odometry inputs and outputs."""

import argparse
import math
import time

import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Imu
from std_msgs.msg import Bool


def finite(values):
    return all(math.isfinite(float(value)) for value in values)


def quaternion_values(q):
    return (q.x, q.y, q.z, q.w)


def vector_values(v):
    return (v.x, v.y, v.z)


class InputHealth(Node):
    def __init__(self):
        super().__init__('ekf_input_health_check')
        self.counts = {
            '/wheel/odom': 0,
            '/imu/data': 0,
            '/odometry/filtered': 0,
            '/visual_slam/tracking/odometry_validated': 0,
        }
        self.invalid = {topic: [] for topic in self.counts}
        self.health_samples = 0
        self.health_true_samples = 0
        self.health_false_after_true = 0
        self.health_last = None
        self.health_transitions = 0
        self.create_subscription(
            Odometry, '/wheel/odom',
            lambda msg: self.check_odom('/wheel/odom', msg, 2.0, 15.0),
            qos_profile_sensor_data,
        )
        self.create_subscription(
            Imu, '/imu/data', self.check_imu, qos_profile_sensor_data
        )
        self.create_subscription(
            Odometry, '/odometry/filtered',
            lambda msg: self.check_odom('/odometry/filtered', msg, 2.0, 15.0),
            qos_profile_sensor_data,
        )
        self.create_subscription(
            Odometry, '/visual_slam/tracking/odometry_validated',
            lambda msg: self.check_odom(
                '/visual_slam/tracking/odometry_validated', msg, 2.0, 15.0
            ),
            qos_profile_sensor_data,
        )
        self.create_subscription(
            Bool, '/vslam/health', self.check_vslam_health, 10
        )

    def check_vslam_health(self, msg):
        value = bool(msg.data)
        self.health_samples += 1
        if value:
            self.health_true_samples += 1
        elif self.health_true_samples > 0:
            self.health_false_after_true += 1
        if self.health_last is not None and value != self.health_last:
            self.health_transitions += 1
        self.health_last = value

    def reject(self, topic, reason):
        reasons = self.invalid[topic]
        if len(reasons) < 8:
            reasons.append(reason)

    def check_odom(self, topic, msg, max_linear, max_angular):
        self.counts[topic] += 1
        pose = msg.pose.pose
        twist = msg.twist.twist
        values = (
            *vector_values(pose.position),
            *quaternion_values(pose.orientation),
            *vector_values(twist.linear),
            *vector_values(twist.angular),
            *msg.pose.covariance,
            *msg.twist.covariance,
        )
        if not finite(values):
            self.reject(topic, 'nonfinite_field_or_covariance')
            return
        qnorm = math.sqrt(sum(value * value for value in quaternion_values(pose.orientation)))
        if qnorm < 0.5 or qnorm > 1.5:
            self.reject(topic, f'invalid_quaternion_norm={qnorm:.6g}')
        linear = math.sqrt(sum(value * value for value in vector_values(twist.linear)))
        angular = math.sqrt(sum(value * value for value in vector_values(twist.angular)))
        if linear > max_linear:
            self.reject(topic, f'linear_speed={linear:.6g}')
        if angular > max_angular:
            self.reject(topic, f'angular_speed={angular:.6g}')

    def check_imu(self, msg):
        topic = '/imu/data'
        self.counts[topic] += 1
        values = (
            *quaternion_values(msg.orientation),
            *vector_values(msg.angular_velocity),
            *vector_values(msg.linear_acceleration),
            *msg.orientation_covariance,
            *msg.angular_velocity_covariance,
            *msg.linear_acceleration_covariance,
        )
        if not finite(values):
            self.reject(topic, 'nonfinite_field_or_covariance')
            return
        qnorm = math.sqrt(sum(value * value for value in quaternion_values(msg.orientation)))
        if qnorm < 0.5 or qnorm > 1.5:
            self.reject(topic, f'invalid_quaternion_norm={qnorm:.6g}')
        angular = math.sqrt(sum(value * value for value in vector_values(msg.angular_velocity)))
        acceleration = math.sqrt(
            sum(value * value for value in vector_values(msg.linear_acceleration))
        )
        if angular > 20.0:
            self.reject(topic, f'angular_speed={angular:.6g}')
        if acceleration > 100.0:
            self.reject(topic, f'acceleration={acceleration:.6g}')


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--duration', type=float, default=30.0)
    parser.add_argument(
        '--require-vslam', action='store_true',
        help='require validated VSLAM data and continuously healthy guard',
    )
    args = parser.parse_args()
    if args.duration <= 0.0:
        raise SystemExit('--duration must be positive')

    rclpy.init()
    node = InputHealth()
    deadline = time.monotonic() + args.duration
    try:
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.1)
    finally:
        node.destroy_node()
        rclpy.shutdown()

    print('===== EKF finite input health =====')
    failed = False
    required = {'/wheel/odom', '/imu/data', '/odometry/filtered'}
    if args.require_vslam:
        required.add('/visual_slam/tracking/odometry_validated')
    for topic, count in node.counts.items():
        reasons = node.invalid[topic]
        state = 'PASS'
        if topic in required and count == 0:
            state = 'FAIL(no_data)'
            failed = True
        elif reasons:
            state = 'FAIL(' + ','.join(reasons) + ')'
            failed = True
        elif count == 0:
            state = 'NO_DATA(optional)'
        print(f'{topic}: samples={count} {state}')

    if node.health_samples == 0:
        health_state = (
            'FAIL(no_data)' if args.require_vslam else 'NO_DATA(optional)'
        )
        failed = failed or args.require_vslam
    elif node.health_true_samples == 0:
        health_state = (
            'FAIL(never_healthy)' if args.require_vslam
            else 'WARN(never_healthy)'
        )
        failed = failed or args.require_vslam
    elif node.health_false_after_true > 0:
        health_state = (
            'FAIL(dropped_after_admission=%d)'
            % node.health_false_after_true
        )
        failed = True
    else:
        health_state = 'PASS'
    print(
        '/vslam/health: samples=%d true=%d transitions=%d %s'
        % (
            node.health_samples,
            node.health_true_samples,
            node.health_transitions,
            health_state,
        )
    )
    print('RESULT=' + ('FAIL' if failed else 'PASS'))
    raise SystemExit(1 if failed else 0)


if __name__ == '__main__':
    main()
