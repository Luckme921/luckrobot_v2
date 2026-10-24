#!/usr/bin/env python3
"""Last navigation gate: missing/stale sensors or localization => zero velocity.

Humble Collision Monitor can ignore an invalid source instead of stopping.
This independent gate is downstream of it, with a steady-clock watchdog.
It never generates a goal and never repeats a cached nonzero command.
"""
import math
import time

import rclpy
from rclpy.clock import Clock, ClockType
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan, Imu
from std_msgs.msg import String
from tf2_msgs.msg import TFMessage

from motion_freshness_core import FreshnessGate, RecoveryRamp, limit_twist


class MotionFreshnessGuard(Node):
    def __init__(self):
        super().__init__('motion_freshness_guard')
        defaults = {
            'input_topic': '/cmd_vel_safe', 'output_topic': '/cmd_vel_drive',
            'map_tf_timeout': 0.75, 'odom_tf_timeout': 0.30,
            'odom_timeout': 0.30, 'lidar_timeout': 0.35, 'camera_timeout': 0.35,
            'wheel_timeout': 0.30, 'imu_timeout': 0.30,
            'command_timeout': 0.20, 'recovery_time': 0.50,
            'max_linear_velocity': 0.30, 'max_reverse_velocity': 0.15,
            'max_angular_velocity': 0.35,
        }
        for name, value in defaults.items():
            self.declare_parameter(name, value)
        p = lambda key: self.get_parameter(key).value
        self.policy = FreshnessGate({
            'map_tf': p('map_tf_timeout'), 'odom_tf': p('odom_tf_timeout'),
            'odom': p('odom_timeout'), 'lidar': p('lidar_timeout'),
            'camera': p('camera_timeout'),
            'wheel': p('wheel_timeout'), 'imu': p('imu_timeout'),
        }, recovery_time=p('recovery_time'))
        self.command_timeout = p('command_timeout')
        self.max_linear = p('max_linear_velocity')
        self.max_reverse = p('max_reverse_velocity')
        self.max_angular = p('max_angular_velocity')
        if any(not math.isfinite(x) or x <= 0 for x in
               (self.command_timeout, self.max_linear, self.max_reverse, self.max_angular)):
            raise ValueError('Velocity/watchdog limits must be finite and positive')
        # Best effort listeners cannot backpressure TF/sensor producers.
        latest = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)
        output_qos = QoSProfile(depth=1, lifespan=Duration(seconds=0.15))
        self.publisher = self.create_publisher(Twist, p('output_topic'), output_qos)
        self.status = self.create_publisher(String, '/motion_freshness_guard/status', 1)
        self.create_subscription(Twist, p('input_topic'), self.on_command, latest)
        self.create_subscription(TFMessage, '/tf', self.on_tf,
                                 QoSProfile(depth=20, reliability=ReliabilityPolicy.BEST_EFFORT))
        self.create_subscription(Odometry, '/odometry/filtered',
                                 lambda m: self.observe('odom', m.header.stamp), latest)
        self.create_subscription(Odometry, '/wheel/odom',
                                 lambda m: self.observe('wheel', m.header.stamp), latest)
        self.create_subscription(Imu, '/imu/data',
                                 lambda m: self.observe('imu', m.header.stamp), latest)
        self.create_subscription(LaserScan, '/scan_filtered',
                                 lambda m: self.observe('lidar', m.header.stamp), latest)
        self.create_subscription(LaserScan, '/scan_camera_safety',
                                 lambda m: self.observe('camera', m.header.stamp), latest)
        self.last_command = None
        self.last_output_time = time.monotonic()
        self.last_output = (0.0, 0.0)
        self.recovery_ramp = RecoveryRamp()
        self.last_status = ''
        self.last_status_time = 0.0
        self.timer = self.create_timer(0.05, self.watchdog,
                                      clock=Clock(clock_type=ClockType.STEADY_TIME))
        self.get_logger().info('Final freshness gate active: %s -> %s; no cached-command replay'
                               % (p('input_topic'), p('output_topic')))

    def observe(self, key, stamp):
        self.policy.observe(key, stamp.sec + stamp.nanosec / 1e9, time.monotonic())

    def on_tf(self, message):
        for tr in message.transforms:
            pair = (tr.header.frame_id.lstrip('/'), tr.child_frame_id.lstrip('/'))
            if pair == ('map', 'odom'):
                self.observe('map_tf', tr.header.stamp)
            elif pair == ('odom', 'base_footprint'):
                self.observe('odom_tf', tr.header.stamp)

    def health(self, now):
        return self.policy.check(self.get_clock().now().nanoseconds / 1e9, now)

    def emit(self, command, now):
        message = Twist()
        message.linear.x, message.angular.z = command
        self.publisher.publish(message)
        self.last_output, self.last_output_time = command, now

    def report(self, reason, now):
        if (reason.split(':')[0] != self.last_status.split(':')[0]
                or now - self.last_status_time >= 1.0):
            self.status.publish(String(data=reason))
            # Age values change every tick; do not flood logs under an outage.
            if reason.split(':')[0] != self.last_status.split(':')[0]:
                self.get_logger().info('Motion gate: ' + reason)
            self.last_status, self.last_status_time = reason, now

    def on_command(self, message):
        now = time.monotonic()
        valid, reason = self.health(now)
        values = (message.linear.x, message.linear.y, message.linear.z,
                  message.angular.x, message.angular.y, message.angular.z)
        if not all(math.isfinite(x) for x in values):
            valid, reason = False, 'command: non-finite'
        if valid:
            target = limit_twist(message.linear.x, message.angular.z,
                                 self.max_linear, self.max_reverse, self.max_angular)
            command = self.recovery_ramp.apply(
                self.last_output, target, now - self.last_output_time)
            self.last_command = now
        else:
            command = (0.0, 0.0)
            self.last_command = None
            self.recovery_ramp.reset()
        self.emit(command, now)
        self.report(reason, now)

    def watchdog(self):
        now = time.monotonic()
        valid, reason = self.health(now)
        if self.last_command is None or now - self.last_command > self.command_timeout:
            valid = False
            if reason == 'ready':
                reason = 'command: waiting for fresh input'
        if not valid:
            self.emit((0.0, 0.0), now)
            self.last_command = None
            self.recovery_ramp.reset()
        self.report(reason, now)


def main():
    rclpy.init()
    node = MotionFreshnessGuard()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            node.emit((0.0, 0.0), time.monotonic())
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
