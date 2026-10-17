#!/usr/bin/env python3
"""Validate and continuously re-anchor cuVSLAM odometry before EKF fusion.

Isaac ROS publishes the useful pose covariance on a separate
PoseWithCovarianceStamped topic.  The raw Odometry covariance can be zero, so
feeding that message directly to robot_localization makes the EKF over-trust a
tracker that has already lost its pose.  This node pairs both messages, rejects
unphysical or uncertain samples, and publishes a guarded odometry stream.

The guarded stream is expressed in the production odom frame.  After a loss it
requires a configurable run of healthy frames and anchors the recovered visual
trajectory to the current production odometry.  It never publishes TF.
"""

import math
from collections import deque

import rclpy
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from geometry_msgs.msg import PoseWithCovarianceStamped, Quaternion
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from std_msgs.msg import Bool


def stamp_seconds(stamp):
    return float(stamp.sec) + float(stamp.nanosec) * 1.0e-9


def yaw_from_quaternion(q):
    return math.atan2(
        2.0 * (q.w * q.z + q.x * q.y),
        1.0 - 2.0 * (q.y * q.y + q.z * q.z),
    )


def wrap_angle(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


def yaw_quaternion(yaw):
    half = 0.5 * yaw
    q = Quaternion()
    q.x = 0.0
    q.y = 0.0
    q.z = math.sin(half)
    q.w = math.cos(half)
    return q


def finite_pose(pose):
    values = (
        pose.position.x,
        pose.position.y,
        pose.position.z,
        pose.orientation.x,
        pose.orientation.y,
        pose.orientation.z,
        pose.orientation.w,
    )
    return all(math.isfinite(value) for value in values)


class VslamOdomGuard(Node):
    def __init__(self):
        super().__init__('vslam_odom_guard')

        self.declare_parameter('raw_odom_topic', '/visual_slam/tracking/odometry')
        self.declare_parameter(
            'pose_covariance_topic',
            '/visual_slam/tracking/vo_pose_covariance',
        )
        self.declare_parameter('reference_odom_topic', '/odometry/filtered')
        self.declare_parameter(
            'validated_odom_topic',
            '/visual_slam/tracking/odometry_validated',
        )
        self.declare_parameter('health_topic', '/vslam/health')
        self.declare_parameter('sync_slop', 0.020)
        self.declare_parameter('max_reference_age', 0.150)
        self.declare_parameter('max_position_std', 0.20)
        self.declare_parameter('max_yaw_std', math.radians(10.0))
        self.declare_parameter('max_translation_step', 0.20)
        self.declare_parameter('max_rotation_step', math.radians(20.0))
        self.declare_parameter('max_linear_speed', 1.00)
        self.declare_parameter('max_angular_speed', 2.50)
        self.declare_parameter('max_reference_position_error', 0.50)
        self.declare_parameter(
            'max_reference_yaw_error', math.radians(30.0)
        )
        self.declare_parameter('recovery_good_samples', 60)
        self.declare_parameter('queue_size', 80)
        self.declare_parameter('reference_queue_size', 300)
        self.declare_parameter('transient_bad_samples', 3)
        self.declare_parameter('stream_timeout', 0.35)

        self.sync_slop = float(self.get_parameter('sync_slop').value)
        self.max_reference_age = float(
            self.get_parameter('max_reference_age').value
        )
        self.max_position_std = float(
            self.get_parameter('max_position_std').value
        )
        self.max_yaw_std = float(self.get_parameter('max_yaw_std').value)
        self.max_translation_step = float(
            self.get_parameter('max_translation_step').value
        )
        self.max_rotation_step = float(
            self.get_parameter('max_rotation_step').value
        )
        self.max_linear_speed = float(
            self.get_parameter('max_linear_speed').value
        )
        self.max_angular_speed = float(
            self.get_parameter('max_angular_speed').value
        )
        self.max_reference_position_error = float(
            self.get_parameter('max_reference_position_error').value
        )
        self.max_reference_yaw_error = float(
            self.get_parameter('max_reference_yaw_error').value
        )
        self.recovery_good_samples = int(
            self.get_parameter('recovery_good_samples').value
        )
        self.queue_size = int(self.get_parameter('queue_size').value)
        self.reference_queue_size = int(
            self.get_parameter('reference_queue_size').value
        )
        self.transient_bad_samples = max(
            1, int(self.get_parameter('transient_bad_samples').value)
        )
        self.stream_timeout = float(
            self.get_parameter('stream_timeout').value
        )

        raw_topic = str(self.get_parameter('raw_odom_topic').value)
        covariance_topic = str(
            self.get_parameter('pose_covariance_topic').value
        )
        reference_topic = str(self.get_parameter('reference_odom_topic').value)
        validated_topic = str(
            self.get_parameter('validated_odom_topic').value
        )
        health_topic = str(self.get_parameter('health_topic').value)

        self.raw_queue = deque(maxlen=self.queue_size)
        self.covariance_queue = deque(maxlen=self.queue_size)
        # VSLAM messages can arrive later than their acquisition timestamp.
        # Retain a short production-odometry history and match by header stamp
        # instead of comparing every visual sample with only the newest EKF
        # message.  This avoids false stale-reference trips under CPU load.
        self.reference_queue = deque(maxlen=self.reference_queue_size)
        self.previous_raw = None
        self.anchor = None
        self.healthy = False
        self.good_run = 0
        self.transient_bad_run = 0
        self.accepted = 0
        self.rejected = 0
        self.last_reason = 'waiting_for_synchronized_vslam'
        self.last_position_std = math.nan
        self.last_yaw_std = math.nan
        self.last_pair_time = None

        self.validated_pub = self.create_publisher(Odometry, validated_topic, 10)
        self.health_pub = self.create_publisher(Bool, health_topic, 10)
        self.diagnostics_pub = self.create_publisher(
            DiagnosticArray, '/diagnostics', 10
        )
        self.create_subscription(
            Odometry, raw_topic, self.raw_callback, qos_profile_sensor_data
        )
        self.create_subscription(
            PoseWithCovarianceStamped,
            covariance_topic,
            self.covariance_callback,
            qos_profile_sensor_data,
        )
        self.create_subscription(
            Odometry, reference_topic, self.reference_callback, 20
        )
        self.create_timer(1.0, self.publish_health)

        self.get_logger().info(
            'VSLAM guard V12.6.3 armed: raw=%s covariance=%s '
            'reference=%s output=%s; '
            'publish_tf=false by design'
            % (raw_topic, covariance_topic, reference_topic, validated_topic)
        )

    def reference_callback(self, msg):
        # Never use a broken production estimate as a recovery anchor.
        if finite_pose(msg.pose.pose):
            self.reference_queue.append(msg)

    def nearest_reference(self, stamp):
        if not self.reference_queue:
            return None
        return min(
            self.reference_queue,
            key=lambda item: abs(stamp_seconds(item.header.stamp) - stamp),
        )

    def raw_callback(self, msg):
        self.raw_queue.append(msg)
        self.match_queues()

    def covariance_callback(self, msg):
        self.covariance_queue.append(msg)
        self.match_queues()

    def match_queues(self):
        while self.raw_queue and self.covariance_queue:
            raw = self.raw_queue[0]
            raw_stamp = stamp_seconds(raw.header.stamp)
            differences = [
                abs(stamp_seconds(item.header.stamp) - raw_stamp)
                for item in self.covariance_queue
            ]
            best_index = min(range(len(differences)), key=differences.__getitem__)
            if differences[best_index] <= self.sync_slop:
                self.raw_queue.popleft()
                covariance = self.covariance_queue[best_index]
                del self.covariance_queue[best_index]
                self.process_pair(raw, covariance)
                continue

            covariance_stamp = stamp_seconds(self.covariance_queue[0].header.stamp)
            if raw_stamp < covariance_stamp - self.sync_slop:
                self.raw_queue.popleft()
                self.reject(
                    'missing_synchronized_covariance', transient=True
                )
                continue
            if covariance_stamp < raw_stamp - self.sync_slop:
                self.covariance_queue.popleft()
                continue
            break

    def process_pair(self, raw, covariance):
        raw_stamp = stamp_seconds(raw.header.stamp)
        self.last_pair_time = self.get_clock().now()
        reference = self.nearest_reference(raw_stamp)
        reason = self.validate_pair(raw, covariance, reference)

        # Always remember the most recent synchronized raw sample.  If cuVSLAM
        # resets to a new origin, the first giant step is rejected; subsequent
        # stable frames may then form a new recovery run and be re-anchored.
        self.previous_raw = raw
        if reason:
            self.reject(
                reason,
                transient=reason in (
                    'missing_reference_odometry',
                    'stale_reference_odometry',
                    'covariance_timestamp_mismatch',
                ),
            )
            return

        self.transient_bad_run = 0
        self.good_run += 1
        if not self.healthy:
            if self.good_run < self.recovery_good_samples:
                self.last_reason = (
                    f'recovering_{self.good_run}_of_{self.recovery_good_samples}'
                )
                return
            self.set_anchor(raw, reference)
            self.healthy = True
            self.last_reason = 'healthy_reanchored'
            self.get_logger().info(
                'VSLAM healthy for %d samples; trajectory re-anchored to %s'
                % (
                    self.recovery_good_samples,
                    reference.header.frame_id or 'odom',
                )
            )

        guarded = self.make_guarded_odometry(raw, covariance, reference)
        self.validated_pub.publish(guarded)
        self.accepted += 1
        self.last_reason = 'healthy'

    def validate_pair(self, raw, covariance, reference):
        if reference is None:
            return 'missing_reference_odometry'

        raw_stamp = stamp_seconds(raw.header.stamp)
        covariance_stamp = stamp_seconds(covariance.header.stamp)
        reference_stamp = stamp_seconds(reference.header.stamp)
        if abs(raw_stamp - covariance_stamp) > self.sync_slop:
            return 'covariance_timestamp_mismatch'
        if abs(raw_stamp - reference_stamp) > self.max_reference_age:
            return 'stale_reference_odometry'

        pose = raw.pose.pose
        if not finite_pose(pose):
            return 'nonfinite_visual_pose'
        quaternion_norm = math.sqrt(
            pose.orientation.x ** 2
            + pose.orientation.y ** 2
            + pose.orientation.z ** 2
            + pose.orientation.w ** 2
        )
        if quaternion_norm < 0.5 or quaternion_norm > 1.5:
            return 'invalid_visual_quaternion'

        pose_covariance = covariance.pose.covariance
        if len(pose_covariance) != 36 or not all(
                math.isfinite(float(value)) for value in pose_covariance):
            return 'nonfinite_visual_covariance_matrix'
        variances = (
            float(pose_covariance[0]),
            float(pose_covariance[7]),
            float(pose_covariance[35]),
        )
        if not all(math.isfinite(value) and value >= 0.0 for value in variances):
            return 'invalid_visual_covariance'
        x_std, y_std, yaw_std = (math.sqrt(value) for value in variances)
        self.last_position_std = max(x_std, y_std)
        self.last_yaw_std = yaw_std
        if self.last_position_std > self.max_position_std:
            return 'visual_position_covariance_high'
        if yaw_std > self.max_yaw_std:
            return 'visual_yaw_covariance_high'

        if self.previous_raw is not None:
            previous_stamp = stamp_seconds(self.previous_raw.header.stamp)
            dt = raw_stamp - previous_stamp
            if dt <= 0.0:
                return 'nonmonotonic_visual_timestamp'
            previous_pose = self.previous_raw.pose.pose
            dx = pose.position.x - previous_pose.position.x
            dy = pose.position.y - previous_pose.position.y
            translation = math.hypot(dx, dy)
            rotation = abs(
                wrap_angle(
                    yaw_from_quaternion(pose.orientation)
                    - yaw_from_quaternion(previous_pose.orientation)
                )
            )
            if translation > self.max_translation_step:
                return 'visual_translation_step_too_large'
            if rotation > self.max_rotation_step:
                return 'visual_rotation_step_too_large'
            if translation / dt > self.max_linear_speed:
                return 'visual_linear_speed_impossible'
            if rotation / dt > self.max_angular_speed:
                return 'visual_angular_speed_impossible'

        if self.healthy and self.anchor is not None:
            guarded_x, guarded_y, guarded_yaw = self.transform_raw_pose(raw)
            reference_pose = reference.pose.pose
            reference_error = math.hypot(
                guarded_x - reference_pose.position.x,
                guarded_y - reference_pose.position.y,
            )
            reference_yaw_error = abs(
                wrap_angle(
                    guarded_yaw
                    - yaw_from_quaternion(reference_pose.orientation)
                )
            )
            if reference_error > self.max_reference_position_error:
                return 'visual_reference_position_disagreement'
            if reference_yaw_error > self.max_reference_yaw_error:
                return 'visual_reference_yaw_disagreement'

        return ''

    def reject(self, reason, transient=False):
        self.rejected += 1
        if transient and self.healthy:
            self.transient_bad_run += 1
            if self.transient_bad_run < self.transient_bad_samples:
                # Drop the unmatched sample, but do not force a full two-second
                # recovery for an isolated DDS scheduling/transport miss.
                self.last_reason = (
                    'healthy_transient_drop_%d_of_%d_%s'
                    % (
                        self.transient_bad_run,
                        self.transient_bad_samples,
                        reason,
                    )
                )
                return

        was_healthy = self.healthy
        self.healthy = False
        self.good_run = 0
        self.transient_bad_run = 0
        self.last_reason = reason
        if was_healthy:
            self.get_logger().warn(
                'VSLAM rejected (%s); visual fusion paused until recovery'
                % reason
            )

    def set_anchor(self, raw, reference):
        raw_pose = raw.pose.pose
        reference_pose = reference.pose.pose
        raw_yaw = yaw_from_quaternion(raw_pose.orientation)
        reference_yaw = yaw_from_quaternion(reference_pose.orientation)
        angle = wrap_angle(reference_yaw - raw_yaw)
        c = math.cos(angle)
        s = math.sin(angle)
        rotated_x = c * raw_pose.position.x - s * raw_pose.position.y
        rotated_y = s * raw_pose.position.x + c * raw_pose.position.y
        self.anchor = (
            reference_pose.position.x - rotated_x,
            reference_pose.position.y - rotated_y,
            angle,
        )

    def transform_raw_pose(self, raw):
        offset_x, offset_y, angle = self.anchor
        pose = raw.pose.pose
        c = math.cos(angle)
        s = math.sin(angle)
        x = offset_x + c * pose.position.x - s * pose.position.y
        y = offset_y + s * pose.position.x + c * pose.position.y
        yaw = wrap_angle(yaw_from_quaternion(pose.orientation) + angle)
        return x, y, yaw

    def make_guarded_odometry(self, raw, covariance, reference):
        # Build a fresh message. Copying the raw Odometry could preserve NaN in
        # currently unused fields and expose a future EKF configuration.
        guarded = Odometry()
        guarded.header.stamp = raw.header.stamp
        x, y, yaw = self.transform_raw_pose(raw)
        guarded.header.frame_id = reference.header.frame_id or 'odom'
        guarded.child_frame_id = reference.child_frame_id or 'base_footprint'
        guarded.pose.pose.position.x = x
        guarded.pose.pose.position.y = y
        guarded.pose.pose.position.z = 0.0
        guarded.pose.pose.orientation = yaw_quaternion(yaw)
        angle = self.anchor[2]
        c = math.cos(angle)
        s = math.sin(angle)
        source_x_variance = max(float(covariance.pose.covariance[0]), 1.0e-6)
        source_y_variance = max(float(covariance.pose.covariance[7]), 1.0e-6)
        rotated_x_variance = c * c * source_x_variance + s * s * source_y_variance
        rotated_y_variance = s * s * source_x_variance + c * c * source_y_variance
        # Only x/y/yaw are meaningful. Use a positive diagonal covariance and
        # mark unused axes uncertain so malformed cross terms cannot upset EKF.
        guarded.pose.covariance = [0.0] * 36
        guarded.pose.covariance[0] = rotated_x_variance
        guarded.pose.covariance[7] = rotated_y_variance
        guarded.pose.covariance[14] = 1.0e6
        guarded.pose.covariance[21] = 1.0e6
        guarded.pose.covariance[28] = 1.0e6
        guarded.pose.covariance[35] = max(
            float(covariance.pose.covariance[35]), 1.0e-6
        )
        guarded.twist.covariance = [0.0] * 36
        for index in (0, 7, 14, 21, 28, 35):
            guarded.twist.covariance[index] = 1.0e6
        return guarded

    def publish_health(self):
        now = self.get_clock().now()
        if self.last_pair_time is None:
            fresh = False
        else:
            fresh = (
                (now - self.last_pair_time).nanoseconds * 1.0e-9
                <= self.stream_timeout
            )
        effective_health = self.healthy and fresh
        if not fresh and self.healthy:
            self.healthy = False
            self.good_run = 0
            self.transient_bad_run = 0
            self.last_reason = 'visual_stream_timeout'

        self.health_pub.publish(Bool(data=effective_health))

        status = DiagnosticStatus()
        status.name = 'vslam_odom_guard: visual odometry health'
        status.hardware_id = 'gemini_336l_cuvslam'
        status.level = (
            DiagnosticStatus.OK if effective_health else DiagnosticStatus.WARN
        )
        status.message = 'healthy' if effective_health else self.last_reason
        status.values = [
            KeyValue(key='healthy', value=str(effective_health).lower()),
            KeyValue(key='state', value=self.last_reason),
            KeyValue(key='good_run', value=str(self.good_run)),
            KeyValue(
                key='transient_bad_run', value=str(self.transient_bad_run)
            ),
            KeyValue(
                key='reference_queue_depth',
                value=str(len(self.reference_queue)),
            ),
            KeyValue(key='accepted_samples', value=str(self.accepted)),
            KeyValue(key='rejected_samples', value=str(self.rejected)),
            KeyValue(
                key='position_std_m', value=f'{self.last_position_std:.6f}'
            ),
            KeyValue(
                key='yaw_std_deg',
                value=f'{math.degrees(self.last_yaw_std):.6f}',
            ),
        ]
        message = DiagnosticArray()
        message.header.stamp = now.to_msg()
        message.status = [status]
        self.diagnostics_pub.publish(message)


def main(args=None):
    rclpy.init(args=args)
    node = VslamOdomGuard()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
