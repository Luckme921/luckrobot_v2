#!/usr/bin/env python3
"""Confirm camera LaserScan returns across frames in a fixed world frame.

The immediate, lightly filtered scan remains available to Collision Monitor.
This node publishes a second scan for costmaps only after a return has spatial
support in multiple frames after odometry-motion compensation.  This prevents
short RGB-D speckles from being amplified by costmap observation persistence.
"""

from __future__ import annotations

import math
from collections import deque

import numpy as np
import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from rclpy.time import Time
from sensor_msgs.msg import LaserScan
from tf2_ros import Buffer, TransformException, TransformListener


class ScanConfirmationFilter(Node):
    def __init__(self):
        super().__init__('camera_scan_confirmation')
        self.declare_parameter('input_topic', '/scan_camera_safety')
        self.declare_parameter('output_topic', '/scan_camera')
        self.declare_parameter('fixed_frame', 'odom')
        self.declare_parameter('confirmation_frames', 5)
        self.declare_parameter('history_duration', 0.50)
        self.declare_parameter('grid_resolution', 0.04)
        self.declare_parameter('neighbor_radius_cells', 1)
        self.declare_parameter('min_cluster_beams', 7)
        self.declare_parameter('cluster_range_tolerance', 0.08)
        self.declare_parameter('stats_period', 5.0)

        self.fixed_frame = str(self.get_parameter('fixed_frame').value)
        self.confirmation_frames = max(
            2, int(self.get_parameter('confirmation_frames').value)
        )
        self.history_duration = max(
            0.10, float(self.get_parameter('history_duration').value)
        )
        self.grid_resolution = max(
            0.01, float(self.get_parameter('grid_resolution').value)
        )
        self.neighbor_radius = max(
            0, int(self.get_parameter('neighbor_radius_cells').value)
        )
        self.min_cluster_beams = max(
            1, int(self.get_parameter('min_cluster_beams').value)
        )
        self.cluster_range_tolerance = max(
            0.01, float(self.get_parameter('cluster_range_tolerance').value)
        )
        stats_period = max(1.0, float(self.get_parameter('stats_period').value))

        input_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=5,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )
        # RViz's existing LaserScan display requests RELIABLE. A reliable
        # publisher is also compatible with Nav2's best-effort sensor
        # subscribers, while the input remains best-effort to accept either
        # camera-filter publisher policy.
        output_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=5,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
        )
        self.publisher = self.create_publisher(
            LaserScan, str(self.get_parameter('output_topic').value), output_qos
        )
        self.subscription = self.create_subscription(
            LaserScan,
            str(self.get_parameter('input_topic').value),
            self.on_scan,
            input_qos,
        )
        self.tf_buffer = Buffer(cache_time=Duration(seconds=5.0))
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.history = deque()
        self.frames_received = 0
        self.frames_published = 0
        self.frames_without_tf = 0
        self.frames_latest_tf = 0
        self.input_beams = 0
        self.confirmed_beams = 0
        self.last_input_beams = 0
        self.last_confirmed_beams = 0
        self.create_timer(stats_period, self.log_stats)

        self.get_logger().info(
            'Camera planning scan confirmation: %d frames / %.2f s, '
            'grid %.3f m, neighbor %d cells, cluster >= %d beams / %.3f m, '
            'frame=%s'
            % (
                self.confirmation_frames,
                self.history_duration,
                self.grid_resolution,
                self.neighbor_radius,
                self.min_cluster_beams,
                self.cluster_range_tolerance,
                self.fixed_frame,
            )
        )

    @staticmethod
    def yaw_from_quaternion(q):
        return math.atan2(
            2.0 * (q.w * q.z + q.x * q.y),
            1.0 - 2.0 * (q.y * q.y + q.z * q.z),
        )

    def empty_copy(self, message):
        output = LaserScan()
        output.header = message.header
        output.angle_min = message.angle_min
        output.angle_max = message.angle_max
        output.angle_increment = message.angle_increment
        output.time_increment = message.time_increment
        output.scan_time = message.scan_time
        output.range_min = message.range_min
        output.range_max = message.range_max
        output.ranges = [math.inf] * len(message.ranges)
        output.intensities = [0.0] * len(message.ranges)
        return output

    def lookup_transform(self, source_frame, stamp):
        try:
            return (
                self.tf_buffer.lookup_transform(
                    self.fixed_frame,
                    source_frame,
                    Time.from_msg(stamp),
                    timeout=Duration(seconds=0.0),
                ),
                False,
            )
        except TransformException as exact_error:
            # Gemini's global timestamp can lead the newest odom TF by tens
            # to hundreds of milliseconds. Waiting cannot create a past TF
            # when EKF is under load, and publishing an empty planning scan
            # made the complete camera layer disappear. Use the latest
            # available transform as a bounded fallback; at the robot's
            # <=0.26 m/s navigation speed the spatial confirmation grid and
            # 4 cm neighbor tolerance absorb the small pose-age error.
            try:
                transform = self.tf_buffer.lookup_transform(
                    self.fixed_frame,
                    source_frame,
                    Time(),
                    timeout=Duration(seconds=0.02),
                )
                self.frames_latest_tf += 1
                if self.frames_latest_tf <= 3:
                    self.get_logger().warning(
                        'Exact stamped TF unavailable; using latest %s <- %s: %s'
                        % (self.fixed_frame, source_frame, exact_error)
                    )
                return transform, True
            except TransformException as latest_error:
                error = '%s | latest: %s' % (exact_error, latest_error)
            self.frames_without_tf += 1
            if self.frames_without_tf <= 3 or self.frames_without_tf % 50 == 0:
                self.get_logger().warning(
                    'No %s <- %s TF for planning scan confirmation: %s'
                    % (self.fixed_frame, source_frame, error)
                )
            return None, False

    def fixed_cells(self, message, valid_indices, valid_ranges, transform):
        angles = (
            float(message.angle_min)
            + valid_indices.astype(np.float64) * float(message.angle_increment)
        )
        local_x = valid_ranges * np.cos(angles)
        local_y = valid_ranges * np.sin(angles)
        yaw = self.yaw_from_quaternion(transform.transform.rotation)
        cosine = math.cos(yaw)
        sine = math.sin(yaw)
        world_x = (
            float(transform.transform.translation.x)
            + cosine * local_x
            - sine * local_y
        )
        world_y = (
            float(transform.transform.translation.y)
            + sine * local_x
            + cosine * local_y
        )
        cell_x = np.floor(world_x / self.grid_resolution).astype(np.int64)
        cell_y = np.floor(world_y / self.grid_resolution).astype(np.int64)
        return list(zip(cell_x.tolist(), cell_y.tolist()))

    def seen_in_frame(self, cell, frame_cells):
        cx, cy = cell
        radius = self.neighbor_radius
        for dx in range(-radius, radius + 1):
            for dy in range(-radius, radius + 1):
                if (cx + dx, cy + dy) in frame_cells:
                    return True
        return False

    def remove_small_clusters(self, mask, ranges):
        """Keep angular clusters that also form one continuous range surface.

        Checking only adjacent beam indices lets a short near-depth artifact
        borrow neighboring beams from a far wall.  The merged angular run can
        then pass ``min_cluster_beams`` even though the near artifact itself is
        only four or five beams wide.  Split a run whenever adjacent ranges
        jump, so every accepted cluster represents one locally continuous
        surface rather than two unrelated depths.
        """
        filtered = np.zeros_like(mask, dtype=bool)
        start = None
        previous_range = None
        for index in range(mask.size + 1):
            active = index < mask.size and bool(mask[index])
            connected = (
                active
                and start is not None
                and previous_range is not None
                and abs(float(ranges[index]) - previous_range)
                <= self.cluster_range_tolerance
            )
            if active and (start is None or not connected):
                if start is not None and index - start >= self.min_cluster_beams:
                    filtered[start:index] = True
                start = index
            elif not active and start is not None:
                if index - start >= self.min_cluster_beams:
                    filtered[start:index] = True
                start = None
            previous_range = float(ranges[index]) if active else None
        return filtered

    def on_scan(self, message):
        self.frames_received += 1
        output = self.empty_copy(message)
        ranges = np.asarray(message.ranges, dtype=np.float64)
        valid = (
            np.isfinite(ranges)
            & (ranges >= float(message.range_min))
            & (ranges <= float(message.range_max))
        )
        # Only spatially supported surfaces may contribute temporal evidence.
        # Otherwise small rejected speckles seed history and a single later
        # large cluster can incorrectly inherit their five-frame confirmation.
        valid = self.remove_small_clusters(valid, ranges)
        valid_indices = np.flatnonzero(valid)
        self.last_input_beams = int(valid_indices.size)
        self.input_beams += self.last_input_beams
        if valid_indices.size == 0:
            self.publisher.publish(output)
            self.frames_published += 1
            return

        transform, used_latest_tf = self.lookup_transform(
            message.header.frame_id, message.header.stamp
        )
        if transform is None:
            # Costmaps get an empty scan until world-frame confirmation is
            # available. Collision Monitor still receives the independent
            # immediate /scan_camera_safety stream.
            self.publisher.publish(output)
            self.frames_published += 1
            return

        if used_latest_tf:
            # Downstream RViz/costmap TF filters must query the same transform
            # epoch used for spatial confirmation. Keeping the camera's
            # future stamp here would make them repeat the same extrapolation
            # failure even though this node already recovered successfully.
            output.header.stamp = transform.header.stamp

        now_ns = self.get_clock().now().nanoseconds
        cutoff_ns = now_ns - int(self.history_duration * 1.0e9)
        while self.history and self.history[0][0] < cutoff_ns:
            self.history.popleft()

        cells = self.fixed_cells(
            message, valid_indices, ranges[valid_indices], transform
        )
        confirmed_valid = np.zeros(valid_indices.size, dtype=bool)
        for point_index, cell in enumerate(cells):
            frame_count = 1
            for _, frame_cells in self.history:
                if self.seen_in_frame(cell, frame_cells):
                    frame_count += 1
                    if frame_count >= self.confirmation_frames:
                        confirmed_valid[point_index] = True
                        break

        current_cells = set(cells)
        self.history.append((now_ns, current_cells))

        confirmed_mask = np.zeros(ranges.size, dtype=bool)
        confirmed_mask[valid_indices[confirmed_valid]] = True
        confirmed_mask = self.remove_small_clusters(confirmed_mask, ranges)
        confirmed_indices = np.flatnonzero(confirmed_mask)
        output_ranges = np.full(ranges.size, math.inf, dtype=np.float32)
        output_ranges[confirmed_indices] = ranges[confirmed_indices].astype(np.float32)
        output.ranges = output_ranges.tolist()
        if len(message.intensities) == ranges.size:
            input_intensities = np.asarray(message.intensities, dtype=np.float32)
            output_intensities = np.zeros(ranges.size, dtype=np.float32)
            output_intensities[confirmed_indices] = input_intensities[
                confirmed_indices
            ]
            output.intensities = output_intensities.tolist()
        self.last_confirmed_beams = int(confirmed_indices.size)
        self.confirmed_beams += self.last_confirmed_beams
        self.publisher.publish(output)
        self.frames_published += 1

    def log_stats(self):
        ratio = (
            100.0 * self.confirmed_beams / self.input_beams
            if self.input_beams else 0.0
        )
        self.get_logger().info(
            'planning scan: rx=%d pub=%d no_tf=%d latest_tf=%d '
            'beams(last %d->%d, total %.1f%%)'
            % (
                self.frames_received,
                self.frames_published,
                self.frames_without_tf,
                self.frames_latest_tf,
                self.last_input_beams,
                self.last_confirmed_beams,
                ratio,
            )
        )


def main(args=None):
    rclpy.init(args=args)
    node = ScanConfirmationFilter()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
