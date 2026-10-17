#!/usr/bin/env python3
"""Project a depth image into a height-aware planar safety LaserScan.

Unlike depthimage_to_laserscan's central-row slice, this node reconstructs
depth pixels in the optical frame, transforms them to ``base_footprint`` and
keeps every point inside the robot's physical collision-height envelope.  It
therefore sees table edges and other overhangs above the camera centre while
rejecting the floor by height rather than by image row.
"""

import math

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
)
from rclpy.time import Time
from sensor_msgs.msg import CameraInfo, Image, LaserScan
from tf2_ros import Buffer, TransformException, TransformListener


def quaternion_matrix(x, y, z, w):
    """Return the 3x3 rotation matrix for a normalized quaternion."""
    norm = math.sqrt(x * x + y * y + z * z + w * w)
    if norm < 1.0e-12:
        return np.eye(3, dtype=np.float32)
    x, y, z, w = x / norm, y / norm, z / norm, w / norm
    return np.asarray(
        [
            [1.0 - 2.0 * (y * y + z * z), 2.0 * (x * y - z * w), 2.0 * (x * z + y * w)],
            [2.0 * (x * y + z * w), 1.0 - 2.0 * (x * x + z * z), 2.0 * (y * z - x * w)],
            [2.0 * (x * z - y * w), 2.0 * (y * z + x * w), 1.0 - 2.0 * (x * x + y * y)],
        ],
        dtype=np.float32,
    )


class DepthHeightScan(Node):
    """Publish the nearest depth point in each base-frame azimuth bin."""

    def __init__(self):
        super().__init__('depth_height_scan')
        self.declare_parameter('depth_topic', '/camera/depth/image_raw')
        self.declare_parameter('camera_info_topic', '/camera/depth/camera_info')
        self.declare_parameter('output_topic', '/scan_camera_height_raw')
        self.declare_parameter('target_frame', 'base_footprint')
        self.declare_parameter('min_obstacle_height', 0.04)
        self.declare_parameter('max_obstacle_height', 0.50)
        self.declare_parameter('depth_min', 0.15)
        self.declare_parameter('depth_max', 3.0)
        self.declare_parameter('angle_min', -1.0)
        self.declare_parameter('angle_max', 1.0)
        self.declare_parameter('beam_count', 641)
        self.declare_parameter('row_stride', 2)
        self.declare_parameter('column_stride', 1)
        self.declare_parameter('min_points_per_bin', 5)
        self.declare_parameter('support_range_tolerance', 0.03)
        self.declare_parameter('stats_period', 5.0)

        self.target_frame = str(self.get_parameter('target_frame').value)
        self.min_height = float(self.get_parameter('min_obstacle_height').value)
        self.max_height = float(self.get_parameter('max_obstacle_height').value)
        self.depth_min = float(self.get_parameter('depth_min').value)
        self.depth_max = float(self.get_parameter('depth_max').value)
        self.angle_min = float(self.get_parameter('angle_min').value)
        self.angle_max = float(self.get_parameter('angle_max').value)
        self.beam_count = max(32, int(self.get_parameter('beam_count').value))
        self.row_stride = max(1, int(self.get_parameter('row_stride').value))
        self.column_stride = max(1, int(self.get_parameter('column_stride').value))
        self.min_points_per_bin = max(
            1, int(self.get_parameter('min_points_per_bin').value)
        )
        self.support_range_tolerance = max(
            0.005, float(self.get_parameter('support_range_tolerance').value)
        )
        stats_period = max(1.0, float(self.get_parameter('stats_period').value))

        if not self.min_height < self.max_height:
            raise ValueError('min_obstacle_height must be below max_obstacle_height')
        if not self.depth_min < self.depth_max:
            raise ValueError('depth_min must be below depth_max')
        if not self.angle_min < self.angle_max:
            raise ValueError('angle_min must be below angle_max')

        self.angle_increment = (
            (self.angle_max - self.angle_min) / float(self.beam_count - 1)
        )
        sensor_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=5,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )

        self.scan_pub = self.create_publisher(
            LaserScan, str(self.get_parameter('output_topic').value), sensor_qos
        )
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.camera_info = None
        self.ray_cache_key = None
        self.u_factor = None
        self.v_factor = None
        self.frames_received = 0
        self.frames_published = 0
        self.frames_bad_encoding = 0
        self.frames_no_info = 0
        self.frames_no_tf = 0
        self.last_finite_bins = 0
        self.last_nearest = math.inf
        self.last_median_support = 0.0
        self.last_stamp_ns = None
        self.resolved_tf_frame = None
        self.resolved_coordinate_mode = None
        self.last_tf_error = 'not attempted'

        # rclpy.Node owns an internal ``_subscriptions`` collection.  Do not
        # overwrite it: doing so removes TransformListener's /tf and
        # /tf_static subscriptions from the executor and makes every frame,
        # including base_footprint, appear nonexistent.
        self._subscription_refs = [
            self.create_subscription(
                CameraInfo,
                str(self.get_parameter('camera_info_topic').value),
                self.on_camera_info,
                sensor_qos,
            ),
            self.create_subscription(
                Image,
                str(self.get_parameter('depth_topic').value),
                self.on_depth,
                sensor_qos,
            ),
        ]
        self.create_timer(stats_period, self.log_stats)
        self.get_logger().info(
            'Height-aware depth safety scan armed: z=[%.3f, %.3f] m in %s, '
            'depth=[%.2f, %.2f] m, %d beams, surface support >=%d within %.3f m'
            % (
                self.min_height,
                self.max_height,
                self.target_frame,
                self.depth_min,
                self.depth_max,
                self.beam_count,
                self.min_points_per_bin,
                self.support_range_tolerance,
            )
        )

    def on_camera_info(self, message):
        if message.width and message.height and message.k[0] > 0.0 and message.k[4] > 0.0:
            self.camera_info = message

    def depth_array(self, message):
        encoding = message.encoding.upper()
        if encoding in ('16UC1', 'MONO16'):
            dtype = np.dtype('>u2' if message.is_bigendian else '<u2')
            row_elements = message.step // dtype.itemsize
            raw = np.frombuffer(message.data, dtype=dtype)
            if raw.size < message.height * row_elements:
                return None
            values = raw[: message.height * row_elements].reshape(
                message.height, row_elements
            )[:, : message.width]
            return values.astype(np.float32) * 0.001
        if encoding == '32FC1':
            dtype = np.dtype('>f4' if message.is_bigendian else '<f4')
            row_elements = message.step // dtype.itemsize
            raw = np.frombuffer(message.data, dtype=dtype)
            if raw.size < message.height * row_elements:
                return None
            values = raw[: message.height * row_elements].reshape(
                message.height, row_elements
            )[:, : message.width]
            return values.astype(np.float32, copy=False)
        return None

    def ray_factors(self, width, height):
        info = self.camera_info
        key = (
            width,
            height,
            float(info.k[0]),
            float(info.k[4]),
            float(info.k[2]),
            float(info.k[5]),
            self.row_stride,
            self.column_stride,
        )
        if key != self.ray_cache_key:
            columns = np.arange(0, width, self.column_stride, dtype=np.float32)
            rows = np.arange(0, height, self.row_stride, dtype=np.float32)
            uu, vv = np.meshgrid(columns, rows)
            self.u_factor = (uu - float(info.k[2])) / float(info.k[0])
            self.v_factor = (vv - float(info.k[5])) / float(info.k[4])
            self.ray_cache_key = key
        return self.u_factor, self.v_factor

    def on_depth(self, message):
        self.frames_received += 1
        if self.camera_info is None:
            self.frames_no_info += 1
            return
        depth = self.depth_array(message)
        if depth is None:
            self.frames_bad_encoding += 1
            return

        source_frame = self.camera_info.header.frame_id or message.header.frame_id
        if not source_frame:
            self.frames_no_tf += 1
            return
        transform, coordinate_mode = self.lookup_camera_transform(source_frame)
        if transform is None:
            self.frames_no_tf += 1
            return

        u_factor, v_factor = self.ray_factors(message.width, message.height)
        sampled = depth[:: self.row_stride, :: self.column_stride]
        valid_depth = (
            np.isfinite(sampled)
            & (sampled >= self.depth_min)
            & (sampled <= self.depth_max)
        )
        if not np.any(valid_depth):
            self.publish_scan(
                message,
                np.full(self.beam_count, np.inf, np.float32),
                np.zeros(self.beam_count, np.int32),
            )
            return

        z_optical = sampled[valid_depth]
        x_optical = u_factor[valid_depth] * z_optical
        y_optical = v_factor[valid_depth] * z_optical

        if coordinate_mode == 'camera_body':
            # REP-103 optical axes are x-right, y-down, z-forward, while the
            # camera body frame is x-forward, y-left, z-up.
            point_x = z_optical
            point_y = -x_optical
            point_z = -y_optical
        else:
            point_x = x_optical
            point_y = y_optical
            point_z = z_optical

        rotation = transform.transform.rotation
        matrix = quaternion_matrix(rotation.x, rotation.y, rotation.z, rotation.w)
        translation = transform.transform.translation
        base_x = (
            matrix[0, 0] * point_x
            + matrix[0, 1] * point_y
            + matrix[0, 2] * point_z
            + float(translation.x)
        )
        base_y = (
            matrix[1, 0] * point_x
            + matrix[1, 1] * point_y
            + matrix[1, 2] * point_z
            + float(translation.y)
        )
        base_z = (
            matrix[2, 0] * point_x
            + matrix[2, 1] * point_y
            + matrix[2, 2] * point_z
            + float(translation.z)
        )
        in_height = (base_z >= self.min_height) & (base_z <= self.max_height)
        if not np.any(in_height):
            self.publish_scan(
                message,
                np.full(self.beam_count, np.inf, np.float32),
                np.zeros(self.beam_count, np.int32),
            )
            return

        base_x = base_x[in_height]
        base_y = base_y[in_height]
        planar_range = np.hypot(base_x, base_y)
        angle = np.arctan2(base_y, base_x)
        in_fov = (
            np.isfinite(planar_range)
            & (angle >= self.angle_min)
            & (angle <= self.angle_max)
        )
        angle = angle[in_fov]
        planar_range = planar_range[in_fov]
        bins = np.rint((angle - self.angle_min) / self.angle_increment).astype(
            np.int32
        )
        bins = np.clip(bins, 0, self.beam_count - 1)

        ranges = np.full(self.beam_count, np.inf, dtype=np.float32)
        np.minimum.at(ranges, bins, planar_range)

        # Count only points supporting the nearest surface in each azimuth
        # bin. The previous implementation counted every point in the bin, so
        # one near flying pixel plus many far wall pixels was incorrectly
        # accepted and the near pixel won the minimum-range projection.
        # Field A/B testing found the persistent empty-scene return had four
        # same-depth samples, while the real plate had about 48. Requiring
        # five therefore rejects that artifact with ample margin for the
        # measured real obstacle.
        supports_nearest = (
            planar_range <= ranges[bins] + self.support_range_tolerance
        )
        support_counts = np.bincount(
            bins[supports_nearest], minlength=self.beam_count
        ).astype(np.int32, copy=False)
        ranges[support_counts < self.min_points_per_bin] = np.inf
        self.publish_scan(message, ranges, support_counts)

    def lookup_camera_transform(self, source_frame):
        """Resolve optical TF, with a REP-103 camera-body fallback.

        Some Orbbec launches expose ``camera_depth_frame`` to downstream
        consumers before (or instead of) making the optical child visible to
        a newly created Python TF listener. The depth image remains a pinhole
        optical image, so the fallback explicitly converts its coordinates.
        """
        candidates = []
        if self.resolved_tf_frame is not None:
            candidates.append(
                (self.resolved_tf_frame, self.resolved_coordinate_mode)
            )
        else:
            candidates.append((source_frame, 'optical'))
            if source_frame.endswith('_optical_frame'):
                candidates.append(
                    (source_frame[: -len('_optical_frame')] + '_frame', 'camera_body')
                )

        errors = []
        for frame, coordinate_mode in candidates:
            try:
                transform = self.tf_buffer.lookup_transform(
                    self.target_frame, frame, Time()
                )
                if self.resolved_tf_frame is None:
                    self.resolved_tf_frame = frame
                    self.resolved_coordinate_mode = coordinate_mode
                    self.get_logger().info(
                        'Height scan TF resolved: %s <- %s (%s coordinates)'
                        % (self.target_frame, frame, coordinate_mode)
                    )
                return transform, coordinate_mode
            except TransformException as error:
                errors.append('%s: %s' % (frame, str(error)))

        # If a previously resolved frame disappears, permit a fresh fallback
        # search on the next image instead of locking the node forever.
        if self.resolved_tf_frame is not None:
            self.resolved_tf_frame = None
            self.resolved_coordinate_mode = None
        self.last_tf_error = ' | '.join(errors)
        return None, None

    def publish_scan(self, image, ranges, support_counts):
        scan = LaserScan()
        scan.header.stamp = image.header.stamp
        scan.header.frame_id = self.target_frame
        scan.angle_min = self.angle_min
        scan.angle_max = self.angle_max
        scan.angle_increment = self.angle_increment
        stamp_ns = image.header.stamp.sec * 1_000_000_000 + image.header.stamp.nanosec
        if self.last_stamp_ns is None or stamp_ns <= self.last_stamp_ns:
            scan.scan_time = 0.0
        else:
            scan.scan_time = (stamp_ns - self.last_stamp_ns) / 1.0e9
        self.last_stamp_ns = stamp_ns
        scan.time_increment = 0.0
        # These ranges are measured from base_footprint, not from the camera.
        scan.range_min = 0.05
        scan.range_max = self.depth_max + 0.5
        scan.ranges = ranges.tolist()
        # Preserve nearest-surface support for diagnostics. Downstream
        # LaserScan consumers ignore intensities unless explicitly requested.
        scan.intensities = support_counts.astype(np.float32).tolist()
        self.scan_pub.publish(scan)
        finite = ranges[np.isfinite(ranges)]
        finite_support = support_counts[np.isfinite(ranges)]
        self.last_finite_bins = int(finite.size)
        self.last_nearest = float(np.min(finite)) if finite.size else math.inf
        self.last_median_support = (
            float(np.median(finite_support)) if finite_support.size else 0.0
        )
        self.frames_published += 1

    def log_stats(self):
        nearest = 'none' if not math.isfinite(self.last_nearest) else '%.3f m' % self.last_nearest
        self.get_logger().info(
            'Height scan stats: rx=%d pub=%d no_info=%d no_tf=%d bad_encoding=%d '
            'finite_bins=%d nearest=%s median_surface_support=%.1f tf=%s'
            % (
                self.frames_received,
                self.frames_published,
                self.frames_no_info,
                self.frames_no_tf,
                self.frames_bad_encoding,
                self.last_finite_bins,
                nearest,
                self.last_median_support,
                self.resolved_tf_frame or self.last_tf_error,
            )
        )


def main(args=None):
    rclpy.init(args=args)
    node = DepthHeightScan()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
