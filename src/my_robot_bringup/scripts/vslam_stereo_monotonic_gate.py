#!/usr/bin/env python3
"""Republish only synchronized, strictly monotonic stereo image pairs."""

import copy
from collections import deque

import rclpy
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
)
from sensor_msgs.msg import CameraInfo, Image


def stamp_ns(message):
    return message.header.stamp.sec * 1_000_000_000 + message.header.stamp.nanosec


class StereoMonotonicGate(Node):
    """Drop repeated/out-of-order Orbbec interleave frames before cuVSLAM."""

    def __init__(self):
        super().__init__('vslam_stereo_monotonic_gate')
        self.declare_parameter('left_image_in', '/camera/left_ir/image_raw')
        self.declare_parameter('right_image_in', '/camera/right_ir/image_raw')
        self.declare_parameter('left_info_in', '/camera/left_ir/camera_info')
        self.declare_parameter('right_info_in', '/camera/right_ir/camera_info')
        self.declare_parameter('left_image_out', '/vslam/stereo/left/image_raw')
        self.declare_parameter('right_image_out', '/vslam/stereo/right/image_raw')
        self.declare_parameter('left_info_out', '/vslam/stereo/left/camera_info')
        self.declare_parameter('right_info_out', '/vslam/stereo/right/camera_info')
        self.declare_parameter('sync_tolerance_ms', 0.1)
        self.declare_parameter('queue_size', 20)
        self.declare_parameter('stats_period', 5.0)

        self.sync_tolerance_ns = int(
            float(self.get_parameter('sync_tolerance_ms').value) * 1_000_000
        )
        queue_size = max(4, int(self.get_parameter('queue_size').value))
        stats_period = max(1.0, float(self.get_parameter('stats_period').value))

        sensor_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=queue_size,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )

        self.left_queue = deque(maxlen=queue_size)
        self.right_queue = deque(maxlen=queue_size)
        self.left_info = None
        self.right_info = None
        self.last_left_stamp = -1
        self.last_right_stamp = -1
        self.received_left = 0
        self.received_right = 0
        self.published_pairs = 0
        self.rejected_nonmonotonic = 0
        self.rejected_unsynchronized = 0
        self.rejected_no_info = 0

        self.left_image_pub = self.create_publisher(
            Image, self.get_parameter('left_image_out').value, sensor_qos
        )
        self.right_image_pub = self.create_publisher(
            Image, self.get_parameter('right_image_out').value, sensor_qos
        )
        self.left_info_pub = self.create_publisher(
            CameraInfo, self.get_parameter('left_info_out').value, sensor_qos
        )
        self.right_info_pub = self.create_publisher(
            CameraInfo, self.get_parameter('right_info_out').value, sensor_qos
        )

        # Node.subscriptions is a read-only rclpy property. Keep explicit
        # references under our own name so Humble cannot garbage-collect them.
        self._subscription_refs = [
            self.create_subscription(
                Image,
                self.get_parameter('left_image_in').value,
                self.on_left_image,
                sensor_qos,
            ),
            self.create_subscription(
                Image,
                self.get_parameter('right_image_in').value,
                self.on_right_image,
                sensor_qos,
            ),
            self.create_subscription(
                CameraInfo,
                self.get_parameter('left_info_in').value,
                self.on_left_info,
                sensor_qos,
            ),
            self.create_subscription(
                CameraInfo,
                self.get_parameter('right_info_in').value,
                self.on_right_info,
                sensor_qos,
            ),
        ]
        self.create_timer(stats_period, self.log_stats)
        self.get_logger().info(
            'VSLAM stereo monotonic gate armed: sync tolerance %.3f ms; '
            'output=/vslam/stereo/{left,right}'
            % (self.sync_tolerance_ns / 1e6)
        )

    def on_left_info(self, message):
        self.left_info = message

    def on_right_info(self, message):
        self.right_info = message

    def on_left_image(self, message):
        self.received_left += 1
        self.left_queue.append(message)
        self.match_pairs()

    def on_right_image(self, message):
        self.received_right += 1
        self.right_queue.append(message)
        self.match_pairs()

    def match_pairs(self):
        while self.left_queue and self.right_queue:
            left = self.left_queue[0]
            right = self.right_queue[0]
            left_stamp = stamp_ns(left)
            right_stamp = stamp_ns(right)
            delta = left_stamp - right_stamp

            if abs(delta) > self.sync_tolerance_ns:
                self.rejected_unsynchronized += 1
                if delta < 0:
                    self.left_queue.popleft()
                else:
                    self.right_queue.popleft()
                continue

            self.left_queue.popleft()
            self.right_queue.popleft()
            if (
                left_stamp <= self.last_left_stamp
                or right_stamp <= self.last_right_stamp
            ):
                self.rejected_nonmonotonic += 1
                continue
            if self.left_info is None or self.right_info is None:
                self.rejected_no_info += 1
                continue

            left_info = copy.deepcopy(self.left_info)
            right_info = copy.deepcopy(self.right_info)
            left_info.header = copy.deepcopy(left.header)
            right_info.header = copy.deepcopy(right.header)

            # Publish calibration first so a newly started cuVSLAM subscriber
            # can initialize before consuming the matching image pair.
            self.left_info_pub.publish(left_info)
            self.right_info_pub.publish(right_info)
            self.left_image_pub.publish(left)
            self.right_image_pub.publish(right)
            self.last_left_stamp = left_stamp
            self.last_right_stamp = right_stamp
            self.published_pairs += 1

    def log_stats(self):
        self.get_logger().info(
            'Stereo gate stats: rx=%d/%d accepted_pairs=%d '
            'drop_nonmonotonic=%d drop_unsynchronized=%d drop_no_info=%d'
            % (
                self.received_left,
                self.received_right,
                self.published_pairs,
                self.rejected_nonmonotonic,
                self.rejected_unsynchronized,
                self.rejected_no_info,
            )
        )


def main(args=None):
    rclpy.init(args=args)
    node = StereoMonotonicGate()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
