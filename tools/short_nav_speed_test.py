#!/usr/bin/env python3
import math
import sys
import time

import rclpy
from rclpy.action import ActionClient
from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import PoseStamped
from tf2_ros import Buffer, TransformListener


def main():
    distance = float(sys.argv[1]) if len(sys.argv) > 1 else 0.4
    if not 0.1 <= distance <= 1.0:
        raise ValueError('distance must be between 0.1 and 1.0 m')
    rclpy.init()
    node = rclpy.create_node('short_nav_speed_test')
    buf = Buffer()
    listener = TransformListener(buf, node)
    end = time.monotonic() + 12
    pose = None
    while time.monotonic() < end:
        rclpy.spin_once(node, timeout_sec=0.2)
        try:
            pose = buf.lookup_transform('map', 'base_footprint', rclpy.time.Time())
            break
        except Exception:
            pass
    if pose is None:
        raise RuntimeError('No map to base_footprint TF; no goal sent')
    p = pose.transform.translation
    q = pose.transform.rotation
    yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
    goal = NavigateToPose.Goal()
    goal.pose = PoseStamped()
    goal.pose.header.frame_id = 'map'
    goal.pose.header.stamp = node.get_clock().now().to_msg()
    goal.pose.pose.position.x = p.x + distance * math.cos(yaw)
    goal.pose.pose.position.y = p.y + distance * math.sin(yaw)
    goal.pose.pose.orientation = q
    print(f'FROM ({p.x:.3f}, {p.y:.3f}, {yaw:.3f}) TO ({goal.pose.pose.position.x:.3f}, {goal.pose.pose.position.y:.3f})', flush=True)
    client = ActionClient(node, NavigateToPose, 'navigate_to_pose')
    if not client.wait_for_server(timeout_sec=5):
        raise RuntimeError('Nav2 action unavailable; no goal sent')
    future = client.send_goal_async(goal)
    rclpy.spin_until_future_complete(node, future, timeout_sec=5)
    handle = future.result()
    if handle is None or not handle.accepted:
        raise RuntimeError('Goal not accepted')
    print('GOAL ACCEPTED', flush=True)
    result = handle.get_result_async()
    rclpy.spin_until_future_complete(node, result, timeout_sec=35)
    if not result.done():
        cancel = handle.cancel_goal_async()
        rclpy.spin_until_future_complete(node, cancel, timeout_sec=5)
        rclpy.spin_until_future_complete(node, result, timeout_sec=5)
        if not result.done():
            raise RuntimeError('TIMEOUT: cancellation not confirmed; stop navigation')
        print(f'TIMEOUT: terminal status {result.result().status}', flush=True)
    else:
        print(f'FINAL STATUS {result.result().status}', flush=True)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
