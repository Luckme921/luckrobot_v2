"""Real SLAM serialization/localization with synthetic stationary room sensors.

ROS_DOMAIN_ID=193 is mandatory. Nav2 lifecycle is simulated; NO hardware launch.
"""
import json
import math
import os
import signal
import subprocess
from pathlib import Path
import sys
import tempfile
import time

REAL_NAV = os.environ.get('MAP_SESSION_REAL_NAV') == '1'
assert os.environ.get('ROS_DOMAIN_ID') == '193', 'isolated test domain required'
sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'scripts'))
import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from lifecycle_msgs.srv import GetState
from nav2_msgs.srv import ManageLifecycleNodes
from action_msgs.srv import CancelGoal
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped, PoseWithCovarianceStamped
from rclpy.parameter import Parameter
from tf2_ros import TransformBroadcaster, StaticTransformBroadcaster
from std_srvs.srv import Trigger
from map_session_manager import Manager


class Fixture(Node):
    def __init__(self):
        super().__init__('stationary_room_fixture')
        self.state = 1
        self.nav_commands = []
        self.scan = self.create_publisher(LaserScan, '/scan_filtered', 10)
        self.odom = self.create_publisher(Odometry, '/odometry/filtered', 10)
        self.tf = TransformBroadcaster(self)
        self.static_tf = StaticTransformBroadcaster(self)
        t = TransformStamped()
        t.header.frame_id, t.child_frame_id = 'base_footprint', 'laser'
        t.transform.rotation.w = 1.
        self.static_tf.sendTransform(t)
        if not REAL_NAV:
            for name in ('controller_server', 'planner_server', 'bt_navigator', 'behavior_server', 'velocity_smoother'):
                self.create_service(GetState, '/' + name + '/get_state', self.get_state)
            self.create_service(ManageLifecycleNodes, '/lifecycle_manager_navigation/manage_nodes', self.manage)
            for action in ('navigate_to_pose', 'navigate_through_poses'):
                self.create_service(CancelGoal, '/' + action + '/_action/cancel_goal', lambda req, res: res)
        self.create_timer(.02, self.sensors)
        self.steps = 0

    def get_state(self, req, res):
        res.current_state.id = self.state
        return res

    def manage(self, req, res):
        self.nav_commands.append(req.command)
        self.state = 2 if req.command == req.PAUSE else 3
        res.success = True
        return res

    def sensors(self):
        stamp = self.get_clock().now().to_msg()
        t = TransformStamped()
        t.header.stamp, t.header.frame_id, t.child_frame_id = stamp, 'odom', 'base_footprint'
        t.transform.rotation.w = 1.
        self.tf.sendTransform(t)
        msg = Odometry()
        msg.header.stamp, msg.header.frame_id, msg.child_frame_id = stamp, 'odom', 'base_footprint'
        msg.pose.pose.orientation.w = 1.
        self.odom.publish(msg)
        self.steps += 1
        if self.steps % 5:
            return
        scan = LaserScan()
        scan.header.stamp, scan.header.frame_id = stamp, 'laser'
        scan.angle_min, scan.angle_max = -math.pi, math.pi
        scan.angle_increment = 2*math.pi/720
        scan.range_min, scan.range_max, scan.scan_time = .2, 12., .1
        scan.ranges = [min(3./max(abs(math.cos(-math.pi+i*scan.angle_increment)), 1e-6),
                           2./max(abs(math.sin(-math.pi+i*scan.angle_increment)), 1e-6)) for i in range(721)]
        self.scan.publish(scan)


def main():
    root = tempfile.mkdtemp(prefix='map-session-integration-')
    rclpy.init(args=['--ros-args', '-p', 'map_root:='+root, '-p', 'auto_explore:=false', '-p', 'boot_mode:=mapping'])
    nav_process = None
    nav_log = None
    if REAL_NAV:
        nav_log = open(root+'/nav2.log', 'w')
        nav_process = subprocess.Popen(['ros2', 'launch', 'my_robot_navigation', 'navigation.launch.py',
            'use_slam:=true', 'navigation_autostart:=false', 'navigation_profile:=luck_full',
            'motion_speed_limit:=0.40', 'motion_angular_limit:=0.80'],
            stdout=nav_log, stderr=subprocess.STDOUT, start_new_session=True)
    fixture = Fixture()
    manager = Manager()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(fixture)
    executor.add_node(manager)

    def wait(state, seconds=100):
        end = time.monotonic()+seconds
        while time.monotonic() < end:
            executor.spin_once(timeout_sec=.05)
            if manager.state == 'ERROR':
                raise AssertionError(manager.status)
            if manager.state == state and not manager.busy and manager.jobs.empty():
                return
        raise AssertionError('timeout waiting '+state+': '+str(manager.status))

    try:
        wait('MAPPING')
        assert fixture.state == 1, 'navigation must stay inactive while preparing map'
        # Exercise the same queued completion transaction the explorer callback invokes.
        manager.jobs.put('complete')
        wait('NAV_READY')
        record = manager.store.current()
        assert record and record['complete']
        assert REAL_NAV or fixture.state == 3
        assert manager.mode == 'localization'
        assert len([n for n in manager.get_node_names() if n == 'slam_toolbox']) == 1
        first = record['map_id']
        assert manager.command('pause', Trigger.Response()).success
        wait('PAUSED')
        assert REAL_NAV or fixture.state == 2
        assert manager.command('resume', Trigger.Response()).success
        wait('NAV_READY')
        assert manager.store.current()['map_id'] == first
        manager.close()
        executor.remove_node(manager)
        manager.destroy_node()
        executor.remove_node(fixture)
        fixture.destroy_node()
        fixture = Fixture()
        if not REAL_NAV:
            fixture.state = 2
        executor.add_node(fixture)
        manager = Manager()
        manager.set_parameters([Parameter('boot_mode', value='auto')])
        executor.add_node(manager)
        wait('WAITING_INITIAL_POSE')
        assert manager.mode is None, 'reboot must not assume saved robot pose'
        seed_pub = fixture.create_publisher(PoseWithCovarianceStamped, '/mapping/initial_pose', 10)
        # Let DDS match this newly created publisher before its single seed.
        end = time.monotonic()+2
        while time.monotonic() < end:
            executor.spin_once(timeout_sec=.05)
        seed = PoseWithCovarianceStamped()
        seed.header.frame_id = 'map'
        seed.header.stamp = fixture.get_clock().now().to_msg()
        seed.pose.pose.orientation.w = 1.
        seed_pub.publish(seed)
        wait('NAV_READY')
        # Test version-update plumbing without starting an explorer or issuing a goal.
        manager.explore = lambda: manager.transition('MAPPING', 'integration_update_without_motion')
        assert manager.command('update_map', Trigger.Response()).success
        wait('MAPPING')
        assert manager.command('cancel', Trigger.Response()).success
        wait('CANCELLED')
        assert manager.store.current()['map_id'] == first, 'cancelled update replaced complete map'
        checkpoint = json.loads((manager.store.root/'checkpoint.json').read_text())
        assert not checkpoint['complete']
        print(json.dumps({'result': 'PASS', 'root': root, 'map_id': first,
                          'states_checked': ['MAPPING', 'SAVING', 'NAV_READY', 'PAUSED', 'NAV_READY',
                                             'WAITING_INITIAL_POSE', 'NAV_READY', 'MAPPING', 'CANCELLED'],
                          'real_nav2': REAL_NAV, 'nav_commands': fixture.nav_commands}, indent=2), flush=True)
    finally:
        manager.close()
        executor.shutdown()
        manager.destroy_node()
        fixture.destroy_node()
        rclpy.shutdown()
        if nav_process and nav_process.poll() is None:
            os.killpg(nav_process.pid, signal.SIGINT)
            try:
                nav_process.wait(timeout=20)
            except subprocess.TimeoutExpired:
                os.killpg(nav_process.pid, signal.SIGTERM)
                nav_process.wait(timeout=10)
        if nav_log:
            nav_log.close()


if __name__ == '__main__':
    main()
