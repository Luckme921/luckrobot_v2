#!/usr/bin/env python3
"""Own the SLAM/explorer processes and publish a persistent map-session state.

Run through managed_mapping.launch.py, never beside an independently owned SLAM.
All motion is performed by the existing explorer/Nav2, never by this node.
"""
import fcntl
import json
import math
import os
from pathlib import Path
import queue
import signal
import subprocess
import threading
import time
import uuid

import yaml
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy
from rclpy.time import Time
from ament_index_python.packages import get_package_share_directory
from action_msgs.srv import CancelGoal
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import Odometry, OccupancyGrid
from nav2_msgs.srv import ManageLifecycleNodes
from lifecycle_msgs.srv import GetState
from slam_toolbox.srv import SerializePoseGraph
from std_msgs.msg import String, Bool
from std_srvs.srv import Trigger
from tf2_ros import Buffer, TransformListener
from explore_lite_msgs.msg import ExploreStatus
from map_versions import MapVersions, atomic_json, validate_pose, write_grid


class Interrupted(RuntimeError):
    pass


class Manager(Node):
    def __init__(self):
        super().__init__('map_session_manager')
        for name, value in [('map_root', '~/ros2_ws/map_store'),
                            ('boot_mode', 'auto'), ('boot_pose_json', ''),
                            ('blocked_retry_seconds', 30.0), ('blocked_retry_limit', 3),
                            ('auto_explore', True)]:
            self.declare_parameter(name, value)
        self.store = MapVersions(self.get_parameter('map_root').value)
        self.lock_file = (self.store.root / 'manager.lock').open('a')
        fcntl.flock(self.lock_file, fcntl.LOCK_EX | fcntl.LOCK_NB)
        self.session = uuid.uuid4().hex
        self.state = 'STARTING'
        self.record = None
        self.mode = None
        self.owns_session = False
        self.children = {}
        self.logs = []
        self.jobs = queue.Queue()
        self.interrupt = threading.Event()
        self.shutdown_event = threading.Event()
        self.busy = False
        self.retry_at = None
        self.retries = 0
        self.explorer_started = False
        self.accept_explore_status = False
        self.last_odom = None
        self.last_map = None
        self.last_pose = None
        self.seed = None
        self.pose_epoch = time.monotonic()
        self.tf = Buffer()
        self.tf_listener = TransformListener(self.tf, self)
        qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.status_pub = self.create_publisher(String, '/mapping/status', qos)
        self.ready_pub = self.create_publisher(Bool, '/mapping/navigation_ready', qos)
        self.map_pub = self.create_publisher(String, '/mapping/map_ready', qos)
        self.resume_pub = self.create_publisher(Bool, '/explore/resume', 10)
        self.create_subscription(Odometry, '/odometry/filtered', self.odom, 10)
        self.create_subscription(OccupancyGrid, '/map', self.map_received, qos)
        self.create_subscription(PoseWithCovarianceStamped, '/pose', self.pose_received, 10)
        self.create_subscription(PoseWithCovarianceStamped, '/mapping/initial_pose', self.initial_pose, 10)
        self.create_subscription(PoseWithCovarianceStamped, '/initialpose', self.initial_pose, 10)
        # Volatile subscription rejects retained completion from a previous explorer.
        self.create_subscription(ExploreStatus, '/explore/status', self.explore_status, 10)
        self.rpc_clients = {}
        for command in ['pause', 'resume', 'cancel', 'save_checkpoint', 'load_latest', 'update_map', 'manual_mapping', 'auto_mapping', 'finish_mapping']:
            self.create_service(Trigger, '/mapping/' + command,
                                lambda req, resp, cmd=command: self.command(cmd, resp))
        self.create_timer(1.0, self.tick)
        self.transition('STARTING', 'manager_created')
        self.worker = threading.Thread(target=self.work, daemon=True)
        self.worker.start()
        self.jobs.put('boot')

    def odom(self, msg):
        self.last_odom = (msg, time.monotonic())

    def map_received(self, msg):
        self.last_map = (msg, time.monotonic())

    def pose_received(self, msg):
        self.last_pose = (msg, time.monotonic())

    def initial_pose(self, msg):
        if self.busy or not self.jobs.empty() or self.state != 'WAITING_INITIAL_POSE' or msg.header.frame_id != 'map':
            return
        age = self.get_clock().now().nanoseconds * 1e-9 - self.stamp(msg.header.stamp)
        if not -0.2 <= age <= 2.0:
            return
        q = msg.pose.pose.orientation
        norm = q.x*q.x+q.y*q.y+q.z*q.z+q.w*q.w
        if not math.isfinite(norm) or abs(norm-1.0) > 0.05:
            return
        try:
            self.seed = validate_pose([msg.pose.pose.position.x, msg.pose.pose.position.y,
                                      math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z))])
        except (ValueError, TypeError):
            return
        self.jobs.put('load_latest')

    def command(self, cmd, response):
        allowed = {
            'manual_mapping': self.state in ('IDLE', 'MAPPING', 'PAUSED', 'NAV_READY', 'INCOMPLETE', 'CANCELLED', 'WAITING_INITIAL_POSE'),
            'auto_mapping': self.state in ('IDLE', 'MAPPING', 'PAUSED', 'NAV_READY', 'INCOMPLETE', 'CANCELLED', 'WAITING_INITIAL_POSE'),
            'finish_mapping': self.mode == 'mapping' and self.state in ('MAPPING', 'PAUSED', 'INCOMPLETE'),
            'resume': self.state in ('PAUSED', 'INCOMPLETE', 'CANCELLED', 'MAPPING'),
            'save_checkpoint': self.mode == 'mapping' and self.state in ('EXPLORING', 'BLOCKED', 'PAUSED', 'MAPPING'),
            'load_latest': self.state in ('IDLE', 'MAPPING', 'PAUSED', 'INCOMPLETE', 'CANCELLED', 'WAITING_INITIAL_POSE', 'NAV_READY'),
            'update_map': self.state == 'NAV_READY' and self.record is not None,
            'pause': self.owns_session, 'cancel': self.owns_session,
        }
        if not allowed.get(cmd, False):
            response.success = False
            response.message = 'command not valid in state ' + self.state
            return response
        if cmd in ('pause', 'cancel'):
            # Drop stale completion/retry jobs before servicing the stop request.
            while not self.jobs.empty():
                try:
                    self.jobs.get_nowait()
                except queue.Empty:
                    break
            self.interrupt.set()
        elif self.busy or not self.jobs.empty():
            response.success = False
            response.message = 'busy; inspect /mapping/status'
            return response
        self.jobs.put(cmd)
        response.success = True
        response.message = 'accepted; final result on /mapping/status'
        return response

    def explore_status(self, msg):
        if not self.accept_explore_status or 'explore' not in self.children:
            return
        if msg.status == 'exploration_started':
            self.explorer_started = True
        elif self.explorer_started and self.state == 'EXPLORING':
            if msg.status == 'exploration_complete':
                self.transition('EXPLORATION_COMPLETE', 'accessible_scope_complete; residual narrow/small unknown retained')
                self.jobs.put('complete')
                self.accept_explore_status = False
            elif msg.status.startswith('exploration_blocked'):
                self.transition('BLOCKED', msg.status)
                self.retry_at = time.monotonic() + float(self.get_parameter('blocked_retry_seconds').value)
            elif msg.status.startswith('exploration_error'):
                self.jobs.put('fault')
            elif msg.status == 'exploration_paused':
                # External control is respected; automatic faults use BLOCKED.
                self.jobs.put('pause')

    def transition(self, state, reason):
        self.state = state
        self.status = dict(schema=1, session_id=self.session, state=state, reason=reason,
                           mode=self.mode, retries=self.retries, map=self.record,
                           updated_unix=time.time())
        atomic_json(self.store.root / 'session.json', self.status)
        self.publish()
        self.get_logger().info(state + ': ' + reason)

    def publish(self):
        self.status_pub.publish(String(data=json.dumps(self.status, ensure_ascii=False)))
        self.ready_pub.publish(Bool(data=self.state == 'NAV_READY'))

    def tick(self):
        self.publish()
        if self.busy or not self.jobs.empty():
            return
        if self.state == 'BLOCKED' and self.retry_at is not None and time.monotonic() >= self.retry_at:
            self.retry_at = None
            self.jobs.put('retry')
        if self.state in ('NAV_READY', 'EXPLORING'):
            dead = [name for name, proc in list(self.children.items()) if proc.poll() is not None]
            if dead:
                self.interrupt.set()
                self.jobs.put('fault')
            elif self.state == 'NAV_READY' and not self.localization_healthy():
                self.jobs.put('relocalize')

    @staticmethod
    def stamp(stamp):
        return stamp.sec + stamp.nanosec * 1e-9

    def pose_now(self):
        t = self.tf.lookup_transform('map', 'base_footprint', Time())
        age = self.get_clock().now().nanoseconds*1e-9 - self.stamp(t.header.stamp)
        if not -0.3 <= age <= 1.0:
            raise RuntimeError('stale map/base transform')
        q = t.transform.rotation
        return validate_pose([t.transform.translation.x, t.transform.translation.y,
                              math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))])

    def localization_healthy(self):
        try:
            p = self.last_pose
            if not p or p[1] <= self.pose_epoch or time.monotonic()-p[1] > 3.0:
                return False
            msg = p[0]
            if msg.header.frame_id != 'map':
                return False
            age = self.get_clock().now().nanoseconds*1e-9 - self.stamp(msg.header.stamp)
            if not -0.3 <= age <= 3.0:
                return False
            cov = msg.pose.covariance
            if not all(math.isfinite(cov[i]) and 0 <= cov[i] <= limit
                       for i, limit in ((0, .25), (7, .25), (35, .12))):
                return False
            self.pose_now()
            return self.last_map is not None and len(self.last_map[0].data) > 0
        except Exception:
            return False

    def check_interrupt(self):
        if self.interrupt.is_set() or self.shutdown_event.is_set():
            raise Interrupted('operation interrupted')

    def call(self, typ, name, request, timeout=30.0, interruptible=True):
        client = self.rpc_clients.get(name)
        if client is None:
            client = self.create_client(typ, name)
            self.rpc_clients[name] = client
        end = time.monotonic()+timeout
        while not client.wait_for_service(timeout_sec=.2):
            if self.shutdown_event.is_set():
                raise Interrupted("shutdown")
            if interruptible:
                self.check_interrupt()
            if time.monotonic() >= end:
                raise RuntimeError(name + ' unavailable')
        future = client.call_async(request)
        while not future.done():
            if self.shutdown_event.is_set():
                raise Interrupted("shutdown")
            if interruptible:
                self.check_interrupt()
            if time.monotonic() >= end:
                raise RuntimeError(name + ' timed out')
            time.sleep(.05)
        if future.exception():
            raise future.exception()
        return future.result()

    def nav_states(self):
        return [self.call(GetState, '/' + name + '/get_state', GetState.Request(),
                          timeout=8, interruptible=False).current_state.id
                for name in ('controller_server', 'planner_server', 'bt_navigator',
                             'behavior_server', 'velocity_smoother')]

    def nav(self, enable):
        states = self.nav_states()
        if (enable and all(s == 3 for s in states)) or (not enable and all(s in (1, 2) for s in states)):
            return
        req = ManageLifecycleNodes.Request()
        req.command = (req.STARTUP if all(s == 1 for s in states) else req.RESUME) if enable else req.PAUSE
        res = self.call(ManageLifecycleNodes, '/lifecycle_manager_navigation/manage_nodes',
                        req, interruptible=False)
        expected = (3,) if enable else (1, 2)
        if not res.success or not all(s in expected for s in self.nav_states()):
            raise RuntimeError('navigation lifecycle transition failed')

    def spawn(self, name, argv):
        self.stop_child(name)
        log = (self.store.root / (self.session + '_' + name + '.log')).open('a')
        self.logs.append(log)
        self.children[name] = subprocess.Popen(argv, stdout=log, stderr=subprocess.STDOUT,
                                                start_new_session=True)

    def stop_child(self, name):
        proc = self.children.pop(name, None)
        if proc is None or proc.poll() is not None:
            return
        os.killpg(proc.pid, signal.SIGINT)
        try:
            proc.wait(timeout=12)
        except subprocess.TimeoutExpired:
            os.killpg(proc.pid, signal.SIGTERM)
            try:
                proc.wait(timeout=5)
            except subprocess.TimeoutExpired:
                os.killpg(proc.pid, signal.SIGKILL)
                proc.wait(timeout=3)

    def hold(self, keep_explorer=False):
        self.accept_explore_status = False
        if keep_explorer and 'explore' in self.children:
            self.resume_pub.publish(Bool(data=False))
        else:
            self.stop_child('explore')
        if 3 in self.nav_states():
            for action in ('navigate_to_pose', 'navigate_through_poses'):
                self.call(CancelGoal, '/' + action + '/_action/cancel_goal',
                          CancelGoal.Request(), timeout=8, interruptible=False)
        self.nav(False)
        end = time.monotonic()+10
        stationary_since = None
        while time.monotonic() < end:
            if self.shutdown_event.is_set():
                raise Interrupted("shutdown")
            odom = self.last_odom
            good = False
            if odom and time.monotonic()-odom[1] < .5:
                msg = odom[0]
                age = self.get_clock().now().nanoseconds*1e-9-self.stamp(msg.header.stamp)
                v, w = msg.twist.twist.linear.x, msg.twist.twist.angular.z
                good = -0.2 <= age <= .5 and math.isfinite(v+w) and abs(v)<.015 and abs(w)<.03
            stationary_since = (stationary_since or time.monotonic()) if good else None
            if stationary_since and time.monotonic()-stationary_since >= 1.0:
                return
            time.sleep(.1)
        raise RuntimeError('stationary odometry not confirmed')

    def start_slam(self, mode, record=None, pose=None):
        self.stop_child('slam')
        # Await DDS removal to avoid overlapping map->odom ownership.
        end = time.monotonic()+8
        while 'slam_toolbox' in self.get_node_names():
            if time.monotonic() >= end:
                raise RuntimeError('another slam_toolbox is still present')
            time.sleep(.2)
        self.tf.clear()
        self.last_map = self.last_pose = None
        self.pose_epoch = time.monotonic()
        share = Path(get_package_share_directory('my_robot_mapping'))
        config = 'mapper_params_localization.yaml' if mode == 'localization' else 'mapper_params_online_async.yaml'
        params = yaml.safe_load((share/'config'/config).read_text())
        values = params['slam_toolbox']['ros__parameters']
        if mode == 'localization':
            # Keep scan-matched pose health observable even while stationary.
            values.update(minimum_travel_distance=0.0, minimum_travel_heading=0.0)
        if record:
            values.update(map_file_name=record['prefix'], map_start_pose=validate_pose(pose),
                          map_start_at_dock=False)
        path = self.store.root / (self.session + '_slam.yaml')
        path.write_text(yaml.safe_dump(params))
        executable = 'localization_slam_toolbox_node' if mode == 'localization' else 'async_slam_toolbox_node'
        self.spawn('slam', ['ros2', 'run', 'slam_toolbox', executable, '--ros-args', '--params-file', str(path)])
        self.mode = mode
        end = time.monotonic()+60
        last_error = "map not received"
        next_report = time.monotonic()+10
        while time.monotonic() < end:
            self.check_interrupt()
            if self.children['slam'].poll() is not None:
                raise RuntimeError('SLAM process exited; inspect session log')
            try:
                self.pose_now()
                if self.last_map and len(self.last_map[0].data):
                    return
            except Exception as exc:
                last_error = str(exc)
            if time.monotonic() >= next_report:
                self.get_logger().info('waiting SLAM map=' + str(self.last_map is not None) +
                    ' pose=' + str(self.last_pose is not None) + ' tf=' + last_error)
                next_report = time.monotonic()+10
            time.sleep(.2)
        raise RuntimeError('SLAM map/TF startup timed out: ' + last_error)

    def explore(self):
        self.nav(True)
        proc = self.children.get('explore')
        if proc and proc.poll() is None:
            self.accept_explore_status = True
            self.transition('EXPLORING', 'resumed_existing_branch_memory')
            self.resume_pub.publish(Bool(data=True))
            return
        self.explorer_started = False
        self.accept_explore_status = True
        self.transition('EXPLORING', 'global_explorer_started')
        self.spawn('explore', ['ros2', 'launch', 'my_robot_mapping', 'auto_explore.launch.py', 'global_regions:=true'])

    def save(self, complete, reason):
        self.transition('STOPPING', reason)
        self.hold(keep_explorer=not complete)
        self.check_interrupt()
        self.transition('SAVING', reason)
        pose = self.pose_now()
        if not self.last_map or time.monotonic()-self.last_map[1] > 10:
            raise RuntimeError('map stream stale; snapshot rejected')
        prefix = self.store.begin()
        req = SerializePoseGraph.Request(filename=str(prefix))
        if self.call(SerializePoseGraph, '/slam_toolbox/serialize_map', req).result != 0:
            raise RuntimeError('pose graph save failed')
        grid = self.last_map[0]
        if grid.header.frame_id != 'map':
            raise RuntimeError('unexpected occupancy frame')
        q = grid.info.origin.orientation
        write_grid(prefix, width=grid.info.width, height=grid.info.height,
                   resolution=grid.info.resolution,
                   origin=[grid.info.origin.position.x, grid.info.origin.position.y,
                           math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z))],
                   data=grid.data)
        self.check_interrupt()
        record = self.store.commit(prefix, complete=complete, session=self.session, pose=pose, reason=reason)
        self.record = record
        if complete:
            self.map_pub.publish(String(data=json.dumps(record)))
            self.transition('MAP_READY', 'complete_map_saved_and_verified')
        else:
            self.transition('CHECKPOINT_SAVED', reason)
        return record, pose

    def localize(self, record, pose):
        self.record = record
        self.map_pub.publish(String(data=json.dumps(record)))
        self.transition('LOCALIZING', 'loading_saved_posegraph')
        self.hold()
        self.start_slam('localization', record, pose)
        self.await_localization()

    def await_localization(self):
        end = time.monotonic()+60
        valid_since = None
        while time.monotonic() < end:
            self.check_interrupt()
            valid_since = (valid_since or time.monotonic()) if self.localization_healthy() else None
            if valid_since and time.monotonic()-valid_since > 3:
                self.nav(True)
                self.transition('NAV_READY', 'localization_health_stable')
                return
            time.sleep(.2)
        raise RuntimeError('localization health did not converge')

    def execute(self, job):
        if job not in ('pause', 'cancel', 'fault'):
            self.check_interrupt()
        if job == 'boot':
            # Never attach to or kill somebody else's running SLAM/explorer.
            time.sleep(2)
            self.check_interrupt()
            if {'slam_toolbox', 'explore_node'} & set(self.get_node_names()):
                raise RuntimeError('existing SLAM/explorer detected; use only managed launch')
            self.owns_session = True
            self.nav(False)
            self.record = self.store.current()
            boot_mode = self.get_parameter('boot_mode').value
            if boot_mode not in ('idle', 'auto', 'mapping', 'localization'):
                raise ValueError('invalid boot_mode')
            if boot_mode == 'idle':
                self.transition('IDLE', 'awaiting_mode_command')
                return
            if self.record and boot_mode != 'mapping':
                raw = self.get_parameter('boot_pose_json').value
                if not raw:
                    self.nav(False)
                    self.transition('WAITING_INITIAL_POSE', 'publish trusted map pose; saved shutdown pose is not reused')
                    return
                pose = validate_pose(json.loads(raw))
                self.localize(self.record, pose)
            elif boot_mode == 'localization':
                raise RuntimeError('no validated complete map')
            else:
                self.transition('MAPPING', 'starting_new_map')
                self.start_slam('mapping')
                if self.get_parameter('auto_explore').value:
                    self.explore()
        elif job in ('manual_mapping', 'auto_mapping'):
            # Preserve an existing paused mapping session; changing mode is
            # not permission to silently discard its map.
            self.hold()
            if self.mode != 'mapping':
                self.record = None
                self.seed = None
                self.start_slam('mapping')
            self.retries = 0
            if job == 'auto_mapping':
                self.explore()
            else:
                self.nav(True)
                self.transition('MAPPING', 'manual_mapping_ready; RViz goals accepted')
        elif job == 'finish_mapping':
            record, pose = self.save(True, 'mapping_finished_by_client')
            self.localize(record, pose)
        elif job == 'complete':
            record, pose = self.save(True, 'exploration_complete')
            self.localize(record, pose)
        elif job == 'retry':
            if self.state != 'BLOCKED':
                return
            if self.retries < self.get_parameter('blocked_retry_limit').value:
                self.retries += 1
                self.hold(keep_explorer=True)
                self.explore()
            else:
                self.save(False, 'blocked_retry_budget_exhausted')
                self.transition('INCOMPLETE', 'checkpoint_saved; remaining regions unresolved')
        elif job in ('pause', 'cancel', 'fault'):
            self.hold(keep_explorer=(job == 'pause'))
            if job == 'cancel' and self.mode == 'mapping':
                self.save(False, 'cancelled_by_client')
            self.transition({'pause': 'PAUSED', 'cancel': 'CANCELLED', 'fault': 'ERROR'}[job], job)
        elif job == 'resume':
            if self.state not in ('PAUSED', 'INCOMPLETE', 'CANCELLED', 'MAPPING'):
                raise RuntimeError('resume requires paused/incomplete/cancelled state')
            self.retries = 0
            if self.mode == 'mapping':
                self.explore()
            else:
                self.await_localization()
        elif job == 'save_checkpoint':
            if self.mode != 'mapping':
                raise RuntimeError('checkpoint requires mapping mode')
            self.save(False, 'checkpoint_requested')
            self.transition('PAUSED', 'checkpoint_saved; resume through API')
        elif job == 'load_latest':
            record = self.store.current()
            if record is None:
                raise RuntimeError('no complete map')
            if self.seed is None:
                self.hold()
                self.transition('WAITING_INITIAL_POSE', 'initial_pose_required')
                return
            self.record = record
            pose, self.seed = self.seed, None
            self.localize(record, pose)
        elif job == 'update_map':
            if self.state != 'NAV_READY' or not self.record:
                raise RuntimeError('update requires validated localized map')
            self.hold()
            pose = self.pose_now()
            self.transition('MAPPING', 'updating_version_from_posegraph')
            self.start_slam('mapping', self.record, pose)
            self.retries = 0
            self.explore()
        elif job == 'relocalize':
            self.transition('LOCALIZING', 'localization_health_lost')
            self.hold()
            self.await_localization()

    def work(self):
        while not self.shutdown_event.is_set():
            try:
                job = self.jobs.get(timeout=.2)
            except queue.Empty:
                continue
            self.busy = True
            if job in ('pause', 'cancel', 'fault'):
                self.interrupt.clear()
            try:
                self.execute(job)
            except Interrupted:
                if self.shutdown_event.is_set() or not rclpy.ok():
                    break
                self.transition('STOPPING', 'interrupted; queued pause/cancel follows')
            except Exception as exc:
                if self.shutdown_event.is_set() or not rclpy.ok():
                    break
                self.accept_explore_status = False
                self.stop_child('explore')
                try:
                    if self.owns_session:
                        self.nav(False)
                except Exception as stop_error:
                    self.get_logger().error('navigation pause failed: ' + str(stop_error))
                self.transition('ERROR', str(exc))
            finally:
                self.busy = False

    def close(self):
        self.shutdown_event.set()
        self.interrupt.set()
        self.worker.join(timeout=60)
        self.stop_child('explore')
        self.stop_child('slam')
        for log in self.logs:
            log.close()
        fcntl.flock(self.lock_file, fcntl.LOCK_UN)
        self.lock_file.close()


def main():
    rclpy.init()
    node = Manager()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.close()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
