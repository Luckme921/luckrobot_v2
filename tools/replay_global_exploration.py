#!/usr/bin/env python3
"""Isolated decision replay. No wheel topics or real navigation servers.

Uses only map/costmap/TF from an existing bag; robot trajectory is historical,
so this validates decision generation, not success of counterfactual motions.
"""
import argparse
import json
import os
from pathlib import Path
import signal
import subprocess
import sys
import time


def servers():
    import rclpy
    from rclpy.action import ActionServer, GoalResponse
    from rclpy.node import Node
    from nav2_msgs.action import ComputePathToPose, NavigateToPose
    rclpy.init()
    node = Node('shadow_action_tripwire')
    def reject(name):
        def callback(request):
            print('REJECTED_ACTION_REQUEST '+name, flush=True)
            return GoalResponse.REJECT
        return callback
    async def execute(handle):
        handle.abort()
        raise RuntimeError('Shadow replay must not execute actions')
    actions = [ActionServer(node, kind, name, execute, goal_callback=reject(name))
               for kind, name in [(ComputePathToPose, 'compute_path_to_pose'),
                                  (NavigateToPose, 'navigate_to_pose')]]
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        for action in actions:
            action.destroy()
        node.destroy_node()


def replay(bag, output, verify_rejection=False):
    output.mkdir(parents=True, exist_ok=True)
    commands = [
        ('tripwire', [sys.executable, __file__, '--servers']),
        ('explore', ['ros2', 'launch', 'my_robot_mapping', 'auto_explore.launch.py',
                     'global_regions:=true',
                     'shadow_only:='+('false' if verify_rejection else 'true'), 'use_sim_time:=true']),
        ('player', ['ros2', 'bag', 'play', str(bag), '--clock', '40', '--rate', '2',
                    '--topics', '/map', '/map_updates', '/global_costmap/costmap',
                    '/tf', '/tf_static'])]
    processes, logs = [], []
    try:
        for name, cmd in commands:
            log = open(output / (name+'.log'), 'w')
            logs.append(log)
            processes.append(subprocess.Popen(cmd, stdout=log, stderr=subprocess.STDOUT,
                                              start_new_session=True))
            time.sleep(1)
        processes[-1].wait(timeout=240)
        time.sleep(2)
    finally:
        for proc in reversed(processes):
            if proc.poll() is None:
                os.killpg(proc.pid, signal.SIGINT)
        for proc in processes:
            try:
                proc.wait(timeout=8)
            except subprocess.TimeoutExpired:
                os.killpg(proc.pid, signal.SIGKILL)
                proc.wait()
        for log in logs:
            log.close()
    decision = (output/'explore.log').read_text()
    tripwire = (output/'tripwire.log').read_text()
    report = {'bag': str(bag), 'domain': os.environ['ROS_DOMAIN_ID'],
              'global_decisions': decision.count('GLOBAL region'),
              'pending_reports': decision.count('GLOBAL pending'),
              'planner_requests': tripwire.count('REJECTED_ACTION_REQUEST compute_path_to_pose'),
              'navigation_requests': tripwire.count('REJECTED_ACTION_REQUEST navigate_to_pose'),
              'mode': 'reject_all_planner_requests' if verify_rejection else 'shadow',
              'scope': 'historical map/pose shadow decisions; not closed-loop validation'}
    (output/'summary.json').write_text(json.dumps(report, indent=2))
    print(json.dumps(report, indent=2))
    if (report['navigation_requests'] or not report['global_decisions'] or
            (not verify_rejection and report['planner_requests']) or
            (verify_rejection and not report['planner_requests'])):
        raise SystemExit('Replay failed: unexpected action or no decisions')


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('--servers', action='store_true')
    parser.add_argument('--bag', type=Path)
    parser.add_argument('--output', type=Path)
    parser.add_argument('--verify-rejection', action='store_true')
    args = parser.parse_args()
    if os.environ.get('ROS_DOMAIN_ID') != '193' or os.environ.get('ROS_LOCALHOST_ONLY') != '1':
        raise SystemExit('Requires isolated ROS_DOMAIN_ID=193 ROS_LOCALHOST_ONLY=1')
    if args.servers:
        servers()
    elif args.bag and args.output:
        replay(args.bag, args.output, args.verify_rejection)
    else:
        parser.error('--bag and --output required')
