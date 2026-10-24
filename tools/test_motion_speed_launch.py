#!/usr/bin/env python3
"""Offline launch substitution regression; never starts ROS nodes."""
import importlib.util
import pathlib
import yaml
from launch import LaunchContext
from launch.substitutions import LaunchConfiguration

root = pathlib.Path(__file__).resolve().parents[1]
spec = importlib.util.spec_from_file_location(
    'navigation_launch', root / 'src/my_robot_navigation/launch/navigation.launch.py')
module = importlib.util.module_from_spec(spec)
spec.loader.exec_module(module)
source = root / 'src/my_robot_navigation/config/nav2_params.yaml'
for speed in ('0.15', '0.20', '0.30', '0.40'):
    context = LaunchContext()
    context.launch_configurations['motion_speed_limit'] = speed
    result = module.MotionLimitedYaml(source_file=str(source), root_key='',
                                     param_rewrites={}, convert_types=True).perform(context)
    data = yaml.safe_load(pathlib.Path(result).read_text())
    configs = data['controller_server']['ros__parameters'].values()
    values = [c['desired_linear_vel'] for c in configs
              if isinstance(c, dict) and 'desired_linear_vel' in c]
    assert values and all(v == float(speed) for v in values), values
    assert data['velocity_smoother']['ros__parameters']['max_velocity'][0] == float(speed)
    assert data['velocity_smoother']['ros__parameters']['max_velocity'][2] == 0.35
    pathlib.Path(result).unlink()
for speed in ('0', '-0.1', 'nan', 'inf', '0.41'):
    context.launch_configurations['motion_speed_limit'] = speed
    try:
        module.MotionLimitedYaml(source_file=str(source), root_key='',
                                 param_rewrites={}).perform(context)
    except ValueError:
        continue
    raise AssertionError(speed)
context.launch_configurations['motion_speed_limit'] = '0.40'
context.launch_configurations['motion_angular_limit'] = '0.70'
result = module.MotionLimitedYaml(source_file=str(source), root_key='',
                                 param_rewrites={}).perform(context)
data = yaml.safe_load(pathlib.Path(result).read_text())
luck = data['controller_server']['ros__parameters']['LuckFollowPath']
smooth = data['velocity_smoother']['ros__parameters']
assert luck['max_angular_vel'] == .70
assert abs(luck['rotate_angular_vel']-.50)<1e-10
assert abs(luck['rotation_breakaway_angular_vel']-.50)<1e-10
assert luck['rotate_angular_vel'] <= luck['rotation_breakaway_angular_vel'] <= luck['max_angular_vel']
assert luck['approach_dist'] == .75
assert luck['linear_braking_decel'] == .35
assert luck['rotate_braking_decel'] == .20
assert luck['reverse_escape_linear_vel'] == .04
assert luck['reverse_escape_timeout'] == 3.0
assert smooth['max_velocity'] == [.4,0.,.7]
assert smooth['min_velocity'][2] == -.7
assert smooth['max_accel'] == [.2,0.,.3]
pathlib.Path(result).unlink()
for yaw in ('0', '-1', 'nan', 'inf', '.71'):
    context.launch_configurations['motion_angular_limit'] = yaw
    try:
        module.MotionLimitedYaml(source_file=str(source), root_key='',
                                 param_rewrites={}).perform(context)
    except ValueError:
        continue
    raise AssertionError(yaw)
print('PASS: baseline and fast limits, longer approach, unchanged physical braking/recovery, invalid caps rejected')
