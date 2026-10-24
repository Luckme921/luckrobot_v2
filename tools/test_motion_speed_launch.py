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
for speed in ('0.15', '0.30'):
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
for speed in ('0', '-0.1', 'nan', 'inf', '0.31'):
    context.launch_configurations['motion_speed_limit'] = speed
    try:
        module.MotionLimitedYaml(source_file=str(source), root_key='',
                                 param_rewrites={}).perform(context)
    except ValueError:
        continue
    raise AssertionError(speed)
print('PASS: shared limits 0.15/0.30, yaw preserved, invalid limits rejected')
