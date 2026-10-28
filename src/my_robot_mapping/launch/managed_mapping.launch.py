"""Exclusive managed SLAM/exploration lifecycle; do not start another system launch."""
import os
import subprocess
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, RegisterEventHandler, EmitEvent, OpaqueFunction
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory


def preflight(context):
    result = subprocess.run(['ros2', 'node', 'list', '--no-daemon', '--spin-time', '2'],
                            capture_output=True, text=True, timeout=12)
    if result.returncode:
        raise RuntimeError('Cannot verify exclusive robot ownership: ' + result.stderr)
    names = {line.strip().split('/')[-1] for line in result.stdout.splitlines()}
    conflicts = names & {'slam_toolbox', 'explore_node', 'map_session_manager',
                         'controller_server', 'chassis_node', 'chassis_driver_node', 'ekf_filter_node'}
    if conflicts:
        raise RuntimeError('Stop existing robot launch before managed startup: ' + ', '.join(sorted(conflicts)))
    return []


def generate_launch_description():
    manager = Node(package='my_robot_mapping', executable='map_session_manager', output='screen',
             parameters=[{'map_root': LaunchConfiguration('map_root'),
                          'boot_mode': LaunchConfiguration('boot_mode'),
                          'boot_pose_json': ParameterValue(LaunchConfiguration('boot_pose_json'), value_type=str),
                          'auto_explore': ParameterValue(LaunchConfiguration('auto_explore'), value_type=bool)}])
    return LaunchDescription([
        DeclareLaunchArgument('map_root', default_value=os.path.expanduser('~/ros2_ws/map_store')),
        DeclareLaunchArgument('boot_mode', default_value='auto', choices=['idle', 'auto', 'mapping', 'localization']),
        DeclareLaunchArgument('boot_pose_json', default_value='',
                              description='Trusted current map [x,y,yaw], e.g. known dock; never assume after carrying robot'),
        DeclareLaunchArgument('auto_explore', default_value='true'),
        OpaqueFunction(function=preflight),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory('my_robot_bringup'), 'launch', 'luckrobot_system.launch.py')),
            launch_arguments={'start_slam': 'false', 'mode': 'mapping', 'navigation_autostart': 'false',
                              'use_rviz2': 'false', 'use_collision_monitor': 'true',
                              'navigation_profile': 'luck_full', 'depth_obstacle_mode': 'height',
                              'motion_speed_limit': '0.40', 'motion_angular_limit': '0.80',
                              'motor_accel_time': '10'}.items()),
        manager,
        RegisterEventHandler(OnProcessExit(target_action=manager,
            on_exit=[EmitEvent(event=Shutdown(reason='map session manager exited'))]))
    ])
