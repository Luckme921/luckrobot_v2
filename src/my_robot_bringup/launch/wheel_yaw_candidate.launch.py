"""Opt-in wheel-yaw EKF comparison. Never publishes TF or motor commands."""
import os
import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    path=os.path.join(get_package_share_directory('my_robot_bringup'),'config','ekf.yaml')
    with open(path,encoding='utf-8') as stream:
        params=yaml.safe_load(stream)['ekf_filter_node']['ros__parameters']
    params.update(publish_tf=False,twist0='/wheel/yaw_validated',
                  twist0_config=[False]*11+[True]+[False]*3,
                  twist0_rejection_threshold=2.0,twist0_queue_size=5)
    return LaunchDescription([
        Node(package='my_robot_bringup',executable='wheel_yaw_guard',output='screen'),
        Node(package='robot_localization',executable='ekf_node',
             name='ekf_wheel_yaw_candidate',parameters=[params],output='screen',
             remappings=[('odometry/filtered','/odometry/wheel_yaw_candidate'),
                         ('accel/filtered','/accel/wheel_yaw_candidate')])])
