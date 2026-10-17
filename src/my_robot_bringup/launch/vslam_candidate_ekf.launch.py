"""Run a no-TF VSLAM fusion candidate alongside the production EKF."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time')
    config = os.path.join(
        get_package_share_directory('my_robot_bringup'),
        'config',
        'ekf_vslam_candidate.yaml',
    )
    guard_config = os.path.join(
        get_package_share_directory('my_robot_bringup'),
        'config',
        'vslam_odom_guard.yaml',
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time', default_value='false',
            description='Entity robot must use false.',
        ),
        Node(
            package='my_robot_bringup',
            executable='vslam_odom_guard',
            name='vslam_odom_guard',
            output='screen',
            parameters=[
                guard_config,
                {
                    'use_sim_time': ParameterValue(use_sim_time, value_type=bool),
                },
            ],
        ),
        Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_filter_node_vslam_candidate',
            output='screen',
            parameters=[
                config,
                {
                    'use_sim_time': ParameterValue(use_sim_time, value_type=bool),
                },
            ],
            remappings=[
                ('odometry/filtered', '/odometry/vslam_candidate'),
                ('accel/filtered', '/accel/vslam_candidate'),
            ],
        ),
    ])
