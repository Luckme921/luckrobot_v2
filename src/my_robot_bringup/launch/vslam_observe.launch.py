"""Isolated Gemini 336L stereo VSLAM bringup.

This launch is deliberately observation-only: cuVSLAM never publishes TF and
does not replace the wheel/IMU EKF.  The optional camera profile disables the
IR projector and high-bandwidth depth/color streams so stereo timing can be
validated without changing the normal navigation launch.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import ComposableNodeContainer, Node
from launch_ros.descriptions import ComposableNode


def generate_launch_description():
    start_camera = LaunchConfiguration('start_camera')
    start_robot_description = LaunchConfiguration('start_robot_description')
    start_vslam = LaunchConfiguration('start_vslam')
    start_motion_reference = LaunchConfiguration('start_motion_reference')
    motion_cmd_vel_topic = LaunchConfiguration('motion_cmd_vel_topic')

    description_dir = get_package_share_directory('my_robot_description')
    bringup_dir = get_package_share_directory('my_robot_bringup')

    start_camera_arg = DeclareLaunchArgument(
        'start_camera',
        default_value='true',
        description='Start an isolated passive-IR Gemini 336L camera profile.',
    )
    start_robot_description_arg = DeclareLaunchArgument(
        'start_robot_description',
        default_value='true',
        description='Publish the base-to-camera static robot transforms.',
    )
    start_vslam_arg = DeclareLaunchArgument(
        'start_vslam',
        default_value='false',
        description=(
            'Start cuVSLAM only in an environment where Isaac ROS release-3.2 '
            'is installed. The safe default validates stereo input only.'
        ),
    )
    start_motion_reference_arg = DeclareLaunchArgument(
        'start_motion_reference',
        default_value='false',
        description=(
            'Start chassis, IMU and the existing wheel/IMU EKF for a bounded '
            'VSLAM motion test. This does not start Nav2 or obstacle avoidance.'
        ),
    )
    motion_cmd_vel_topic_arg = DeclareLaunchArgument(
        'motion_cmd_vel_topic',
        default_value='/cmd_vel_vslam_test',
        description='Isolated velocity input used only by the VSLAM motion test.',
    )

    robot_description = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(description_dir, 'launch', 'display_robot.launch.py')
        ),
        launch_arguments={'use_rviz': 'false'}.items(),
        condition=IfCondition(start_robot_description),
    )

    camera = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('orbbec_camera'),
                'launch',
                'gemini_330_series.launch.py',
            )
        ),
        launch_arguments={
            # Standalone timing/VO profile. Do not use this launch as the final
            # depth-obstacle profile; the IR projector is intentionally off.
            'enable_color': 'false',
            'enable_depth': 'false',
            'enable_point_cloud': 'false',
            'enable_left_ir': 'true',
            'left_ir_width': '640',
            'left_ir_height': '400',
            'left_ir_fps': '30',
            'left_ir_format': 'ANY',
            'left_ir_qos': 'SENSOR_DATA',
            'enable_right_ir': 'true',
            'right_ir_width': '640',
            'right_ir_height': '400',
            'right_ir_fps': '30',
            'right_ir_format': 'ANY',
            'right_ir_qos': 'SENSOR_DATA',
            'enable_laser': 'false',
            'enable_frame_sync': 'true',
            'enable_sync_host_time': 'false',
            'time_domain': 'global',
            'publish_tf': 'true',
            'show_fps_enable': 'true',
            'log_level': 'info',
        }.items(),
        condition=IfCondition(start_camera),
    )

    # Optional low-level motion reference for controlled VSLAM validation.
    # It deliberately excludes Nav2, lidar and collision_monitor so it cannot
    # interfere with the production navigation stack. Keep it disabled unless
    # the robot is in a clear test area and use only bounded test commands.
    chassis = Node(
        package='chassis_driver',
        executable='chassis_node',
        name='chassis_driver_node',
        remappings=[('cmd_vel', motion_cmd_vel_topic)],
        output='screen',
        condition=IfCondition(start_motion_reference),
    )

    imu = Node(
        package='imu_driver',
        executable='imu_driver_node',
        name='imu_driver_node',
        output='screen',
        condition=IfCondition(start_motion_reference),
    )

    wheel_imu_ekf = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[os.path.join(bringup_dir, 'config', 'ekf.yaml')],
        condition=IfCondition(start_motion_reference),
    )

    visual_slam = ComposableNode(
        name='visual_slam_node',
        package='isaac_ros_visual_slam',
        plugin='nvidia::isaac_ros::visual_slam::VisualSlamNode',
        parameters=[{
            'num_cameras': 2,
            'min_num_images': 2,
            'rectified_images': False,
            'enable_image_denoising': True,
            'enable_imu_fusion': False,
            'enable_localization_n_mapping': False,
            'enable_ground_constraint_in_odometry': True,
            'enable_ground_constraint_in_slam': False,
            'sync_matching_threshold_ms': 5.0,
            'image_jitter_threshold_ms': 40.0,
            'image_buffer_size': 100,
            'image_qos': 'SENSOR_DATA',
            'override_publishing_stamp': False,
            'base_frame': 'base_footprint',
            'odom_frame': 'odom',
            'map_frame': 'map',
            'camera_optical_frames': [
                'camera_left_ir_optical_frame',
                'camera_right_ir_optical_frame',
            ],
            # TF ownership remains with robot_localization and slam_toolbox.
            'publish_map_to_odom_tf': False,
            'publish_odom_to_base_tf': False,
            'enable_slam_visualization': True,
            'enable_observations_view': True,
            'enable_landmarks_view': False,
            'verbosity': 1,
        }],
        remappings=[
            ('visual_slam/image_0', '/camera/left_ir/image_raw'),
            ('visual_slam/camera_info_0', '/camera/left_ir/camera_info'),
            ('visual_slam/image_1', '/camera/right_ir/image_raw'),
            ('visual_slam/camera_info_1', '/camera/right_ir/camera_info'),
        ],
    )

    container = ComposableNodeContainer(
        name='luckrobot_vslam_observe_container',
        namespace='',
        package='rclcpp_components',
        executable='component_container_mt',
        composable_node_descriptions=[visual_slam],
        output='screen',
        condition=IfCondition(start_vslam),
    )

    return LaunchDescription([
        start_camera_arg,
        start_robot_description_arg,
        start_vslam_arg,
        start_motion_reference_arg,
        motion_cmd_vel_topic_arg,
        robot_description,
        camera,
        chassis,
        imu,
        wheel_imu_ekf,
        container,
    ])
