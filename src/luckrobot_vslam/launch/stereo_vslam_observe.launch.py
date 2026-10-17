"""Run Gemini 336L stereo cuVSLAM without taking ownership of the robot TF tree."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode


def generate_launch_description():
    left_image = LaunchConfiguration('left_image')
    left_info = LaunchConfiguration('left_info')
    right_image = LaunchConfiguration('right_image')
    right_info = LaunchConfiguration('right_info')

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
            # Interleaved laser-off stereo is 15 Hz (about 66.7 ms). This is
            # only a warning threshold; 80 ms prevents expected log flooding.
            'image_jitter_threshold_ms': 80.0,
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
            # Production ownership stays with slam_toolbox and robot_localization.
            'publish_map_to_odom_tf': False,
            'publish_odom_to_base_tf': False,
            'enable_slam_visualization': True,
            'enable_observations_view': True,
            'enable_landmarks_view': False,
            'verbosity': 1,
        }],
        remappings=[
            ('visual_slam/image_0', left_image),
            ('visual_slam/camera_info_0', left_info),
            ('visual_slam/image_1', right_image),
            ('visual_slam/camera_info_1', right_info),
        ],
    )

    container = ComposableNodeContainer(
        name='luckrobot_vslam_observe_container',
        namespace='',
        package='rclcpp_components',
        executable='component_container_mt',
        composable_node_descriptions=[visual_slam],
        output='screen',
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'left_image', default_value='/vslam/stereo/left/image_raw'),
        DeclareLaunchArgument(
            'left_info', default_value='/vslam/stereo/left/camera_info'),
        DeclareLaunchArgument(
            'right_image', default_value='/vslam/stereo/right/image_raw'),
        DeclareLaunchArgument(
            'right_info', default_value='/vslam/stereo/right/camera_info'),
        container,
    ])
