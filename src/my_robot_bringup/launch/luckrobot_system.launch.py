"""Canonical one-command LuckRobot system bringup.

The production path remains ``smart_navigation.launch.py``.  This wrapper
exposes the same navigation/mapping arguments and adds an optional VSLAM
health guard.  The second EKF is a separate, opt-in experiment and remains
disabled during guard validation so a numerical failure cannot share the
production navigation startup path.

cuVSLAM itself is intentionally not started here: it lives in the persistent
Isaac ROS Docker container on this Jetson.  Start that container separately
only when ``enable_stereo_ir:=true`` and the candidate A/B output is requested.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    bringup_dir = get_package_share_directory('my_robot_bringup')

    mode = LaunchConfiguration('mode')
    posegraph = LaunchConfiguration('posegraph')
    use_rviz = LaunchConfiguration('use_rviz2')
    use_collision_monitor = LaunchConfiguration('use_collision_monitor')
    use_sim_time = LaunchConfiguration('use_sim_time')
    start_at_dock = LaunchConfiguration('start_at_dock')
    navigation_profile = LaunchConfiguration('navigation_profile')
    enable_stereo_ir = LaunchConfiguration('enable_stereo_ir')
    vslam_camera_profile = LaunchConfiguration('vslam_camera_profile')
    depth_scan_height = LaunchConfiguration('depth_scan_height')
    depth_obstacle_mode = LaunchConfiguration('depth_obstacle_mode')
    depth_min_obstacle_height = LaunchConfiguration('depth_min_obstacle_height')
    depth_max_obstacle_height = LaunchConfiguration('depth_max_obstacle_height')
    start_vslam_guard = LaunchConfiguration('start_vslam_guard')
    start_vslam_candidate = LaunchConfiguration('start_vslam_candidate')
    start_vslam_candidate_ekf = LaunchConfiguration('start_vslam_candidate_ekf')

    guard_requested = PythonExpression([
        "'", start_vslam_guard, "' == 'true' or '",
        start_vslam_candidate, "' == 'true' or '",
        start_vslam_candidate_ekf, "' == 'true'",
    ])

    smart_navigation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(bringup_dir, 'launch', 'smart_navigation.launch.py')
        ),
        launch_arguments={
            'mode': mode,
            'posegraph': posegraph,
            'use_rviz2': use_rviz,
            'use_collision_monitor': use_collision_monitor,
            'use_sim_time': use_sim_time,
            'motion_speed_limit': LaunchConfiguration('motion_speed_limit'),
            'motor_accel_time': LaunchConfiguration('motor_accel_time'),
            'start_at_dock': start_at_dock,
            'navigation_profile': navigation_profile,
            'enable_stereo_ir': enable_stereo_ir,
            'vslam_camera_profile': vslam_camera_profile,
            'depth_scan_height': depth_scan_height,
            'depth_obstacle_mode': depth_obstacle_mode,
            'depth_min_obstacle_height': depth_min_obstacle_height,
            'depth_max_obstacle_height': depth_max_obstacle_height,
        }.items(),
    )

    guard_config = os.path.join(
        bringup_dir, 'config', 'vslam_odom_guard.yaml'
    )
    vslam_guard = Node(
        package='my_robot_bringup',
        executable='vslam_odom_guard',
        name='vslam_odom_guard',
        output='screen',
        condition=IfCondition(guard_requested),
        parameters=[
            guard_config,
            {'use_sim_time': ParameterValue(use_sim_time, value_type=bool)},
        ],
    )

    candidate_config = os.path.join(
        bringup_dir, 'config', 'ekf_vslam_candidate.yaml'
    )
    candidate_ekf = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node_vslam_candidate',
        output='screen',
        condition=IfCondition(start_vslam_candidate_ekf),
        parameters=[
            candidate_config,
            {'use_sim_time': ParameterValue(use_sim_time, value_type=bool)},
        ],
        remappings=[
            ('odometry/filtered', '/odometry/vslam_candidate'),
            ('accel/filtered', '/accel/vslam_candidate'),
        ],
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'mode', default_value='mapping',
            choices=['mapping', 'localization', 'lifelong'],
            description='SLAM 工作模式',
        ),
        DeclareLaunchArgument(
            'posegraph',
            default_value=os.path.expanduser('~/ros2_ws/maps/my_room_map'),
            description='定位/持续建图 posegraph 路径（不带后缀）',
        ),
        DeclareLaunchArgument(
            'use_rviz2', default_value='true',
            description='是否启动 RViz2',
        ),
        DeclareLaunchArgument(
            'use_collision_monitor', default_value='true',
            description='是否启用 Collision Monitor 近场安全链',
        ),
        DeclareLaunchArgument(
            'use_sim_time', default_value='false',
            description='实体机器人必须为 false',
        ),
        DeclareLaunchArgument('motor_accel_time', default_value='50'),
        DeclareLaunchArgument('motion_speed_limit', default_value='0.30',
                              description='最终速度门控上限；首次验证用 0.15 m/s'),
        DeclareLaunchArgument(
            'start_at_dock', default_value='true',
            description='加载 posegraph 时是否位于首节点/充电桩',
        ),
        DeclareLaunchArgument(
            'navigation_profile', default_value='stable',
            choices=['stable', 'luck_controller', 'luck_full'],
            description='Nav2 档位；实测自定义导航使用 luck_full',
        ),
        DeclareLaunchArgument(
            'enable_stereo_ir', default_value='false',
            description=(
                '开启相机左右 IR 640x400@30Hz，为容器内 cuVSLAM 提供输入；'
                '不改变 TF 或 Nav2'
            ),
        ),
        DeclareLaunchArgument(
            'vslam_camera_profile',
            default_value='navigation',
            choices=['navigation', 'passive', 'interleaved'],
            description=(
                'Gemini 发射器档位；VSLAM 动态验收使用 interleaved，'
                '普通导航保持 navigation'
            ),
        ),
        DeclareLaunchArgument(
            'depth_scan_height', default_value='60',
            description=(
                '旧版中央行深度伪激光垂直采样行数；仅供 legacy/shadow 模式。'
            ),
        ),
        DeclareLaunchArgument(
            'depth_obstacle_mode', default_value='legacy',
            choices=['legacy', 'shadow', 'height'],
            description=(
                'legacy=稳定基线；shadow=旁路验证完整高度扫描；'
                'height=验证通过后由完整高度扫描接管避障。'
            ),
        ),
        DeclareLaunchArgument(
            'depth_min_obstacle_height', default_value='0.12',
            description='深度补充避障最低高度；2D雷达覆盖与其扫描平面相交的低位障碍。',
        ),
        DeclareLaunchArgument(
            'depth_max_obstacle_height', default_value='0.50',
            description='相对 base_footprint 的最高碰撞包络。',
        ),
        DeclareLaunchArgument(
            'start_vslam_guard', default_value='false',
            description=(
                '仅启动 VSLAM 健康门控；不会发布 TF，也不会接管生产 EKF'
            ),
        ),
        DeclareLaunchArgument(
            'start_vslam_candidate', default_value='false',
            description=(
                '兼容旧命令的别名：当前仅启动健康门控，不启动候选 EKF'
            ),
        ),
        DeclareLaunchArgument(
            'start_vslam_candidate_ekf', default_value='false',
            description=(
                '危险的实验开关；门控长时验收前必须保持 false'
            ),
        ),
        smart_navigation,
        vslam_guard,
        candidate_ekf,
    ])
