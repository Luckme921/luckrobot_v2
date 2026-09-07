"""One-command bringup for mapping, localization or lifelong mapping with Nav2."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    bringup_dir = get_package_share_directory('my_robot_bringup')
    mapping_dir = get_package_share_directory('my_robot_mapping')
    navigation_dir = get_package_share_directory('my_robot_navigation')

    mode = LaunchConfiguration('mode')
    posegraph = LaunchConfiguration('posegraph')
    use_rviz = LaunchConfiguration('use_rviz2')
    use_collision_monitor = LaunchConfiguration('use_collision_monitor')
    use_sim_time = LaunchConfiguration('use_sim_time')
    start_at_dock = LaunchConfiguration('start_at_dock')
    navigation_profile = LaunchConfiguration('navigation_profile')
    use_sim_time_param = ParameterValue(use_sim_time, value_type=bool)
    start_at_dock_param = ParameterValue(start_at_dock, value_type=bool)

    mode_arg = DeclareLaunchArgument(
        'mode',
        default_value='mapping',
        choices=['mapping', 'localization', 'lifelong'],
        description=(
            'mapping=从空白新建地图；localization=只定位不永久改图；'
            'lifelong=加载 posegraph 并继续更新地图'
        ),
    )
    posegraph_arg = DeclareLaunchArgument(
        'posegraph',
        default_value=os.path.expanduser('~/ros2_ws/maps/my_room_map'),
        description='slam_toolbox 序列化地图路径，不带 .posegraph/.data 后缀',
    )
    use_rviz_arg = DeclareLaunchArgument(
        'use_rviz2', default_value='true', description='是否启动 RViz2'
    )
    collision_arg = DeclareLaunchArgument(
        'use_collision_monitor',
        default_value='true',
        description='是否启用 Nav2 Collision Monitor 近场安全制动',
    )
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time', default_value='false', description='实体机器人必须为 false'
    )
    start_at_dock_arg = DeclareLaunchArgument(
        'start_at_dock',
        default_value='true',
        description='加载 posegraph 时机器人是否位于首节点/充电桩',
    )
    navigation_profile_arg = DeclareLaunchArgument(
        'navigation_profile',
        default_value='stable',
        choices=['stable', 'luck_controller', 'luck_full'],
        description=(
            '导航算法档位：stable 为实车基线；luck_controller 只试验控制器；'
            'luck_full 再加入自定义规划器'
        ),
    )

    base_cmd_vel_topic = PythonExpression([
        "'/cmd_vel_safe' if '", use_collision_monitor,
        "'.lower() == 'true' else '/cmd_vel'"
    ])

    hardware = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(bringup_dir, 'launch', 'test_bringup.launch.py')
        ),
        launch_arguments={
            'use_rviz2': use_rviz,
            'cmd_vel_topic': base_cmd_vel_topic,
        }.items(),
    )

    mapping_slam = Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        condition=IfCondition(PythonExpression(["'", mode, "' == 'mapping'"])),
        parameters=[
            os.path.join(mapping_dir, 'config', 'mapper_params_online_async.yaml'),
            {'use_sim_time': use_sim_time_param},
        ],
    )

    localization_slam = Node(
        package='slam_toolbox',
        executable='localization_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        condition=IfCondition(PythonExpression(["'", mode, "' == 'localization'"])),
        parameters=[
            os.path.join(mapping_dir, 'config', 'mapper_params_localization.yaml'),
            {
                'use_sim_time': use_sim_time_param,
                'mode': 'localization',
                'map_file_name': posegraph,
                'map_start_at_dock': start_at_dock_param,
            },
        ],
    )

    lifelong_slam = Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        condition=IfCondition(PythonExpression(["'", mode, "' == 'lifelong'"])),
        parameters=[
            os.path.join(mapping_dir, 'config', 'mapper_params_online_async.yaml'),
            {
                'use_sim_time': use_sim_time_param,
                'mode': 'mapping',
                'map_file_name': posegraph,
                'map_start_at_dock': start_at_dock_param,
            },
        ],
    )

    navigation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(navigation_dir, 'launch', 'navigation.launch.py')
        ),
        launch_arguments={
            # slam_toolbox 在三种模式下都负责 map、map->odom；绝不启动 AMCL。
            'use_slam': 'true',
            'use_sim_time': use_sim_time,
            # Orbbec 启动文件也声明了同名 log_level，且默认值为 none。
            # 显式覆盖，防止 none 透传给 Nav2 并导致所有核心节点启动失败。
            'log_level': 'info',
            'navigation_profile': navigation_profile,
        }.items(),
    )

    collision_monitor = Node(
        package='nav2_collision_monitor',
        executable='collision_monitor',
        name='collision_monitor',
        output='screen',
        condition=IfCondition(use_collision_monitor),
        parameters=[
            os.path.join(bringup_dir, 'config', 'collision_monitor.yaml'),
            {'use_sim_time': use_sim_time_param},
        ],
    )
    collision_lifecycle = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_collision_monitor',
        output='screen',
        condition=IfCondition(use_collision_monitor),
        parameters=[{
            'use_sim_time': use_sim_time_param,
            'autostart': True,
            'node_names': ['collision_monitor'],
        }],
    )

    # 给 TF、雷达与 EKF 留出初始化窗口；节点本身仍会等待生命周期依赖就绪。
    delayed_navigation = TimerAction(period=5.0, actions=[navigation])

    return LaunchDescription([
        mode_arg,
        posegraph_arg,
        use_rviz_arg,
        collision_arg,
        use_sim_time_arg,
        start_at_dock_arg,
        navigation_profile_arg,
        hardware,
        mapping_slam,
        localization_slam,
        lifelong_slam,
        collision_monitor,
        collision_lifecycle,
        delayed_navigation,
    ])
