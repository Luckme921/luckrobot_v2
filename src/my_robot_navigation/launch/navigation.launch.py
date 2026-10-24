import os
import math
import tempfile
import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch.conditions import IfCondition
from nav2_common.launch import RewrittenYaml


class MotionLimitedYaml(RewrittenYaml):
    """Keep controller prediction and downstream speed authority consistent."""

    def perform(self, context):
        speed = float(LaunchConfiguration('motion_speed_limit').perform(context))
        if not math.isfinite(speed) or not 0.0 < speed <= 0.40:
            raise ValueError('motion_speed_limit must be in (0, 0.40] m/s')
        yaw = float(LaunchConfiguration('motion_angular_limit', default='0.35').perform(context))
        if not math.isfinite(yaw) or not 0.0 < yaw <= 0.70:
            raise ValueError('motion_angular_limit must be in (0, 0.70] rad/s')
        source = super().perform(context)
        with open(source, encoding='utf-8') as stream:
            data = yaml.safe_load(stream)
        controller = data['controller_server']['ros__parameters']
        for config in controller.values():
            if isinstance(config, dict) and 'desired_linear_vel' in config:
                config['desired_linear_vel'] = speed
                if 'max_angular_vel' in config:
                    config['max_angular_vel'] = yaw
                    config['rotate_angular_vel'] = yaw * (0.25 / 0.35)
                    # configure() requires breakaway >= normal spin. Do not
                    # boost it beyond that to the full tracking yaw maximum.
                    config['rotation_breakaway_angular_vel'] = min(
                        yaw, max(0.35, config['rotate_angular_vel']))
                if speed > 0.30:
                    # More approach distance, not an invented stronger brake.
                    if 'approach_dist' in config:
                        config['approach_dist'] = max(config['approach_dist'], 0.75)
                    if 'approach_velocity_scaling_dist' in config:
                        config['approach_velocity_scaling_dist'] = max(
                            config['approach_velocity_scaling_dist'], 0.75)
        smoother = data['velocity_smoother']['ros__parameters']
        smoother['max_velocity'][0] = speed
        smoother['max_velocity'][2] = yaw
        smoother['min_velocity'][2] = -yaw
        smoother['min_velocity'][0] = max(smoother['min_velocity'][0], -speed)
        with tempfile.NamedTemporaryFile(mode='w', suffix='.yaml', delete=False) as stream:
            yaml.safe_dump(data, stream)
            return stream.name

def generate_launch_description():
    # 获取包路径
    nav_dir = get_package_share_directory('my_robot_navigation')
    nav2_bringup_dir = get_package_share_directory('nav2_bringup')
    
    # 默认的地图和参数文件路径
    default_map_path = os.path.join(nav_dir, 'maps', 'my_room_map.yaml')
    default_params_path = os.path.join(nav_dir, 'config', 'nav2_params.yaml')
    stable_nav_to_pose_bt = os.path.join(
        nav_dir, 'behavior_trees', 'navigate_to_pose_general_goal_checker.xml')
    luck_controller_nav_to_pose_bt = os.path.join(
        nav_dir, 'behavior_trees', 'navigate_to_pose_luck_controller.xml')
    luck_full_nav_to_pose_bt = os.path.join(
        nav_dir, 'behavior_trees', 'navigate_to_pose_luck_full.xml')
    manual_nav_through_poses_bt = os.path.join(
        nav_dir, 'behavior_trees',
        'navigate_through_poses_general_goal_checker.xml')

    # 声明 Launch 参数
    use_slam = LaunchConfiguration('use_slam')
    map_yaml_file = LaunchConfiguration('map')
    params_file = LaunchConfiguration('params_file')
    use_sim_time = LaunchConfiguration('use_sim_time')
    log_level = LaunchConfiguration('log_level')
    navigation_profile = LaunchConfiguration('navigation_profile')

    selected_nav_to_pose_bt = PythonExpression([
        "'", luck_controller_nav_to_pose_bt, "' if '", navigation_profile,
        "' == 'luck_controller' else ('", luck_full_nav_to_pose_bt,
        "' if '", navigation_profile, "' == 'luck_full' else '",
        stable_nav_to_pose_bt, "')",
    ])

    # Humble's stock trees leave FollowPath.goal_checker_id empty. That is
    # accepted only while controller_server has one goal checker. Rewrite the
    # two YAML placeholders to installed absolute paths so RViz/manual goals
    # always select general_goal_checker; automatic exploration supplies its
    # own BT and explicitly selects explore_goal_checker.
    configured_params = MotionLimitedYaml(
        source_file=params_file,
        root_key='',
        param_rewrites={
            'default_nav_to_pose_bt_xml': selected_nav_to_pose_bt,
            'default_nav_through_poses_bt_xml': manual_nav_through_poses_bt,
        },
        convert_types=True,
    )

    # 🌟 核心开关：默认处于 "建图模式" (true)
    declare_use_slam_cmd = DeclareLaunchArgument(
        'use_slam',
        default_value='true',
        description='是否处于建图模式。True则不加载老地图和AMCL；False则加载。')

    declare_map_yaml_cmd = DeclareLaunchArgument(
        'map',
        default_value=default_map_path,
        description='Full path to map yaml file to load')

    declare_params_file_cmd = DeclareLaunchArgument(
        'params_file',
        default_value=default_params_path,
        description='Full path to the ROS2 parameters file to use')

    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation (Gazebo) clock if true')

    declare_log_level_cmd = DeclareLaunchArgument(
        'log_level',
        default_value='info',
        description='Nav2 logging level')

    declare_navigation_profile_cmd = DeclareLaunchArgument(
        'navigation_profile',
        default_value='stable',
        choices=['stable', 'luck_controller', 'luck_full'],
        description=(
            'stable=Smac2D+RPP; luck_controller=Smac2D+LuckController; '
            'luck_full=LuckPlanner+LuckController'))

    # ==============================================================
    # 1. 始终启动：导航核心栈 (Planner, Controller, Behavior, etc.)
    # ==============================================================
    nav_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_bringup_dir, 'launch', 'navigation_launch.py')
        ),
        launch_arguments={
            'params_file': configured_params,
            'use_sim_time': use_sim_time,
            'log_level': log_level
        }.items()
    )

    # ==============================================================
    # 2. 条件启动：定位核心栈 (Map Server, AMCL)
    # 仅当 use_slam 为 false 时启动！彻底解决建图时加载老地图的冲突！
    # ==============================================================
    loc_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_bringup_dir, 'launch', 'localization_launch.py')
        ),
        condition=IfCondition(PythonExpression(["not '", use_slam, "' == 'true'"])),
        launch_arguments={
            'map': map_yaml_file,
            'params_file': configured_params,
            'use_sim_time': use_sim_time
        }.items()
    )

    return LaunchDescription([
        DeclareLaunchArgument('motion_angular_limit', default_value='0.35',
                              description='Shared tracking yaw cap; fast candidate upper bound 0.70'),
        DeclareLaunchArgument('motion_speed_limit', default_value='0.30',
                              description='Shared forward limit for controller, smoother and guard'),
        declare_use_slam_cmd,
        declare_map_yaml_cmd,
        declare_params_file_cmd,
        declare_use_sim_time_cmd,
        declare_log_level_cmd,
        declare_navigation_profile_cmd,
        nav_cmd,
        loc_cmd
    ])
