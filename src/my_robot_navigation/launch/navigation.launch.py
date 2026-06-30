import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    # 获取包路径
    nav_dir = get_package_share_directory('my_robot_navigation')
    nav2_bringup_dir = get_package_share_directory('nav2_bringup')
    
    # 默认的地图和参数文件路径
    default_map_path = os.path.join(nav_dir, 'maps', 'my_room_map.yaml')
    default_params_path = os.path.join(nav_dir, 'config', 'nav2_params.yaml')

    # 声明 Launch 参数
    map_yaml_file = LaunchConfiguration('map')
    params_file = LaunchConfiguration('params_file')
    use_sim_time = LaunchConfiguration('use_sim_time')

    declare_map_yaml_cmd = DeclareLaunchArgument(
        'map',
        default_value=default_map_path,
        description='Full path to map yaml file to load')

    declare_params_file_cmd = DeclareLaunchArgument(
        'params_file',
        default_value=default_params_path,
        description='Full path to the ROS2 parameters file to use for all launched nodes')

    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation (Gazebo) clock if true')

    # 包含 Nav2 官方的 bringup_launch.py
    # 这一句相当于召唤了自动驾驶的“千军万马”
    nav2_bringup_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_bringup_dir, 'launch', 'bringup_launch.py')
        ),
        launch_arguments={
            'map': map_yaml_file,
            'params_file': params_file,
            'use_sim_time': use_sim_time
        }.items()
    )

    return LaunchDescription([
        declare_map_yaml_cmd,
        declare_params_file_cmd,
        declare_use_sim_time_cmd,
        nav2_bringup_cmd
    ])