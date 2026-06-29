import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition
from launch_ros.actions import Node

def generate_launch_description():
    pkg_name = 'my_robot_description'
    urdf_file_name = 'my_robot.urdf'
    
    # 获取 URDF 文件路径
    urdf_path = os.path.join(
        get_package_share_directory(pkg_name),
        'urdf',
        urdf_file_name)
    
    # 读取 URDF 内容
    with open(urdf_path, 'r') as infp:
        robot_desc = infp.read()

    # 获取 RViz2 配置文件路径
    rviz_config_path = os.path.join(
        get_package_share_directory(pkg_name),
        'rviz_config',
        'rviz2_config.rviz')

    # 1. 声明一个变量 'use_rviz'，默认值为 'false'
    use_rviz_arg = DeclareLaunchArgument(
        'use_rviz',
        default_value='false',
        description='是否自动启动 RViz2 可视化界面'
    )

    # 2. 机器人状态发布节点 (始终启动)
    rsp_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_desc}]
    )
    
    # 3. RViz2 节点 (受条件控制)
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config_path],
        # 仅当 use_rviz 参数为 'true' 时才启动
        condition=IfCondition(LaunchConfiguration('use_rviz'))
    )

    return LaunchDescription([
        use_rviz_arg,
        rsp_node,
        rviz_node
    ])