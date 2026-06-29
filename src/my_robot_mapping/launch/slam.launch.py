import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    
    # 获取配置文件的绝对路径
    mapping_dir = get_package_share_directory('my_robot_mapping')
    config_file = os.path.join(mapping_dir, 'config', 'mapper_params_online_async.yaml')
    
    # 创建 slam_toolbox 节点
    # 我们使用的是 online_async_node，它会在后台异步构建地图，绝不会卡死底盘和 EKF 的响应
    slam_node = Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        parameters=[
            config_file,
            {'use_sim_time': False} # 实体机器人，绝对不能使用仿真时间
        ]
    )

    return LaunchDescription([
        slam_node
    ])