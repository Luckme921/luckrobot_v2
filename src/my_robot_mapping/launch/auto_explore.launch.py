import os
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # 启动 ROS 2 官方前沿探索节点 (Frontier Exploration)
        Node(
            package='explore_lite',
            executable='explore',
            name='explore_node',
            output='screen',
            parameters=[{
                'robot_base_frame': 'base_footprint',
                
                # 🚨 核心修复 1：绝对不能用 global_costmap！
                # 必须用 SLAM 原生的纯净地图，这样算法才能清晰看到 0(空闲) 和 -1(未知) 的锐利边界
                'costmap_topic': '/map',
                'costmap_updates_topic': '/map_updates',
                
                'visualize': True,           # 在 RViz 中显示绿色的探索目标点
                'planner_frequency': 0.5,    # 每 2 秒重新规划一次探索路径
                'progress_timeout': 30.0,    # 如果 30 秒都没实质性移动，就放弃当前目标换一个
                'potential_scale': 3.0,      # 倾向于去未知区域的权重
                'orientation_scale': 0.0,    # 不强制要求特定朝向
                'gain_scale': 1.0,           # 区域大小权重
                'transform_tolerance': 0.3,  # TF 延迟容忍度
                
                # 🚨 核心修复 2：降低边界长度阈值 (从 0.5 降到 0.25)
                # 家用环境下门框、桌腿比较多，0.25米(25厘米)的未知缝隙也能被它当成目标去探索
                'min_frontier_size': 0.25,   
            }]
        )
    ])