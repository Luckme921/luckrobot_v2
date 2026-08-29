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
                'costmap_topic': '/map',
                'costmap_updates_topic': '/map_updates',
                
                'visualize': True,
                
                # 🌟 事件驱动模式：不到达目标或彻底失败，绝不主动换目标
                'planner_frequency': 0.0,    
                
                # 🌟 核心破局机制：黑名单快刀！
                # 如果小车在死胡同前卡顿挣扎超过 10 秒没有靠近目标，
                # 探索算法会强行撤回目标，并把这个不可达区域“拉黑”，去找下一个边界！
                'progress_timeout': 10.0,    
                
                # 调大距离权重，优先探索眼前的屋子
                'potential_scale': 5.0,      
                'orientation_scale': 0.0,    
                'gain_scale': 1.0,           
                'transform_tolerance': 0.3,  
                
                # 忽略小于 50 厘米的灰斑死角
                'min_frontier_size': 0.50,   
            }]
        )
    ])