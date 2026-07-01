#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import os
import time
import subprocess
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped

class VacuumManager(Node):
    def __init__(self):
        super().__init__('vacuum_manager')
        self.get_logger().info("🤖 扫地机大管家节点已启动！")
        
        # 创建 Nav2 的目标点发布器 (用于一键返航)
        self.goal_pub = self.create_publisher(PoseStamped, '/goal_pose', 10)
        
        # 充电桩坐标 (假设以第一次开机点为原点 0,0)
        self.home_x = 0.0
        self.home_y = 0.0
        self.home_yaw = 0.0
        
        # 记录 explore_lite 的进程
        self.explore_process = None

    def start_exploration(self):
        if self.explore_process is None or self.explore_process.poll() is not None:
            self.get_logger().info("🚀 启动全自动探索建图...")
            # 在后台异步拉起 auto_explore.launch.py
            self.explore_process = subprocess.Popen(
                ["ros2", "launch", "my_robot_mapping", "auto_explore.launch.py"]
            )
            print("\n✅ 小车已经开始自主扫建，请在 RViz 中观察！")
        else:
            print("\n⚠️ 探索正在进行中，请勿重复启动。")

    def stop_and_save_map(self):
        self.get_logger().info("💾 正在结束探索并保存地图...")
        
        # 1. 杀掉探索进程，让小车停下
        if self.explore_process is not None:
            self.explore_process.terminate()
            self.explore_process.wait()
            self.explore_process = None
            
        # 2. 调用 map_saver_cli 自动保存地图
        map_dir = os.path.expanduser("~/ros2_ws/src/my_robot_navigation/maps")
        os.makedirs(map_dir, exist_ok=True)
        map_path = os.path.join(map_dir, "my_room_map")
        
        print("\n⏳ 正在固化 2D 栅格地图，请稍候...")
        ret = os.system(f"ros2 run nav2_map_server map_saver_cli -f {map_path}")
        
        if ret == 0:
            print(f"\n🎉 地图保存成功！路径: {map_path}.yaml")
        else:
            print("\n❌ 地图保存失败，请检查 SLAM 节点是否正常运行。")

    def go_home(self):
        self.get_logger().info("🏠 收到返航指令，正在自动规划回充路线...")
        
        # 杀掉探索进程防止冲突
        if self.explore_process is not None:
            self.explore_process.terminate()
            self.explore_process = None
            
        goal_msg = PoseStamped()
        goal_msg.header.frame_id = 'map'
        goal_msg.header.stamp = self.get_clock().now().to_msg()
        goal_msg.pose.position.x = self.home_x
        goal_msg.pose.position.y = self.home_y
        goal_msg.pose.orientation.w = 1.0 # 简化处理，朝向为 0 度
        
        self.goal_pub.publish(goal_msg)
        print("\n✅ 目标已下发！小车正在自动躲避障碍物，返回充电桩 (0,0)！")

def main(args=None):
    rclpy.init(args=args)
    manager = VacuumManager()

    menu = """
    ======================================
         🤖 智能扫地机器人大管家控制台
    ======================================
    [1] 🚀 开始全自动探索建图 (Auto Explore)
    [2] 💾 结束建图并自动保存 (Save Map)
    [3] 🏠 结束工作并自动返航 (Go Home)
    [0] ❌ 退出控制台
    ======================================
    """

    try:
        while rclpy.ok():
            print(menu)
            choice = input("👉 请输入您的指令 (0-3): ")
            
            if choice == '1':
                manager.start_exploration()
            elif choice == '2':
                manager.stop_and_save_map()
            elif choice == '3':
                manager.go_home()
            elif choice == '0':
                print("👋 退出管家系统。")
                if manager.explore_process:
                    manager.explore_process.terminate()
                break
            else:
                print("⚠️ 无效输入，请重新选择。")
                
            # 处理底层的 ROS 事件
            rclpy.spin_once(manager, timeout_sec=0.1)
            time.sleep(1) # 给人类一点反应时间

    except KeyboardInterrupt:
        pass
    finally:
        if manager.explore_process:
            manager.explore_process.terminate()
        manager.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()