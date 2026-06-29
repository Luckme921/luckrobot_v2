#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import sys
import select
import termios
import tty
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

# 终端提示信息
msg = """
=============================================
🤖 极简底盘键盘遥控器 (配合底层速度平滑器)
=============================================
按键映射 (单按改变目标速度，无需长按):
        w (加速前进)
a (左转)  s/空格 (急停)  d (右转)
        x (减速/后退)

退出请按: Ctrl + C
=============================================
"""

# 步进参数（每次按键改变的幅度）
LIN_STEP = 0.05  # 线速度每次改变 0.05 m/s
ANG_STEP = 0.1   # 角速度每次改变 0.1 rad/s

class KeyboardTeleop(Node):
    def __init__(self):
        super().__init__('simple_keyboard_teleop')
        self.publisher_ = self.create_publisher(Twist, '/cmd_vel', 10)
        
        # 记录当前的目标速度
        self.target_linear = 0.0
        self.target_angular = 0.0
        
        # 为了防止看门狗超时，使用定时器以 10Hz 持续发送当前状态
        self.timer = self.create_timer(0.1, self.publish_velocity)
        
        self.get_logger().info("键盘遥控节点已启动，正在向 /cmd_vel 发布数据...")
        print(msg)
        self.print_status()

    def publish_velocity(self):
        twist = Twist()
        twist.linear.x = self.target_linear
        twist.angular.z = self.target_angular
        self.publisher_.publish(twist)

    def print_status(self):
        # 使用回车符 \r 原地刷新状态，不刷屏
        sys.stdout.write(f"\r✅ 当前目标速度: [ 线速度 X: {self.target_linear:.2f} m/s | 角速度 Z: {self.target_angular:.2f} rad/s ]   ")
        sys.stdout.flush()

    def update_speed(self, key):
        if key == 'w':
            self.target_linear = round(self.target_linear + LIN_STEP, 2)
        elif key == 'x':
            self.target_linear = round(self.target_linear - LIN_STEP, 2)
        elif key == 'a':
            self.target_angular = round(self.target_angular + ANG_STEP, 2)
        elif key == 'd':
            self.target_angular = round(self.target_angular - ANG_STEP, 2)
        elif key == 's' or key == ' ':
            # 软制动归零
            self.target_linear = 0.0
            self.target_angular = 0.0
        else:
            return # 按了其他键，忽略

        self.print_status()

# 读取终端键盘输入的底层魔法 (Linux 环境)
def get_key(settings):
    tty.setraw(sys.stdin.fileno())
    # 非阻塞读取，等待 0.1 秒
    rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
    if rlist:
        key = sys.stdin.read(1)
    else:
        key = ''
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key

def main(args=None):
    # 保存终端原本的设置，退出时恢复
    settings = termios.tcgetattr(sys.stdin)
    
    rclpy.init(args=args)
    node = KeyboardTeleop()

    try:
        while rclpy.ok():
            # 获取键盘按键
            key = get_key(settings)
            
            # 检测 Ctrl+C (ASCII 码为 \x03)
            if key == '\x03':
                break
            
            # 如果有按键按下，更新速度
            if key:
                node.update_speed(key.lower())
                
            # 处理 ROS 2 的后台事件（如定时器发送）
            rclpy.spin_once(node, timeout_sec=0)
            
    except Exception as e:
        print(f"\n发生错误: {e}")
    finally:
        # 退出前发送速度为 0 的停止指令
        stop_twist = Twist()
        node.publisher_.publish(stop_twist)
        print("\n\n🛑 已发送最终停止指令。安全退出程序。")
        
        node.destroy_node()
        rclpy.shutdown()
        # 恢复终端回显设置
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)

if __name__ == '__main__':
    main()