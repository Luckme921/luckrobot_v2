#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
import serial
import struct
import math
import sys

# ==========================================
# IMU 串口配置
# ==========================================
PORT = '/dev/serial/by-id/usb-Silicon_Labs_CP2102_USB_to_UART_Bridge_Controller_0001-if00-port0'
BAUD = 921600

def euler_to_quaternion(roll, pitch, yaw):
    """
    将 欧拉角(弧度) 转换为 四元数
    ROS 2 标准使用的是 XYZ 顺序的四元数
    """
    qx = math.sin(roll/2) * math.cos(pitch/2) * math.cos(yaw/2) - math.cos(roll/2) * math.sin(pitch/2) * math.sin(yaw/2)
    qy = math.cos(roll/2) * math.sin(pitch/2) * math.cos(yaw/2) + math.sin(roll/2) * math.cos(pitch/2) * math.sin(yaw/2)
    qz = math.cos(roll/2) * math.cos(pitch/2) * math.sin(yaw/2) - math.sin(roll/2) * math.sin(pitch/2) * math.cos(yaw/2)
    qw = math.cos(roll/2) * math.cos(pitch/2) * math.cos(yaw/2) + math.sin(roll/2) * math.sin(pitch/2) * math.sin(yaw/2)
    return qx, qy, qz, qw

class IMUDriverNode(Node):
    def __init__(self):
        super().__init__('imu_driver_node')
        
        # 发布标准的 Imu 消息
        self.imu_pub = self.create_publisher(Imu, '/imu/data', 10)
        
        # 缓存 0x40 帧的原始数据，等 0x41 帧到来时一并打包发布
        self.ax, self.ay, self.az = 0.0, 0.0, 0.0
        self.gx, self.gy, self.gz = 0.0, 0.0, 0.0
        
        try:
            self.ser = serial.Serial(PORT, BAUD, timeout=0.5)
            self.get_logger().info(f"✅ 成功连接 IMU 串口: {PORT}")
        except Exception as e:
            self.get_logger().error(f"❌ 串口打开失败: {e}")
            sys.exit(1)
            
        # 开启一个后台定时器，高频轮询读取串口 (模拟独立线程)
        self.timer = self.create_timer(0.002, self.read_serial_data)

    def read_serial_data(self):
        # 一次读取尽量多的数据，防止缓冲区堵塞
        while self.ser.in_waiting > 0:
            if self.ser.read(1) != b'\xfc':
                continue
                
            head_type = self.ser.read(1)
            check_len = self.ser.read(1)
            self.ser.read(4) # 跳过 SN 和 CRC 等校验位
            
            if not head_type or not check_len:
                continue
                
            msg_type = head_type[0]
            payload_len = check_len[0]
            
            payload = self.ser.read(payload_len)
            if len(payload) != payload_len:
                continue

            # --- 解析 0x40: 提取加速度和陀螺仪 ---
            if msg_type == 0x40 and payload_len == 56:
                data = struct.unpack('<12f2i', payload)
                self.gx, self.gy, self.gz = data[0:3]
                self.ax, self.ay, self.az = data[3:6]
                
            # --- 解析 0x41: 提取姿态，并触发 ROS 2 发布 ---
            elif msg_type == 0x41 and payload_len == 48:
                data = struct.unpack('<10f2i', payload)
                roll, pitch, yaw = data[3:6]
                
                # 转换四元数
                qx, qy, qz, qw = euler_to_quaternion(roll, pitch, yaw)
                
                # 组装并发布 ROS 2 消息
                self.publish_imu_msg(qx, qy, qz, qw)

    def publish_imu_msg(self, qx, qy, qz, qw):
        msg = Imu()
        
        # 1. 赋予时间戳与坐标系名称 (原汁原味的数据就在 imu_link 下)
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'imu_link'
        
        # 2. 填入四元数姿态
        msg.orientation.x = qx
        msg.orientation.y = qy
        msg.orientation.z = qz
        msg.orientation.w = qw
        
        # 3. 填入角速度 (rad/s)
        msg.angular_velocity.x = self.gx
        msg.angular_velocity.y = self.gy
        msg.angular_velocity.z = self.gz
        
        if abs(self.gz) < 0.015:
            msg.angular_velocity.z = 0.0
        else:
            msg.angular_velocity.z = self.gz

        # 4. 填入线加速度 (m/s^2)
        msg.linear_acceleration.x = self.ax
        msg.linear_acceleration.y = self.ay
        msg.linear_acceleration.z = self.az
        
        # 5. 填入协方差矩阵 (对角线赋予一个小的值，代表置信度，防止 EKF 滤波报错)
        covariance = [0.01, 0.0, 0.0, 
                      0.0, 0.01, 0.0, 
                      0.0, 0.0, 0.01]
        msg.orientation_covariance = covariance
        msg.angular_velocity_covariance = covariance
        msg.linear_acceleration_covariance = covariance
        
        self.imu_pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = IMUDriverNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("🛑 IMU 节点安全退出")
    finally:
        node.ser.close()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()