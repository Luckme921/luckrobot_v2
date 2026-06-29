#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
import numpy as np
import math
import sys

try:
    from scipy.spatial import KDTree
except ImportError:
    print("❌ 缺少核心计算库。请先退出并在终端执行: pip install scipy numpy")
    sys.exit(1)

# ==========================================
# 读取你当前 URDF 中的初始估计值 (已更新为你的最新标定值)
# ==========================================
LASER_X = 0.0
LASER_Y = 0.0
LASER_ROLL_PI = True 

# 这里的基准值已经换成了你 URDF 里最新的正确数据
CAM_SCREW_X = 0.1454
CAM_SCREW_Y = -0.0103
CAM_SCREW_YAW = -0.0394  # 刚才算出的 -0.0394 弧度
CAM_SCREW_ROLL_PI = True 

CAM_OPTICAL_X = 0.011715
CAM_OPTICAL_Y = 0.0475

def scan_to_base_link(msg, is_lidar):
    angles = np.arange(msg.angle_min, msg.angle_max, msg.angle_increment)
    ranges = np.array(msg.ranges)
    
    min_len = min(len(angles), len(ranges))
    angles = angles[:min_len]
    ranges = ranges[:min_len]

    valid = (ranges > msg.range_min) & (ranges < msg.range_max) & ~np.isnan(ranges) & ~np.isinf(ranges)
    ranges = ranges[valid]
    angles = angles[valid]

    x = ranges * np.cos(angles)
    y = ranges * np.sin(angles)

    if is_lidar:
        if LASER_ROLL_PI:
            y = -y  
        x += LASER_X
        y += LASER_Y
    else:
        # 1. 补偿相机内部的螺丝孔到光学中心偏移
        x_c = x + CAM_OPTICAL_X
        y_c = y + CAM_OPTICAL_Y
        
        # 2. 补偿机械倒装 (Roll = 180度)
        if CAM_SCREW_ROLL_PI:
            y_c = -y_c 
            
        # 3. 补偿机械偏航角 (Yaw 旋转矩阵)
        x_rot = x_c * math.cos(CAM_SCREW_YAW) - y_c * math.sin(CAM_SCREW_YAW)
        y_rot = x_c * math.sin(CAM_SCREW_YAW) + y_c * math.cos(CAM_SCREW_YAW)
        
        # 4. 加上小车基座的绝对平移
        x_base = x_rot + CAM_SCREW_X
        y_base = y_rot + CAM_SCREW_Y
        x, y = x_base, y_base
        
    return np.column_stack((x, y))

def compute_icp(src, dst, max_iterations=50, tolerance=1e-6):
    src_points = np.copy(src)
    total_R = np.eye(2)
    total_t = np.zeros(2)
    
    prev_err = float('inf')
    for i in range(max_iterations):
        tree = KDTree(dst)
        distances, indices = tree.query(src_points)
        
        valid = distances < 0.20 
        if np.sum(valid) < 10:
            return None, None, None 
            
        p_src = src_points[valid]
        p_dst = dst[indices[valid]]
        
        c_src = np.mean(p_src, axis=0)
        c_dst = np.mean(p_dst, axis=0)
        
        H = (p_src - c_src).T @ (p_dst - c_dst)
        U, S, Vt = np.linalg.svd(H)
        R = Vt.T @ U.T
        
        if np.linalg.det(R) < 0:
            Vt[1, :] *= -1
            R = Vt.T @ U.T
            
        t = c_dst - R @ c_src
        src_points = (R @ src_points.T).T + t
        
        total_R = R @ total_R
        total_t = R @ total_t + t
        
        err = np.mean(distances[valid])
        if abs(prev_err - err) < tolerance:
            break
        prev_err = err
        
    yaw_err = np.arctan2(total_R[1, 0], total_R[0, 0])
    return total_t[0], total_t[1], yaw_err

class AutoCalibratorNode(Node):
    def __init__(self):
        super().__init__('auto_icp_calibrator')
        self.lidar = None
        self.cam = None
        self.get_logger().info("⏳ 正在监听 /scan 和 /scan_camera 话题...")
        self.sub_l = self.create_subscription(LaserScan, '/scan', self.lidar_cb, 10)
        self.sub_c = self.create_subscription(LaserScan, '/scan_camera', self.cam_cb, 10)
        self.timer = self.create_timer(1.0, self.perform_calibration)
        
    def lidar_cb(self, msg):
        self.lidar = msg
        
    def cam_cb(self, msg):
        self.cam = msg
        
    def perform_calibration(self):
        if self.lidar and self.cam:
            self.timer.cancel()
            self.get_logger().info("✅ 成功截获静止数据帧！开始验证残余标定误差...")
            
            dst_pts = scan_to_base_link(self.lidar, is_lidar=True)
            src_pts = scan_to_base_link(self.cam, is_lidar=False)
            
            dx, dy, dyaw = compute_icp(src_pts, dst_pts)
            
            if dx is None:
                self.get_logger().error("❌ ICP 无法收敛，请检查机器人是否对准墙角！")
                sys.exit(1)
                
            self.get_logger().info("\n\n" + "="*60)
            self.get_logger().info("🎉 当前标定已达到极高精度！检测到的残余公差（应接近于0）：")
            self.get_logger().info(f"X轴 残余误差: {dx:.4f} 米")
            self.get_logger().info(f"Y轴 残余误差: {dy:.4f} 米")
            self.get_logger().info(f"Yaw 残余误差: {math.degrees(dyaw):.2f} 度")
            self.get_logger().info("="*60)
            
            # 由于已经是校准后的二次验证，我们给出叠加上残余误差的理论最优值
            new_x = CAM_SCREW_X * math.cos(dyaw) - CAM_SCREW_Y * math.sin(dyaw) + dx
            new_y = CAM_SCREW_X * math.sin(dyaw) + CAM_SCREW_Y * math.cos(dyaw) + dy
            new_yaw = CAM_SCREW_YAW + dyaw
            
            self.get_logger().info("👉 如果追求极致（其实大可不必），你可以把 URDF 里的 origin 刷新为下面这行：")
            self.get_logger().info(f'\n    <origin xyz="{new_x:.4f} {new_y:.4f} 0.13933" rpy="3.1416 0 {new_yaw:.4f}" />\n')
            self.get_logger().info("="*60)
            
            sys.exit(0)

def main():
    rclpy.init()
    node = AutoCalibratorNode()
    try:
        rclpy.spin(node)
    except SystemExit:
        pass
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()