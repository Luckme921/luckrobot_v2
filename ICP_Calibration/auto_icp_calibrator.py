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

# 与 src/my_robot_description/urdf/my_robot.urdf 保持一致。
CAM_SCREW_X = 0.1434
CAM_SCREW_Y = -0.0002
CAM_SCREW_YAW = -0.0467
CAM_SCREW_ROLL_PI = True 

CAM_OPTICAL_X = 0.011715
CAM_OPTICAL_Y = 0.0475

# 单帧 ICP 很容易受墙面高度差、玻璃/反光和稀疏点影响。
# 一次运行自动采集多组独立帧，使用中位数给出诊断结论。
SAMPLE_COUNT = 10
MAX_ATTEMPTS = 30
SAMPLE_INTERVAL = 1.0

def scan_to_base_link(msg, is_lidar):
    ranges = np.array(msg.ranges)
    angles = msg.angle_min + np.arange(len(ranges)) * msg.angle_increment

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
    if len(src) < 10 or len(dst) < 10:
        return None, None, None

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
        self.last_pair_stamp = None
        self.results = []
        self.attempts = 0
        self.get_logger().info("⏳ 正在监听 /scan_filtered 和 /scan_camera 话题...")
        self.get_logger().info(
            f"将自动采集 {SAMPLE_COUNT} 组静止数据；请保持机器人和周围物体不动，并关闭 RViz。"
        )
        self.sub_l = self.create_subscription(LaserScan, '/scan_filtered', self.lidar_cb, 10)
        self.sub_c = self.create_subscription(LaserScan, '/scan_camera', self.cam_cb, 10)
        self.timer = self.create_timer(SAMPLE_INTERVAL, self.perform_calibration)
        
    def lidar_cb(self, msg):
        self.lidar = msg
        
    def cam_cb(self, msg):
        self.cam = msg
        
    def perform_calibration(self):
        if self.lidar is None or self.cam is None:
            return

        pair_stamp = (
            self.lidar.header.stamp.sec,
            self.lidar.header.stamp.nanosec,
            self.cam.header.stamp.sec,
            self.cam.header.stamp.nanosec,
        )
        if pair_stamp == self.last_pair_stamp:
            return
        self.last_pair_stamp = pair_stamp
        self.attempts += 1

        dst_pts = scan_to_base_link(self.lidar, is_lidar=True)
        src_pts = scan_to_base_link(self.cam, is_lidar=False)
        dx, dy, dyaw = compute_icp(src_pts, dst_pts)

        if dx is None:
            self.get_logger().warning(
                f"第 {self.attempts} 次 ICP 未收敛，等待下一组数据。"
            )
            if self.attempts >= MAX_ATTEMPTS:
                self.timer.cancel()
                self.get_logger().error("有效帧不足：请让相机和雷达同时看到墙角后重试。")
                raise SystemExit(1)
            return

        self.results.append((dx, dy, math.degrees(dyaw)))
        self.get_logger().info(
            f"样本 {len(self.results)}/{SAMPLE_COUNT}: "
            f"dx={dx:+.4f} m, dy={dy:+.4f} m, dyaw={math.degrees(dyaw):+.2f} deg"
        )

        if len(self.results) < SAMPLE_COUNT:
            return

        self.timer.cancel()
        values = np.asarray(self.results)
        medians = np.median(values, axis=0)
        spreads = np.std(values, axis=0)
        dx_med, dy_med, yaw_med = medians
        translation = math.hypot(dx_med, dy_med)

        stable = max(spreads[0], spreads[1]) <= 0.015 and spreads[2] <= 0.50
        if translation <= 0.020 and abs(yaw_med) <= 0.50:
            grade = "PASS"
            conclusion = "残差很小，当前外参可保留。"
        elif translation <= 0.050 and abs(yaw_med) <= 1.00:
            grade = "WARN"
            conclusion = "存在小偏差，对当前低速 Nav2 通常可用，但需换场景复测。"
        else:
            grade = "FAIL"
            conclusion = "偏差较大，需要检查 TF、安装尺寸和扫描高度。"

        self.get_logger().info("\n" + "=" * 64)
        self.get_logger().info(f"多帧 ICP 诊断: {grade}")
        self.get_logger().info(
            f"中位残差: X={dx_med:+.4f} m, Y={dy_med:+.4f} m, "
            f"Yaw={yaw_med:+.2f} deg, 平移合量={translation:.4f} m"
        )
        self.get_logger().info(
            f"样本标准差: X={spreads[0]:.4f} m, Y={spreads[1]:.4f} m, "
            f"Yaw={spreads[2]:.2f} deg"
        )
        self.get_logger().info(f"样本稳定性: {'合格' if stable else '不合格'}")
        self.get_logger().info(conclusion)
        if not stable:
            self.get_logger().warning("场景约束不足或噪声较大，本次残差不能用于修改 URDF。")
        else:
            self.get_logger().warning(
                "本工具只做诊断，不会生成可直接替换的 URDF。"
                "只有在第二个不同墙角得到同方向、近似数值后才应进入外参调整。"
            )
        self.get_logger().info("=" * 64)
        raise SystemExit(0)

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
