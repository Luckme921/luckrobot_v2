#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import serial
import struct
import math
import sys

# ==========================================
# IMU 串口配置 (根据你 ls 提供的信息)
# ==========================================
PORT = '/dev/serial/by-id/usb-Silicon_Labs_CP2102_USB_to_UART_Bridge_Controller_0001-if00-port0'
BAUD = 921600

def main():
    try:
        # 打开串口
        ser = serial.Serial(PORT, BAUD, timeout=0.5)
        print(f"✅ 成功连接 IMU 串口: {PORT}")
        print("==================================================")
        print("💡 测试提示: 请用手抬起小车各个轴，观察数值的符号变化")
        print("==================================================\n")
    except Exception as e:
        print(f"❌ 串口打开失败，请检查权限或路径: {e}")
        return

    try:
        while True:
            # 1. 寻找帧头 0xFC (单字节匹配，效率极高)
            if ser.read(1) != b'\xfc':
                continue
            
            # 2. 读取协议头部 (跳过不需要的 SN 和 CRC)
            head_type = ser.read(1)
            check_len = ser.read(1)
            ser.read(4) # 跳过 SN, CRC8, CRC16_H, CRC16_L
            
            if not head_type or not check_len:
                continue
                
            msg_type = head_type[0]
            payload_len = check_len[0]
            
            # 3. 读取真实载荷数据
            payload = ser.read(payload_len)
            if len(payload) != payload_len:
                continue

            # ==================================================
            # 解析 0x40 (IMU 原始数据: 加速度 + 陀螺仪)
            # ==================================================
            if msg_type == 0x40 and payload_len == 56:
                # '<12f2i' 代表 Little-Endian，12个浮点数，2个整数
                data = struct.unpack('<12f2i', payload)
                gx, gy, gz = data[0:3]
                ax, ay, az = data[3:6]
                
                # 原地刷新打印 (不刷屏)
                sys.stdout.write(f"\r[IMU原始] 加速度: X={ax:5.2f} Y={ay:5.2f} Z={az:5.2f} | 陀螺仪: X={gx:5.2f} Y={gy:5.2f} Z={gz:5.2f}   ")
                sys.stdout.flush()
                
            # ==================================================
            # 解析 0x41 (AHRS 姿态数据: 芯片内解算的欧拉角与四元数)
            # ==================================================
            elif msg_type == 0x41 and payload_len == 48:
                data = struct.unpack('<10f2i', payload)
                roll, pitch, yaw = data[3:6] # 数据原生单位为 弧度(rad)
                
                # 转为角度(Degree)，方便肉眼判断方向是否符合机械预期
                r_deg = math.degrees(roll)
                p_deg = math.degrees(pitch)
                y_deg = math.degrees(yaw)
                
                # 换行打印姿态角
                print(f"\n[AHRS姿态] Roll(翻滚): {r_deg:6.1f}° | Pitch(俯仰): {p_deg:6.1f}° | Yaw(航向): {y_deg:6.1f}°")
                
    except KeyboardInterrupt:
        print("\n\n🛑 测试中止，关闭串口。")
    finally:
        ser.close()

if __name__ == '__main__':
    main()