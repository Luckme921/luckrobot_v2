#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
工业底盘单电机编码器精准标定脚本
原理：直接脱机控制单个电机，监听编码器累加脉冲。当累计脉冲达到指定圈数时瞬间制动。
用于验证底层的单圈脉冲数是否严格等于 32768。
"""

import serial
import time
import sys
import math

# ================= 硬件配置 =================
SERIAL_PORT = "/dev/serial/by-id/usb-FTDI_FT232R_USB_UART_ABADW8KY-if00-port0"
BAUDRATE = 115200

RIGHT_WHEEL_ID = 0x03
LEFT_WHEEL_ID = 0x04
PULSES_PER_REV = 32768.0  # 我们代码里假设的理论单圈脉冲数

def crc8_maxim(data):
    """CRC-8/MAXIM 校验"""
    crc = 0x00
    for byte in data:
        crc ^= byte
        for _ in range(8):
            if crc & 0x01:
                crc = (crc >> 1) ^ 0x8C
            else:
                crc >>= 1
    return crc

def build_speed_frame(motor_id, rpm, brake=False):
    """构建控制帧"""
    frame = bytearray(9)
    frame[0] = motor_id
    frame[1] = 0x64  # Func code
    
    # 速度转 16 bit 补码
    if rpm < 0:
        rpm_val = 0x10000 + rpm
    else:
        rpm_val = rpm
        
    frame[2] = (rpm_val >> 8) & 0xFF
    frame[3] = rpm_val & 0xFF
    
    frame[6] = 0x32 # 硬件微插值
    frame[7] = 0xFF if brake else 0x00 # 刹车控制
    
    crc = crc8_maxim(frame)
    frame.append(crc)
    return frame

def test_motor(ser, motor_id, test_rpm, target_revolutions):
    """执行电机测试"""
    target_pulses = target_revolutions * PULSES_PER_REV
    
    print(f"\n🚀 开始测试！目标圈数: {target_revolutions} 圈 (理论脉冲: {target_pulses})")
    print("⏳ 等待读取初始位置...")

    # 清空缓冲区
    ser.reset_input_buffer()
    
    first_read = True
    last_raw_pos = 0
    accumulated_pulses = 0.0
    
    start_time = time.time()
    
    try:
        while True:
            # 1. 持续下发转速指令
            cmd_frame = build_speed_frame(motor_id, test_rpm)
            ser.write(cmd_frame)
            ser.flush()
            
            # 2. 等待并读取 10 字节返回帧
            resp = ser.read(10)
            if len(resp) == 10 and resp[0] == motor_id and resp[9] == crc8_maxim(resp[:-1]):
                # 解析当前位置 (0-32767)
                raw_pos = (resp[6] << 8) | resp[7]
                
                if first_read:
                    last_raw_pos = raw_pos
                    first_read = False
                    continue
                
                # 解卷逻辑 (和 C++ 节点严格保持一致)
                delta = raw_pos - last_raw_pos
                if delta > 16384:
                    delta -= 32768
                elif delta < -16384:
                    delta += 32768
                    
                last_raw_pos = raw_pos
                accumulated_pulses += delta
                
                # 计算当前已转圈数
                current_revs = abs(accumulated_pulses) / PULSES_PER_REV
                
                # 打印进度
                sys.stdout.write(f"\r🎯 进度: {current_revs:6.2f} / {target_revolutions} 圈 | 当前累计脉冲: {abs(accumulated_pulses):8.0f} ")
                sys.stdout.flush()
                
                # 3. 检查是否达到目标圈数
                if abs(accumulated_pulses) >= target_pulses:
                    print("\n\n✅ 达到目标脉冲数！下发急刹指令！")
                    # 瞬间下发 0 RPM 和 抱死指令
                    stop_frame = build_speed_frame(motor_id, 0, brake=True)
                    ser.write(stop_frame)
                    ser.flush()
                    break
                    
            time.sleep(0.005) # 短暂休眠避免死循环霸占CPU
            
    except KeyboardInterrupt:
        print("\n\n🛑 用户手动强制终止！下发急刹指令！")
        stop_frame = build_speed_frame(motor_id, 0, brake=True)
        ser.write(stop_frame)
        ser.flush()

def main():
    print("==================================================")
    print("      🏭 工业底盘电机单圈编码器标定工具")
    print("==================================================")
    
    # 关闭正在占用的 ROS2 节点
    print("⚠️  请确保已经关闭了 ROS2 底盘驱动节点！\n")
    
    try:
        ser = serial.Serial(SERIAL_PORT, BAUDRATE, timeout=0.1)
    except Exception as e:
        print(f"❌ 无法打开串口 {SERIAL_PORT}: {e}")
        return

    while True:
        print("\n选择测试电机:")
        print("1. 左轮 (LEFT_WHEEL, ID: 0x04)")
        print("2. 右轮 (RIGHT_WHEEL, ID: 0x03)")
        choice = input("请输入 1 或 2 (输入 q 退出): ")
        
        if choice.lower() == 'q':
            break
            
        motor_id = LEFT_WHEEL_ID if choice == '1' else RIGHT_WHEEL_ID if choice == '2' else None
        if not motor_id:
            print("无效输入！")
            continue
            
        try:
            revs_input = float(input("👉 请输入目标圈数 (例如 10): "))
            rpm_input = int(input("👉 请输入测试转速 (RPM, 建议 30-60, 右轮可能需要输入负数): "))
        except ValueError:
            print("输入格式错误！")
            continue
            
        print("\n🚨 请将轮胎做好物理标记！")
        input("按回车键立即开始转动...")
        
        test_motor(ser, motor_id, rpm_input, revs_input)
        
        print("\n👀 请肉眼检查轮胎物理标记：")
        print("1. 如果刚好回到原点 -> 编码器 32768 完全准确！")
        print("2. 如果没到原点 / 超过了原点 -> 编码器分辨率错误！(可能带了减速器，我们需重新计算)")

if __name__ == "__main__":
    main()