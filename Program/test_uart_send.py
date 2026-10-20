#!/usr/bin/env python3
"""
STM32 UART 测试工具 - 模拟 NUC 发送障碍物数据
用于验证 STM32 的 NUC_Obstacle 模块能否正确接收和解析数据
"""

import serial
import struct
import time
import sys

def calculate_checksum(xh, xl, yh, yl):
    """计算校验和"""
    return (xh + xl + yh + yl) & 0xFF

def create_frame(x_mm, y_mm):
    """
    创建障碍物数据帧
    x_mm: 前向距离 (mm, int16)
    y_mm: 横向偏移 (mm, int16, 正=左)
    返回: 8字节数组
    """
    # int16 大端序
    x_bytes = struct.pack('>h', x_mm)  # Big-endian signed short
    y_bytes = struct.pack('>h', y_mm)

    xh, xl = x_bytes[0], x_bytes[1]
    yh, yl = y_bytes[0], y_bytes[1]

    chk = calculate_checksum(xh, xl, yh, yl)

    frame = bytes([0xAA, 0xBB, xh, xl, yh, yl, chk, 0x55])
    return frame

def main():
    # 配置串口（需要根据实际设备修改）
    PORT = 'COM3'  # 修改为实际的 STM32 串口号
    BAUDRATE = 115200

    if len(sys.argv) > 1:
        PORT = sys.argv[1]

    print(f"=== STM32 UART 测试工具 ===")
    print(f"串口: {PORT}, 波特率: {BAUDRATE}")
    print(f"协议: AA BB | XH XL | YH YL | CHK | 55")
    print(f"按 Ctrl+C 停止\n")

    try:
        ser = serial.Serial(PORT, BAUDRATE, timeout=1)
        print(f"[OK] Serial port opened\n")

        # 测试场景
        test_cases = [
            (450, 0, "正前方 450mm"),
            (400, 100, "左前方 400mm, 偏左100mm"),
            (500, -80, "右前方 500mm, 偏右80mm"),
            (420, 50, "左前方 420mm, 偏左50mm"),
        ]

        frame_count = 0

        while True:
            for x, y, desc in test_cases:
                frame = create_frame(x, y)
                ser.write(frame)
                frame_count += 1

                hex_str = ' '.join(f'{b:02X}' for b in frame)
                print(f"[{frame_count:04d}] {desc}")
                print(f"       发送: {hex_str}")
                print(f"       X={x:+5d}mm  Y={y:+4d}mm\n")

                time.sleep(0.5)  # 每 0.5 秒发送一帧

            time.sleep(1)  # 每组测试间隔 1 秒

    except serial.SerialException as e:
        print(f"[ERROR] Serial port error: {e}")
        print(f"\nPlease check:")
        print(f"  1. STM32 connected?")
        print(f"  2. Correct port? (current: {PORT})")
        print(f"  3. Port in use by another program?")
        sys.exit(1)

    except KeyboardInterrupt:
        print(f"\n\nStopped. Total frames sent: {frame_count}")
        ser.close()
        sys.exit(0)

if __name__ == '__main__':
    main()
