#!/usr/bin/env python3
"""
STM32 UART8 Monitor - 监控与NUC的通信
通过串口监听STM32发送的调试信息
"""
import serial
import serial.tools.list_ports
import time
import struct

def find_stm32_port():
    """查找STM32的串口"""
    ports = serial.tools.list_ports.comports()
    print("可用串口:")
    for i, port in enumerate(ports):
        print(f"{i+1}. {port.device} - {port.description}")

    if not ports:
        print("未找到串口设备")
        return None

    choice = input(f"\n选择串口 (1-{len(ports)}): ")
    try:
        idx = int(choice) - 1
        return ports[idx].device
    except:
        return None

def parse_nuc_frame(data):
    """解析NUC数据帧: AA BB XH XL YH YL CHK 55"""
    if len(data) != 8:
        return None

    if data[0] != 0xAA or data[1] != 0xBB or data[7] != 0x55:
        return None

    # 大端序解析
    x = struct.unpack('>h', bytes([data[2], data[3]]))[0]
    y = struct.unpack('>h', bytes([data[4], data[5]]))[0]
    chk = data[6]

    # 验证校验和
    calc_chk = (data[2] + data[3] + data[4] + data[5]) & 0xFF

    return {
        'x': x,
        'y': y,
        'checksum_ok': chk == calc_chk,
        'raw': data.hex(' ')
    }

def main():
    port = find_stm32_port()
    if not port:
        return

    print(f"\n正在连接 {port}...")

    try:
        ser = serial.Serial(
            port=port,
            baudrate=115200,  # UART8波特率
            bytesize=8,
            parity='N',
            stopbits=1,
            timeout=1
        )

        print(f"已连接到 {port} @ 115200")
        print("监控UART8数据... (Ctrl+C退出)\n")

        buffer = bytearray()
        frame_count = 0
        error_count = 0

        while True:
            if ser.in_waiting > 0:
                data = ser.read(ser.in_waiting)
                buffer.extend(data)

                # 查找帧头 AA BB
                while len(buffer) >= 8:
                    idx = buffer.find(b'\xAA\xBB')
                    if idx == -1:
                        buffer.clear()
                        break

                    if idx > 0:
                        # 丢弃帧头前的数据
                        print(f"[丢弃] {buffer[:idx].hex(' ')}")
                        buffer = buffer[idx:]

                    if len(buffer) < 8:
                        break

                    # 提取一帧
                    frame = buffer[:8]
                    buffer = buffer[8:]

                    # 解析
                    result = parse_nuc_frame(frame)
                    if result:
                        frame_count += 1
                        status = "✓" if result['checksum_ok'] else "✗"
                        print(f"[{frame_count:04d}] {status} X={result['x']:5d}mm  Y={result['y']:5d}mm  [{result['raw']}]")
                    else:
                        error_count += 1
                        print(f"[ERR{error_count:04d}] 无效帧: {frame.hex(' ')}")

            time.sleep(0.01)

    except KeyboardInterrupt:
        print("\n\n已停止监控")
        print(f"统计: 成功={frame_count}, 错误={error_count}")

    except Exception as e:
        print(f"错误: {e}")

    finally:
        if 'ser' in locals() and ser.is_open:
            ser.close()

if __name__ == "__main__":
    main()
