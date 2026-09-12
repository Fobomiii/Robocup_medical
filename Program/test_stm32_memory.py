#!/usr/bin/env python3
"""
通过 OpenOCD telnet 接口实时读取 STM32 内存
监控 NUC_Obstacle 模块的数据接收状态
"""
import socket
import time
import struct

OPENOCD_HOST = 'localhost'
OPENOCD_PORT = 4444

# 变量地址（从 nm 输出获取）
ADDR_NUC_VALID = 0x2400005C
ADDR_NUC_X     = 0x2400005E
ADDR_NUC_Y     = 0x24000060

def openocd_cmd(sock, cmd):
    """发送命令到 OpenOCD 并返回响应"""
    sock.sendall((cmd + '\n').encode())
    time.sleep(0.05)
    response = b''
    sock.settimeout(0.1)
    try:
        while True:
            chunk = sock.recv(4096)
            if not chunk:
                break
            response += chunk
    except socket.timeout:
        pass
    return response.decode('utf-8', errors='ignore')

def read_u8(sock, addr):
    """读取 uint8"""
    resp = openocd_cmd(sock, f'mdb {addr:#x}')
    # 解析 "0x2400005c: 01" 格式
    for line in resp.split('\n'):
        if f'{addr:#x}' in line.lower():
            parts = line.split(':')
            if len(parts) > 1:
                hex_val = parts[1].strip().split()[0]
                return int(hex_val, 16)
    return None

def read_i16(sock, addr):
    """读取 int16 小端序"""
    resp = openocd_cmd(sock, f'mdh {addr:#x}')
    for line in resp.split('\n'):
        if f'{addr:#x}' in line.lower():
            parts = line.split(':')
            if len(parts) > 1:
                hex_val = parts[1].strip().split()[0]
                val = int(hex_val, 16)
                # 转换为有符号
                if val >= 0x8000:
                    val -= 0x10000
                return val
    return None

def main():
    print("Connecting to OpenOCD...")

    # 启动 OpenOCD（如果未运行）
    import subprocess
    openocd_proc = subprocess.Popen(
        ['openocd', '-f', 'interface/cmsis-dap.cfg', '-f', 'target/stm32h7x.cfg'],
        stdout=subprocess.DEVNULL,
        stderr=subprocess.DEVNULL,
        cwd='C:/Users/ASUS/Desktop/Robocup医疗/Program/MDK-ARM'
    )
    time.sleep(2)

    try:
        sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        sock.connect((OPENOCD_HOST, OPENOCD_PORT))
        print(f"Connected to OpenOCD telnet {OPENOCD_HOST}:{OPENOCD_PORT}\n")

        # 复位并运行目标
        openocd_cmd(sock, 'reset init')
        openocd_cmd(sock, 'resume')
        time.sleep(0.5)

        print("Monitoring NUC_Obstacle data reception (Ctrl+C to exit)")
        print("=" * 60)

        last_x, last_y, last_valid = None, None, None
        frame_count = 0

        while True:
            valid = read_u8(sock, ADDR_NUC_VALID)
            x = read_i16(sock, ADDR_NUC_X)
            y = read_i16(sock, ADDR_NUC_Y)

            # 只在数据变化时打印
            if (x != last_x or y != last_y or valid != last_valid):
                timestamp = time.strftime('%H:%M:%S')
                status = "[VALID]" if valid == 1 else "[TIMEOUT]"

                if valid == 1:
                    frame_count += 1
                    print(f"[{timestamp}] [{frame_count:04d}] {status}  X={x:5d}mm  Y={y:+5d}mm")
                else:
                    print(f"[{timestamp}] {status} (>200ms no data)")

                last_x, last_y, last_valid = x, y, valid

            time.sleep(0.1)

    except KeyboardInterrupt:
        print("\n\nMonitoring stopped")
        print(f"Total frames received: {frame_count}")

    except Exception as e:
        print(f"错误: {e}")

    finally:
        if 'sock' in locals():
            sock.close()
        if 'openocd_proc' in locals():
            openocd_proc.terminate()
            openocd_proc.wait()

if __name__ == "__main__":
    main()
