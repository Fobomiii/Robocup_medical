#!/usr/bin/env python3
"""
障碍物检测节点
- 订阅 /livox/lidar (PointCloud2)
- 过滤前方 40-50cm、横向宽度符合雪糕筒特征的点云
- 通过 USB-TTL 串口发送质心 X/Y 坐标给 STM32H7
"""
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2 as pc2
import struct, threading
import serial
import serial.tools.list_ports
import numpy as np
import time


PING_FRAME = bytes((0xA5, 0x5A, 0x01, 0x00))
PONG_FRAME = bytes((0xA5, 0x5A, 0x01, 0x01))


def _parse_cloud(msg) -> np.ndarray:
    """兼容带额外字段（intensity/tag 等）的结构化 dtype"""
    gen = pc2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True)
    arr = list(gen)
    if not arr:
        return np.empty((0, 3), dtype=np.float32)
    a = np.array(arr)
    if a.dtype.names:
        return np.column_stack([a["x"], a["y"], a["z"]]).astype(np.float32)
    return a.reshape(-1, 3).astype(np.float32)


class ObstacleDetector(Node):
    def __init__(self):
        super().__init__("obstacle_detector")

        # ── 参数声明 ──────────────────────────────────────────────
        self.declare_parameter("topic",         "/livox/lidar")
        self.declare_parameter("dist_min",       0.40)   # 前向检测近端 (m)
        self.declare_parameter("dist_max",       0.50)   # 前向检测远端 (m)
        self.declare_parameter("y_limit",        0.30)   # 横向粗过滤范围 ±(m)
        self.declare_parameter("z_min",         -0.10)   # 高度下限 (m)
        self.declare_parameter("z_max",          0.50)   # 高度上限 (m)
        self.declare_parameter("min_points",     5)      # 最少有效点数
        self.declare_parameter("max_points",     80)     # 最多有效点数（超过=大物体）
        self.declare_parameter("y_span_max",     0.25)   # 横向宽度上限 (m)
        self.declare_parameter("serial_port",   "/dev/ttyUSB0")
        self.declare_parameter("baud_rate",      115200)
        self.declare_parameter("send_interval",  0.05)   # 最小发送间隔 (s)

        topic           = self.get_parameter("topic").value
        self.dmin       = self.get_parameter("dist_min").value
        self.dmax       = self.get_parameter("dist_max").value
        self.ylim       = self.get_parameter("y_limit").value
        self.zmin       = self.get_parameter("z_min").value
        self.zmax       = self.get_parameter("z_max").value
        self.nmin       = self.get_parameter("min_points").value
        self.nmax       = self.get_parameter("max_points").value
        self.y_span_max = self.get_parameter("y_span_max").value
        self._port      = self.get_parameter("serial_port").value
        self._baud      = self.get_parameter("baud_rate").value
        self.dt         = self.get_parameter("send_interval").value

        self.ser      = None
        self._ser_lock = threading.Lock()
        self._serial_stop = threading.Event()

        # 后台线程持续重试打开串口（设备可能开机晚于节点）
        threading.Thread(target=self._serial_watchdog, daemon=True).start()
        threading.Thread(target=self._serial_reader, daemon=True).start()

        # ROS2 订阅：若 /livox/lidar 还未发布，会自动等待，不需要额外处理
        self.sub = self.create_subscription(
            PointCloud2, topic, self._cb, 10)
        self._last_send = 0.0

        self.get_logger().info(
            f"障碍物检测启动 | 话题:{topic} | "
            f"范围 {self.dmin}-{self.dmax}m | "
            f"点数 {self.nmin}-{self.nmax} | "
            f"横宽<{self.y_span_max}m | "
            f"串口 {self._port}@{self._baud}")

    # ── 串口看门狗（后台线程）────────────────────────────────────────
    def _serial_watchdog(self):
        """
        持续尝试打开串口，直到成功。
        若串口中途断开，等待 5s 后重试。
        这样 obstacle_detector 比 USB-TTL 先启动时不会永久失去串口。
        """
        retry_interval = 5  # 秒
        while not self._serial_stop.is_set() and rclpy.ok():
            with self._ser_lock:
                alive = self.ser is not None and self.ser.is_open
            if not alive:
                try:
                    s = serial.Serial(self._port, self._baud,
                                      timeout=0.02, write_timeout=0.1)
                    with self._ser_lock:
                        self.ser = s
                    self.get_logger().info(f"串口 {self._port} 已打开")
                except serial.SerialException:
                    ports = [p.device for p in serial.tools.list_ports.comports()]
                    self.get_logger().warn(
                        f"串口 {self._port} 不可用，{retry_interval}s 后重试。"
                        f"当前可用: {ports}")
                    time.sleep(retry_interval)
                    continue
            time.sleep(1)   # 串口正常时每秒检查一次是否断开

    def _serial_reader(self):
        """Reply to STM32 link probes without depending on lidar data."""
        ping_index = 0

        while not self._serial_stop.is_set() and rclpy.ok():
            try:
                with self._ser_lock:
                    if self.ser is None or not self.ser.is_open:
                        data = b""
                    else:
                        data = self.ser.read(1)
            except serial.SerialException as e:
                self.get_logger().error(f"Serial read failed: {e}")
                with self._ser_lock:
                    self.ser = None
                time.sleep(0.05)
                continue

            if not data:
                time.sleep(0.005)
                continue

            byte = data[0]
            if byte == PING_FRAME[ping_index]:
                ping_index += 1
                if ping_index == len(PING_FRAME):
                    ping_index = 0
                    self._write_serial(PONG_FRAME)
            elif byte == PING_FRAME[0]:
                ping_index = 1
            else:
                ping_index = 0

    def _write_serial(self, frame: bytes):
        """Write one complete UART frame while excluding the reader thread."""
        try:
            with self._ser_lock:
                if self.ser is None or not self.ser.is_open:
                    return False
                self.ser.write(frame)
            return True
        except serial.SerialException as e:
            self.get_logger().error(f"Serial write failed: {e}")
            with self._ser_lock:
                self.ser = None
            return False

    # ── 点云回调 ───────────────────────────────────────────────────
    def _cb(self, msg: PointCloud2):
        now = time.monotonic()
        if now - self._last_send < self.dt:
            return

        pts = _parse_cloud(msg)
        if len(pts) == 0:
            self._send(0.0, 0.0)
            self._last_send = now
            return

        x, y, z = pts[:, 0], pts[:, 1], pts[:, 2]

        # 空间过滤
        mask = (
            (x >= self.dmin) & (x <= self.dmax) &
            (np.abs(y) <= self.ylim) &
            (z >= self.zmin) & (z <= self.zmax)
        )
        filtered = pts[mask]
        n = len(filtered)

        if n < self.nmin:
            self._send(0.0, 0.0)
            self._last_send = now
            return
        if n > self.nmax:
            self.get_logger().debug(f"忽略大物体 ({n} 点)")
            self._send(0.0, 0.0)
            self._last_send = now
            return

        y_span = float(filtered[:, 1].max() - filtered[:, 1].min()) if n > 1 else 0.0
        if y_span > self.y_span_max:
            self.get_logger().debug(
                f"忽略宽物体 (横向={y_span:.3f}m > {self.y_span_max}m)")
            self._send(0.0, 0.0)
            self._last_send = now
            return

        cx = float(np.mean(filtered[:, 0]))
        cy = float(np.mean(filtered[:, 1]))
        self.get_logger().info(
            f"雪糕筒 ({n}点, 横宽={y_span:.3f}m): x={cx:.3f}m  y={cy:+.3f}m")

        self._send(cx, cy)
        self._last_send = now

    # ── 串口发送 ───────────────────────────────────────────────────
    def _send(self, x: float, y: float):
        """
        帧格式（8字节）：
          AA BB [x_mm int16 BE] [y_mm int16 BE] [checksum uint8] 55
        """
        x_mm = max(-32768, min(32767, int(round(x * 1000))))
        y_mm = max(-32768, min(32767, int(round(y * 1000))))
        payload = struct.pack(">hh", x_mm, y_mm)
        chk = sum(payload) & 0xFF
        frame = bytes([0xAA, 0xBB]) + payload + bytes([chk, 0x55])

        if not self._write_serial(frame):
            self.get_logger().debug(
                f"[no-serial] frame: {frame.hex(' ').upper()}")

    def destroy_node(self):
        self._serial_stop.set()
        with self._ser_lock:
            if self.ser and self.ser.is_open:
                self.ser.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = ObstacleDetector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
