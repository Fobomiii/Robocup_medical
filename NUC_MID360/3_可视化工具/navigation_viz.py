#!/usr/bin/env python3
"""Field navigation monitor for the RoboCup medical robot."""

import json
import math
import os
import sys
import threading

import yaml
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseArray, PoseStamped
from nav_msgs.msg import Path
from std_msgs.msg import String

from PyQt5.QtCore import Qt, QTimer
from PyQt5.QtGui import QColor, QPalette
from PyQt5.QtWidgets import (
    QApplication,
    QFrame,
    QGridLayout,
    QLabel,
    QMainWindow,
    QSizePolicy,
    QVBoxLayout,
    QWidget,
)

import matplotlib
matplotlib.use("Qt5Agg")
matplotlib.rcParams["font.sans-serif"] = ["Noto Sans CJK JP", "DejaVu Sans"]
matplotlib.rcParams["axes.unicode_minus"] = False
from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg as FigureCanvas
from matplotlib.figure import Figure
from matplotlib.patches import Circle, Rectangle


BG = "#101417"
PANEL = "#191f23"
GRID = "#415058"
TEXT = "#e7ecef"
MUTED = "#9aa7ad"
GREEN = "#35c48d"
RED = "#ef6262"
PATH_COLOR = "#37a7e8"
OBSTACLE = "#e84d4d"


def find_map_config():
    candidates = [
        os.environ.get("MEDICAL_NAV_MAP", ""),
        os.path.expanduser("~/livox_ws/src/obstacle_detector/config/field_map.yaml"),
        os.path.expanduser("~/livox_ws/install/obstacle_detector/share/obstacle_detector/config/field_map.yaml"),
        os.path.normpath(
            os.path.join(os.path.dirname(__file__), "..", "2_ROS2包_obstacle_detector", "config", "field_map.yaml")
        ),
    ]
    for path in candidates:
        if path and os.path.isfile(path):
            return path
    raise FileNotFoundError("field_map.yaml not found; set MEDICAL_NAV_MAP")


def field_yaw_from_quaternion(pose):
    z = pose.orientation.z
    w = pose.orientation.w
    ros_yaw = math.atan2(2.0 * w * z, 1.0 - 2.0 * z * z)
    return -math.degrees(ros_yaw)


class NavigationSubscriber(Node):
    def __init__(self):
        super().__init__("medical_navigation_visualizer")
        self.lock = threading.Lock()
        self.pose = None
        self.path = []
        self.obstacles = []
        self.status = {}
        self.revision = 0
        self.create_subscription(PoseStamped, "/medical_nav/robot_pose", self._pose, 10)
        self.create_subscription(Path, "/medical_nav/path", self._path, 10)
        self.create_subscription(PoseArray, "/medical_nav/obstacles", self._obstacles, 10)
        self.create_subscription(String, "/medical_nav/status", self._status, 10)

    def _pose(self, message):
        with self.lock:
            self.pose = (
                message.pose.position.x * 1000.0,
                message.pose.position.y * 1000.0,
                field_yaw_from_quaternion(message.pose),
            )
            self.revision += 1

    def _path(self, message):
        with self.lock:
            self.path = [
                (pose.pose.position.x * 1000.0, pose.pose.position.y * 1000.0)
                for pose in message.poses
            ]
            self.revision += 1

    def _obstacles(self, message):
        with self.lock:
            self.obstacles = [
                (
                    pose.position.x * 1000.0,
                    pose.position.y * 1000.0,
                    max(80.0, pose.position.z * 1000.0),
                )
                for pose in message.poses
            ]
            self.revision += 1

    def _status(self, message):
        try:
            status = json.loads(message.data)
        except json.JSONDecodeError:
            status = {"message": message.data}
        with self.lock:
            self.status = status
            self.revision += 1

    def snapshot(self):
        with self.lock:
            return self.revision, self.pose, list(self.path), list(self.obstacles), dict(self.status)


class FieldCanvas(FigureCanvas):
    def __init__(self, field_map):
        self.field_map = field_map
        self.figure = Figure(figsize=(7.2, 7.2), facecolor=BG)
        self.axes = self.figure.add_subplot(111, facecolor="#151b1f")
        super().__init__(self.figure)
        self.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)
        self.figure.subplots_adjust(left=0.10, right=0.97, top=0.96, bottom=0.09)

    def redraw(self, pose, path, obstacles, inflation_mm=0.0):
        ax = self.axes
        ax.clear()
        bounds = self.field_map["field"]
        xmin, xmax = bounds["x_min_mm"], bounds["x_max_mm"]
        ymin, ymax = bounds["y_min_mm"], bounds["y_max_mm"]
        ax.set_xlim(xmin, xmax)
        ax.set_ylim(ymin, ymax)
        ax.set_aspect("equal", adjustable="box")
        ax.set_xlabel("X / mm", color=MUTED)
        ax.set_ylabel("Y / mm", color=MUTED)
        ax.tick_params(colors=MUTED, labelsize=9)
        ax.grid(color=GRID, linewidth=0.6, alpha=0.45)
        for spine in ax.spines.values():
            spine.set_color(GRID)

        ax.add_patch(
            Rectangle(
                (xmin, ymin),
                xmax - xmin,
                ymax - ymin,
                fill=False,
                edgecolor="#d4dde1",
                linewidth=2.0,
                zorder=1,
            )
        )

        for fixture in self.field_map.get("fixtures", []):
            width = fixture["width_mm"]
            height = fixture["height_mm"]
            left = fixture["x_mm"] - width / 2.0
            bottom = fixture["y_mm"] - height / 2.0
            color = "#59646a" if fixture["name"].startswith("bed") else "#78694c"
            ax.add_patch(
                Rectangle(
                    (left, bottom),
                    width,
                    height,
                    facecolor=color,
                    edgecolor="#c8d0d4",
                    linewidth=1.0,
                    alpha=0.78,
                    zorder=2,
                )
            )
            ax.text(
                fixture["x_mm"],
                fixture["y_mm"],
                fixture.get("label", fixture["name"]),
                color=TEXT,
                ha="center",
                va="center",
                fontsize=9,
                zorder=3,
            )

        for name, goal in self.field_map["goals"].items():
            x, y = goal["x_mm"], goal["y_mm"]
            if name in ("bed1", "bed3"):
                ax.add_patch(Circle((x, y), 300, fill=False, edgecolor="#f0c44f", linewidth=1.3, zorder=4))
            ax.scatter([x], [y], s=32, color="#f0c44f", edgecolors="#fff2ba", zorder=5)
            ax.text(x + 90, y + 90, goal.get("label", name), color="#f5dc89", fontsize=8, zorder=5)

        ax.axhline(3800, color="#c3a14c", linestyle="--", linewidth=1.0, alpha=0.65, zorder=1)

        if path:
            px = [point[0] for point in path]
            py = [point[1] for point in path]
            ax.plot(px, py, color=PATH_COLOR, linewidth=2.6, zorder=6)
            ax.scatter(px, py, s=34, color=PATH_COLOR, edgecolors="#d8f1ff", zorder=7)
            for index, (x, y) in enumerate(path[1:], start=1):
                ax.text(x + 70, y - 130, str(index), color="#d8f1ff", fontsize=8, zorder=7)

        for x, y, radius in obstacles:
            if inflation_mm > 0.0:
                ax.add_patch(
                    Circle(
                        (x, y),
                        radius + inflation_mm,
                        fill=False,
                        edgecolor="#f59b9b",
                        linestyle="--",
                        linewidth=1.0,
                        alpha=0.55,
                        zorder=7,
                    )
                )
            ax.add_patch(
                Circle(
                    (x, y), radius, facecolor=OBSTACLE,
                    edgecolor="#ffd4d4", alpha=0.9, zorder=8,
                )
            )

        if pose is not None:
            x, y, yaw_deg = pose
            yaw = math.radians(yaw_deg)
            dx = 300.0 * math.sin(yaw)
            dy = 300.0 * math.cos(yaw)
            ax.add_patch(Circle((x, y), 220, facecolor=GREEN, edgecolor="#d8fff0", alpha=0.88, zorder=9))
            ax.arrow(
                x,
                y,
                dx,
                dy,
                width=30,
                head_width=150,
                head_length=150,
                color="#f4ffff",
                length_includes_head=True,
                zorder=10,
            )

        self.draw_idle()


class MainWindow(QMainWindow):
    def __init__(self, node, field_map):
        super().__init__()
        self.node = node
        self.last_revision = -1
        self.setWindowTitle("医疗机器人场地导航")
        self.resize(1180, 820)

        root = QWidget()
        layout = QGridLayout(root)
        layout.setContentsMargins(14, 14, 14, 14)
        layout.setHorizontalSpacing(14)
        self.canvas = FieldCanvas(field_map)
        layout.addWidget(self.canvas, 0, 0)

        panel = QFrame()
        panel.setFixedWidth(270)
        panel.setStyleSheet(f"QFrame {{ background:{PANEL}; border:1px solid {GRID}; }}")
        panel_layout = QVBoxLayout(panel)
        panel_layout.setContentsMargins(18, 18, 18, 18)
        panel_layout.setSpacing(12)
        title = QLabel("导航状态")
        title.setStyleSheet(f"color:{TEXT}; font-size:20px; font-weight:600; border:none;")
        panel_layout.addWidget(title)

        self.serial_label = self._label(panel_layout)
        self.pose_label = self._label(panel_layout)
        self.path_label = self._label(panel_layout)
        self.task_label = self._label(panel_layout)
        self.position_label = self._label(panel_layout)
        self.obstacle_label = self._label(panel_layout)
        self.planner_label = self._label(panel_layout)
        panel_layout.addStretch(1)
        footer = QLabel("固定路径验证模式")
        footer.setStyleSheet(f"color:{MUTED}; font-size:12px; border:none;")
        panel_layout.addWidget(footer)
        layout.addWidget(panel, 0, 1)
        layout.setColumnStretch(0, 1)
        self.setCentralWidget(root)

        self.setStyleSheet(f"QMainWindow, QWidget {{ background:{BG}; color:{TEXT}; }}")
        timer = QTimer(self)
        timer.timeout.connect(self._tick)
        timer.start(100)
        self.timer = timer
        self.canvas.redraw(None, [], [])

    @staticmethod
    def _label(layout):
        label = QLabel()
        label.setWordWrap(True)
        label.setMinimumHeight(38)
        label.setStyleSheet(f"color:{TEXT}; font-size:14px; border:none;")
        layout.addWidget(label)
        return label

    def _tick(self):
        revision, pose, path, obstacles, status = self.node.snapshot()
        if revision == self.last_revision:
            return
        self.last_revision = revision
        self.canvas.redraw(pose, path, obstacles, float(status.get("inflation_mm", 0.0)))

        serial_ok = bool(status.get("serial"))
        pose_ok = bool(status.get("pose"))
        self.serial_label.setText(f"串口  {'在线' if serial_ok else '离线'}")
        self.serial_label.setStyleSheet(f"color:{GREEN if serial_ok else RED}; font-size:14px; border:none;")
        self.pose_label.setText(f"位姿  {'有效' if pose_ok else '等待'}")
        self.path_label.setText(
            f"路径  #{status.get('path_id', 0)}\n航点  {status.get('waypoint_index', 0)} / {status.get('waypoints', 0)}"
        )
        self.task_label.setText(
            f"任务状态  {status.get('task_state', '-')}\n导航状态  {status.get('nav_status', '-')}"
        )
        if pose is None:
            self.position_label.setText("位置  --")
        else:
            self.position_label.setText(f"位置  X {pose[0]:.0f}  Y {pose[1]:.0f}\n航向  {pose[2]:.1f} deg")
        self.obstacle_label.setText(f"当前检测障碍  {len(obstacles)}")
        planner_mode = "A* 动态规划" if status.get("planner_enabled") else "固定路径"
        self.planner_label.setText(
            f"规划  {planner_mode}\n状态  {status.get('planner_state', '-')}  "
            f"重规划 {status.get('replans', 0)} 次"
        )


def spin_ros(node):
    rclpy.spin(node)


def main():
    field_map_path = find_map_config()
    with open(field_map_path, "r", encoding="utf-8") as stream:
        field_map = yaml.safe_load(stream)

    rclpy.init()
    node = NavigationSubscriber()
    thread = threading.Thread(target=spin_ros, args=(node,), daemon=True)
    thread.start()

    app = QApplication(sys.argv)
    palette = QPalette()
    palette.setColor(QPalette.Window, QColor(BG))
    palette.setColor(QPalette.WindowText, QColor(TEXT))
    app.setPalette(palette)
    window = MainWindow(node, field_map)
    window.show()
    result = app.exec_()

    if rclpy.ok():
        rclpy.shutdown()
    thread.join(timeout=1.0)
    node.destroy_node()
    raise SystemExit(result)


if __name__ == "__main__":
    main()
