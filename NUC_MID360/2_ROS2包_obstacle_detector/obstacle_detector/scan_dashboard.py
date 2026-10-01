#!/usr/bin/env python3
"""Fullscreen bedside scan dashboard for the NUC display."""

import signal
import sys
import time

import rclpy
from PyQt5.QtCore import QSize, Qt, QTimer
from PyQt5.QtGui import QFont, QPixmap
from PyQt5.QtWidgets import (
    QApplication,
    QFrame,
    QHBoxLayout,
    QLabel,
    QMainWindow,
    QPushButton,
    QSizePolicy,
    QVBoxLayout,
    QWidget,
)
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import String

from .health_ble_core import parse_health_status
from .nav_protocol import (
    SCAN_CONTEXT_BED1,
    SCAN_CONTEXT_BED3,
    SCAN_CONTEXT_ORDER,
    START_WAIT_TASK_STATES,
)
from .scan_dashboard_core import (
    describe_start_status,
    parse_bridge_status,
    parse_scan_result,
    parse_scan_status,
    parse_tele_scan_state,
)


class CameraView(QLabel):
    def __init__(
        self,
        aspect_ratio: float,
        placeholder: str = "等待摄像头画面…",
    ) -> None:
        super().__init__(placeholder)
        self.aspect_ratio = aspect_ratio
        self.setObjectName("cameraView")
        self.setAlignment(Qt.AlignCenter)
        self.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Fixed)
        self.setMinimumWidth(260)

    def hasHeightForWidth(self) -> bool:
        return True

    def heightForWidth(self, width: int) -> int:
        return max(140, round(width / self.aspect_ratio))

    def sizeHint(self) -> QSize:
        width = max(320, super().sizeHint().width())
        return QSize(width, self.heightForWidth(width))


class ScanDashboardWindow(QMainWindow):
    def __init__(self) -> None:
        super().__init__()
        self.setWindowFlags(Qt.Window | Qt.FramelessWindowHint)
        self.setWindowTitle("医疗机器人扫码信息")
        self.setObjectName("medicalScanDashboard")
        self._camera_pixmap = None
        self._tele_camera_pixmap = None
        self._tele_scan_state = "standby"
        self._hide_order_value = True
        self._last_health_status_monotonic = 0.0

        root = QWidget()
        root.setObjectName("root")
        layout = QHBoxLayout(root)
        layout.setContentsMargins(32, 18, 32, 18)
        layout.setSpacing(28)

        screen = QApplication.primaryScreen()
        screen_height = screen.geometry().height() if screen is not None else 720
        screen_width = screen.geometry().width() if screen is not None else 1280
        camera_width = max(260, round((screen_width - 92) / 4.0))
        title_size = max(16, min(22, round(screen_height * 0.020)))
        nurse_value_size = max(20, min(28, round(screen_height * 0.028)))
        bed_value_size = max(30, min(46, round(screen_height * 0.045)))
        health_value_size = max(42, min(60, round(screen_height * 0.070)))

        information = QWidget()
        information_layout = QVBoxLayout(information)
        information_layout.setContentsMargins(0, 0, 0, 0)
        information_layout.setSpacing(12)

        health_card = self._make_health_card(
            health_value_size, title_size
        )
        bed1_card, self.bed1_value = self._make_card(
            "1床条码", bed_value_size, title_size
        )
        bed3_card, self.bed3_value = self._make_card(
            "3床条码", bed_value_size, title_size
        )
        information_layout.addWidget(health_card, 3)
        information_layout.addWidget(bed1_card, 2)
        information_layout.addWidget(bed3_card, 2)

        camera_panel = QWidget()
        camera_layout = QVBoxLayout(camera_panel)
        camera_layout.setContentsMargins(0, 0, 0, 0)
        camera_layout.setSpacing(6)

        self.start_value = QLabel("等待")
        self.start_value.setObjectName("startStatus")
        # Match the red 等待 that set_start_status applies, so the very first
        # frames before any bridge status do not flash the grey stylesheet
        # colour and then snap to red.
        self.start_value.setStyleSheet("color: #e53935;")
        self.start_value.setFont(
            QFont("Noto Sans CJK SC", max(28, bed_value_size // 2), QFont.Bold)
        )
        self.start_value.setAlignment(Qt.AlignCenter)

        camera_title = QLabel("扫码摄像头")
        camera_title.setFont(QFont("Noto Sans CJK SC", title_size, QFont.Bold))
        camera_title.setAlignment(Qt.AlignCenter)
        self.camera_view = CameraView(16.0 / 10.0)
        self.camera_view.setFixedHeight(self.camera_view.heightForWidth(camera_width))
        tele_camera_title = QLabel("长焦辅助摄像头")
        tele_camera_title.setFont(
            QFont("Noto Sans CJK SC", title_size, QFont.Bold)
        )
        tele_camera_title.setAlignment(Qt.AlignCenter)
        self.tele_camera_view = CameraView(
            16.0 / 9.0,
            "等待长焦摄像头画面…",
        )
        self.tele_camera_view.setFixedHeight(
            self.tele_camera_view.heightForWidth(camera_width)
        )
        nurse_card, self.nurse_value = self._make_card(
            "护士台二维码", nurse_value_size, title_size, minimum_height=84
        )
        nurse_card.setMaximumHeight(max(96, round(screen_height * 0.13)))
        camera_layout.addWidget(self.start_value)
        camera_layout.addWidget(camera_title)
        camera_layout.addWidget(self.camera_view, 1)
        camera_layout.addWidget(tele_camera_title)
        camera_layout.addWidget(self.tele_camera_view, 1)
        camera_layout.addWidget(nurse_card)
        camera_layout.addStretch(1)

        layout.addWidget(information, 3)
        layout.addWidget(camera_panel, 1)

        self.exit_button = QPushButton("×", root)
        self.exit_button.setObjectName("exitButton")
        self.exit_button.setFixedSize(72, 72)
        self.exit_button.setCursor(Qt.PointingHandCursor)
        self.exit_button.setToolTip("退出扫码界面")
        self.exit_button.clicked.connect(self.close)
        self.exit_button.raise_()

        self.setCentralWidget(root)
        self.setStyleSheet(
            """
            QWidget#root {
                background: white;
                color: #172033;
            }
            QFrame#scanCard {
                background: #f7f9fc;
                border: 2px solid #d9e0eb;
                border-radius: 18px;
            }
            QLabel#scanValue {
                color: #0068d9;
                background: transparent;
            }
            QLabel#healthValue {
                color: #0068d9;
                background: transparent;
            }
            QLabel#cameraView {
                color: #667085;
                background: white;
                border: 2px solid #d9e0eb;
                border-radius: 14px;
            }
            QLabel#startStatus {
                background: transparent;
            }
            QPushButton#exitButton {
                color: white;
                background: #e53935;
                border: 3px solid white;
                border-radius: 36px;
                font-family: "DejaVu Sans";
                font-size: 42px;
                font-weight: bold;
                padding-bottom: 7px;
            }
            QPushButton#exitButton:hover {
                background: #c62828;
            }
            QPushButton#exitButton:pressed {
                background: #991f1f;
            }
            """
        )

        self._health_status_watchdog = QTimer(self)
        self._health_status_watchdog.timeout.connect(
            self._check_health_status_timeout
        )
        self._health_status_watchdog.start(1000)

    @staticmethod
    def _make_card(
        title: str, value_size: int, title_size: int, minimum_height: int = 0
    ):
        card = QFrame()
        card.setObjectName("scanCard")
        if minimum_height:
            card.setMinimumHeight(minimum_height)
        layout = QVBoxLayout(card)
        layout.setContentsMargins(24, 10, 24, 12)
        layout.setSpacing(2)

        title_label = QLabel(title)
        title_label.setFont(QFont("Noto Sans CJK SC", title_size, QFont.Bold))
        value_label = QLabel("--")
        value_label.setObjectName("scanValue")
        value_label.setFont(QFont("DejaVu Sans", value_size, QFont.Bold))
        value_label.setAlignment(Qt.AlignCenter)
        value_label.setTextInteractionFlags(Qt.TextSelectableByMouse)
        value_label.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)

        layout.addWidget(title_label)
        layout.addWidget(value_label, 1)
        return card, value_label

    def _make_health_card(self, value_size: int, title_size: int) -> QFrame:
        card = QFrame()
        card.setObjectName("scanCard")
        layout = QVBoxLayout(card)
        layout.setContentsMargins(24, 12, 24, 16)
        layout.setSpacing(8)

        title_label = QLabel("生命体征")
        title_label.setFont(QFont("Noto Sans CJK SC", title_size, QFont.Bold))
        layout.addWidget(title_label)

        self.health_connection_status = QLabel("等待重连")
        self.health_connection_status.setObjectName("healthConnectionStatus")
        self.health_connection_status.setFont(
            QFont("Noto Sans CJK SC", title_size, QFont.Bold)
        )
        self.health_connection_status.setAlignment(Qt.AlignCenter)
        self.health_connection_status.setStyleSheet("color: #e53935;")
        layout.addWidget(self.health_connection_status)

        metrics = QHBoxLayout()
        metrics.setSpacing(24)
        heart_layout, self.heart_rate_value = self._make_health_metric(
            "心率：", "-- bpm", value_size, title_size
        )
        temperature_layout, self.temperature_value = self._make_health_metric(
            "体温：", "-- °C", value_size, title_size
        )
        metrics.addLayout(heart_layout, 1)
        metrics.addLayout(temperature_layout, 1)
        layout.addLayout(metrics, 1)
        return card

    @staticmethod
    def _make_health_metric(
        title: str, placeholder: str, value_size: int, title_size: int
    ):
        layout = QVBoxLayout()
        layout.setSpacing(2)
        title_label = QLabel(title)
        title_label.setFont(QFont("Noto Sans CJK SC", title_size, QFont.Bold))
        value_label = QLabel(placeholder)
        value_label.setObjectName("healthValue")
        value_label.setFont(QFont("DejaVu Sans", value_size, QFont.Bold))
        value_label.setAlignment(Qt.AlignCenter)
        value_label.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)
        layout.addWidget(title_label)
        layout.addWidget(value_label, 1)
        return layout, value_label

    def _set_health_connection_state(self, connected: bool) -> None:
        if connected:
            self.health_connection_status.setText("连接成功")
            self.health_connection_status.setStyleSheet("color: #138a36;")
        else:
            self.health_connection_status.setText("等待重连")
            self.health_connection_status.setStyleSheet("color: #e53935;")

    def _clear_health_values(self) -> None:
        self.heart_rate_value.setText("-- bpm")
        self.temperature_value.setText("-- °C")

    def _check_health_status_timeout(self) -> None:
        if (
            self._last_health_status_monotonic <= 0.0
            or time.monotonic() - self._last_health_status_monotonic > 3.0
        ):
            self._set_health_connection_state(False)
            self._clear_health_values()

    def set_health_status(self, payload: str) -> None:
        status = parse_health_status(payload)
        if status is None:
            self._set_health_connection_state(False)
            self._clear_health_values()
            return
        self._last_health_status_monotonic = time.monotonic()
        if not status["ble_link"]:
            self._set_health_connection_state(False)
            self._clear_health_values()
            return
        self._set_health_connection_state(True)
        if not status["connected"]:
            self._clear_health_values()
            return
        heart_rate = status["heart_rate_bpm"]
        temperature = status["temperature_c"]
        self.heart_rate_value.setText(
            f"{heart_rate:d} bpm" if heart_rate is not None else "-- bpm"
        )
        self.temperature_value.setText(
            f"{temperature:.2f} °C" if temperature is not None else "-- °C"
        )

    def set_scan_value(self, context: int, value: str) -> None:
        if context == SCAN_CONTEXT_ORDER and self._hide_order_value:
            return
        labels = {
            SCAN_CONTEXT_ORDER: self.nurse_value,
            SCAN_CONTEXT_BED1: self.bed1_value,
            SCAN_CONTEXT_BED3: self.bed3_value,
        }
        label = labels.get(context)
        if label is not None:
            label.setText(value or "--")

    def set_task_state(self, task_state: int) -> None:
        self._hide_order_value = task_state in START_WAIT_TASK_STATES
        if self._hide_order_value:
            self.nurse_value.setText("--")

    def set_start_status(self, payload: str) -> None:
        status = parse_bridge_status(payload)
        if status is not None:
            self.set_task_state(status["task_state"])
        # 等待 / 发车 / 运行 / 成功, all from the one task state the bridge
        # forwards.  Before the first bridge status the C button cannot start
        # the robot, so the label stays on the red 等待 placeholder.
        _, text, color = describe_start_status(status)
        self.start_value.setText(text)
        self.start_value.setStyleSheet(f"color: {color};")

    def set_camera_frame(self, data: bytes) -> None:
        pixmap = QPixmap()
        if not pixmap.loadFromData(data, "JPEG"):
            return
        self._camera_pixmap = pixmap
        self._refresh_camera_view(self.camera_view, self._camera_pixmap)

    def set_tele_camera_frame(self, data: bytes) -> None:
        if self._tele_scan_state == "failed":
            return
        pixmap = QPixmap()
        if not pixmap.loadFromData(data, "JPEG"):
            return
        self._tele_camera_pixmap = pixmap
        self._refresh_camera_view(self.tele_camera_view, self._tele_camera_pixmap)

    def set_tele_scan_state(self, state: str) -> None:
        self._tele_scan_state = state
        if state == "failed":
            self._tele_camera_pixmap = None
            self.tele_camera_view.clear()
            self.tele_camera_view.setText("失败")
            self.tele_camera_view.setStyleSheet(
                "color: #e53935; background: #fff1f0; "
                "border: 4px solid #e53935; border-radius: 14px; "
                "font-size: 48px; font-weight: 700;"
            )
            return
        self.tele_camera_view.setStyleSheet("")
        if state == "scanning" and self._tele_camera_pixmap is None:
            self.tele_camera_view.setText("等待长焦摄像头画面…")
        elif self._tele_camera_pixmap is not None:
            self._refresh_camera_view(
                self.tele_camera_view, self._tele_camera_pixmap
            )

    @staticmethod
    def _refresh_camera_view(view: CameraView, pixmap: QPixmap) -> None:
        if pixmap is None:
            return
        size = view.contentsRect().size()
        if size.width() <= 0 or size.height() <= 0:
            return
        scaled = pixmap.scaled(
            size, Qt.KeepAspectRatioByExpanding, Qt.SmoothTransformation
        )
        left = max(0, (scaled.width() - size.width()) // 2)
        top = max(0, (scaled.height() - size.height()) // 2)
        view.setPixmap(scaled.copy(left, top, size.width(), size.height()))

    def _refresh_cameras(self) -> None:
        self._refresh_camera_view(self.camera_view, self._camera_pixmap)
        self._refresh_camera_view(self.tele_camera_view, self._tele_camera_pixmap)

    def enter_fullscreen(self) -> None:
        screen = self.screen() or QApplication.primaryScreen()
        if screen is not None:
            self.setGeometry(screen.geometry())
        self.showFullScreen()
        self.raise_()
        self.activateWindow()

    def resizeEvent(self, event) -> None:
        super().resizeEvent(event)
        root = self.centralWidget()
        if root is not None:
            self.exit_button.move(root.width() - self.exit_button.width() - 12, 12)
            self.exit_button.raise_()
        self._refresh_cameras()

    def keyPressEvent(self, event) -> None:
        if event.key() == Qt.Key_Escape:
            self.close()
            return
        if event.key() == Qt.Key_F11:
            self.showNormal() if self.isFullScreen() else self.enter_fullscreen()
            return
        super().keyPressEvent(event)


class ScanDashboardNode(Node):
    def __init__(self, window: ScanDashboardWindow) -> None:
        super().__init__("scan_dashboard")
        self.window = window
        self.create_subscription(
            String, "/medical_nav/scan_result", self._scan_result, 10
        )
        self.create_subscription(
            String, "/medical_nav/scanner_status", self._scanner_status, 10
        )
        self.create_subscription(
            String, "/medical_nav/bridge_status", self._bridge_status, 10
        )
        self.create_subscription(
            String, "/medical_nav/health_status", self._health_status, 10
        )
        self.create_subscription(
            CompressedImage,
            "/medical_nav/scanner_preview/compressed",
            self._camera_frame,
            2,
        )
        self.create_subscription(
            CompressedImage,
            "/medical_nav/tele_scanner_preview/compressed",
            self._tele_camera_frame,
            2,
        )

    def _scan_result(self, message: String) -> None:
        parsed = parse_scan_result(message.data)
        if parsed is not None:
            self.window.set_scan_value(*parsed)

    def _camera_frame(self, message: CompressedImage) -> None:
        self.window.set_camera_frame(bytes(message.data))

    def _tele_camera_frame(self, message: CompressedImage) -> None:
        self.window.set_tele_camera_frame(bytes(message.data))

    def _scanner_status(self, message: String) -> None:
        values = parse_scan_status(message.data)
        if values is not None:
            for context, value in values.items():
                self.window.set_scan_value(context, value)
        tele_state = parse_tele_scan_state(message.data)
        if tele_state is not None:
            self.window.set_tele_scan_state(tele_state)

    def _bridge_status(self, message: String) -> None:
        self.window.set_start_status(message.data)

    def _health_status(self, message: String) -> None:
        self.window.set_health_status(message.data)


def main(args=None) -> None:
    rclpy.init(args=args)
    application = QApplication(sys.argv)
    application.setApplicationName("medical-scan-dashboard")
    window = ScanDashboardWindow()
    node = ScanDashboardNode(window)

    spin_timer = QTimer()
    spin_timer.timeout.connect(lambda: rclpy.spin_once(node, timeout_sec=0.0))
    spin_timer.start(10)
    signal.signal(signal.SIGINT, lambda *_: application.quit())

    window.enter_fullscreen()
    QTimer.singleShot(300, window.enter_fullscreen)
    exit_code = application.exec_()
    spin_timer.stop()
    node.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()
    raise SystemExit(exit_code)


if __name__ == "__main__":
    main()
