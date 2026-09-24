#!/usr/bin/env python3
"""Fullscreen bedside scan dashboard for the NUC display."""

import signal
import sys

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

from .nav_protocol import (
    SCAN_CONTEXT_BED1,
    SCAN_CONTEXT_BED3,
    SCAN_CONTEXT_ORDER,
)
from .scan_dashboard_core import parse_scan_result, parse_scan_status


class CameraView(QLabel):
    ASPECT_RATIO = 16.0 / 10.0

    def __init__(self) -> None:
        super().__init__("等待摄像头画面…")
        self.setObjectName("cameraView")
        self.setAlignment(Qt.AlignCenter)
        self.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Preferred)
        self.setMinimumWidth(260)

    def hasHeightForWidth(self) -> bool:
        return True

    def heightForWidth(self, width: int) -> int:
        return max(160, round(width / self.ASPECT_RATIO))

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

        root = QWidget()
        root.setObjectName("root")
        layout = QHBoxLayout(root)
        layout.setContentsMargins(32, 18, 32, 18)
        layout.setSpacing(28)

        screen = QApplication.primaryScreen()
        screen_height = screen.geometry().height() if screen is not None else 720
        title_size = max(16, min(22, round(screen_height * 0.020)))
        nurse_value_size = max(26, min(38, round(screen_height * 0.035)))
        bed_value_size = max(44, min(68, round(screen_height * 0.063)))

        information = QWidget()
        information_layout = QVBoxLayout(information)
        information_layout.setContentsMargins(0, 0, 0, 0)
        information_layout.setSpacing(12)

        nurse_card, self.nurse_value = self._make_card(
            "护士台二维码", nurse_value_size, title_size, minimum_height=100
        )
        bed1_card, self.bed1_value = self._make_card(
            "1床条码", bed_value_size, title_size
        )
        bed3_card, self.bed3_value = self._make_card(
            "3床条码", bed_value_size, title_size
        )
        information_layout.addWidget(nurse_card, 1)
        information_layout.addWidget(bed1_card, 2)
        information_layout.addWidget(bed3_card, 2)

        camera_panel = QWidget()
        camera_layout = QVBoxLayout(camera_panel)
        camera_layout.setContentsMargins(0, 0, 0, 0)
        camera_layout.setSpacing(12)

        camera_title = QLabel("扫码摄像头")
        camera_title.setFont(QFont("Noto Sans CJK SC", title_size, QFont.Bold))
        camera_title.setAlignment(Qt.AlignCenter)
        self.camera_view = CameraView()
        camera_layout.addStretch(1)
        camera_layout.addWidget(camera_title)
        camera_layout.addWidget(self.camera_view)
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
            QLabel#cameraView {
                color: #667085;
                background: white;
                border: 2px solid #d9e0eb;
                border-radius: 14px;
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

    def set_scan_value(self, context: int, value: str) -> None:
        labels = {
            SCAN_CONTEXT_ORDER: self.nurse_value,
            SCAN_CONTEXT_BED1: self.bed1_value,
            SCAN_CONTEXT_BED3: self.bed3_value,
        }
        label = labels.get(context)
        if label is not None:
            label.setText(value or "--")

    def set_camera_frame(self, data: bytes) -> None:
        pixmap = QPixmap()
        if not pixmap.loadFromData(data, "JPEG"):
            return
        self._camera_pixmap = pixmap
        self._refresh_camera()

    def _refresh_camera(self) -> None:
        if self._camera_pixmap is None:
            return
        size = self.camera_view.contentsRect().size()
        scaled = self._camera_pixmap.scaled(
            size, Qt.KeepAspectRatioByExpanding, Qt.SmoothTransformation
        )
        left = max(0, (scaled.width() - size.width()) // 2)
        top = max(0, (scaled.height() - size.height()) // 2)
        self.camera_view.setPixmap(
            scaled.copy(left, top, size.width(), size.height())
        )

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
        self._refresh_camera()

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
            CompressedImage,
            "/medical_nav/scanner_preview/compressed",
            self._camera_frame,
            2,
        )

    def _scan_result(self, message: String) -> None:
        parsed = parse_scan_result(message.data)
        if parsed is not None:
            self.window.set_scan_value(*parsed)

    def _camera_frame(self, message: CompressedImage) -> None:
        self.window.set_camera_frame(bytes(message.data))

    def _scanner_status(self, message: String) -> None:
        values = parse_scan_status(message.data)
        if values is not None:
            for context, value in values.items():
                self.window.set_scan_value(context, value)


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
