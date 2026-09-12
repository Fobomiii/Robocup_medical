#!/usr/bin/env python3
"""
Livox Mid360 障碍物可视化调试工具 v3
运行: source ~/livox_ws/install/setup.bash && python3 ~/lidar_viz.py
"""
import os, sys, threading, time, subprocess
# 确保与 systemd 服务使用同一个 RMW，避免重启后找不到话题
os.environ.setdefault('RMW_IMPLEMENTATION', 'rmw_fastrtps_cpp')
os.environ.setdefault('ROS_DOMAIN_ID', '0')

import numpy as np

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2 as pc2

from PyQt5.QtWidgets import (
    QApplication, QMainWindow, QWidget,
    QVBoxLayout, QHBoxLayout, QGridLayout,
    QLabel, QSlider, QPushButton, QLineEdit,
    QRadioButton, QButtonGroup,
    QStatusBar, QFrame, QSizePolicy,
    QScrollArea, QDialog
)
from PyQt5.QtCore import Qt, QTimer
from PyQt5.QtGui import QFont, QCursor

import matplotlib
matplotlib.use('Qt5Agg')
from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg as FigureCanvas
from matplotlib.figure import Figure
import matplotlib.patches as mpatches


# ═══════════════════════════════════════════════════════════════
#  颜色常量
# ═══════════════════════════════════════════════════════════════
BG    = "#0f0f1a"
PANEL = "#1a1a2e"
BORDER= "#2a3a5a"
ACCENT= "#88ccff"
WARN  = "#ffcc44"
OK    = "#44ee88"
ERR_C = "#ff5555"
TEXT  = "#ccddee"
DIM   = "#5577aa"
VAL_C = "#ffcc66"


# ═══════════════════════════════════════════════════════════════
#  点云解析（兼容结构化 dtype）
# ═══════════════════════════════════════════════════════════════
def parse_cloud(msg) -> np.ndarray:
    gen = pc2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True)
    arr = list(gen)
    if not arr:
        return np.empty((0, 3), dtype=np.float32)
    a = np.array(arr)
    if a.dtype.names:
        return np.column_stack([a["x"], a["y"], a["z"]]).astype(np.float32)
    return a.reshape(-1, 3).astype(np.float32)


# ═══════════════════════════════════════════════════════════════
#  ROS2 后台线程
# ═══════════════════════════════════════════════════════════════
class LidarSubscriber(Node):
    def __init__(self):
        super().__init__('lidar_viz_node')
        self.raw_pts = np.empty((0, 3), dtype=np.float32)
        self.lock = threading.Lock()
        self.last_recv_time = 0.0          # 0 = 从未收到过
        self.sub = self.create_subscription(
            PointCloud2, '/livox/lidar', self._cb, 10)

    def _cb(self, msg):
        pts = parse_cloud(msg)
        if pts.shape[0] > 0:
            with self.lock:
                self.raw_pts = pts
                self.last_recv_time = time.time()

    def get_raw(self):
        with self.lock:
            return self.raw_pts.copy()

    def data_age(self) -> float:
        """距上次收到数据的秒数；从未收到返回 inf"""
        if self.last_recv_time == 0.0:
            return float('inf')
        return time.time() - self.last_recv_time


def ros_spin(node):
    rclpy.spin(node)


# ═══════════════════════════════════════════════════════════════
#  极坐标画布
# ═══════════════════════════════════════════════════════════════
class PolarCanvas(FigureCanvas):
    def __init__(self):
        self.fig = Figure(figsize=(5.5, 5.5), facecolor=BG)
        super().__init__(self.fig)
        self.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)
        self.ax = self.fig.add_subplot(111, polar=True, facecolor='#16213e')
        self._style()

    def _style(self):
        ax = self.ax
        ax.set_theta_zero_location('N')
        ax.set_theta_direction(-1)
        ax.tick_params(colors='#8899aa', labelsize=9)
        ax.spines['polar'].set_color(BORDER)
        for g in ax.yaxis.get_gridlines() + ax.xaxis.get_gridlines():
            g.set_color('#2a3a5a'); g.set_linewidth(0.6)
        ax.set_rlabel_position(22.5)
        self.fig.subplots_adjust(top=0.95, bottom=0.05, left=0.05, right=0.95)

    def redraw(self, raw, filtered, obstacle, dmin, dmax,
               show_raw, show_filt, max_r=2.0):
        ax = self.ax
        ax.cla(); self._style()
        ax.set_rmax(max_r)
        ax.set_rticks([0.25, 0.5, 0.75, 1.0, 1.5, 2.0])

        th = np.linspace(0, 2*np.pi, 300)
        ax.fill_between(th, dmin, dmax, color='#ffaa00', alpha=0.08, zorder=1)
        ax.plot(th, [dmin]*300, color='#ffaa00', lw=0.8, ls='--', alpha=0.5)
        ax.plot(th, [dmax]*300, color='#ffaa00', lw=0.8, ls='--', alpha=0.5)

        handles = []

        def to_polar(pts):
            r = np.sqrt(pts[:,0]**2 + pts[:,1]**2)
            t = np.pi/2 - np.arctan2(pts[:,1], pts[:,0])
            return r, t

        if show_raw and len(raw) > 0:
            r, t = to_polar(raw)
            m = r < max_r
            ax.scatter(t[m], r[m], s=1.5, c='#4488cc', alpha=0.3, zorder=2, rasterized=True)
            handles.append(mpatches.Patch(color='#4488cc', label=f'原始点云 ({m.sum()}点)'))

        if show_filt and len(filtered) > 0:
            r, t = to_polar(filtered)
            ax.scatter(t, r, s=14, c='#ff6600', alpha=0.9, zorder=3, rasterized=True)
            handles.append(mpatches.Patch(color='#ff6600', label=f'过滤点云 ({len(filtered)}点)'))

        if obstacle is not None:
            cx, cy = obstacle
            r_o = np.sqrt(cx**2 + cy**2)
            t_o = np.pi/2 - np.arctan2(cy, cx)
            ax.plot(t_o, r_o, '*', ms=24, color='#ffee00',
                    zorder=5, mew=1.5, markeredgecolor='#ff8800')
            ax.annotate(f'  x={cx:.3f}m\n  y={cy:+.3f}m',
                        xy=(t_o, r_o), fontsize=9, color='#ffee00',
                        xytext=(12, 12), textcoords='offset points')
            handles.append(mpatches.Patch(color='#ffee00', label='障碍物质心 ★'))

        if handles:
            ax.legend(handles=handles, loc='lower left', fontsize=8,
                      framealpha=0.3, labelcolor='white', facecolor=PANEL)

        ax.text(np.pi/2, max_r*1.08, '▲ 前方',
                ha='center', va='bottom', color='#88aacc', fontsize=9)
        self.draw_idle()

    def show_no_signal(self, msg="等待 Mid360 数据..."):
        ax = self.ax
        ax.cla(); self._style()
        ax.set_rmax(2.0)
        ax.text(0, 1.0, msg, ha='center', va='center',
                color=ERR_C, fontsize=14, fontweight='bold',
                transform=ax.transAxes)
        self.draw_idle()


# ═══════════════════════════════════════════════════════════════
#  参数滑块
# ═══════════════════════════════════════════════════════════════
class NumpadDialog(QDialog):
    """数字键盘弹出框，定位于屏幕右下角"""
    def __init__(self, label, lo, hi, current, unit, parent=None):
        super().__init__(parent, Qt.Tool | Qt.FramelessWindowHint)
        self._lo = lo; self._hi = hi
        self.setFixedSize(290, 370)
        self.setStyleSheet(
            f"background:{PANEL};border:2px solid {BORDER};border-radius:10px;")

        vbox = QVBoxLayout(self)
        vbox.setContentsMargins(10, 10, 10, 10)
        vbox.setSpacing(6)

        title = QLabel(f"{label}  [{lo:.2f} ~ {hi:.2f}{unit}]")
        title.setStyleSheet(f"color:{ACCENT};font-size:12px;font-weight:bold;")
        title.setAlignment(Qt.AlignCenter)
        vbox.addWidget(title)

        self.display = QLineEdit(f"{current:.2f}")
        self.display.setReadOnly(True)
        self.display.setAlignment(Qt.AlignRight)
        self.display.setMinimumHeight(48)
        self.display.setStyleSheet(
            f"background:#0f0f25;color:{VAL_C};border:1px solid {ACCENT};"
            "border-radius:6px;padding:4px 10px;font-size:24px;font-weight:bold;")
        vbox.addWidget(self.display)

        grid = QGridLayout(); grid.setSpacing(5)

        def ks(bg=PANEL, fg=TEXT):
            return (f"background:{bg};color:{fg};border-radius:6px;"
                    f"font-size:18px;font-weight:bold;border:1px solid #3a4a6a;")

        def key(txt, bg=PANEL, fg=TEXT):
            b = QPushButton(txt); b.setFixedHeight(52); b.setStyleSheet(ks(bg, fg))
            return b

        for i, ch in enumerate(["7","8","9","4","5","6","1","2","3"]):
            r, c = divmod(i, 3)
            b = key(ch); b.clicked.connect(lambda _, x=ch: self._press(x))
            grid.addWidget(b, r, c)

        bm = key("-", "#2a2a3a", WARN)
        bm.clicked.connect(lambda: self._press("-"))
        grid.addWidget(bm, 3, 0)

        b0 = key("0"); b0.clicked.connect(lambda: self._press("0"))
        grid.addWidget(b0, 3, 1)

        bd = key("."); bd.clicked.connect(lambda: self._press("."))
        grid.addWidget(bd, 3, 2)

        bk = key("⌫", "#3a2a1a", WARN)
        bk.clicked.connect(lambda: self._press("⌫"))
        grid.addWidget(bk, 4, 0)

        ok = key("确认", "#1a4a1a", OK)
        ok.clicked.connect(self.accept)
        grid.addWidget(ok, 4, 1, 1, 2)

        vbox.addLayout(grid)

        cancel = QPushButton("取消")
        cancel.setFixedHeight(36)
        cancel.setStyleSheet(ks("#3a1a1a", ERR_C))
        cancel.clicked.connect(self.reject)
        vbox.addWidget(cancel)

        # 定位到屏幕右下角
        sg = QApplication.primaryScreen().geometry()
        self.move(sg.width() - self.width() - 8, sg.height() - self.height() - 36)

    def _press(self, ch):
        cur = self.display.text()
        if ch == "⌫":
            self.display.setText(cur[:-1] if len(cur) > 1 else "0")
        elif ch == "-":
            self.display.setText(cur[1:] if cur.startswith("-") else "-" + cur)
        elif ch == ".":
            if "." not in cur:
                self.display.setText(cur + ".")
        else:
            if cur == "0":
                self.display.setText(ch)
            else:
                self.display.setText(cur + ch)

    def get_value(self):
        try:
            v = float(self.display.text().rstrip(".") or "0")
            return max(self._lo, min(self._hi, v))
        except ValueError:
            return None


class PSlider(QWidget):
    def __init__(self, label, lo, hi, default, scale, unit):
        super().__init__()
        self.scale = scale
        self._lo = lo; self._hi = hi
        self._unit = unit; self._label = label
        row = QHBoxLayout(self)
        row.setContentsMargins(0, 1, 0, 1); row.setSpacing(4)

        lbl = QLabel(label)
        lbl.setFixedWidth(58)
        lbl.setStyleSheet(f"color:{TEXT};font-size:12px;")
        row.addWidget(lbl)

        self.sl = QSlider(Qt.Horizontal)
        self.sl.setRange(int(lo*scale), int(hi*scale))
        self.sl.setValue(int(default*scale))
        self.sl.setFixedHeight(26)
        self.sl.setStyleSheet("""
QSlider::groove:horizontal {
    height: 8px; border-radius: 4px; background: #2a3a5a; margin: 0 2px;
}
QSlider::sub-page:horizontal { background: #88ccff; border-radius: 4px; }
QSlider::handle:horizontal {
    width: 24px; height: 24px; border-radius: 12px;
    background: #88ccff; margin: -8px -1px; border: 2px solid #aaddff;
}""")
        row.addWidget(self.sl)

        # 数值标签：点击弹出数字键盘
        self.vl = QPushButton(f"{default:.2f}{unit}")
        self.vl.setFixedWidth(56)
        self.vl.setFixedHeight(26)
        self.vl.setCursor(QCursor(Qt.PointingHandCursor))
        self.vl.setStyleSheet(
            f"color:{VAL_C};font-size:12px;font-weight:bold;"
            f"background:#22294a;border:1px solid #3a4a6a;border-radius:4px;"
            "padding:0 2px; text-align:right;")
        self.vl.clicked.connect(self._open_numpad)
        row.addWidget(self.vl)

        self.sl.valueChanged.connect(
            lambda v: self.vl.setText(f"{v/scale:.2f}{unit}"))

    def _open_numpad(self):
        dlg = NumpadDialog(self._label, self._lo, self._hi,
                           self.value(), self._unit, self.window())
        if dlg.exec_() == QDialog.Accepted:
            v = dlg.get_value()
            if v is not None:
                self.set_value(v)

    def value(self):         return self.sl.value() / self.scale
    def set_value(self, v):  self.sl.setValue(int(v * self.scale))


# ═══════════════════════════════════════════════════════════════
#  服务控制器（systemctl）
# ═══════════════════════════════════════════════════════════════
class ServiceCtrl:
    MID360   = 'livox-mid360.service'
    OBSTACLE = 'obstacle-detector.service'

    @staticmethod
    def status(svc) -> str:
        r = subprocess.run(['systemctl', 'is-active', svc],
                           capture_output=True, text=True)
        return r.stdout.strip()

    @staticmethod
    def start(svc):
        subprocess.Popen(['sudo', 'systemctl', 'start', svc])

    @staticmethod
    def stop(svc):
        subprocess.Popen(['sudo', 'systemctl', 'stop', svc])


# ═══════════════════════════════════════════════════════════════
#  右侧面板：服务控制 + 调参说明
# ═══════════════════════════════════════════════════════════════
class RightPanel(QWidget):
    def __init__(self, parent=None):
        super().__init__(parent)
        self.setFixedWidth(195)
        self.setStyleSheet(f"background:{PANEL};border-radius:8px;")
        root = QVBoxLayout(self)
        root.setContentsMargins(6, 6, 6, 4)
        root.setSpacing(4)

        # ─── 标题 ────────────────────────────────────────
        t = QLabel("雷达连接 & 说明")
        t.setStyleSheet(f"color:{ACCENT};font-size:13px;font-weight:bold;")
        t.setAlignment(Qt.AlignCenter)
        root.addWidget(t)
        root.addWidget(self._hline())

        # ─── Mid360 驱动控制 ──────────────────────────────
        root.addWidget(self._sec("Mid360 驱动"))

        row1 = QHBoxLayout()
        self.dot_mid = QLabel("●")
        self.dot_mid.setStyleSheet(f"color:{ERR_C};font-size:16px;")
        self.lbl_mid = QLabel("未运行")
        self.lbl_mid.setStyleSheet(f"color:{TEXT};font-size:11px;")
        row1.addWidget(self.dot_mid); row1.addWidget(self.lbl_mid); row1.addStretch()
        root.addLayout(row1)

        self.btn_mid = QPushButton("▶  启动驱动")
        self.btn_mid.setStyleSheet(self._btn_style("#1a4a1a", OK))
        self.btn_mid.clicked.connect(self._toggle_mid)
        root.addWidget(self.btn_mid)

        root.addWidget(self._hline())

        # ─── 障碍物检测控制 ───────────────────────────────
        root.addWidget(self._sec("障碍物检测"))

        row2 = QHBoxLayout()
        self.dot_obs = QLabel("●")
        self.dot_obs.setStyleSheet(f"color:{ERR_C};font-size:16px;")
        self.lbl_obs = QLabel("未运行")
        self.lbl_obs.setStyleSheet(f"color:{TEXT};font-size:11px;")
        row2.addWidget(self.dot_obs); row2.addWidget(self.lbl_obs); row2.addStretch()
        root.addLayout(row2)

        self.btn_obs = QPushButton("▶  启动检测")
        self.btn_obs.setStyleSheet(self._btn_style("#1a4a1a", OK))
        self.btn_obs.clicked.connect(self._toggle_obs)
        root.addWidget(self.btn_obs)

        self.btn_all = QPushButton("⚡  一键启动全部")
        self.btn_all.setStyleSheet(self._btn_style("#1a3a5a", ACCENT))
        self.btn_all.clicked.connect(self._start_all)
        root.addWidget(self.btn_all)

        root.addWidget(self._hline())

        # ─── 调参说明（可滚动）────────────────────────────
        root.addWidget(self._sec("参数说明"))

        scroll = QScrollArea()
        scroll.setWidgetResizable(True)
        scroll.setStyleSheet(f"""
QScrollArea {{
    background:{BG};
    border:1px solid {BORDER};
    border-radius:4px;
}}
QScrollBar:vertical {{
    background:#1a1a2e;
    width:28px;
    border-radius:14px;
    margin:4px 2px;
}}
QScrollBar::handle:vertical {{
    background:#4a6a9a;
    min-height:50px;
    border-radius:12px;
    margin:2px;
}}
QScrollBar::handle:vertical:hover {{
    background:#6a9acc;
}}
QScrollBar::add-line:vertical,
QScrollBar::sub-line:vertical {{ height:0; }}
QScrollBar::add-page:vertical,
QScrollBar::sub-page:vertical {{ background:none; }}
""")
        scroll.setHorizontalScrollBarPolicy(Qt.ScrollBarAlwaysOff)
        scroll.setLayoutDirection(Qt.RightToLeft)   # 滚动条移到左侧

        inner = QWidget()
        inner.setLayoutDirection(Qt.LeftToRight)    # 文字仍然左到右
        inner.setStyleSheet(f"background:{BG};")
        desc_layout = QVBoxLayout(inner)
        desc_layout.setContentsMargins(6, 6, 6, 6)
        desc_layout.setSpacing(4)

        for title, body in PARAM_DESCS:
            tl = QLabel(title)
            tl.setStyleSheet(
                f"color:{WARN};font-size:13px;font-weight:bold;padding-top:6px;")
            tl.setWordWrap(True)
            desc_layout.addWidget(tl)

            bl = QLabel(body)
            bl.setStyleSheet(f"color:#99aabb;font-size:12px;")
            bl.setWordWrap(True)
            desc_layout.addWidget(bl)

        desc_layout.addStretch()
        scroll.setWidget(inner)
        root.addWidget(scroll, 1)

        # 每 2 秒刷新状态指示灯
        self.timer = QTimer()
        self.timer.timeout.connect(self._refresh_status)
        self.timer.start(2000)
        self._refresh_status()

    # ── 内部方法 ──────────────────────────────────────────
    def _hline(self):
        f = QFrame(); f.setFrameShape(QFrame.HLine)
        f.setStyleSheet(f"color:{BORDER};"); return f

    def _sec(self, t):
        l = QLabel(f"▸ {t}")
        l.setStyleSheet(f"color:{DIM};font-size:11px;"); return l

    def _btn_style(self, bg, fg):
        return (f"background:{bg};color:{fg};border:1px solid {fg}44;"
                "border-radius:5px;padding:8px 4px;font-size:13px;font-weight:bold;")

    def _refresh_status(self):
        mid_st = ServiceCtrl.status(ServiceCtrl.MID360)
        obs_st = ServiceCtrl.status(ServiceCtrl.OBSTACLE)

        def apply(dot, lbl, btn, st, start_txt, stop_txt):
            running = (st == 'active')
            dot.setStyleSheet(f"color:{OK if running else ERR_C};font-size:16px;")
            lbl.setText("运行中" if running else "已停止")
            lbl.setStyleSheet(
                f"color:{OK if running else '#888899'};font-size:11px;")
            btn.setText(stop_txt if running else start_txt)
            btn.setStyleSheet(
                self._btn_style("#4a1a1a", ERR_C) if running
                else self._btn_style("#1a4a1a", OK))
            btn.setEnabled(True)  # 解除操作锁

        apply(self.dot_mid, self.lbl_mid, self.btn_mid,
              mid_st, "▶  启动驱动", "■  停止驱动")
        apply(self.dot_obs, self.lbl_obs, self.btn_obs,
              obs_st, "▶  启动检测", "■  停止检测")

    def _toggle_mid(self):
        self.btn_mid.setEnabled(False)          # 防止连击
        self.btn_mid.setText("⏳ 操作中...")
        st = ServiceCtrl.status(ServiceCtrl.MID360)
        if st == 'active':
            ServiceCtrl.stop(ServiceCtrl.MID360)
        else:
            ServiceCtrl.start(ServiceCtrl.MID360)
        QTimer.singleShot(2500, self._refresh_status)  # 等服务切换完成

    def _toggle_obs(self):
        self.btn_obs.setEnabled(False)
        self.btn_obs.setText("⏳ 操作中...")
        st = ServiceCtrl.status(ServiceCtrl.OBSTACLE)
        if st == 'active':
            ServiceCtrl.stop(ServiceCtrl.OBSTACLE)
        else:
            ServiceCtrl.start(ServiceCtrl.OBSTACLE)
        QTimer.singleShot(2500, self._refresh_status)

    def _start_all(self):
        self.btn_all.setEnabled(False)
        self.btn_all.setText("⏳  启动中(6s)...")
        ServiceCtrl.start(ServiceCtrl.MID360)
        QTimer.singleShot(5000, lambda: ServiceCtrl.start(ServiceCtrl.OBSTACLE))
        QTimer.singleShot(7000, self._refresh_status)
        QTimer.singleShot(7000, lambda: (
            self.btn_all.setText("⚡  一键启动全部"),
            self.btn_all.setEnabled(True)
        ))

    def auto_start_if_needed(self):
        """启动时自动检测，若服务未运行则自动拉起"""
        mid_st = ServiceCtrl.status(ServiceCtrl.MID360)
        if mid_st != 'active':
            self._start_all()
            return True
        return False


# ═══════════════════════════════════════════════════════════════
#  调参说明文字
# ═══════════════════════════════════════════════════════════════
PARAM_DESCS = [
    ("📏 近端 / 远端",
     "前方检测距离范围。\n雪糕筒放 40-50cm 处。太宽易误检，太窄漏检。"),
    ("↔ 横向范围",
     "y 轴粗过滤，±范围内的点才参与检测。建议 ±0.30m。"),
    ("📐 横宽上限",
     "最关键过滤参数。\n雪糕筒≈0.10-0.15m，箱子>0.30m，床>1m。\n建议设 0.20-0.25m。"),
    ("⬆ 高度 下限/上限",
     "Z 轴过滤，排除地面和过高的点。\n下限-0.10m 忽略地面，上限0.50m。"),
    ("🔢 最少/最多点数",
     "区域内点数范围。\n太少误检噪声，太多认为大物体。建议 5-80。"),
    ("🎨 显示模式",
     "原始点云=所有雷达点(蓝)\n过滤点云=通过条件的点(橙)\n两者叠加最直观"),
    ("⭐ 质心标记",
     "黄色★ = 障碍物质心。\nx=前向距离 y=横向偏移(正=左)"),
]


# ═══════════════════════════════════════════════════════════════
#  左侧参数面板
# ═══════════════════════════════════════════════════════════════
class LeftPanel(QWidget):
    def __init__(self):
        super().__init__()
        self.setFixedWidth(185)
        self.setStyleSheet(f"background:{PANEL};border-radius:8px;")
        pl = QVBoxLayout(self)
        pl.setContentsMargins(6, 6, 6, 6); pl.setSpacing(5)

        t = QLabel("参数调整")
        t.setStyleSheet(f"color:{ACCENT};font-size:13px;font-weight:bold;")
        t.setAlignment(Qt.AlignCenter); pl.addWidget(t)
        pl.addWidget(self._hline())

        pl.addWidget(self._sec("检测距离 (m)"))
        self.s_dmin  = PSlider("近端",    0.10, 1.50,  0.40, 100, "m"); pl.addWidget(self.s_dmin)
        self.s_dmax  = PSlider("远端",    0.10, 2.00,  0.50, 100, "m"); pl.addWidget(self.s_dmax)
        pl.addWidget(self._hline())

        pl.addWidget(self._sec("尺寸过滤"))
        self.s_yspan = PSlider("横宽上限",0.05, 1.00,  0.25, 100, "m"); pl.addWidget(self.s_yspan)
        self.s_ylim  = PSlider("横向范围",0.10, 1.00,  0.30, 100, "m"); pl.addWidget(self.s_ylim)
        self.s_zmin  = PSlider("高度下限",-0.50,0.20, -0.10, 100, "m"); pl.addWidget(self.s_zmin)
        self.s_zmax  = PSlider("高度上限",0.10, 1.50,  0.50, 100, "m"); pl.addWidget(self.s_zmax)
        pl.addWidget(self._hline())

        pl.addWidget(self._sec("有效点数范围"))
        self.s_nmin  = PSlider("最少点数", 1,  50,   5, 1, ""); pl.addWidget(self.s_nmin)
        self.s_nmax  = PSlider("最多点数",10, 300,  80, 1, ""); pl.addWidget(self.s_nmax)
        pl.addWidget(self._hline())

        pl.addWidget(self._sec("显示模式"))
        self.rb_raw  = QRadioButton("仅原始点云")
        self.rb_filt = QRadioButton("仅过滤点云")
        self.rb_both = QRadioButton("两者叠加"); self.rb_both.setChecked(True)
        grp = QButtonGroup(self)
        for rb in [self.rb_raw, self.rb_filt, self.rb_both]:
            rb.setStyleSheet(f"color:{TEXT};font-size:12px;")
            rb.setMaximumHeight(26)
            grp.addButton(rb); pl.addWidget(rb)
        pl.addWidget(self._hline())

        btn = QPushButton("↺  重置默认参数")
        btn.setMaximumHeight(30)
        btn.setStyleSheet(
            f"background:#252540;color:{TEXT};border-radius:5px;padding:4px;font-size:12px;")
        btn.clicked.connect(self._reset); pl.addWidget(btn)

    def _hline(self):
        f = QFrame(); f.setFrameShape(QFrame.HLine)
        f.setStyleSheet(f"color:{BORDER};"); return f

    def _sec(self, t):
        l = QLabel(f"▸ {t}")
        l.setStyleSheet(f"color:{DIM};font-size:11px;padding-top:4px;"); return l

    def _reset(self):
        self.s_dmin.set_value(0.40); self.s_dmax.set_value(0.50)
        self.s_yspan.set_value(0.25); self.s_ylim.set_value(0.30)
        self.s_zmin.set_value(-0.10); self.s_zmax.set_value(0.50)
        self.s_nmin.set_value(5);    self.s_nmax.set_value(80)


# ═══════════════════════════════════════════════════════════════
#  主窗口
# ═══════════════════════════════════════════════════════════════
STALE_TIMEOUT = 2.5   # 超过此秒数无数据则冻结画面

class MainWindow(QMainWindow):
    def __init__(self, ros_node):
        super().__init__()
        self.ros_node = ros_node
        self.setWindowTitle("Livox Mid360  障碍物可视化调试  v5")
        self.setStyleSheet(f"background:{BG};color:{TEXT};")
        self.showFullScreen()

        central = QWidget(); self.setCentralWidget(central)
        root = QHBoxLayout(central)
        root.setContentsMargins(4,4,4,4); root.setSpacing(4)

        self.left   = LeftPanel()
        self.canvas = PolarCanvas()
        self.right  = RightPanel()

        root.addWidget(self.left)
        root.addWidget(self.canvas, 1)
        root.addWidget(self.right)

        self.sb = QStatusBar()
        self.sb.setFixedHeight(22)
        self.sb.setStyleSheet(f"background:{PANEL};color:#88aacc;font-size:11px;padding:0 4px;")
        self.setStatusBar(self.sb)
        self.sb.showMessage("⏳  正在连接 /livox/lidar ...")

        # 主刷新定时器（100ms）
        self.timer = QTimer()
        self.timer.timeout.connect(self._tick)
        self.timer.start(100)

        # 启动 3s 后自动检查，若服务未运行则自动拉起
        QTimer.singleShot(3000, self._auto_start_check)

    def _auto_start_check(self):
        if self.ros_node.data_age() == float('inf'):
            # 还没有收到数据 → 检查服务
            did_start = self.right.auto_start_if_needed()
            if did_start:
                self.sb.showMessage("🚀 服务未运行，已自动启动 Mid360 + 障碍物检测...")

    def _tick(self):
        age = self.ros_node.data_age()

        # ── 无数据 / 数据过期：冻结画面 ─────────────────────────────
        if age > STALE_TIMEOUT:
            if age == float('inf'):
                self.sb.showMessage(
                    "⏳  等待 Mid360 数据 — 请确认驱动已启动（右侧一键启动）")
            else:
                self.sb.showMessage(
                    f"⚠  信号丢失 {int(age)}s — 驱动已停止或雷达断开")
            self.canvas.show_no_signal(
                "等待 Mid360 数据..." if age == float('inf')
                else f"信号丢失 {int(age)}s")
            return

        # ── 有效数据：正常处理 ────────────────────────────────────────
        raw = self.ros_node.get_raw()
        lp  = self.left

        dmin  = lp.s_dmin.value();  dmax  = lp.s_dmax.value()
        ylim  = lp.s_ylim.value();  yspan = lp.s_yspan.value()
        zmin  = lp.s_zmin.value();  zmax  = lp.s_zmax.value()
        nmin  = max(1, int(lp.s_nmin.value()))
        nmax  = int(lp.s_nmax.value())

        x, y, z = raw[:,0], raw[:,1], raw[:,2]
        mask = (x>=dmin)&(x<=dmax)&(np.abs(y)<=ylim)&(z>=zmin)&(z<=zmax)
        filt = raw[mask]; n = len(filt)

        obstacle = None; reason = ""
        if n < nmin:
            reason = f"点数不足 ({n} < {nmin})"
        elif n > nmax:
            reason = f"点数过多 ({n} > {nmax}) → 大物体"
        else:
            ys = float(filt[:,1].max()-filt[:,1].min()) if n>1 else 0.0
            if ys > yspan:
                reason = f"横向过宽 {ys:.2f}m > {yspan:.2f}m → 大物体"
            else:
                obstacle = (float(np.mean(filt[:,0])), float(np.mean(filt[:,1])))

        show_raw  = lp.rb_raw.isChecked()  or lp.rb_both.isChecked()
        show_filt = lp.rb_filt.isChecked() or lp.rb_both.isChecked()
        self.canvas.redraw(raw, filt, obstacle, dmin, dmax, show_raw, show_filt)

        if obstacle:
            cx, cy = obstacle
            r = np.sqrt(cx**2+cy**2)
            ang = np.degrees(np.arctan2(cy, cx))
            self.sb.showMessage(
                f"✓ 检测到障碍物  |  x={cx:.3f}m  y={cy:+.3f}m  "
                f"|  距离={r:.3f}m  角度={ang:+.1f}°  |  {n}点  "
                f"|  原始总点数 {len(raw)}")
        else:
            self.sb.showMessage(
                f"✗ 无目标  |  {reason}  "
                f"|  原始 {len(raw)} 点  区域内 {n} 点")


# ═══════════════════════════════════════════════════════════════
#  入口
# ═══════════════════════════════════════════════════════════════
def main():
    rclpy.init()
    node = LidarSubscriber()
    threading.Thread(target=ros_spin, args=(node,), daemon=True).start()

    app = QApplication(sys.argv)
    app.setStyle('Fusion')
    win = MainWindow(node)
    win.show()
    ret = app.exec_()
    rclpy.shutdown()
    sys.exit(ret)


if __name__ == '__main__':
    main()
