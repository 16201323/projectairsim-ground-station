"""
Project AirSim 地面站控制界面
基于PyQt6框架，深色霓虹科幻风格
提供飞机类型选择、控制模式切换、启动/着陆/退出操作、
实时日志显示、UDP参数监控、相机视频流显示等功能。

功能概述：
1. 深色霓虹科幻风格UI界面
2. 支持四旋翼/六旋翼/倾斜旋翼(VTOL)三种无人机型号选择
3. 键盘手动控制 / UDP自动控制双模式
4. 启动/着陆/退出按钮操作
5. 实时流动日志显示（带颜色区分）
6. UDP模式下完整参数实时显示
7. 前视/下视相机视频流切换显示
8. 传感器数据面板（IMU/GPS/高度表/大气机/雷达等）
9. 退出后可重新启动
10. UDP超时自动悬停保护
"""

import ctypes
import os
import threading
import time
from datetime import datetime

from pynput import keyboard as pynput_keyboard
from pynput.keyboard import Key as PynputKey, KeyCode as PynputKeyCode

from PyQt6.QtWidgets import (
    QApplication, QMainWindow, QWidget, QVBoxLayout, QHBoxLayout,
    QLabel, QPushButton, QRadioButton, QButtonGroup, QTextEdit,
    QGroupBox, QProgressBar, QSplitter, QFrame, QGridLayout,
    QSizePolicy, QComboBox, QScrollArea, QStackedWidget
)
from PyQt6.QtCore import (
    Qt, QThread, pyqtSignal, pyqtSlot, QTimer, QSize, QRect
)
from PyQt6.QtGui import (
    QPainter, QColor, QPen, QBrush, QFont, QLinearGradient,
    QRadialGradient, QPalette, QImage, QPixmap
)

from core.constants import (
    DRONE_MODELS, DEFAULT_SPEED, DEFAULT_YAW_SPEED,
    SPEED_STEP, MIN_SPEED, MAX_SPEED, CONTROL_DURATION,
    UDP_DEFAULT_IP, UDP_DEFAULT_PORT, UDP_MULTICAST_ADDR,
    UDP_HOME_GEO_POINT, CAMERA_WIDTH, CAMERA_HEIGHT, VIDEO_FPS,
    DATA_BASE_DIR, LOG_SAVE_DIR,
    COLOR_BG_MAIN, COLOR_BG_PANEL, COLOR_BG_PANEL_LIGHT,
    COLOR_BORDER, COLOR_BORDER_GLOW,
    COLOR_NEON_CYAN, COLOR_NEON_PURPLE, COLOR_NEON_GREEN,
    COLOR_NEON_YELLOW, COLOR_NEON_RED, COLOR_NEON_ORANGE,
    COLOR_TEXT_MAIN, COLOR_TEXT_SECOND, COLOR_TEXT_DIM,
    WINDOW_WIDTH, WINDOW_HEIGHT, LEFT_PANEL_WIDTH,
    RIGHT_PANEL_WIDTH, LIDAR_AREA_HEIGHT,
)
from core.control_thread import DroneControlThread
from ui.widgets import NeonLabel, StatusIndicator
from ui.video_widget import VideoWidget
from ui.sensor_panel import SensorPanel
from ui.lidar_widgets import Lidar3DWidget
from sensors import SensorData



# ==============================================================================
# 科幻风格QSS样式表
# ==============================================================================

NEON_STYLESHEET = f"""
QMainWindow {{
    background-color: {COLOR_BG_MAIN};
}}
QWidget {{
    color: {COLOR_TEXT_MAIN};
    font-family: "Microsoft YaHei", "Consolas", monospace;
}}
QGroupBox {{
    background-color: {COLOR_BG_PANEL};
    border: 1px solid {COLOR_BORDER};
    border-radius: 6px;
    margin-top: 8px;
    padding: 6px;
    padding-top: 14px;
    font-size: 11px;
    font-weight: bold;
    color: {COLOR_NEON_CYAN};
}}
QGroupBox::title {{
    subcontrol-origin: margin;
    left: 10px;
    padding: 0 4px;
    color: {COLOR_NEON_CYAN};
}}
QRadioButton {{
    color: {COLOR_TEXT_MAIN};
    font-size: 11px;
    spacing: 6px;
    padding: 3px 0;
}}
QRadioButton::indicator {{
    width: 14px;
    height: 14px;
    border: 2px solid {COLOR_BORDER};
    border-radius: 8px;
    background-color: {COLOR_BG_PANEL};
}}
QRadioButton::indicator:checked {{
    border-color: {COLOR_NEON_CYAN};
    background-color: {COLOR_NEON_CYAN};
}}
QPushButton {{
    background-color: {COLOR_BG_PANEL_LIGHT};
    border: 1px solid {COLOR_BORDER};
    border-radius: 4px;
    padding: 8px 16px;
    font-size: 12px;
    font-weight: bold;
    color: {COLOR_TEXT_MAIN};
    min-height: 28px;
}}
QPushButton:hover {{
    border-color: {COLOR_NEON_CYAN};
    color: {COLOR_NEON_CYAN};
    background-color: #1e2d50;
}}
QPushButton:pressed {{
    background-color: {COLOR_NEON_CYAN};
    color: {COLOR_BG_MAIN};
}}
QPushButton:disabled {{
    background-color: #0d1117;
    border-color: #1a1f2e;
    color: {COLOR_TEXT_DIM};
}}
QPushButton#btnStart {{
    border-color: {COLOR_NEON_GREEN};
    color: {COLOR_NEON_GREEN};
}}
QPushButton#btnStart:hover {{
    background-color: #0a2a1a;
    border-color: {COLOR_NEON_GREEN};
}}
QPushButton#btnStart:pressed {{
    background-color: {COLOR_NEON_GREEN};
    color: {COLOR_BG_MAIN};
}}
QPushButton#btnLand {{
    border-color: {COLOR_NEON_YELLOW};
    color: {COLOR_NEON_YELLOW};
}}
QPushButton#btnLand:hover {{
    background-color: #2a2500;
    border-color: {COLOR_NEON_YELLOW};
}}
QPushButton#btnLand:pressed {{
    background-color: {COLOR_NEON_YELLOW};
    color: {COLOR_BG_MAIN};
}}
QPushButton#btnExit {{
    border-color: {COLOR_NEON_RED};
    color: {COLOR_NEON_RED};
}}
QPushButton#btnExit:hover {{
    background-color: #2a0a0a;
    border-color: {COLOR_NEON_RED};
}}
QPushButton#btnExit:pressed {{
    background-color: {COLOR_NEON_RED};
    color: {COLOR_BG_MAIN};
}}
QTextEdit {{
    background-color: #0d1117;
    border: 1px solid {COLOR_BORDER};
    border-radius: 4px;
    color: {COLOR_TEXT_MAIN};
    font-family: "Consolas", monospace;
    font-size: 10px;
    padding: 4px;
    selection-background-color: {COLOR_NEON_CYAN};
    selection-color: {COLOR_BG_MAIN};
}}
QProgressBar {{
    background-color: {COLOR_BG_PANEL};
    border: 1px solid {COLOR_BORDER};
    border-radius: 4px;
    text-align: center;
    color: {COLOR_NEON_CYAN};
    font-size: 10px;
    min-height: 16px;
}}
QProgressBar::chunk {{
    background-color: {COLOR_NEON_CYAN};
    border-radius: 3px;
}}
QComboBox {{
    background-color: {COLOR_BG_PANEL_LIGHT};
    border: 1px solid {COLOR_BORDER};
    border-radius: 4px;
    padding: 4px 8px;
    color: {COLOR_TEXT_MAIN};
    font-size: 11px;
    min-height: 22px;
}}
QComboBox:hover {{
    border-color: {COLOR_NEON_CYAN};
}}
QComboBox::drop-down {{
    border: none;
    width: 20px;
}}
QComboBox QAbstractItemView {{
    background-color: {COLOR_BG_PANEL};
    border: 1px solid {COLOR_BORDER};
    color: {COLOR_TEXT_MAIN};
    selection-background-color: {COLOR_NEON_CYAN};
    selection-color: {COLOR_BG_MAIN};
}}
QSplitter::handle {{
    background-color: {COLOR_BORDER};
}}
"""

# ==============================================================================
# 主窗口
# ==============================================================================

class GroundStationWindow(QMainWindow):
    """
    地面站主窗口
    深色霓虹科幻风格UI
    
    布局结构：
    ┌────────────┬────────────────────────────────────────────────────────────────┐
    │  ◆ LOGO  [启动][着陆][退出]  |  [连接][飞行][位置]  |  时钟                │
    ├────────────┼────────────────────────────────────────────────────────────────┤
    │            │                    视频显示区域                                │
    │  左侧面板  │  [前视/下视切换] [拍照] [VTOL切换]                              │
    │  飞机类型  │  ┌──────────────────────────────────────────────────────────┐  │
    │  控制模式  │  │                    相机视频                              │  │
    │  飞行速度  │  └──────────────────────────────────────────────────────────┘  │
    │  UDP参数   ├──────────────────────────────────────────────────────────────────┤
    │  传感器    │                         运行日志                                 │
    └────────────┴──────────────────────────────────────────────────────────────────┘
    
    交互逻辑：
    1. 用户选择飞机类型和控制模式
    2. 点击"启动"按钮创建控制线程并连接仿真环境
    3. 飞行中可通过键盘/按钮进行控制
    4. 点击"着陆"执行着陆，点击"退出"断开连接
    5. 退出后界面恢复初始状态，可再次启动
    """

    def __init__(self):
        super().__init__()
        self.setWindowTitle("AIRSIM GROUND STATION")
        self.setGeometry(100, 100, WINDOW_WIDTH, WINDOW_HEIGHT)
        self.setMinimumSize(1000, 700)
        # 设置强焦点策略，确保窗口能接收键盘事件
        self.setFocusPolicy(Qt.FocusPolicy.StrongFocus)
        self.control_thread = None
        self.keys_pressed = set()
        self._modifier_keys_pressed = set()
        # Linux窗口标题缓存（避免频繁调用xdotool）
        self._last_window_check_time = 0.0
        self._cached_window_title = ""
        self._init_ui()
        self._apply_stylesheet()

        # 安装PyQt6全局事件过滤器，捕获地面站窗口内的键盘事件
        QApplication.instance().installEventFilter(self)

        # 启动pynput系统级键盘监听器，捕获所有应用程序的键盘事件（包括UE窗口）
        # 即使地面站窗口不在前台，也能响应键盘控制
        self._pynput_listener = pynput_keyboard.Listener(
            on_press=self._on_pynput_key_press,
            on_release=self._on_pynput_key_release
        )
        self._pynput_listener.start()

        self.keyboard_timer = QTimer()
        self.keyboard_timer.timeout.connect(self._process_keyboard)
        # 键盘轮询间隔10ms，与控制循环频率匹配
        # 控制循环10ms + 键盘定时器10ms = 最小延迟
        self.keyboard_timer.setInterval(10)

        # 相机帧拉取定时器：替代frame_signal的推送模式
        # 原方案：每帧通过pyqtSignal跨线程传递2.7MB图像，4相机×20fps=80次/秒
        # 新方案：UI以15fps主动拉取缓存帧，零跨线程信号开销
        self._frame_pull_timer = QTimer()
        self._frame_pull_timer.timeout.connect(self._pull_camera_frames)
        # 15fps≈66ms间隔，人眼足够流畅，同时大幅减少UI重绘次数
        self._frame_pull_timer.setInterval(66)

    def _init_ui(self):
        """
        初始化UI布局（三栏式）
        布局结构：
        ┌──────────┬───────────────────────────────────────┬───────────────────┐
        │ ◆ AIRSIM GROUND STATION [▶启动][⬇着陆][✕退出] [连接][飞行] 时钟    │
        ├──────────┼───────────────────────────────────────┼───────────────────┤
        │          │                                       │                   │
        │  控制面板 │              视频显示区域               │   传感器仪表盘     │
        │  飞机类型 │  ┌───────────────────────────────┐   │  ┌──────┬──────┐  │
        │  控制模式 │  │                               │   │  │IMU   │GPS   │  │
        │  飞行速度 │  │       相机视频                 │   │  ├──────┼──────┤  │
        │  UDP参数  │  │                               │   │  │无线电│大气机│  │
        │          │  └───────────────────────────────┘   │  ├──────┼──────┤  │
        │          │  [第三人称▾] [📷拍照]                │  │激光  │超声波│  │
        │          ├──────────────────────────────────────┤  ├──────┼──────┤  │
        │          │        ◆ LiDAR 点云区域              │  │雷达  │LiDAR │  │
        │          │  ┌───────────────────────────────┐   │  └──────┴──────┘  │
        ├──────────┤  │         (待实现)               │   │         ◀        │
        │          │  └───────────────────────────────┘   │      收起按钮     │
        │ ◆ 运行日志│                                       │                   │
        │ [INFO].. │                                       │                   │
        └──────────┴───────────────────────────────────────┴───────────────────┘
          200px                  900px                          300px
        """
        central = QWidget()
        self.setCentralWidget(central)
        main_layout = QVBoxLayout(central)
        main_layout.setContentsMargins(4, 4, 4, 4)
        main_layout.setSpacing(4)

        main_layout.addWidget(self._create_title_bar())

        left_panel = self._create_left_panel()

        middle_panel = self._create_middle_panel()

        self.right_panel = self._create_right_sensor_panel()

        self.main_split = QSplitter(Qt.Orientation.Horizontal)
        self.main_split.addWidget(left_panel)
        self.main_split.addWidget(middle_panel)
        self.main_split.addWidget(self.right_panel)
        self.main_split.setSizes([
            LEFT_PANEL_WIDTH,
            WINDOW_WIDTH - LEFT_PANEL_WIDTH - RIGHT_PANEL_WIDTH,
            RIGHT_PANEL_WIDTH
        ])
        self.main_split.setHandleWidth(2)
        self.main_split.setChildrenCollapsible(False)

        main_layout.addWidget(self.main_split, 1)

    def _create_title_bar(self):
        """
        创建顶部标题栏
        布局：[LOGO] ... [启动][着陆][退出](居中) ... [连接状态][飞行状态][位置][时钟]
        操作按钮居中显示，系统状态框贴近右侧时钟
        """
        frame = QFrame()
        frame.setFixedHeight(70)
        frame.setStyleSheet(f"QFrame {{ background-color: {COLOR_BG_PANEL}; border: 1px solid {COLOR_BORDER}; border-radius: 4px; }}")
        layout = QHBoxLayout(frame)
        layout.setContentsMargins(10, 0, 10, 0)
        layout.setSpacing(6)

        # 左侧：程序标题（固定宽度靠左对齐）
        title = QLabel("◆ AIRSIM GROUND STATION")
        title.setFont(QFont("Consolas", 14, QFont.Weight.Bold))
        title.setStyleSheet(f"color: {COLOR_NEON_CYAN};")
        title.setFixedWidth(310)
        layout.addWidget(title)

        # 左侧弹性空间，将按钮推到中间
        layout.addStretch()

        # 操作按钮（居中显示，间距加大）
        self.btn_start = QPushButton("▶ 启动")
        self.btn_start.setObjectName("btnStart")
        self.btn_start.clicked.connect(self._on_start)
        self.btn_start.setFixedSize(80, 34)
        layout.addWidget(self.btn_start)

        layout.addSpacing(16)

        self.btn_land = QPushButton("⬇ 着陆")
        self.btn_land.setObjectName("btnLand")
        self.btn_land.setEnabled(False)
        self.btn_land.clicked.connect(self._on_land)
        self.btn_land.setFixedSize(80, 34)
        layout.addWidget(self.btn_land)

        layout.addSpacing(16)

        self.btn_exit = QPushButton("✕ 退出")
        self.btn_exit.setObjectName("btnExit")
        self.btn_exit.setEnabled(False)
        self.btn_exit.clicked.connect(self._on_exit)
        self.btn_exit.setFixedSize(80, 34)
        layout.addWidget(self.btn_exit)

        # 右侧弹性空间，将状态框推到右边
        layout.addStretch()

        # 系统状态框（贴近时钟）
        status_frame = QFrame()
        status_frame.setStyleSheet(f"QFrame {{ background-color: {COLOR_BG_PANEL_LIGHT}; border: 1px solid {COLOR_BORDER}; border-radius: 4px; padding: 0px; }}")
        status_layout = QHBoxLayout(status_frame)
        status_layout.setContentsMargins(4, 1, 4,1)#8, 2, 8, 2
        status_layout.setSpacing(0)

        status_layout.addStretch()
        self.ind_conn = StatusIndicator("未连接", COLOR_NEON_RED, size=15)
        status_layout.addWidget(self.ind_conn)
        status_layout.addSpacing(1)
        self.ind_flight = StatusIndicator("空闲", COLOR_TEXT_DIM, size=15)
        status_layout.addWidget(self.ind_flight)
        layout.addWidget(status_frame)

        layout.addSpacing(8)

        self.btn_toggle_sensor = QPushButton("◀")
        self.btn_toggle_sensor.setFixedSize(42, 22)
        self.btn_toggle_sensor.setToolTip("显示/隐藏传感器面板")
        self.btn_toggle_sensor.setStyleSheet(f"""
            QPushButton {{
                background-color: {COLOR_BG_PANEL_LIGHT};
                border: 1px solid {COLOR_BORDER};
                border-radius: 3px;
                color: {COLOR_TEXT_SECOND};
                font-size: 10px;
                padding: 2px 6px;
            }}
            QPushButton:hover {{
                border-color: {COLOR_NEON_CYAN};
                color: {COLOR_NEON_CYAN};
            }}
        """)
        self.btn_toggle_sensor.clicked.connect(self._on_toggle_sensor_panel)
        layout.addWidget(self.btn_toggle_sensor)

        layout.addSpacing(4)

        # 右侧：实时时钟
        self.time_label = QLabel()
        self.time_label.setFont(QFont("Consolas", 10))
        self.time_label.setStyleSheet(f"color: {COLOR_TEXT_SECOND};")
        layout.addWidget(self.time_label)

        clock = QTimer()
        clock.timeout.connect(self._update_clock)
        clock.start(1000)
        self._update_clock()
        self._clock_timer = clock

        return frame

    def _create_left_panel(self):
        """
        创建左侧面板（200px）：上=控制面板 + 下=运行日志
        """
        panel = QWidget()
        panel.setFixedWidth(LEFT_PANEL_WIDTH)
        layout = QVBoxLayout(panel)
        layout.setContentsMargins(2, 2, 2, 2)
        layout.setSpacing(0)

        left_split = QSplitter(Qt.Orientation.Vertical)
        left_split.setHandleWidth(2)
        left_split.setChildrenCollapsible(False)

        left_split.addWidget(self._create_control_panel())
        left_split.addWidget(self._create_log_panel())
        left_split.setSizes([450, 350])

        layout.addWidget(left_split)
        return panel

    def _create_control_panel(self):
        """
        创建控制面板：飞机类型、控制模式、飞行速度、UDP参数
        """
        panel = QWidget()
        layout = QVBoxLayout(panel)
        layout.setContentsMargins(2, 2, 2, 2)
        layout.setSpacing(2)

        drone_grp = QGroupBox("◆ 飞机类型")
        dl = QVBoxLayout(drone_grp)
        dl.setSpacing(1)
        dl.setContentsMargins(4, 11, 4, 2)
        self.drone_btn_group = QButtonGroup()
        for idx, (key, text) in enumerate([
            ("1", "四旋翼"),
            ("2", "六旋翼"),
            ("3", "倾斜旋翼VTOL"),
        ]):
            rb = QRadioButton(text)
            rb.setProperty("drone_key", key)
            self.drone_btn_group.addButton(rb, int(key))
            dl.addWidget(rb)
            if idx == 0:
                rb.setChecked(True)
        layout.addWidget(drone_grp)

        mode_grp = QGroupBox("◆ 控制模式")
        ml = QVBoxLayout(mode_grp)
        ml.setSpacing(1)
        ml.setContentsMargins(4, 11, 4, 2)
        self.mode_btn_group = QButtonGroup()
        self.radio_kb = QRadioButton("键盘控制")
        self.radio_udp = QRadioButton("UDP自动控制")
        self.mode_btn_group.addButton(self.radio_kb, 1)
        self.mode_btn_group.addButton(self.radio_udp, 2)
        self.radio_kb.setChecked(True)
        ml.addWidget(self.radio_kb)
        ml.addWidget(self.radio_udp)
        self.radio_kb.toggled.connect(self._on_mode_changed)
        self.radio_udp.toggled.connect(self._on_mode_changed)
        layout.addWidget(mode_grp)

        spd_grp = QGroupBox("◆ 飞行速度")
        spl = QVBoxLayout(spd_grp)
        spl.setSpacing(2)
        spl.setContentsMargins(4, 11, 4, 2)
        self.speed_bar = QProgressBar()
        self.speed_bar.setRange(int(MIN_SPEED * 10), int(MAX_SPEED * 10))
        self.speed_bar.setValue(int(DEFAULT_SPEED * 10))
        self.speed_bar.setFormat(f"{DEFAULT_SPEED:.1f} m/s")
        self.speed_bar.setFixedHeight(16)
        spl.addWidget(self.speed_bar)
        self.lbl_speed = NeonLabel(f"当前: {DEFAULT_SPEED:.1f} m/s", COLOR_NEON_CYAN, 9)
        spl.addWidget(self.lbl_speed)
        layout.addWidget(spd_grp)

        self.udp_grp = QGroupBox("◆ UDP 飞行参数")
        ul = QGridLayout(self.udp_grp)
        ul.setSpacing(4)
        ul.setContentsMargins(4, 11, 4, 2)
        self.udp_labels = {}
        udp_params = [
            ("lon", "经度:", 0, 0, "°"),
            ("lat", "纬度:", 1, 0, "°"),
            ("alt", "相对高度:", 2, 0, "m"),
            ("height", "绝对高度:", 2, 3, "m"),
            ("theta", "俯仰角:", 3, 0, "°"),
            ("phi", "滚转角:", 3, 3, "°"),
            ("psi", "偏航角:", 4, 0, "°"),
            ("Vt", "真空速:", 4, 3, "m/s"),
            ("vn", "北向速度:", 5, 0, "m/s"),
            ("ve", "东向速度:", 5, 3, "m/s"),
            ("Hdot", "升降速度:", 6, 0, "m/s"),
            ("Vd", "地速:", 6, 3, "m/s"),
        ]
        self.udp_units = {}
        for key, label, row, col, unit in udp_params:
            lbl = QLabel(label)
            lbl.setFont(QFont("Microsoft YaHei", 9))
            lbl.setStyleSheet(f"color: {COLOR_TEXT_SECOND};")
            ul.addWidget(lbl, row, col)
            val = NeonLabel("0.00", COLOR_NEON_GREEN, 10, bold=True)
            self.udp_labels[key] = val
            if key in ("lon", "lat"):
                ul.addWidget(val, row, col + 1, 1, 4)
            else:
                ul.addWidget(val, row, col + 1)
            if unit:
                unit_lbl = QLabel(unit)
                unit_lbl.setFont(QFont("Microsoft YaHei", 8))
                unit_lbl.setStyleSheet(f"color: {COLOR_TEXT_DIM};")
                if key in ("lon", "lat"):
                    ul.addWidget(unit_lbl, row, col + 5)
                else:
                    ul.addWidget(unit_lbl, row, col + 2)
                self.udp_units[key] = unit_lbl
        self.udp_grp.setVisible(False)
        layout.addWidget(self.udp_grp)

        layout.addStretch()
        return panel

    def _create_log_panel(self):
        """
        创建运行日志面板
        """
        panel = QWidget()
        layout = QVBoxLayout(panel)
        layout.setContentsMargins(2, 2, 2, 2)
        layout.setSpacing(2)

        log_grp = QGroupBox("◆ 运行日志")
        ll = QVBoxLayout(log_grp)
        ll.setContentsMargins(2, 11, 2, 2)
        self.log_text = QTextEdit()
        self.log_text.setReadOnly(True)
        self.log_text.setLineWrapMode(QTextEdit.LineWrapMode.NoWrap)
        self.log_text.setHorizontalScrollBarPolicy(Qt.ScrollBarPolicy.ScrollBarAsNeeded)
        self._log_count = 0
        self._max_log_count = 1000
        ll.addWidget(self.log_text)
        layout.addWidget(log_grp, 1)

        return panel

    def _create_middle_panel(self):
        """
        创建中间面板：上=视频显示 + 下=LiDAR占位区域
        """
        panel = QWidget()
        layout = QVBoxLayout(panel)
        layout.setContentsMargins(0, 0, 0, 0)
        layout.setSpacing(0)

        mid_split = QSplitter(Qt.Orientation.Vertical)
        mid_split.setHandleWidth(2)
        mid_split.setChildrenCollapsible(False)

        mid_split.addWidget(self._create_video_panel())
        mid_split.addWidget(self._create_lidar_panel())
        mid_split.setSizes([WINDOW_HEIGHT - LIDAR_AREA_HEIGHT - 120, LIDAR_AREA_HEIGHT])

        layout.addWidget(mid_split)
        return panel

    def _create_lidar_panel(self):
        panel = QWidget()
        panel.setMinimumHeight(200)
        layout = QVBoxLayout(panel)
        layout.setContentsMargins(2, 2, 2, 2)
        layout.setSpacing(2)

        grp = QGroupBox("◆ LiDAR 3D 点云")
        gl = QVBoxLayout(grp)
        gl.setContentsMargins(2, 11, 2, 2)

        self.lidar_widget = Lidar3DWidget()
        gl.addWidget(self.lidar_widget)

        layout.addWidget(grp, 1)
        return panel

    def _create_video_panel(self):
        """
        创建视频显示面板
        包含：相机切换 + 拍照按钮 + VTOL切换按钮 + 视频显示
        按钮浮于视频右上角，不影响视频显示
        """
        container = QWidget()
        container.setStyleSheet(f"background-color: {COLOR_BG_MAIN};")

        layout = QVBoxLayout(container)
        layout.setContentsMargins(0, 0, 0, 0)
        layout.setSpacing(0)

        self.video_stack = QStackedWidget()
        self.video_widget_chase = VideoWidget()
        self.video_widget_stereo_left = VideoWidget()
        self.video_widget_stereo_right = VideoWidget()
        self.video_widget_down = VideoWidget()
        self.video_stack.addWidget(self.video_widget_chase)
        self.video_stack.addWidget(self.video_widget_stereo_left)
        self.video_stack.addWidget(self.video_widget_stereo_right)
        self.video_stack.addWidget(self.video_widget_down)
        layout.addWidget(self.video_stack, 1)

        self.cam_switch_btn = QPushButton("第三人称")
        self.cam_switch_btn.setObjectName("btnCamSwitch")
        self.cam_switch_btn.setFixedSize(80, 24)
        self.cam_switch_btn.clicked.connect(self._on_camera_switch)
        self.cam_switch_btn.setParent(container)
        self.cam_switch_btn.raise_()

        self.btn_photo = QPushButton("📷 拍照")
        self.btn_photo.setEnabled(False)
        self.btn_photo.setFixedSize(80, 24)
        self.btn_photo.clicked.connect(self._on_video_photo)
        self.btn_photo.setParent(container)
        self.btn_photo.raise_()

        self.btn_vtol_video = QPushButton("🔄 VTOL切换")
        self.btn_vtol_video.setEnabled(False)
        self.btn_vtol_video.setVisible(False)
        self.btn_vtol_video.setFixedSize(120, 24)
        self.btn_vtol_video.clicked.connect(lambda: self._action("vtol"))
        self.btn_vtol_video.setParent(container)
        self.btn_vtol_video.raise_()

        self._current_camera_idx = 0
        self._camera_names = ["第三人称", "双目左", "双目右", "下视深度"]
        self._camera_keys = ["chase", "stereo_left", "stereo_right", "down"]
        self._camera_btns = [self.btn_photo, self.cam_switch_btn]

        container.resizeEvent = self._on_video_panel_resize

        return container

    def _on_video_panel_resize(self, event):
        w = event.size().width()
        x_offset = 8
        y_offset = 8
        gap = 6
        if self.btn_vtol_video.isVisible():
            self.btn_vtol_video.move(w - x_offset - self.btn_vtol_video.width(), y_offset)
            x_offset += self.btn_vtol_video.width() + gap
        self.btn_photo.move(w - x_offset - self.btn_photo.width(), y_offset)
        x_offset += self.btn_photo.width() + gap
        self.cam_switch_btn.move(w - x_offset - self.cam_switch_btn.width(), y_offset)

    def _on_camera_switch(self):
        self._current_camera_idx = (self._current_camera_idx + 1) % len(self._camera_names)
        self.video_stack.setCurrentIndex(self._current_camera_idx)
        self.cam_switch_btn.setText(self._camera_names[self._current_camera_idx])

    def _switch_to_camera(self, idx):
        if 0 <= idx < len(self._camera_names):
            self._current_camera_idx = idx
            self.video_stack.setCurrentIndex(idx)
            self.cam_switch_btn.setText(self._camera_names[idx])

    def _set_chase_gimbal(self, roll, pitch, yaw):
        if self.control_thread and self.control_thread.isRunning():
            self.control_thread.request_set_chase_gimbal(roll, pitch, yaw)

    def _on_video_photo(self):
        camera_key = self._camera_keys[self._current_camera_idx]
        if camera_key == "stereo_left":
            self._action("photo_stereo_left")
        elif camera_key == "down":
            self._action("photo_down")
        elif camera_key == "chase":
            self._action("photo_chase")
        elif camera_key == "stereo_right":
            self._action("photo_stereo_right")

    def _create_right_sensor_panel(self):
        """
        创建右侧传感器仪表盘（300px，可隐藏）
        包含：传感器标题 + 折叠按钮 + 紧凑2列传感器卡片网格
        """
        panel = QWidget()
        panel.setObjectName("rightSensorPanel")
        layout = QVBoxLayout(panel)
        layout.setContentsMargins(2, 2, 2, 2)
        layout.setSpacing(2)

        title = QLabel("◆ 传感器数据")
        title.setFont(QFont("Microsoft YaHei", 10, QFont.Weight.Bold))
        title.setStyleSheet(f"color: {COLOR_NEON_CYAN}; padding: 2px;")
        layout.addWidget(title)

        self.sensor_panel = SensorPanel()
        layout.addWidget(self.sensor_panel, 1)

        panel.setFixedWidth(RIGHT_PANEL_WIDTH)
        return panel

    def _on_toggle_sensor_panel(self):
        """
        切换右侧传感器面板的显示/隐藏
        """
        if self.right_panel.isVisible():
            self.right_panel.setVisible(False)
            self.btn_toggle_sensor.setText("▶")
            sizes = self.main_split.sizes()
            if len(sizes) >= 2:
                sizes[-1] = 0
                self.main_split.setSizes(sizes)
        else:
            self.right_panel.setVisible(True)
            self.btn_toggle_sensor.setText("◀")
            sizes = self.main_split.sizes()
            if len(sizes) >= 3:
                total = sum(sizes)
                sizes[0] = LEFT_PANEL_WIDTH
                sizes[2] = RIGHT_PANEL_WIDTH
                sizes[1] = total - LEFT_PANEL_WIDTH - RIGHT_PANEL_WIDTH
                self.main_split.setSizes(sizes)

    def focusOutEvent(self, event):
        """窗口失去焦点时清除所有按键状态，防止按键粘滞"""
        self.keys_pressed.clear()
        self._modifier_keys_pressed.clear()
        super().focusOutEvent(event)

    def changeEvent(self, event):
        """
        窗口状态变化事件处理
        捕获WindowDeactivate事件（窗口变为非活动状态），清除按键状态
        比focusOutEvent更可靠：focusOutEvent仅在子控件焦点变化时触发，
        而changeEvent在窗口整体失活时触发（如Win+D、Alt+Tab等）
        """
        if event.type() == event.Type.WindowDeactivate:
            self.keys_pressed.clear()
            self._modifier_keys_pressed.clear()
        super().changeEvent(event)

    def _apply_stylesheet(self):
        """应用全局QSS样式表（深色霓虹科幻风格）"""
        self.setStyleSheet(NEON_STYLESHEET)

    def _update_clock(self):
        """更新标题栏时钟显示（每秒调用一次）"""
        self.time_label.setText(datetime.now().strftime("%Y-%m-%d %H:%M:%S"))

    # ---- 按钮事件 ----

    def _on_start(self):
        """
        启动按钮点击事件处理
        
        两种场景：
        1. 首次启动：创建控制线程并连接仿真环境
        2. 着陆后再次起飞：向已有线程发送起飞请求
        """
        # 场景2：线程已运行且已着陆，请求再次起飞
        if self.control_thread and self.control_thread.isRunning():
            self.control_thread.request_takeoff()
            self._append_log("已发送再次起飞指令", "INFO")
            return

        # 场景1：首次启动
        key = str(self.drone_btn_group.checkedId())
        robot_cfg, model_name, is_vtol = DRONE_MODELS[key]
        mode = "键盘控制" if self.radio_kb.isChecked() else "UDP自动控制"
        sim_cfg_path = os.path.join(os.path.dirname(__file__), "sim_config")

        self.control_thread = DroneControlThread(
            robot_config=robot_cfg, drone_model_name=model_name,
            is_vtol=is_vtol, control_mode=mode, sim_config_path=sim_cfg_path,
            udp_multicast_addr=UDP_MULTICAST_ADDR)

        self.control_thread.log_signal.connect(self._on_log)
        self.control_thread.status_signal.connect(self._on_status)
        self.control_thread.udp_param_signal.connect(self._on_udp)
        # frame_signal已不再使用：相机帧改为拉取模式（_pull_camera_frames定时器）
        # 保留信号定义但不连接，避免控制线程emit时报错
        # self.control_thread.frame_signal.connect(self._on_frame)
        self.control_thread.sensor_data_signal.connect(self._on_sensor_data)
        self.control_thread.finished_signal.connect(self._on_finished)

        self.btn_start.setEnabled(False)
        self.btn_land.setEnabled(True)
        self.btn_exit.setEnabled(True)
        self.btn_photo.setEnabled(True)
        if is_vtol:
            self.btn_vtol_video.setEnabled(True)
            self.btn_vtol_video.setVisible(True)

        for b in self.drone_btn_group.buttons():
            b.setEnabled(False)
        for b in self.mode_btn_group.buttons():
            b.setEnabled(False)

        self.keyboard_timer.start()
        # 启动相机帧拉取定时器（替代frame_signal推送模式）
        self._frame_pull_timer.start()
        self.control_thread.start()
        self._append_log("地面站已启动", "INFO")

    def _on_mode_changed(self, checked):
        """控制模式切换事件：UDP模式显示参数框，键盘模式隐藏"""
        if self.udp_grp:
            self.udp_grp.setVisible(self.radio_udp.isChecked())

    def _on_land(self):
        """着陆按钮点击事件：向控制线程发送着陆请求"""
        if self.control_thread and self.control_thread.isRunning():
            self._append_log(f"点击着陆: 线程运行中, _land_requested={self.control_thread._land_requested}", "INFO")
            self.control_thread.request_land()
            self._append_log("已发送着陆指令", "WARNING")

    def _on_exit(self):
        """退出按钮点击事件：向控制线程发送停止请求，断开连接"""
        if self.control_thread and self.control_thread.isRunning():
            self.control_thread.request_stop()
            self._append_log("正在退出...", "WARNING")

    def _action(self, act):
        """
        发送快捷操作请求到控制线程

        参数：
            act: 操作类型标识
                "photo_stereo_left" - 双目左相机拍照
                "photo_down"        - 下视拍照
                "photo_chase"       - 第三人称拍照
                "photo_stereo_right"- 双目右相机拍照
                "vtol"              - VTOL模式切换
        """
        if not self.control_thread or not self.control_thread.isRunning():
            return
        if act == "photo_front" or act == "photo_stereo_left":
            self.control_thread.request_photo_stereo_left()
        elif act == "photo_down":
            self.control_thread.request_photo_down()
        elif act == "photo_chase":
            self.control_thread.request_photo_chase()
        elif act == "photo_stereo_right":
            self.control_thread.request_photo_stereo_right()
        elif act == "vtol":
            self.control_thread.request_vtol_toggle()

    # ---- 键盘控制 ----

    # 控制键集合：仅拦截这些键，其他键正常传递给子控件
    _CONTROL_KEYS = {
        Qt.Key.Key_W, Qt.Key.Key_S, Qt.Key.Key_A, Qt.Key.Key_D,
        Qt.Key.Key_Up, Qt.Key.Key_Down, Qt.Key.Key_Left, Qt.Key.Key_Right,
        Qt.Key.Key_Plus, Qt.Key.Key_Equal, Qt.Key.Key_Minus,
        Qt.Key.Key_F, Qt.Key.Key_G, Qt.Key.Key_L,
        Qt.Key.Key_T, Qt.Key.Key_V, Qt.Key.Key_Q,
        Qt.Key.Key_1, Qt.Key.Key_2, Qt.Key.Key_3, Qt.Key.Key_4,
        Qt.Key.Key_5, Qt.Key.Key_6, Qt.Key.Key_7, Qt.Key.Key_8, Qt.Key.Key_9,
    }

    # pynput特殊键到Qt.Key的映射（方向键等）
    _PYNPUT_SPECIAL_MAP = {
        PynputKey.up: Qt.Key.Key_Up,
        PynputKey.down: Qt.Key.Key_Down,
        PynputKey.left: Qt.Key.Key_Left,
        PynputKey.right: Qt.Key.Key_Right,
    }

    # pynput修饰键集合：这些键按住时，后续普通按键不触发无人机控制
    _PYNPUT_MODIFIER_KEYS = {
        PynputKey.cmd_l, PynputKey.cmd_r,
        PynputKey.ctrl_l, PynputKey.ctrl_r,
        PynputKey.alt_l, PynputKey.alt_r,
        PynputKey.alt_gr,
    }

    # pynput字母键和符号键到Qt.Key的映射（加减号用KeyCode表示）
    _PYNPUT_CHAR_MAP = {
        'w': Qt.Key.Key_W, 's': Qt.Key.Key_S,
        'a': Qt.Key.Key_A, 'd': Qt.Key.Key_D,
        'f': Qt.Key.Key_F, 'g': Qt.Key.Key_G,
        'l': Qt.Key.Key_L, 't': Qt.Key.Key_T,
        'v': Qt.Key.Key_V, 'q': Qt.Key.Key_Q,
        '+': Qt.Key.Key_Plus, '=': Qt.Key.Key_Equal,
        '-': Qt.Key.Key_Minus,
        '1': Qt.Key.Key_1, '2': Qt.Key.Key_2, '3': Qt.Key.Key_3,
        '4': Qt.Key.Key_4, '5': Qt.Key.Key_5, '6': Qt.Key.Key_6,
        '7': Qt.Key.Key_7, '8': Qt.Key.Key_8, '9': Qt.Key.Key_9,
    }

    def _convert_pynput_key(self, key):
        """
        将pynput的Key/KeyCode转换为Qt.Key
        支持特殊键（方向键等）和字母键的映射
        """
        if isinstance(key, PynputKey):
            return self._PYNPUT_SPECIAL_MAP.get(key)
        elif isinstance(key, PynputKeyCode):
            if key.char:
                return self._PYNPUT_CHAR_MAP.get(key.char.lower())
        return None

    def _on_pynput_key_press(self, key):
        """
        pynput系统级键盘按下回调
        仅在手动模式下，且当前前台窗口为本程序或UE程序时响应键盘事件
        修饰键（Win/Ctrl/Alt）按住时，忽略普通按键，防止Win+D等组合键误触发控制
        """
        if isinstance(key, PynputKey) and key in self._PYNPUT_MODIFIER_KEYS:
            self._modifier_keys_pressed.add(key)
            return
        if not self._is_target_window_active():
            return
        if self._modifier_keys_pressed:
            return
        qt_key = self._convert_pynput_key(key)
        if qt_key is not None:
            self.keys_pressed.add(qt_key)
            self._handle_single_key_action(qt_key)

    def _on_pynput_key_release(self, key):
        """
        pynput系统级键盘释放回调
        始终处理释放事件，不受窗口焦点限制
        同时清除修饰键状态，确保组合键释放后恢复正常控制
        """
        if isinstance(key, PynputKey) and key in self._PYNPUT_MODIFIER_KEYS:
            self._modifier_keys_pressed.discard(key)
        qt_key = self._convert_pynput_key(key)
        if qt_key is not None:
            self.keys_pressed.discard(qt_key)

    def _is_target_window_active(self):
        """
        检查当前前台窗口是否为本程序或UE程序
        仅在手动模式下允许pynput响应键盘事件

        判断逻辑：
        1. 非手动模式（UDP模式）直接返回False，不响应pynput键盘
        2. 手动模式下，检查前台窗口标题是否包含本程序或UE相关关键字
        3. Linux平台使用xdotool/subprocess获取窗口标题（带缓存，避免频繁调用）
        4. Windows平台使用ctypes.windll获取窗口标题

        性能优化：
        - 窗口标题查询结果缓存0.5秒，避免每次按键都调用subprocess
        - subprocess调用开销约5-10ms，如果每次按键都调用会显著增加延迟

        返回：
            bool: True表示应响应键盘事件，False表示忽略
        """
        if self.control_thread and self.control_thread.control_mode != "键盘控制":
            return False
        try:
            import sys
            if sys.platform == "linux":
                # Linux平台：使用xdotool获取活动窗口标题（带0.5秒缓存）
                now = time.monotonic()
                if now - self._last_window_check_time > 0.5:
                    import subprocess
                    result = subprocess.run(
                        ["xdotool", "getactivewindow", "getwindowname"],
                        capture_output=True, text=True, timeout=0.5
                    )
                    self._cached_window_title = result.stdout.strip().lower()
                    self._last_window_check_time = now
                title = self._cached_window_title
            else:
                # Windows平台：使用ctypes获取前台窗口标题
                hwnd = ctypes.windll.user32.GetForegroundWindow()
                if hwnd == 0:
                    return False
                length = ctypes.windll.user32.GetWindowTextLengthW(hwnd)
                if length == 0:
                    return False
                buf = ctypes.create_unicode_buffer(length + 1)
                ctypes.windll.user32.GetWindowTextW(hwnd, buf, length + 1)
                title = buf.value.lower()
            target_keywords = ["地面站", "airsim", "dynamiccity", "eolicpark",
                               "unreal", "ue4", "ue5", "projectairsim", "sim",
                               "ground station"]
            return any(kw in title for kw in target_keywords)
        except Exception:
            # 获取窗口标题失败时，默认允许响应（避免在Linux上pynput完全失效）
            return True

    def _handle_single_key_action(self, qt_key):
        """
        处理单次触发的快捷键动作
        PyQt6的keyPressEvent和pynput回调共用此方法，避免重复代码

        快捷键说明：
        - +/-: 加速/减速
        - ↑: 起飞
        - F: 双目左相机拍照
        - G: 下视相机拍照
        - T: 着陆
        - V: VTOL切换
        - Q: 退出
        - 1/2/3/4: 切换到第三人称/双目左/双目右/下视深度相机
        - 5/6/7/8/9: 追踪相机云台视角切换（前/后/左/右/俯仰重置）
        """
        if qt_key in (Qt.Key.Key_Plus, Qt.Key.Key_Equal) and self.control_thread and self.control_thread.isRunning():
            self.control_thread.request_speed_up()
        elif qt_key == Qt.Key.Key_Minus and self.control_thread and self.control_thread.isRunning():
            self.control_thread.request_speed_down()
        elif qt_key == Qt.Key.Key_Up and self.control_thread and self.control_thread.isRunning():
            self.control_thread.request_takeoff()
        elif qt_key == Qt.Key.Key_F:
            self._action("photo_stereo_left")
        elif qt_key == Qt.Key.Key_G:
            self._action("photo_down")
        elif qt_key == Qt.Key.Key_T:
            self._on_land()
        elif qt_key == Qt.Key.Key_V:
            self._action("vtol")
        elif qt_key == Qt.Key.Key_Q:
            self._on_exit()
        elif qt_key == Qt.Key.Key_1:
            self._switch_to_camera(0)
        elif qt_key == Qt.Key.Key_2:
            self._switch_to_camera(1)
        elif qt_key == Qt.Key.Key_3:
            self._switch_to_camera(2)
        elif qt_key == Qt.Key.Key_4:
            self._switch_to_camera(3)
        elif qt_key == Qt.Key.Key_5:
            self._set_chase_gimbal(0, -15, 0)
        elif qt_key == Qt.Key.Key_6:
            self._set_chase_gimbal(0, -15, 180)
        elif qt_key == Qt.Key.Key_7:
            self._set_chase_gimbal(0, -15, 90)
        elif qt_key == Qt.Key.Key_8:
            self._set_chase_gimbal(0, -15, -90)
        elif qt_key == Qt.Key.Key_9:
            self._set_chase_gimbal(0, -60, 0)

    def eventFilter(self, obj, event):
        """
        全局事件过滤器
        拦截控制键的键盘按下/释放事件，确保即使焦点在子控件上也能响应键盘控制
        非控制键（如文本输入键）正常传递，不影响子控件功能
        修饰键（Ctrl/Alt/Meta）按住时，忽略普通按键，防止组合键误触发控制
        
        参数：
            obj: 事件源对象
            event: 事件对象
        """
        if event.type() == event.Type.KeyPress and event.key() in self._CONTROL_KEYS:
            if event.modifiers() & (Qt.KeyboardModifier.ControlModifier |
                                    Qt.KeyboardModifier.AltModifier |
                                    Qt.KeyboardModifier.MetaModifier):
                return False
            self.keyPressEvent(event)
            return True
        elif event.type() == event.Type.KeyRelease and event.key() in self._CONTROL_KEYS:
            self.keyReleaseEvent(event)
            return True
        return super().eventFilter(obj, event)

    def keyPressEvent(self, event):
        """
        键盘按下事件处理（PyQt6窗口内）
        记录按下的键到集合中，同时处理单次触发的快捷键
        修饰键（Ctrl/Alt/Meta/Win）按住时，忽略普通按键，防止组合键误触发控制
        
        快捷键映射：
        +/-: 加速/减速
        F: 双目左相机拍照
        G: 下视拍照
        T: 着陆
        V: VTOL切换
        Q: 退出
        1/2/3/4: 切换到第三人称/双目左/双目右/下视深度相机
        5/6/7/8/9: 追踪相机云台视角（前/后/左/右/俯视）
        """
        if event.modifiers() & (Qt.KeyboardModifier.ControlModifier |
                                Qt.KeyboardModifier.AltModifier |
                                Qt.KeyboardModifier.MetaModifier):
            super().keyPressEvent(event)
            return
        self.keys_pressed.add(event.key())
        if event.key() in self._CONTROL_KEYS:
            self._handle_single_key_action(event.key())
        super().keyPressEvent(event)

    def keyReleaseEvent(self, event):
        """键盘释放事件处理：从按键集合中移除已释放的键"""
        self.keys_pressed.discard(event.key())
        super().keyReleaseEvent(event)

    def _process_keyboard(self):
        """
        定时处理键盘状态（50ms周期）
        根据当前按下的方向键计算速度分量，发送给控制线程
        
        键盘控制映射：
        W/S: 前进/后退（vx = ±1）
        A/D: 左移/右移（vy = ±1）
        ↑/↓: 上升/下降（vz = ±1，NED坐标系中负值为上升）
        ←/→: 左转/右转（yaw = ±1）
        """
        if not self.control_thread or not self.control_thread.isRunning():
            return
        if self.control_thread.control_mode != "键盘控制":
            return
        if not self._is_target_window_active():
            if self.keys_pressed:
                self.keys_pressed.clear()
                self._modifier_keys_pressed.clear()
            return
        vx = (1 if Qt.Key.Key_W in self.keys_pressed else -1 if Qt.Key.Key_S in self.keys_pressed else 0)
        vy = (1 if Qt.Key.Key_D in self.keys_pressed else -1 if Qt.Key.Key_A in self.keys_pressed else 0)
        vz = (-1 if Qt.Key.Key_Up in self.keys_pressed else 1 if Qt.Key.Key_Down in self.keys_pressed else 0)
        yaw = (-1 if Qt.Key.Key_Left in self.keys_pressed else 1 if Qt.Key.Key_Right in self.keys_pressed else 0)
        self.control_thread.update_keyboard(vx, vy, vz, yaw)

    def _pull_camera_frames(self):
        """
        相机帧拉取定时器回调（15fps）
        从SensorManager的缓存中主动拉取最新帧，更新到VideoWidget

        性能优化说明：
        - 原方案：相机回调→frame_signal.emit()→UI主线程处理
          每帧2.7MB图像跨线程传递，4相机×20fps=80次/秒
        - 新方案：UI定时器主动拉取SensorManager缓存帧
          零跨线程信号开销，刷新频率可控（15fps）
        - 拉取模式还避免了帧积压问题：如果UI处理慢，
          推送模式会积压大量未处理信号，拉取模式只取最新帧
        """
        if not self.control_thread or not self.control_thread.isRunning():
            return
        if not hasattr(self.control_thread, '_sensor_manager') or \
                self.control_thread._sensor_manager is None:
            return

        sm = self.control_thread._sensor_manager

        # 从各相机回调中拉取最新帧并更新到对应的VideoWidget
        # 传感器名称映射到(VideoWidget, 显示标签)：
        # - Chase: 追踪相机（CameraCallback）
        # - DownCamera: 下视相机（CameraCallback）
        # - StereoCamera: 双目相机组（StereoCameraCallback，含左右帧）
        camera_widget_map = {
            "Chase": (self.video_widget_chase, "chase"),
            "DownCamera": (self.video_widget_down, "down"),
        }

        for sensor_name, (widget, camera_key) in camera_widget_map.items():
            callback = sm.get_sensor(sensor_name)
            if callback is not None and hasattr(callback, 'get_latest_frame'):
                frame = callback.get_latest_frame()
                if frame is not None:
                    widget.update_frame(camera_key, frame)

        # 双目相机组单独处理（StereoCameraCallback含左右帧）
        stereo_callback = sm.get_sensor("StereoCamera")
        if stereo_callback is not None and hasattr(stereo_callback, 'get_latest_left_frame'):
            left_frame = stereo_callback.get_latest_left_frame()
            if left_frame is not None:
                self.video_widget_stereo_left.update_frame("stereo_left", left_frame)
            right_frame = stereo_callback.get_latest_right_frame()
            if right_frame is not None:
                self.video_widget_stereo_right.update_frame("stereo_right", right_frame)

        if hasattr(self, 'lidar_widget') and self.lidar_widget is not None:
            lidar_data = self.control_thread.get_latest_lidar_data()
            if lidar_data is not None:
                self.lidar_widget.update_points(lidar_data)

    # ---- 信号处理 ----

    @pyqtSlot(str, str)
    def _on_log(self, msg, level):
        """日志信号处理：将控制线程的日志消息转发到日志显示区域"""
        self._append_log(msg, level)

    @pyqtSlot(str, str)
    def _on_status(self, item, value):
        """
        状态信号处理：根据状态项和值更新界面指示器
        
        状态项说明：
        - "connection": 连接状态（connected/connecting/disconnected）
        - "flight": 飞行状态（idle/taking_off/flying/landing）
        - "speed": 飞行速度（数值字符串）
        - "position": 位置信息（坐标字符串）
        - "vtol": VTOL模式（multirotor/fixedwing）
        """
        if item == "connection":
            if value == "connected":
                self.ind_conn.set_label("已连接")
                self.ind_conn.set_color(COLOR_NEON_GREEN)
            elif value == "connecting":
                self.ind_conn.set_label("连接中...")
                self.ind_conn.set_color(COLOR_NEON_YELLOW)
            else:
                self.ind_conn.set_label("未连接")
                self.ind_conn.set_color(COLOR_NEON_RED)
        elif item == "flight":
            mapping = {
                "idle": ("空闲", COLOR_TEXT_DIM),
                "taking_off": ("起飞中", COLOR_NEON_YELLOW),
                "flying": ("飞行中", COLOR_NEON_GREEN),
                "landing": ("着陆中", COLOR_NEON_ORANGE),
                "landed": ("已着陆", COLOR_NEON_CYAN),
            }
            if value in mapping:
                self.ind_flight.set_label(mapping[value][0])
                self.ind_flight.set_color(mapping[value][1])
            # 着陆完成后启用启动按钮，允许再次起飞
            if value == "landed":
                self.btn_start.setEnabled(True)
                self.btn_land.setEnabled(False)
        elif item == "speed":
            sv = float(value)
            self.speed_bar.setValue(int(sv * 10))
            self.speed_bar.setFormat(f"{sv:.1f} m/s")
            self.lbl_speed.setText(f"当前: {sv:.1f} m/s")
        elif item == "vtol":
            self._append_log(f"VTOL模式: {'多旋翼' if value == 'multirotor' else '固定翼'}", "INFO")

    @pyqtSlot(dict)
    def _on_udp(self, params):
        """
        处理UDP参数更新并更新UI标签
        """
        field_formats = {
            "lon": ".6f",
            "lat": ".6f",
            "alt": ".2f",
            "height": ".2f",
            "theta": ".2f",
            "phi": ".2f",
            "psi": ".2f",
            "Vt": ".2f",
            "vn": ".2f",
            "ve": ".2f",
            "Hdot": ".2f",
            "Vd": ".2f",
        }
        for key, fmt in field_formats.items():
            if key in params and key in self.udp_labels:
                self.udp_labels[key].setText(f"{params[key]:{fmt}}")

    @pyqtSlot(object)
    def _on_frame(self, data):
        """
        视频帧信号处理：根据相机名称分发到对应的视频显示控件
        双目左相机帧 → video_widget_stereo_left
        下视深度相机帧 → video_widget_down
        第三人称帧 → video_widget_chase
        双目右相机帧 → video_widget_stereo_right
        """
        camera_name, frame = data
        if camera_name == "stereo_left":
            self.video_widget_stereo_left.update_frame(camera_name, frame)
        elif camera_name == "down":
            self.video_widget_down.update_frame(camera_name, frame)
        elif camera_name == "chase":
            self.video_widget_chase.update_frame(camera_name, frame)
        elif camera_name == "stereo_right":
            self.video_widget_stereo_right.update_frame(camera_name, frame)

    @pyqtSlot(str, object)
    def _on_sensor_data(self, sensor_name, data):
        """
        传感器数据更新信号处理
        将传感器数据更新到左侧面板的传感器数据区域

        参数：
            sensor_name: 传感器名称（如"IMU1"、"GPS"等）
            data: SensorData对象，包含传感器类型和载荷数据
        """
        try:
            if isinstance(data, SensorData):
                # 优先通过SensorManager获取格式化的显示字段
                # 这样可以使用各传感器回调类中定义的中文标签和格式化数值
                fields = None
                if (self.control_thread is not None
                        and hasattr(self.control_thread, '_sensor_manager')
                        and self.control_thread._sensor_manager is not None):
                    callback = self.control_thread._sensor_manager.get_sensor(sensor_name)
                    if callback is not None:
                        fields = callback.get_display_fields()
                # 回退：直接从payload构建显示字段
                if fields is None:
                    fields = {}
                    for key, value in data.payload.items():
                        if isinstance(value, float):
                            fields[key] = f"{value:.4f}"
                        elif isinstance(value, bool):
                            fields[key] = "有效" if value else "无效"
                        else:
                            fields[key] = str(value)
                self.sensor_panel.update_sensor_data(
                    sensor_name, fields, data.sensor_type
                )
        except Exception:
            pass

    @pyqtSlot(str)
    def _on_finished(self, reason):
        """
        控制线程结束信号处理
        恢复界面到初始状态，允许用户重新启动
        
        恢复内容：
        1. 停止键盘定时器，清空按键状态
        2. 恢复按钮启用/禁用状态
        3. 恢复飞机类型和控制模式的选择能力
        4. 重置状态指示灯为初始状态
        5. 清空控制线程引用
        """
        self.keyboard_timer.stop()
        # 停止相机帧拉取定时器
        self._frame_pull_timer.stop()
        self.keys_pressed.clear()
        self._modifier_keys_pressed.clear()

        # 清除所有视频显示
        self.video_widget_stereo_left.clear_frame()
        self.video_widget_down.clear_frame()
        self.video_widget_chase.clear_frame()
        self.video_widget_stereo_right.clear_frame()

        if hasattr(self, 'lidar_widget') and self.lidar_widget is not None:
            self.lidar_widget.clear_points()

        # 重置传感器数据面板
        self.sensor_panel.reset()

        self.btn_start.setEnabled(True)
        self.btn_land.setEnabled(False)
        self.btn_exit.setEnabled(False)
        self.btn_photo.setEnabled(False)
        self.btn_vtol_video.setEnabled(False)
        self.btn_vtol_video.setVisible(False)

        for b in self.drone_btn_group.buttons():
            b.setEnabled(True)
        for b in self.mode_btn_group.buttons():
            b.setEnabled(True)

        self.ind_conn.set_label("未连接")
        self.ind_conn.set_color(COLOR_NEON_RED)
        self.ind_flight.set_label("空闲")
        self.ind_flight.set_color(COLOR_TEXT_DIM)

        self.control_thread = None
        self._append_log("已退出，可重新启动", "INFO")

    def _append_log(self, message, level="INFO"):
        """
        添加日志消息到日志区域
        带颜色区分：INFO=青色，WARNING=黄色，ERROR=红色
        自动滚动到底部，超过最大条数时删除旧日志
        
        参数：
            message: 日志消息内容
            level: 日志级别（"INFO"/"WARNING"/"ERROR"）
        """
        color_map = {
            "INFO": COLOR_NEON_CYAN,
            "WARNING": COLOR_NEON_YELLOW,
            "ERROR": COLOR_NEON_RED,
        }
        color = color_map.get(level, COLOR_TEXT_MAIN)
        self.log_text.append(f'<span style="color: {color};">{message}</span>')
        # 日志数量限制：超过最大条数时删除最早的日志
        self._log_count += 1
        if self._log_count > self._max_log_count:
            cursor = self.log_text.textCursor()
            cursor.movePosition(cursor.MoveOperation.Start)
            cursor.movePosition(cursor.MoveOperation.Down, cursor.MoveMode.KeepAnchor, 1)
            cursor.removeSelectedText()
            cursor.deleteChar()
            self._log_count -= 1
        self.log_text.verticalScrollBar().setValue(
            self.log_text.verticalScrollBar().maximum()
        )

    def closeEvent(self, event):
        """
        窗口关闭事件处理
        如果控制线程正在运行，先请求停止并等待最多5秒
        停止pynput系统级键盘监听器
        确保资源正确释放后再关闭窗口
        """
        if self.control_thread and self.control_thread.isRunning():
            self.control_thread.request_stop()
            self.control_thread.wait(5000)
        if self._pynput_listener:
            self._pynput_listener.stop()
        event.accept()


# ==============================================================================
# 程序入口
# ==============================================================================

def main():
    """
    程序入口：创建PyQt6应用并启动地面站
    
    初始化流程：
    1. 创建QApplication实例
    2. 设置Fusion风格（最适合自定义QSS）
    3. 配置深色调色板（全局默认颜色）
    4. 创建主窗口并显示
    5. 进入Qt事件循环
    """
    app = QApplication([])
    app.setStyle("Fusion")

    # 设置深色调色板
    palette = QPalette()
    palette.setColor(QPalette.ColorRole.Window, QColor(COLOR_BG_MAIN))
    palette.setColor(QPalette.ColorRole.WindowText, QColor(COLOR_TEXT_MAIN))
    palette.setColor(QPalette.ColorRole.Base, QColor("#0d1117"))
    palette.setColor(QPalette.ColorRole.Text, QColor(COLOR_TEXT_MAIN))
    palette.setColor(QPalette.ColorRole.Button, QColor(COLOR_BG_PANEL))
    palette.setColor(QPalette.ColorRole.ButtonText, QColor(COLOR_TEXT_MAIN))
    palette.setColor(QPalette.ColorRole.Highlight, QColor(COLOR_NEON_CYAN))
    palette.setColor(QPalette.ColorRole.HighlightedText, QColor(COLOR_BG_MAIN))
    app.setPalette(palette)

    window = GroundStationWindow()
    window.show()
    app.exec()


if __name__ == "__main__":
    main()
