"""
Project AirSim 地面站控制界面
基于PyQt6框架，深色霓虹科幻风格
提供飞机类型选择、控制模式切换、启动/着陆/退出操作、
实时日志显示、UDP参数监控、相机视频流显示、LiDAR点云可视化等功能。

功能概述：
1. 深色霓虹科幻风格UI界面
2. 支持四旋翼/六旋翼/倾斜旋翼(VTOL)三种无人机型号选择
3. 键盘手动控制 / UDP自动控制双模式
4. 启动/着陆/退出按钮操作
5. 实时流动日志显示（带颜色区分）
6. UDP模式下完整参数实时显示
7. 前视/下视相机视频流切换显示
8. LiDAR点云2D/3D可视化
9. 传感器数据面板（IMU/GPS/高度表/大气机/雷达等）
10. 退出后可重新启动
11. UDP超时自动悬停保护
"""

import ctypes
import os
import threading
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
    WINDOW_WIDTH, WINDOW_HEIGHT, LEFT_PANEL_WIDTH, BOTTOM_PANEL_HEIGHT,
)
from core.control_thread import DroneControlThread
from ui.widgets import NeonLabel, StatusIndicator
from ui.lidar_widgets import Lidar3DWidget
from ui.video_widget import VideoWidget
from ui.sensor_panel import SensorPanel
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
    │  UDP参数   ├──────────────────────────┬─────────────────────────────────────┤
    │  传感器    │      运行日志             │       LiDAR 3D [快照]              │
    └────────────┴──────────────────────────┴─────────────────────────────────────┘
    
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
        self.keyboard_timer.setInterval(50)

    def _init_ui(self):
        """
        初始化UI布局
        布局结构：
        ┌────────────┬────────────────────────────────────────────────────────────────┐
        │  ◆ LOGO  [启动][着陆][退出]  |  [连接][飞行][位置]  |  时钟                │
        ├────────────┼────────────────────────────────────────────────────────────────┤
        │            │                    视频显示区域                                │
        │  左侧面板  │  [前视/下视切换] [拍照] [VTOL切换]                              │
        │  飞机类型  │  ┌──────────────────────────────────────────────────────────┐  │
        │  控制模式  │  │                    相机视频                              │  │
        │  飞行速度  │  └──────────────────────────────────────────────────────────┘  │
        │  UDP参数   ├──────────────────────────┬─────────────────────────────────────┤
        │  传感器    │      运行日志             │       LiDAR 3D [快照]              │
        └────────────┴──────────────────────────┴─────────────────────────────────────┘
        """
        central = QWidget()
        self.setCentralWidget(central)
        main_layout = QVBoxLayout(central)
        main_layout.setContentsMargins(4, 4, 4, 4)
        main_layout.setSpacing(4)

        main_layout.addWidget(self._create_title_bar())

        # 右侧区域：上方视频 + 下方(日志 | LiDAR 3D)
        right_split = QSplitter(Qt.Orientation.Vertical)
        right_split.addWidget(self._create_video_panel())
        right_bottom = self._create_bottom_panel()
        right_split.addWidget(right_bottom)
        right_split.setSizes([WINDOW_HEIGHT - BOTTOM_PANEL_HEIGHT - 50, BOTTOM_PANEL_HEIGHT])

        # 主水平分割器：左侧面板(贯穿全高) | 右侧区域
        main_split = QSplitter(Qt.Orientation.Horizontal)
        main_split.addWidget(self._create_left_panel())
        main_split.addWidget(right_split)
        main_split.setSizes([LEFT_PANEL_WIDTH, WINDOW_WIDTH - LEFT_PANEL_WIDTH])

        main_layout.addWidget(main_split, 1)

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

        layout.addSpacing(1)

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
        创建左侧控制面板（固定宽度240px）
        包含：飞机类型、控制模式、飞行速度、UDP参数
        操作按钮已移至顶部标题栏
        """
        panel = QWidget()
        panel.setFixedWidth(LEFT_PANEL_WIDTH)
        layout = QVBoxLayout(panel)
        layout.setContentsMargins(2, 2, 2, 2)
        layout.setSpacing(2)

        scroll = QScrollArea()
        scroll.setWidgetResizable(True)
        scroll.setFrameShape(QFrame.Shape.NoFrame)
        scroll.setHorizontalScrollBarPolicy(Qt.ScrollBarPolicy.ScrollBarAlwaysOff)
        scroll_content = QWidget()
        scroll_layout = QVBoxLayout(scroll_content)
        scroll_layout.setContentsMargins(2, 2, 2, 2)
        scroll_layout.setSpacing(2)

        # 飞机类型
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
        scroll_layout.addWidget(drone_grp)

        # 控制模式
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
        scroll_layout.addWidget(mode_grp)

        # 飞行速度
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
        scroll_layout.addWidget(spd_grp)

        # UDP控制参数（仅在UDP模式下可见）
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
        scroll_layout.addWidget(self.udp_grp)

        # 传感器数据面板（显示IMU/GPS/高度表/大气机/雷达等传感器数据）
        self.sensor_panel = SensorPanel()
        scroll_layout.addWidget(self.sensor_panel, 1)

        scroll.setWidget(scroll_content)
        layout.addWidget(scroll)
        return panel

    def _create_video_panel(self):
        """
        创建视频显示面板（中间右侧主区域）
        包含：相机切换 + 拍照按钮 + VTOL切换按钮 + 视频显示
        按钮浮于视频右上角，不影响视频显示
        """
        container = QWidget()
        container.setStyleSheet(f"background-color: {COLOR_BG_MAIN};")

        # 使用绝对定位实现按钮浮于视频右上角
        layout = QVBoxLayout(container)
        layout.setContentsMargins(0, 0, 0, 0)
        layout.setSpacing(0)

        # 视频显示（使用QStackedWidget切换第三人称/双目左/双目右/下视深度）
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

        # 悬浮按钮（绝对定位在右上角）
        # 相机切换按钮
        self.cam_switch_btn = QPushButton("第三人称")
        self.cam_switch_btn.setObjectName("btnCamSwitch")
        self.cam_switch_btn.setFixedSize(80, 24)
        self.cam_switch_btn.clicked.connect(self._on_camera_switch)
        self.cam_switch_btn.setParent(container)
        self.cam_switch_btn.raise_()

        # 拍照按钮
        self.btn_photo = QPushButton("📷 拍照")
        self.btn_photo.setEnabled(False)
        self.btn_photo.setFixedSize(80, 24)
        self.btn_photo.clicked.connect(self._on_video_photo)
        self.btn_photo.setParent(container)
        self.btn_photo.raise_()

        # VTOL切换按钮（仅倾斜旋翼时可见）
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
        self._camera_btns = [self.btn_photo, self.cam_switch_btn]  # 快照和切换按钮

        # 监听容器大小变化，重新定位按钮
        container.resizeEvent = self._on_video_panel_resize

        return container

    def _on_video_panel_resize(self, event):
        """视频面板大小变化时，重新定位右上角悬浮按钮"""
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
        """切换第三人称→双目左→双目右→下视深度相机显示"""
        self._current_camera_idx = (self._current_camera_idx + 1) % len(self._camera_names)
        self.video_stack.setCurrentIndex(self._current_camera_idx)
        self.cam_switch_btn.setText(self._camera_names[self._current_camera_idx])

    def _switch_to_camera(self, idx):
        """快捷键切换到指定索引的相机视图"""
        if 0 <= idx < len(self._camera_names):
            self._current_camera_idx = idx
            self.video_stack.setCurrentIndex(idx)
            self.cam_switch_btn.setText(self._camera_names[idx])

    def _set_chase_gimbal(self, roll, pitch, yaw):
        """设置追踪相机云台角度（度）"""
        if self.control_thread and self.control_thread.isRunning():
            self.control_thread.request_set_chase_gimbal(roll, pitch, yaw)

    def _on_video_photo(self):
        """视频区域拍照按钮：根据当前显示的相机拍照"""
        camera_key = self._camera_keys[self._current_camera_idx]
        if camera_key == "stereo_left":
            self._action("photo_stereo_left")
        elif camera_key == "down":
            self._action("photo_down")
        elif camera_key == "chase":
            self._action("photo_chase")
        elif camera_key == "stereo_right":
            self._action("photo_stereo_right")

    def _create_bottom_panel(self):
        """
        创建右侧下方面板
        两列布局：运行日志 | LiDAR 3D（平分宽度）
        LiDAR快照按钮浮于右上角
        """
        panel = QWidget()
        layout = QHBoxLayout(panel)
        layout.setContentsMargins(2, 2, 2, 2)
        layout.setSpacing(4)

        # 左列：运行日志
        log_grp = QGroupBox("◆ 运行日志")
        ll = QVBoxLayout(log_grp)
        ll.setContentsMargins(2, 11, 2, 2)
        self.log_text = QTextEdit()
        self.log_text.setReadOnly(True)
        self._log_count = 0
        self._max_log_count = 1000
        ll.addWidget(self.log_text)
        layout.addWidget(log_grp, 1)

        # 右列：LiDAR 3D（快照按钮浮于右上角）
        lid3d_grp = QGroupBox("◆ LiDAR 3D")
        l3l = QVBoxLayout(lid3d_grp)
        l3l.setContentsMargins(2, 11, 2, 2)
        self.lidar3d_widget = Lidar3DWidget()
        l3l.addWidget(self.lidar3d_widget, 1)
        self.btn_lidar3d_snap = QPushButton("3D快照")
        self.btn_lidar3d_snap.setEnabled(False)
        self.btn_lidar3d_snap.clicked.connect(lambda: self._action("lidar"))
        self.btn_lidar3d_snap.setFixedSize(80, 18)
        self.btn_lidar3d_snap.setParent(lid3d_grp)
        self.btn_lidar3d_snap.raise_()
        self._lid3d_grp = lid3d_grp
        lid3d_grp.resizeEvent = self._on_lid3d_grp_resize
        layout.addWidget(lid3d_grp, 1)

        return panel

    def _on_lid3d_grp_resize(self, event):
        """LiDAR 3D分组框大小变化时，重新定位右上角快照按钮"""
        w = event.size().width()
        self.btn_lidar3d_snap.move(w - self.btn_lidar3d_snap.width() - 46, 40)

    def focusOutEvent(self, event):
        """窗口失去焦点时清除所有按键状态，防止按键粘滞"""
        self.keys_pressed.clear()
        super().focusOutEvent(event)

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
        self.control_thread.frame_signal.connect(self._on_frame)
        self.control_thread.lidar_signal.connect(self._on_lidar)
        self.control_thread.sensor_data_signal.connect(self._on_sensor_data)
        self.control_thread.finished_signal.connect(self._on_finished)

        self.btn_start.setEnabled(False)
        self.btn_land.setEnabled(True)
        self.btn_exit.setEnabled(True)
        self.btn_photo.setEnabled(True)
        self.btn_lidar3d_snap.setEnabled(True)
        if is_vtol:
            self.btn_vtol_video.setEnabled(True)
            self.btn_vtol_video.setVisible(True)

        for b in self.drone_btn_group.buttons():
            b.setEnabled(False)
        for b in self.mode_btn_group.buttons():
            b.setEnabled(False)

        self.keyboard_timer.start()
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
                "lidar"             - LiDAR快照
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
        elif act == "lidar":
            self.control_thread.request_lidar_snapshot()
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
        其他窗口下的键盘输入将被忽略，避免干扰其他应用
        """
        if not self._is_target_window_active():
            return
        qt_key = self._convert_pynput_key(key)
        if qt_key is not None:
            self.keys_pressed.add(qt_key)
            self._handle_single_key_action(qt_key)

    def _on_pynput_key_release(self, key):
        """
        pynput系统级键盘释放回调
        仅在手动模式下，且当前前台窗口为本程序或UE程序时响应
        """
        if not self._is_target_window_active():
            return
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

        返回：
            bool: True表示应响应键盘事件，False表示忽略
        """
        if self.control_thread and self.control_thread.control_mode != "键盘控制":
            return False
        try:
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
                               "unreal", "ue4", "ue5", "projectairsim", "sim"]
            return any(kw in title for kw in target_keywords)
        except Exception:
            return False

    def _handle_single_key_action(self, qt_key):
        """
        处理单次触发的快捷键动作
        PyQt6的keyPressEvent和pynput回调共用此方法，避免重复代码

        快捷键说明：
        - +/-: 加速/减速
        - ↑: 起飞
        - F: 双目左相机拍照
        - G: 下视相机拍照
        - L: LiDAR快照
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
        elif qt_key == Qt.Key.Key_L:
            self._action("lidar")
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
        
        参数：
            obj: 事件源对象
            event: 事件对象
        """
        if event.type() == event.Type.KeyPress and event.key() in self._CONTROL_KEYS:
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
        
        快捷键映射：
        +/-: 加速/减速
        F: 双目左相机拍照
        G: 下视拍照
        L: LiDAR快照
        T: 着陆
        V: VTOL切换
        Q: 退出
        1/2/3/4: 切换到第三人称/双目左/双目右/下视深度相机
        5/6/7/8/9: 追踪相机云台视角（前/后/左/右/俯视）
        """
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
        vx = (1 if Qt.Key.Key_W in self.keys_pressed else -1 if Qt.Key.Key_S in self.keys_pressed else 0)
        vy = (1 if Qt.Key.Key_D in self.keys_pressed else -1 if Qt.Key.Key_A in self.keys_pressed else 0)
        vz = (-1 if Qt.Key.Key_Up in self.keys_pressed else 1 if Qt.Key.Key_Down in self.keys_pressed else 0)
        yaw = (-1 if Qt.Key.Key_Left in self.keys_pressed else 1 if Qt.Key.Key_Right in self.keys_pressed else 0)
        self.control_thread.update_keyboard(vx, vy, vz, yaw)

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

    @pyqtSlot(object)
    def _on_lidar(self, lidar_data):
        """LiDAR数据信号处理：更新3D点云可视化控件"""
        self.lidar3d_widget.update_points(lidar_data)

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
        self.keys_pressed.clear()

        # 清除所有视频和LiDAR显示
        self.video_widget_stereo_left.clear_frame()
        self.video_widget_down.clear_frame()
        self.video_widget_chase.clear_frame()
        self.video_widget_stereo_right.clear_frame()
        self.lidar3d_widget.clear_points()

        # 重置传感器数据面板
        self.sensor_panel.reset()

        self.btn_start.setEnabled(True)
        self.btn_land.setEnabled(False)
        self.btn_exit.setEnabled(False)
        self.btn_photo.setEnabled(False)
        self.btn_lidar3d_snap.setEnabled(False)
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
