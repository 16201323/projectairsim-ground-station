"""UI模块 - 传感器数据面板（双列紧凑卡片布局）

布局设计（240px宽，每行两个参数，紧凑无滚动）：
┌──────────────────────┐
│ ◆ IMU                │
│ 滚转 12.34°  俯仰 -5°│
│ 偏航 123.4°  加速X 0  │
│ 加速Y 0.01  加速Z 9.8 │
├──────────────────────┤
│ ◆ GPS                │
│ 纬度 29.34°  经度 116°│
│ 海拔 25.3m   地速 0.0 │
├──────────────────────┤
│ ◆ 无高               │
│ 高度 25.27m  量程 500m│
│ 状态 有效             │
├──────────────────────┤
│ ◆ 大气机             │
│ 气高 25.1m   空速 0.0 │
│ 气压 1013hPa QNH 1013│
│ 差压 0.0Pa            │
└──────────────────────┘
"""

from PyQt6.QtWidgets import (
    QWidget, QVBoxLayout, QGroupBox, QGridLayout,
    QLabel, QScrollArea, QFrame, QSizePolicy,
    QRadioButton, QButtonGroup, QHBoxLayout
)
from PyQt6.QtGui import QFont
from PyQt6.QtCore import Qt, pyqtSignal

from sensors.base import SensorType, SensorData
from typing import Dict, Optional


COLOR_BG_PANEL = "#141b2d"
COLOR_BORDER = "#1e3a5f"
COLOR_NEON_CYAN = "#00d4ff"
COLOR_NEON_GREEN = "#00ff88"
COLOR_NEON_YELLOW = "#ffd700"
COLOR_NEON_RED = "#ff4444"
COLOR_NEON_ORANGE = "#ff8c00"
COLOR_NEON_PURPLE = "#7b2cbf"
COLOR_TEXT_MAIN = "#e0e6ed"
COLOR_TEXT_SECOND = "#8892a0"
COLOR_TEXT_DIM = "#4a5568"

SENSOR_DISPLAY_CONFIG = {
    SensorType.CAMERA: ("相机", COLOR_NEON_CYAN),
    SensorType.DEPTH_CAMERA: ("深相", COLOR_NEON_PURPLE),
    SensorType.STEREO_CAMERA: ("双目", COLOR_NEON_PURPLE),
    SensorType.LIDAR: ("激光雷达", COLOR_NEON_GREEN),
    SensorType.RADAR: ("毫米波雷达", COLOR_NEON_ORANGE),
    SensorType.IMU: ("IMU", COLOR_NEON_CYAN),
    SensorType.GPS: ("GPS", COLOR_NEON_GREEN),
    SensorType.RADIO_ALTIMETER: ("无高", COLOR_NEON_YELLOW),
    SensorType.LASER_ALTIMETER: ("激高", COLOR_NEON_YELLOW),
    SensorType.ULTRASONIC_ALTIMETER: ("超高", COLOR_NEON_YELLOW),
    SensorType.BAROMETER: ("大气机", COLOR_NEON_ORANGE),
    SensorType.AIRSPEED: ("空速", COLOR_NEON_ORANGE),
    SensorType.DISTANCE_SENSOR: ("距离", COLOR_NEON_YELLOW),
}


class SensorGroupBox(QGroupBox):

    def __init__(self, sensor_name: str, sensor_type: SensorType,
                 color: str = COLOR_NEON_CYAN):
        super().__init__(f"◆ {sensor_name}")
        self.setStyleSheet(f"""
            QGroupBox {{
                background-color: {COLOR_BG_PANEL};
                border: 1px solid {COLOR_BORDER};
                border-radius: 4px;
                margin-top: 5px;
                padding: 2px;
                padding-top: 12px;
                font-size: 10px;
                font-weight: bold;
                color: {color};
            }}
            QGroupBox::title {{
                subcontrol-origin: margin;
                left: 5px;
                padding: 0 2px;
                color: {color};
            }}
        """)
        self._sensor_name = sensor_name
        self._sensor_type = sensor_type
        self._color = color
        self._value_labels: Dict[str, QLabel] = {}
        self._grid = QGridLayout(self)
        self._grid.setSpacing(1)
        self._grid.setContentsMargins(3, 6, 3, 2)
        self._row = 0
        self._col = 0
        self._fields_per_row = 2

    def add_field(self, label_text: str, initial_value: str = "N/A"):
        col_offset = self._col * 2
        lbl = QLabel(label_text)
        lbl.setFont(QFont("Microsoft YaHei", 7))
        lbl.setStyleSheet(f"color: {COLOR_TEXT_SECOND}; padding-right: 5px;")
        lbl.setSizePolicy(QSizePolicy.Policy.Maximum, QSizePolicy.Policy.Preferred)
        self._grid.addWidget(lbl, self._row, col_offset)
        val = QLabel(initial_value)
        val.setFont(QFont("Consolas", 8, QFont.Weight.Bold))
        val.setStyleSheet(f"color: {self._color};")
        self._grid.addWidget(val, self._row, col_offset + 1)
        self._value_labels[label_text] = val
        self._col += 1
        if self._col >= self._fields_per_row:
            self._col = 0
            self._row += 1

    def update_fields(self, fields: Dict[str, str]):
        for label_text, value in fields.items():
            if label_text in self._value_labels:
                self._value_labels[label_text].setText(value)


class SensorPanel(QWidget):
    """
    传感器数据面板（双列紧凑卡片）+ 数据发送控制
    适用于右侧面板（240px宽×全高），每行两个参数

    信号：
        nav_udp_start_requested: 组合导航UDP发送启动请求
        nav_udp_stop_requested: 组合导航UDP发送停止请求
        lidar_udp_start_requested: 激光点云UDP发送启动请求
        lidar_udp_stop_requested: 激光点云UDP发送停止请求
    """

    nav_udp_start_requested = pyqtSignal()
    nav_udp_stop_requested = pyqtSignal()
    lidar_udp_start_requested = pyqtSignal()
    lidar_udp_stop_requested = pyqtSignal()

    SENSOR_NAME_MAP = {
        "IMU1": "IMU",
        "GPS": "GPS",
        "RadioAltimeter": "无高",
        "LaserAltimeter": "激高",
        "UltrasonicAltimeter": "超高",
        "Atmosphere": "大气机",
        "lidar1": "激光雷达",
        "Radar1": "毫米波雷达",
        "StereoCamera": "双目",
        "FrontCamera": "前视",
        "DownCamera": "下视",
        "Chase": "追踪",
    }

    SENSOR_DISPLAY_ORDER = [
        "IMU1",
        "GPS",
        "RadioAltimeter",
        "LaserAltimeter",
        "UltrasonicAltimeter",
        "Atmosphere",
        "Radar1",
        "lidar1",
        "StereoCamera",
        "FrontCamera",
        "DownCamera",
        "Chase",
    ]

    SENSOR_FIELDS_MAP = {
        "IMU1": ["滚转", "俯仰", "偏航", "加速X", "加速Y", "加速Z"],
        "GPS": ["纬度", "经度", "海拔", "地速", "定位"],
        "RadioAltimeter": ["高度", "量程", "状态"],
        "LaserAltimeter": ["高度", "量程", "状态"],
        "UltrasonicAltimeter": ["高度", "量程", "状态"],
        "Atmosphere": ["气高", "空速", "气压", "QNH", "差压"],
        "lidar1": ["线数", "测距", "点频", "水平", "垂直", "频率"],
        "Radar1": ["目标", "距离", "方位", "仰角"],
        "StereoCamera": ["左相机", "右相机", "基线", "视差"],
        "FrontCamera": ["分辨率"],
        "DownCamera": ["分辨率"],
        "Chase": ["分辨率"],
    }

    def __init__(self, parent=None):
        super().__init__(parent)
        self._sensor_groups: Dict[str, SensorGroupBox] = {}
        self._init_ui()

    def _init_ui(self):
        layout = QVBoxLayout(self)
        layout.setContentsMargins(0, 0, 0, 0)
        layout.setSpacing(0)

        scroll = QScrollArea()
        scroll.setWidgetResizable(True)
        scroll.setFrameShape(QFrame.Shape.NoFrame)
        scroll.setHorizontalScrollBarPolicy(Qt.ScrollBarPolicy.ScrollBarAlwaysOff)
        scroll.setStyleSheet("QScrollArea { background: transparent; }")

        scroll_content = QWidget()
        scroll_layout = QVBoxLayout(scroll_content)
        scroll_layout.setContentsMargins(0, 0, 0, 0)
        scroll_layout.setSpacing(2)

        for sensor_name in self.SENSOR_DISPLAY_ORDER:
            display_name = self.SENSOR_NAME_MAP.get(sensor_name, sensor_name)
            sensor_type = self._infer_sensor_type(sensor_name)
            display_config = SENSOR_DISPLAY_CONFIG.get(
                sensor_type, ("传感器", COLOR_NEON_CYAN)
            )
            color = display_config[1]
            group = SensorGroupBox(display_name, sensor_type, color)
            fields = self.SENSOR_FIELDS_MAP.get(sensor_name, [])
            for field_name in fields:
                group.add_field(field_name)
            self._sensor_groups[sensor_name] = group
            group.setVisible(False)
            scroll_layout.addWidget(group)

        # 数据发送控制分组框（传感器列表下方）
        send_grp = QGroupBox("◆ 数据发送")
        send_grp.setStyleSheet(f"""
            QGroupBox {{
                background-color: {COLOR_BG_PANEL};
                border: 1px solid {COLOR_BORDER};
                border-radius: 4px;
                margin-top: 5px;
                padding: 2px;
                padding-top: 12px;
                font-size: 10px;
                font-weight: bold;
                color: {COLOR_NEON_CYAN};
            }}
            QGroupBox::title {{
                subcontrol-origin: margin;
                left: 5px;
                padding: 0 2px;
                color: {COLOR_NEON_CYAN};
            }}
            QRadioButton {{
                color: {COLOR_TEXT_MAIN};
                font-size: 9px;
                spacing: 3px;
            }}
            QRadioButton::indicator {{
                width: 10px;
                height: 10px;
            }}
        """)
        send_layout = QVBoxLayout(send_grp)
        send_layout.setSpacing(2)
        send_layout.setContentsMargins(4, 12, 4, 4)

        # 组合导航单选组
        nav_label = QLabel("组合导航(IMU+GPS)")
        nav_label.setFont(QFont("Microsoft YaHei", 8))
        nav_label.setStyleSheet(f"color: {COLOR_TEXT_SECOND};")
        send_layout.addWidget(nav_label)

        nav_row = QHBoxLayout()
        nav_row.setSpacing(8)
        self._nav_btn_group = QButtonGroup()
        self.radio_nav_send = QRadioButton("发送")
        self.radio_nav_stop = QRadioButton("停止")
        self.radio_nav_stop.setChecked(True)  # 默认停止
        self._nav_btn_group.addButton(self.radio_nav_send, 1)
        self._nav_btn_group.addButton(self.radio_nav_stop, 2)
        nav_row.addWidget(self.radio_nav_send)
        nav_row.addWidget(self.radio_nav_stop)
        nav_row.addStretch()
        send_layout.addLayout(nav_row)

        # 激光雷达点云单选组
        lidar_label = QLabel("激光雷达点云")
        lidar_label.setFont(QFont("Microsoft YaHei", 8))
        lidar_label.setStyleSheet(f"color: {COLOR_TEXT_SECOND};")
        send_layout.addWidget(lidar_label)

        lidar_row = QHBoxLayout()
        lidar_row.setSpacing(8)
        self._lidar_btn_group = QButtonGroup()
        self.radio_lidar_send = QRadioButton("发送")
        self.radio_lidar_stop = QRadioButton("停止")
        self.radio_lidar_stop.setChecked(True)  # 默认停止
        self._lidar_btn_group.addButton(self.radio_lidar_send, 1)
        self._lidar_btn_group.addButton(self.radio_lidar_stop, 2)
        lidar_row.addWidget(self.radio_lidar_send)
        lidar_row.addWidget(self.radio_lidar_stop)
        lidar_row.addStretch()
        send_layout.addLayout(lidar_row)

        # 连接单选框信号
        self.radio_nav_send.toggled.connect(self._on_nav_send_toggled)
        self.radio_lidar_send.toggled.connect(self._on_lidar_send_toggled)

        scroll_layout.addWidget(send_grp)

        scroll_layout.addStretch()
        scroll.setWidget(scroll_content)
        layout.addWidget(scroll, 1)
        self._content_layout = scroll_layout

    @staticmethod
    def _infer_sensor_type(sensor_name: str) -> Optional[SensorType]:
        name_map = {
            "IMU1": SensorType.IMU,
            "GPS": SensorType.GPS,
            "RadioAltimeter": SensorType.RADIO_ALTIMETER,
            "LaserAltimeter": SensorType.LASER_ALTIMETER,
            "UltrasonicAltimeter": SensorType.ULTRASONIC_ALTIMETER,
            "Atmosphere": SensorType.BAROMETER,
            "lidar1": SensorType.LIDAR,
            "Radar1": SensorType.RADAR,
            "StereoCamera": SensorType.STEREO_CAMERA,
            "FrontCamera": SensorType.CAMERA,
            "DownCamera": SensorType.CAMERA,
            "Chase": SensorType.CAMERA,
        }
        return name_map.get(sensor_name)

    def update_sensor_data(self, sensor_name: str, data: Dict[str, str],
                           sensor_type: Optional[SensorType] = None):
        if sensor_name in self._sensor_groups:
            group = self._sensor_groups[sensor_name]
            group.update_fields(data)
            group.setVisible(True)
        elif sensor_type is not None:
            group = self._create_sensor_group(sensor_name, sensor_type)
            group.update_fields(data)
            group.setVisible(True)

    def _create_sensor_group(self, sensor_name: str,
                             sensor_type: SensorType) -> SensorGroupBox:
        display_name = self.SENSOR_NAME_MAP.get(sensor_name, sensor_name)
        display_config = SENSOR_DISPLAY_CONFIG.get(
            sensor_type, ("传感器", COLOR_NEON_CYAN)
        )
        color = display_config[1]
        group = SensorGroupBox(display_name, sensor_type, color)
        fields = self.SENSOR_FIELDS_MAP.get(sensor_name, [])
        for field_name in fields:
            group.add_field(field_name)
        self._sensor_groups[sensor_name] = group
        count = self._content_layout.count()
        self._content_layout.insertWidget(count - 1, group)
        return group

    def update_all_sensors(self, all_fields: Dict[str, Dict[str, str]],
                           type_map: Optional[Dict[str, SensorType]] = None):
        if type_map is None:
            type_map = {}
        for sensor_name, fields in all_fields.items():
            sensor_type = type_map.get(sensor_name)
            if sensor_type is not None:
                self.update_sensor_data(sensor_name, fields, sensor_type)

    def _on_nav_send_toggled(self, checked):
        """组合导航单选框切换事件：选中「发送」时发射启动信号，否则发射停止信号"""
        if checked:
            self.nav_udp_start_requested.emit()
        else:
            self.nav_udp_stop_requested.emit()

    def _on_lidar_send_toggled(self, checked):
        """激光点云单选框切换事件：选中「发送」时发射启动信号，否则发射停止信号"""
        if checked:
            self.lidar_udp_start_requested.emit()
        else:
            self.lidar_udp_stop_requested.emit()

    def set_send_controls_enabled(self, enabled: bool):
        """设置数据发送控件的启用/禁用状态（仿真未启动时禁用）"""
        self.radio_nav_send.setEnabled(enabled)
        self.radio_nav_stop.setEnabled(enabled)
        self.radio_lidar_send.setEnabled(enabled)
        self.radio_lidar_stop.setEnabled(enabled)

    def reset_send_controls(self):
        """重置数据发送控件到默认状态（停止）"""
        self.radio_nav_stop.setChecked(True)
        self.radio_lidar_stop.setChecked(True)

    def reset(self):
        for group in self._sensor_groups.values():
            group.setVisible(False)
            for label in group._value_labels.values():
                label.setText("N/A")
