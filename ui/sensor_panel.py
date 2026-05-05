"""
UI模块 - 传感器数据面板

本模块实现传感器数据的UI显示面板：
SensorPanel：在左侧面板中显示所有传感器的实时数据

布局设计：
左侧面板中的传感器数据区域，每个传感器一个分组框：
┌─────────────────────┐
│ ◆ IMU 惯性测量单元   │
│ 滚转角: 12.34°      │
│ 俯仰角: -5.67°      │
│ 偏航角: 123.45°     │
├─────────────────────┤
│ ◆ GPS 全球定位       │
│ 纬度: 29.340789°    │
│ 经度: 116.715986°   │
│ 海拔: 25.3m         │
├─────────────────────┤
│ ◆ 无线电高度表       │
│ 高度: 25.27m        │
│ 状态: 有效           │
├─────────────────────┤
│ ◆ 大气机            │
│ 气压高度: 25.1m     │
│ 指示空速: 0.00m/s   │
└─────────────────────┘
"""

from PyQt6.QtWidgets import (
    QWidget, QVBoxLayout, QGroupBox, QGridLayout,
    QLabel, QScrollArea, QFrame
)
from PyQt6.QtGui import QFont
from PyQt6.QtCore import Qt, pyqtSignal, QObject

from sensors.base import SensorType, SensorData
from typing import Dict, Optional


# 颜色常量（与drone_ground_station.py保持一致）
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


# 传感器类型→显示名称和颜色映射
SENSOR_DISPLAY_CONFIG = {
    SensorType.CAMERA: ("相机", COLOR_NEON_CYAN),
    SensorType.DEPTH_CAMERA: ("深度相机", COLOR_NEON_PURPLE),
    SensorType.STEREO_CAMERA: ("双目相机", COLOR_NEON_PURPLE),
    SensorType.LIDAR: ("激光雷达", COLOR_NEON_GREEN),
    SensorType.RADAR: ("毫米波雷达", COLOR_NEON_ORANGE),
    SensorType.IMU: ("惯性测量单元", COLOR_NEON_CYAN),
    SensorType.GPS: ("全球定位", COLOR_NEON_GREEN),
    SensorType.RADIO_ALTIMETER: ("无线电高度表", COLOR_NEON_YELLOW),
    SensorType.LASER_ALTIMETER: ("激光高度表", COLOR_NEON_YELLOW),
    SensorType.ULTRASONIC_ALTIMETER: ("超声波高度表", COLOR_NEON_YELLOW),
    SensorType.BAROMETER: ("大气机", COLOR_NEON_ORANGE),
    SensorType.AIRSPEED: ("空速传感器", COLOR_NEON_ORANGE),
    SensorType.DISTANCE_SENSOR: ("距离传感器", COLOR_NEON_YELLOW),
}


class SensorDataLabel(QLabel):
    """
    传感器数据值标签
    用于显示传感器数据的数值，带颜色高亮
    """

    def __init__(self, text="N/A", color=COLOR_NEON_GREEN, font_size=9):
        super().__init__(text)
        self.setFont(QFont("Consolas", font_size, QFont.Weight.Bold))
        self.setStyleSheet(f"color: {color};")


class SensorGroupBox(QGroupBox):
    """
    传感器分组框
    每个传感器一个分组框，包含标签和数值的网格布局
    """

    def __init__(self, sensor_name: str, sensor_type: SensorType,
                 color: str = COLOR_NEON_CYAN):
        """
        初始化传感器分组框

        参数：
            sensor_name: 传感器显示名称（如"前视相机"、"下视相机"）
            sensor_type: 传感器类型
            color: 显示颜色（十六进制颜色值）
        """
        super().__init__(f"◆ {sensor_name}")
        self.setStyleSheet(f"""
            QGroupBox {{
                background-color: {COLOR_BG_PANEL};
                border: 1px solid {COLOR_BORDER};
                border-radius: 4px;
                margin-top: 6px;
                padding: 4px;
                padding-top: 12px;
                font-size: 10px;
                font-weight: bold;
                color: {color};
            }}
            QGroupBox::title {{
                subcontrol-origin: margin;
                left: 6px;
                padding: 0 3px;
                color: {color};
            }}
        """)
        self._sensor_name = sensor_name
        self._sensor_type = sensor_type
        self._color = color
        self._value_labels: Dict[str, SensorDataLabel] = {}
        self._grid = QGridLayout(self)
        self._grid.setSpacing(2)
        self._grid.setContentsMargins(4, 8, 4, 2)
        self._row = 0

    def add_field(self, label_text: str, initial_value: str = "N/A"):
        """
        添加一个显示字段

        参数：
            label_text: 字段标签文本
            initial_value: 初始值
        """
        lbl = QLabel(label_text)
        lbl.setFont(QFont("Microsoft YaHei", 8))
        lbl.setStyleSheet(f"color: {COLOR_TEXT_SECOND};")
        self._grid.addWidget(lbl, self._row, 0)
        val = SensorDataLabel(initial_value, self._color, 9)
        self._grid.addWidget(val, self._row, 1)
        self._value_labels[label_text] = val
        self._row += 1

    def update_fields(self, fields: Dict[str, str]):
        """
        更新显示字段的值

        参数：
            fields: 字段字典，{标签: 值}
        """
        for label_text, value in fields.items():
            if label_text in self._value_labels:
                self._value_labels[label_text].setText(value)


class SensorPanel(QWidget):
    """
    传感器数据面板
    在左侧面板中显示所有传感器的实时数据

    工作流程：
    1. 初始化时创建所有传感器的分组框
    2. 通过update_sensor_data()方法更新传感器数据
    3. 每个传感器分组框独立更新，互不影响

    信号：
        无（被动更新，由主窗口定时调用update_sensor_data）
    """

    # 传感器名称→显示名称映射
    # 用于将传感器ID转换为友好的显示名称
    SENSOR_NAME_MAP = {
        "IMU1": "IMU 惯性测量单元",
        "GPS": "GPS 全球定位",
        "RadioAltimeter": "无线电高度表",
        "LaserAltimeter": "激光高度表",
        "UltrasonicAltimeter": "超声波高度表",
        "Atmosphere": "大气机",
        "lidar1": "激光雷达",
        "Radar1": "毫米波雷达",
        "StereoCamera": "双目相机（左/右）",
        "FrontCamera": "前视相机（机头）",
        "DownCamera": "下视相机（机腹）",
        "Chase": "第三人称追踪相机",
    }

    # 传感器显示顺序（按sensor_name排列）
    SENSOR_DISPLAY_ORDER = [
        "IMU1",
        "GPS",
        "RadioAltimeter",
        "LaserAltimeter",
        "UltrasonicAltimeter",
        "Atmosphere",
        "lidar1",
        "Radar1",
        "StereoCamera",
        "FrontCamera",
        "DownCamera",
        "Chase",
    ]

    # 传感器名称→预设显示字段映射
    SENSOR_FIELDS_MAP = {
        "IMU1": ["滚转角", "俯仰角", "偏航角", "加速度X", "加速度Y", "加速度Z"],
        "GPS": ["纬度", "经度", "海拔", "地速", "卫星数"],
        "RadioAltimeter": ["高度", "量程", "状态"],
        "LaserAltimeter": ["高度", "量程", "状态"],
        "UltrasonicAltimeter": ["高度", "量程", "状态"],
        "Atmosphere": ["气压高度", "指示空速", "气压"],
        "lidar1": ["点数", "距离范围"],
        "Radar1": ["目标数", "最近距离", "方位角", "仰角"],
        "StereoCamera": ["基线距离", "视差范围", "平均视差"],
        "FrontCamera": ["分辨率", "帧数"],
        "DownCamera": ["分辨率", "帧数"],
        "Chase": ["分辨率", "帧数"],
    }

    def __init__(self, parent=None):
        super().__init__(parent)
        self._sensor_groups: Dict[str, SensorGroupBox] = {}
        self._init_ui()

    def _init_ui(self):
        """初始化UI布局"""
        layout = QVBoxLayout(self)
        layout.setContentsMargins(0, 0, 0, 0)
        layout.setSpacing(2)

        # 标题
        title = QLabel("◆ 传感器数据")
        title.setFont(QFont("Microsoft YaHei", 10, QFont.Weight.Bold))
        title.setStyleSheet(f"color: {COLOR_NEON_CYAN}; padding: 2px;")
        layout.addWidget(title)

        # 滚动区域
        scroll = QScrollArea()
        scroll.setWidgetResizable(True)
        scroll.setFrameShape(QFrame.Shape.NoFrame)
        scroll.setHorizontalScrollBarPolicy(Qt.ScrollBarPolicy.ScrollBarAlwaysOff)
        scroll.setStyleSheet(f"QScrollArea {{ background: transparent; }}")

        scroll_content = QWidget()
        self._scroll_layout = QVBoxLayout(scroll_content)
        self._scroll_layout.setContentsMargins(2, 2, 2, 2)
        self._scroll_layout.setSpacing(2)

        # 预创建所有已知传感器的分组框
        for sensor_name in self.SENSOR_DISPLAY_ORDER:
            display_name = self.SENSOR_NAME_MAP.get(sensor_name, sensor_name)
            # 从SENSOR_DISPLAY_CONFIG获取颜色，需要推断sensor_type
            sensor_type = self._infer_sensor_type(sensor_name)
            display_config = SENSOR_DISPLAY_CONFIG.get(
                sensor_type, ("传感器", COLOR_NEON_CYAN)
            )
            color = display_config[1]
            group = SensorGroupBox(display_name, sensor_type, color)
            # 添加预设字段
            fields = self.SENSOR_FIELDS_MAP.get(sensor_name, [])
            for field_name in fields:
                group.add_field(field_name)
            self._sensor_groups[sensor_name] = group
            # 初始隐藏，有数据时才显示
            group.setVisible(False)
            self._scroll_layout.addWidget(group)

        self._scroll_layout.addStretch()
        scroll.setWidget(scroll_content)
        layout.addWidget(scroll, 1)

    @staticmethod
    def _infer_sensor_type(sensor_name: str) -> Optional[SensorType]:
        """
        根据传感器名称推断传感器类型
        用于获取正确的颜色配置

        参数：
            sensor_name: 传感器名称

        返回：
            SensorType枚举值
        """
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
        """
        更新传感器数据显示

        参数：
            sensor_name: 传感器名称（用于定位分组框）
            data: 显示字段字典，{标签: 值}
            sensor_type: 传感器类型（可选，用于动态创建新分组框）
        """
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
        """
        动态创建传感器分组框
        当收到未知传感器的数据时自动创建

        参数：
            sensor_name: 传感器名称
            sensor_type: 传感器类型

        返回：
            新创建的SensorGroupBox
        """
        display_name = self.SENSOR_NAME_MAP.get(sensor_name, sensor_name)
        display_config = SENSOR_DISPLAY_CONFIG.get(
            sensor_type, ("传感器", COLOR_NEON_CYAN)
        )
        color = display_config[1]
        group = SensorGroupBox(display_name, sensor_type, color)
        # 添加预设字段
        fields = self.SENSOR_FIELDS_MAP.get(sensor_name, [])
        for field_name in fields:
            group.add_field(field_name)
        self._sensor_groups[sensor_name] = group
        # 插入到stretch之前
        count = self._scroll_layout.count()
        self._scroll_layout.insertWidget(count - 1, group)
        return group

    def update_all_sensors(self, all_fields: Dict[str, Dict[str, str]],
                           type_map: Optional[Dict[str, SensorType]] = None):
        """
        批量更新所有传感器数据

        参数：
            all_fields: 所有传感器的显示字段，{传感器名称: {标签: 值}}
            type_map: 传感器名称→类型映射
        """
        if type_map is None:
            type_map = {}

        for sensor_name, fields in all_fields.items():
            sensor_type = type_map.get(sensor_name)
            if sensor_type is not None:
                self.update_sensor_data(sensor_name, fields, sensor_type)

    def reset(self):
        """重置所有传感器数据显示（退出时调用）"""
        for group in self._sensor_groups.values():
            group.setVisible(False)
            for label in group._value_labels.values():
                label.setText("N/A")
