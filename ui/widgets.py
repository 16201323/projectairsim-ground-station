"""
UI模块 - 基础控件

本模块包含地面站使用的基础自定义Qt控件：
NeonLabel：霓虹发光标签，用于显示带颜色的参数值
StatusIndicator：圆形发光状态指示灯，用于显示系统连接和飞行状态
"""

from PyQt6.QtWidgets import QLabel, QWidget
from PyQt6.QtCore import Qt
from PyQt6.QtGui import QPainter, QColor, QFont, QRadialGradient, QBrush, QPen

from core.constants import (
    COLOR_NEON_CYAN, COLOR_NEON_RED, COLOR_TEXT_MAIN
)


class NeonLabel(QLabel):
    """
    霓虹发光标签控件
    用于显示带颜色的参数值和状态文字
    支持自定义颜色和字号，常用于UDP参数显示和速度显示
    """

    def __init__(self, text="", color=COLOR_NEON_CYAN, font_size=11, bold=False):
        super().__init__(text)
        self.setFont(QFont("Consolas", font_size, QFont.Weight.Bold if bold else QFont.Weight.Normal))
        self.setStyleSheet(f"color: {color};")


class StatusIndicator(QWidget):
    """
    圆形发光状态指示灯控件
    用于显示系统连接状态、飞行状态等

    视觉效果：
    - 外层：径向渐变发光光晕（半透明扩散）
    - 内层：径向渐变圆形指示灯（高光→主色→暗边）
    - 右侧：状态文字标签

    颜色含义：
    - 绿色(#00ff88)：已连接/飞行中
    - 黄色(#ffd700)：连接中/起飞中
    - 红色(#ff4444)：未连接/错误
    - 灰色(#4a5568)：空闲
    """

    def __init__(self, label="", color=COLOR_NEON_RED, size=12):
        super().__init__()
        self.label_text = label
        self.indicator_color = QColor(color)
        self.indicator_size = size
        self.setFixedHeight(size + 6)
        self.setMinimumWidth(140)

    def set_color(self, color_str):
        self.indicator_color = QColor(color_str)
        self.update()

    def set_label(self, text):
        self.label_text = text
        self.update()

    def paintEvent(self, event):
        painter = QPainter(self)
        painter.setRenderHint(QPainter.RenderHint.Antialiasing)

        # 发光效果
        gr = self.indicator_size + 8
        gradient = QRadialGradient(self.indicator_size / 2 + 3, self.indicator_size / 2 + 3, gr)
        gc = QColor(self.indicator_color)
        gc.setAlpha(50)
        gradient.setColorAt(0, gc)
        gc.setAlpha(0)
        gradient.setColorAt(1, gc)
        painter.setBrush(QBrush(gradient))
        painter.setPen(Qt.PenStyle.NoPen)
        painter.drawEllipse(3, 3, gr, gr)

        # 指示灯主体
        gradient = QRadialGradient(self.indicator_size / 2, self.indicator_size / 2, self.indicator_size / 2)
        gradient.setColorAt(0, self.indicator_color.lighter(160))
        gradient.setColorAt(0.7, self.indicator_color)
        gradient.setColorAt(1, self.indicator_color.darker(150))
        painter.setBrush(QBrush(gradient))
        painter.setPen(QPen(self.indicator_color.darker(120), 1))
        painter.drawEllipse(3, 3, self.indicator_size, self.indicator_size)

        # 文字
        painter.setPen(QColor(COLOR_TEXT_MAIN))
        painter.setFont(QFont("Microsoft YaHei", 9))
        painter.drawText(self.indicator_size + 14, 0,
                         self.width() - self.indicator_size - 14, self.height(),
                         Qt.AlignmentFlag.AlignVCenter | Qt.AlignmentFlag.AlignLeft,
                         self.label_text)
        painter.end()
