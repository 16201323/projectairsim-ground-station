"""
UI模块 - 视频显示控件

本模块实现相机视频流的Qt显示控件：
VideoWidget：接收并显示OpenCV图像帧，支持自动缩放和NO SIGNAL提示
"""

import cv2
import numpy as np

from PyQt6.QtWidgets import QWidget
from PyQt6.QtCore import Qt
from PyQt6.QtGui import QPainter, QColor, QFont, QImage

from core.constants import COLOR_NEON_CYAN, COLOR_TEXT_DIM, COLOR_BORDER


class VideoWidget(QWidget):
    """
    视频显示控件：显示OpenCV图像帧

    功能：
    - 接收并显示前视/下视相机的视频帧
    - 自动缩放图像以适应控件大小（保持宽高比）
    - 无信号时显示"NO SIGNAL"提示
    - 左上角显示当前相机名称标签

    图像处理流程：
    BGR(OpenCV) → RGB(QImage) → 缩放 → 居中绘制
    """

    def __init__(self, parent=None):
        super().__init__(parent)
        self.setMinimumSize(320, 160)
        self.current_frame = None
        self.camera_label = "前视相机"
        self.setStyleSheet(f"background-color: #0d1117; border: 1px solid {COLOR_BORDER}; border-radius: 4px;")

    def update_frame(self, camera_name, frame):
        if frame is not None:
            label_map = {"front": "前视相机", "down": "下视相机", "chase": "第三人称"}
            self.camera_label = label_map.get(camera_name, "相机")
            self.current_frame = frame.copy()
            self.update()

    def clear_frame(self):
        """清除视频帧，恢复NO SIGNAL显示"""
        self.current_frame = None
        self.camera_label = "前视相机"
        self.update()

    def paintEvent(self, event):
        painter = QPainter(self)
        painter.setRenderHint(QPainter.RenderHint.Antialiasing)

        if self.current_frame is not None:
            try:
                rgb = cv2.cvtColor(self.current_frame, cv2.COLOR_BGR2RGB)
                h, w, ch = rgb.shape
                q_img = QImage(rgb.data, w, h, ch * w, QImage.Format.Format_RGB888)
                scaled = q_img.scaled(self.width(), self.height(),
                                      Qt.AspectRatioMode.KeepAspectRatio,
                                      Qt.TransformationMode.SmoothTransformation)
                x = (self.width() - scaled.width()) // 2
                y = (self.height() - scaled.height()) // 2
                painter.drawImage(x, y, scaled)
            except Exception:
                painter.fillRect(0, 0, self.width(), self.height(), QColor("#0d1117"))
        else:
            painter.fillRect(0, 0, self.width(), self.height(), QColor("#0d1117"))
            painter.setPen(QColor(COLOR_TEXT_DIM))
            painter.setFont(QFont("Consolas", 12))
            painter.drawText(0, 0, self.width(), self.height(),
                             Qt.AlignmentFlag.AlignCenter, "NO SIGNAL")

        painter.setPen(QColor(COLOR_NEON_CYAN))
        painter.setFont(QFont("Microsoft YaHei", 8))
        painter.drawText(5, 12, self.camera_label)
        painter.end()
