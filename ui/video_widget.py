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
        """
        更新视频帧（拉取模式优化版）

        优化说明：
        - 不再每帧都copy()，因为拉取模式下UI定时器控制了刷新频率（15fps）
        - 帧数据在paintEvent中才做BGR→RGB转换，避免重复转换
        - 仅当帧确实变化时才触发重绘（update()）

        参数：
            camera_name: 相机标识键（如"chase"、"down"）
            frame: OpenCV BGR格式的图像帧
        """
        if frame is not None:
            label_map = {"front": "前视相机", "down": "下视相机", "chase": "第三人称",
                         "stereo_left": "双目左", "stereo_right": "双目右"}
            self.camera_label = label_map.get(camera_name, "相机")
            # 直接引用帧，不做copy()（拉取模式下刷新频率可控）
            self.current_frame = frame
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
                                      Qt.AspectRatioMode.KeepAspectRatioByExpanding,
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
