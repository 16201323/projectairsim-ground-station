"""UI模块 - LiDAR点云3D可视化控件 (Open3D嵌入版)

Lidar3DWidget：使用Open3D渲染引擎嵌入PyQt6窗口

处理逻辑与test_lidar_ls120s3.py完全一致：
1. 累积30帧原始点云（环形缓冲区）
2. 合并后随机降采样至200k点（保留建筑整体形态）
3. Z轴强度灰度着色（地面暗→高空亮）

与test脚本的唯一差异：
- test：LidarDisplay独立Open3D窗口
- 本控件：Open3D Visualizer窗口嵌入Qt Widget
"""

import ctypes
from ctypes import wintypes

import numpy as np
import threading
import time as _time

from PyQt6.QtWidgets import QWidget, QVBoxLayout
from PyQt6.QtCore import Qt, QTimer
from PyQt6.QtGui import QPainter, QColor, QFont

from core.constants import COLOR_NEON_YELLOW, COLOR_BORDER


class Lidar3DWidget(QWidget):
    """
    LiDAR点云3D可视化控件（Open3D嵌入版）

    配置与test_lidar_ls120s3.py完全一致：
    - ACCUM_FRAMES = 30
    - MAX_DISPLAY_POINTS = 200000
    - 随机降采样
    - Z轴灰度强度着色
    """

    ACCUM_FRAMES = 30
    MAX_DISPLAY_POINTS = 200000

    def __init__(self, parent=None):
        super().__init__(parent)
        self.setMinimumSize(200, 160)

        self._accum_buffer = []
        self._lock = threading.Lock()
        self._need_redraw = False
        self._hwnd = None
        self._vis = None
        self._view_control = None
        self._has_gl = False
        self._first_geom = True

        try:
            import open3d
            self._has_gl = True
        except ImportError:
            self._has_gl = False

        if self._has_gl:
            self._init_open3d()
            self._timer = QTimer()
            self._timer.timeout.connect(self._redraw)
            self._timer.start(200)

            self._resize_timer = QTimer()
            self._resize_timer.setSingleShot(True)
            self._resize_timer.timeout.connect(self._sync_window_size)
        else:
            self.setStyleSheet(
                f"background-color: #0d1117; border: 1px solid {COLOR_BORDER}; border-radius: 4px;"
            )

    def _init_open3d(self):
        import open3d as o3d
        self._o3d = o3d

        self._vis = o3d.visualization.Visualizer()
        self._vis.create_window(
            window_name="_lidar_embedded_",
            width=640,
            height=360,
            left=0,
            top=0,
            visible=False,
        )

        ctrl = self._vis.get_view_control()
        self._view_control = ctrl
        ctrl.set_zoom(0.15)
        ctrl.set_up([0, 0, -1])

        self._embed_into_qt()

    def _embed_into_qt(self):
        user32 = ctypes.windll.user32
        u32 = user32
        u32.FindWindowW.argtypes = [wintypes.LPCWSTR, wintypes.LPCWSTR]
        u32.FindWindowW.restype = wintypes.HWND
        u32.SetParent.argtypes = [wintypes.HWND, wintypes.HWND]
        u32.SetParent.restype = wintypes.HWND
        u32.GetWindowLongW.argtypes = [wintypes.HWND, ctypes.c_int]
        u32.GetWindowLongW.restype = ctypes.c_long
        u32.SetWindowLongW.argtypes = [wintypes.HWND, ctypes.c_int, ctypes.c_long]
        u32.SetWindowLongW.restype = ctypes.c_long

        GWL_STYLE = -16
        GWL_EXSTYLE = -20
        WS_POPUP = 0x80000000
        WS_CHILD = 0x40000000
        WS_CAPTION = 0x00C00000
        WS_SYSMENU = 0x00080000
        WS_THICKFRAME = 0x00040000
        WS_EX_NOACTIVATE = 0x08000000

        for _ in range(20):
            hwnd = u32.FindWindowW(None, "_lidar_embedded_")
            if not hwnd:
                hwnd = u32.FindWindowW("GLFW30", None)
            if hwnd:
                self._hwnd = hwnd
                qt_hwnd = int(self.winId())
                u32.SetParent(hwnd, wintypes.HWND(qt_hwnd))
                style = u32.GetWindowLongW(hwnd, GWL_STYLE)
                style = (style & ~(WS_POPUP | WS_CAPTION | WS_SYSMENU | WS_THICKFRAME)) | WS_CHILD
                u32.SetWindowLongW(hwnd, GWL_STYLE, style)
                ex_style = u32.GetWindowLongW(hwnd, GWL_EXSTYLE)
                u32.SetWindowLongW(hwnd, GWL_EXSTYLE, ex_style | WS_EX_NOACTIVATE)
                u32.ShowWindow(hwnd, 5)
                return
            _time.sleep(0.05)

    def _sync_window_size(self):
        if not self._hwnd or not self._vis:
            return
        try:
            r = self.rect()
            rw = max(r.width(), 1)
            rh = max(r.height(), 1)
            if rw > 1 and rh > 1:
                ctypes.windll.user32.MoveWindow(self._hwnd, 0, 0, rw, rh, True)
        except Exception:
            pass

    def resizeEvent(self, event):
        super().resizeEvent(event)
        if hasattr(self, '_resize_timer'):
            self._resize_timer.start(100)

    def update_points(self, lidar_data):
        """
        累积原始点云帧——与test_lidar_ls120s3.py的lidar_callback完全一致
        """
        try:
            if lidar_data and "point_cloud" in lidar_data:
                pc = lidar_data["point_cloud"]
                if not pc:
                    return

                pts = np.array(pc, dtype=np.float32).reshape(-1, 3)
                mask = ~np.all(pts == 0, axis=1)
                pts = pts[mask]
                if len(pts) == 0:
                    return

                with self._lock:
                    self._accum_buffer.append(pts)
                    while len(self._accum_buffer) > self.ACCUM_FRAMES:
                        self._accum_buffer.pop(0)

                    self._need_redraw = True
        except Exception:
            pass

    def clear_points(self):
        with self._lock:
            self._accum_buffer.clear()
        self._need_redraw = False
        self._first_geom = True
        if self._vis:
            try:
                self._vis.clear_geometries()
                self._vis.poll_events()
                self._vis.update_renderer()
            except Exception:
                pass

    def _redraw(self):
        """
        合并渲染——与test_lidar_ls120s3.py的get_accumulated_cloud完全一致

        1. 合并30帧
        2. 随机降采样至200k
        3. Z轴强度灰度
        4. 推入Open3D渲染
        """
        if not self._vis or not self._hwnd:
            return

        # 窗口正在关闭或已关闭时不再渲染
        try:
            if not self.isVisible() or self.width() < 10 or self.height() < 10:
                return
        except RuntimeError:
            return

        with self._lock:
            if not self._need_redraw:
                return
            if not self._accum_buffer:
                return
            all_pts = np.concatenate(self._accum_buffer, axis=0)
            self._need_redraw = False

        if len(all_pts) > self.MAX_DISPLAY_POINTS:
            idx = np.random.choice(len(all_pts), self.MAX_DISPLAY_POINTS, replace=False)
            all_pts = all_pts[idx]

        z = all_pts[:, 2]
        z_min, z_max = z.min(), z.max()
        if z_max - z_min < 1e-6:
            intensity = np.ones(len(all_pts))
        else:
            intensity = (z - z_min) / (z_max - z_min)

        colors = np.stack([intensity, intensity, intensity], axis=1)

        try:
            pcd = self._o3d.geometry.PointCloud()
            pcd.points = self._o3d.utility.Vector3dVector(all_pts)
            pcd.colors = self._o3d.utility.Vector3dVector(colors)

            self._vis.clear_geometries()
            self._vis.add_geometry(pcd, reset_bounding_box=self._first_geom)
            self._first_geom = False
            self._vis.poll_events()
            self._vis.update_renderer()
        except Exception:
            pass

    def closeEvent(self, event):
        # 停止定时器，防止断开后仍在回调中操作已销毁的OpenGL上下文
        if hasattr(self, '_timer') and self._timer:
            self._timer.stop()
        if hasattr(self, '_resize_timer') and self._resize_timer:
            self._resize_timer.stop()
        if self._vis:
            try:
                self._vis.destroy_window()
            except Exception:
                pass
            self._vis = None
            self._hwnd = None

    def paintEvent(self, event):
        if not self._has_gl:
            painter = QPainter(self)
            painter.fillRect(0, 0, self.width(), self.height(), QColor("#0d1117"))
            painter.setPen(QColor(COLOR_NEON_YELLOW))
            painter.setFont(QFont("Microsoft YaHei", 9))
            painter.drawText(self.rect(), Qt.AlignmentFlag.AlignCenter,
                           "请安装Open3D:\npip install open3d")
            painter.end()
