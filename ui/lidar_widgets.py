"""
UI模块 - LiDAR点云3D可视化控件

Lidar3DWidget：3D立体视图，使用matplotlib实现可旋转的3D点云显示

NED坐标系说明：
- X=北（前），Y=东（右），Z=下
- 地面点Z > 0（在无人机下方），天空点Z < 0（在无人机上方）
- 建筑底部Z大，建筑顶部Z小
"""

import numpy as np

from PyQt6.QtWidgets import QWidget, QVBoxLayout
from PyQt6.QtCore import Qt, QTimer
from PyQt6.QtGui import QPainter, QColor, QFont

from core.constants import COLOR_NEON_YELLOW, COLOR_BORDER


class Lidar3DWidget(QWidget):
    """
    LiDAR点云3D可视化控件

    核心特性：
    1. 多帧累积：缓存最近N帧点云，构建稠密3D地图
    2. 高度着色：基于NED坐标系Z轴，建筑按高度分层显色
    3. 深度着色：远处点自动变暗，增强立体感
    4. 降采样：超过阈值时随机降采样，保证渲染流畅
    5. 增量渲染：仅数据变化时重绘，避免空转
    """

    ACCUM_FRAMES = 4
    MAX_DISPLAY_POINTS = 15000
    MAX_RAW_POINTS = 8000
    POINT_SIZE = 4
    DPI = 80
    REDRAW_INTERVAL_MS = 200

    def __init__(self, parent=None):
        super().__init__(parent)
        self.setMinimumSize(200, 160)

        self._accum_buffer = []
        self._all_pts = np.zeros((0, 3))
        self._all_colors = np.zeros((0, 4))
        self._need_redraw = False
        self._is_drawing = False

        try:
            from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg as FigureCanvas
            from matplotlib.figure import Figure
            self._has_mpl = True
        except ImportError:
            self._has_mpl = False

        if self._has_mpl:
            from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg as FigureCanvas
            from matplotlib.figure import Figure
            self.fig = Figure(facecolor='#0d1117', dpi=self.DPI)
            self.canvas = FigureCanvas(self.fig)
            self.ax = self.fig.add_subplot(111, projection='3d')
            self._setup_axes()

            layout = QVBoxLayout(self)
            layout.setContentsMargins(0, 0, 0, 0)
            layout.setSpacing(0)
            layout.addWidget(self.canvas)

            self._timer = QTimer()
            self._timer.timeout.connect(self._redraw)
            self._timer.start(self.REDRAW_INTERVAL_MS)
        else:
            self.setStyleSheet(f"background-color: #0d1117; border: 1px solid {COLOR_BORDER}; border-radius: 4px;")

    def _setup_axes(self):
        self.ax.set_facecolor('#0d1117')
        self.ax.set_axis_off()
        self.ax.view_init(elev=30, azim=-45)
        self.fig.subplots_adjust(left=0, right=1, bottom=0, top=1)

    def update_points(self, lidar_data):
        try:
            if lidar_data and "point_cloud" in lidar_data:
                pts_raw = np.array(lidar_data["point_cloud"])
                if len(pts_raw) == 0 or len(pts_raw) % 3 != 0:
                    return
                pts = pts_raw.reshape(-1, 3)

                mask = ~np.all(pts == 0, axis=1)
                pts = pts[mask]
                if len(pts) == 0:
                    return

                if len(pts) > self.MAX_RAW_POINTS:
                    idx = np.random.choice(len(pts), self.MAX_RAW_POINTS, replace=False)
                    pts = pts[idx]

                colors = self._compute_colors(pts)

                self._accum_buffer.append((pts, colors))
                while len(self._accum_buffer) > self.ACCUM_FRAMES:
                    self._accum_buffer.pop(0)

                self._need_redraw = True
        except Exception:
            pass

    def _compute_colors(self, pts):
        """
        计算点云颜色：基于NED坐标系Z轴高度

        NED坐标系：Z轴向下为正
        - 地面/低处：Z值大（正）→ 蓝色/青色
        - 中等高度：Z值中等 → 绿色
        - 建筑顶部/高处：Z值小（负）→ 红色/橙色

        这样建筑从底到顶形成 蓝→绿→红 的色带，轮廓清晰
        """
        n = len(pts)
        colors = np.zeros((n, 4))

        z_vals = pts[:, 2]

        if n > 10:
            z_low = np.percentile(z_vals, 5)
            z_high = np.percentile(z_vals, 95)
        else:
            z_low = np.min(z_vals)
            z_high = np.max(z_vals)

        z_span = z_high - z_low
        if z_span < 1.0:
            z_span = 1.0

        z_norm = np.clip((z_vals - z_low) / z_span, 0, 1)

        dist_xy = np.sqrt(pts[:, 0]**2 + pts[:, 1]**2)
        dist_max = np.percentile(dist_xy, 95) if n > 10 else 50.0
        dist_max = max(dist_max, 10.0)
        dist_norm = np.clip(dist_xy / dist_max, 0, 1)
        brightness = np.clip(1.0 - 0.3 * dist_norm, 0.3, 1.0)

        # z_norm=0(高处/建筑顶) → 红/橙, z_norm=0.5(中间) → 绿, z_norm=1(低处/地面) → 蓝/青
        colors[:, 0] = np.clip((0.5 - z_norm) * 2, 0, 1) * brightness
        colors[:, 1] = np.clip(1 - np.abs(z_norm - 0.5) * 2, 0, 1) * brightness
        colors[:, 2] = np.clip((z_norm - 0.5) * 2, 0, 1) * brightness
        colors[:, 3] = 0.85

        return colors

    def clear_points(self):
        self._accum_buffer.clear()
        self._all_pts = np.zeros((0, 3))
        self._all_colors = np.zeros((0, 4))
        self._need_redraw = False
        if self._has_mpl:
            self.ax.cla()
            self._setup_axes()
            self.canvas.draw_idle()

    def _redraw(self):
        if not self._has_mpl or self._is_drawing:
            return

        if not self._need_redraw:
            return

        self._need_redraw = False
        self._is_drawing = True

        try:
            all_pts_list = []
            all_colors_list = []
            for idx, (pts, colors) in enumerate(self._accum_buffer):
                age_ratio = (idx + 1) / max(len(self._accum_buffer), 1)
                faded_colors = colors.copy()
                faded_colors[:, 3] = colors[:, 3] * (0.3 + 0.7 * age_ratio)
                all_pts_list.append(pts)
                all_colors_list.append(faded_colors)

            if all_pts_list:
                self._all_pts = np.vstack(all_pts_list)
                self._all_colors = np.vstack(all_colors_list)
            else:
                self._all_pts = np.zeros((0, 3))
                self._all_colors = np.zeros((0, 4))

            self.ax.cla()
            self._setup_axes()

            pts = self._all_pts
            colors = self._all_colors
            n = len(pts)

            if n > 0:
                if n > self.MAX_DISPLAY_POINTS:
                    idx = np.random.choice(n, self.MAX_DISPLAY_POINTS, replace=False)
                    pts = pts[idx]
                    colors = colors[idx]
                    n = len(pts)

                self.ax.scatter(
                    pts[:, 0], pts[:, 1], pts[:, 2],
                    c=colors, s=self.POINT_SIZE,
                    depthshade=True,
                    edgecolors='none',
                    alpha=0.85
                )

                max_range = max(
                    np.abs(pts[:, 0]).max(),
                    np.abs(pts[:, 1]).max(),
                    np.abs(pts[:, 2]).max()
                )
                max_range = max(max_range * 1.2, 15.0)
                max_range = min(max_range, 200.0)
            else:
                max_range = 30

            self.ax.set_xlim(-max_range, max_range)
            self.ax.set_ylim(-max_range, max_range)
            self.ax.set_zlim(-max_range, max_range)

            self.canvas.draw_idle()
        finally:
            self._is_drawing = False

    def paintEvent(self, event):
        if not self._has_mpl:
            painter = QPainter(self)
            painter.fillRect(0, 0, self.width(), self.height(), QColor("#0d1117"))
            painter.setPen(QColor(COLOR_NEON_YELLOW))
            painter.setFont(QFont("Microsoft YaHei", 9))
            painter.drawText(self.rect(), Qt.AlignmentFlag.AlignCenter,
                           "请安装matplotlib:\npip install matplotlib")
            painter.end()
