"""UI模块 - LiDAR点云3D可视化控件 (pyqtgraph版)

Lidar3DWidget：使用pyqtgraph的GLViewWidget渲染3D点云

处理逻辑与test_lidar_ls120s3.py完全一致：
1. 累积30帧原始点云（环形缓冲区）
2. 合并后随机降采样至200k点（保留建筑整体形态）
3. Z轴强度plasma色图着色（与LidarDisplay的COLOR_INTENSITY模式一致）

与test脚本的差异：
- test：LidarDisplay独立Open3D窗口（多进程+mp.Queue）
- 本控件：pyqtgraph GLViewWidget嵌入Qt布局（原生集成，无需Win32 hack）

优势：
- 跨平台兼容（Win11 + Ubuntu 24.04）
- 鼠标交互原生支持（左键旋转/滚轮缩放/中键平移）
- 无需Win32 API窗口嵌入
- 轻量依赖（pyqtgraph ~5MB vs open3d ~200MB）
- 内置坐标轴和参考网格
- 视角预设切换（透视/前视/俯视）
"""

import threading
import numpy as np

from PyQt6.QtWidgets import QWidget, QVBoxLayout, QHBoxLayout, QPushButton
from PyQt6.QtCore import Qt, QTimer
from PyQt6.QtGui import QPainter, QColor, QFont, QVector3D

from core.constants import (
    COLOR_NEON_YELLOW,
    COLOR_BORDER,
    COLOR_NEON_CYAN,
    COLOR_BG_PANEL,
    LIDAR_VIEW_PERSPECTIVE,
    LIDAR_VIEW_FORWARD,
    LIDAR_VIEW_TOPDOWN,
)


class Lidar3DWidget(QWidget):
    """
    LiDAR点云3D可视化控件（pyqtgraph版）

    配置与test_lidar_ls120s3.py完全一致：
    - ACCUM_FRAMES = 30（累积30帧点云）
    - MAX_DISPLAY_POINTS = 200000（降采样上限）
    - 随机降采样（保留建筑整体形态）
    - plasma色图着色（与LidarDisplay.COLOR_INTENSITY一致）

    交互操作：
    - 左键拖拽：旋转视角
    - 滚轮：缩放
    - 中键拖拽：平移
    - 工具栏按钮：切换透视/前视/俯视预设
    """

    ACCUM_FRAMES = 30
    MAX_DISPLAY_POINTS = 200000

    def __init__(self, parent=None):
        super().__init__(parent)
        self.setMinimumSize(200, 160)

        self._accum_buffer = []
        self._lock = threading.Lock()
        self._need_redraw = False
        self._has_gl = False
        self._scatter = None
        self._axis_item = None
        self._grid_item = None
        self._gl_widget = None
        self._current_view = LIDAR_VIEW_PERSPECTIVE
        self._plasma_cmap = None

        try:
            import pyqtgraph.opengl as gl
            self._has_gl = True
            self._gl_module = gl
        except ImportError:
            self._has_gl = False

        if self._has_gl:
            self._init_pyqtgraph()
            self._timer = QTimer()
            self._timer.timeout.connect(self._redraw)
            self._timer.start(200)
        else:
            self.setStyleSheet(
                f"background-color: #0d1117; border: 1px solid {COLOR_BORDER}; border-radius: 4px;"
            )

    def _init_pyqtgraph(self):
        """初始化pyqtgraph OpenGL视图：坐标轴+参考网格+点云散点图+视角工具栏"""
        gl = self._gl_module

        layout = QVBoxLayout(self)
        layout.setContentsMargins(0, 0, 0, 0)
        layout.setSpacing(0)

        self._gl_widget = gl.GLViewWidget()
        self._gl_widget.setBackgroundColor(0, 0, 0)
        # 透视视角：距离80m, 仰角30度, 方位135度（从东北方向看）
        # OpenGL: X=右(NED东), Y=前(NED北), Z=上
        self._gl_widget.setCameraPosition(distance=80, elevation=30, azimuth=135)
        # 旋转中心固定在OpenGL Y轴前方20米处（NED北=前）
        # 前向LiDAR只扫描前方120°，旋转轴心放前方中间避免旋转时后方空白
        self._gl_widget.opts['center'] = QVector3D(0, 20, 0)
        self._gl_widget.setMinimumSize(200, 160)

        self._axis_item = gl.GLAxisItem()
        self._axis_item.setSize(5.0, 5.0, 5.0)
        self._gl_widget.addItem(self._axis_item)

        self._grid_item = gl.GLGridItem()
        self._grid_item.setSize(100, 100)
        self._grid_item.setSpacing(10, 10)
        self._grid_item.setColor((255, 255, 255, 30))
        self._gl_widget.addItem(self._grid_item)

        self._scatter = gl.GLScatterPlotItem()
        self._scatter.setGLOptions('translucent')
        self._gl_widget.addItem(self._scatter)

        toolbar = self._create_view_toolbar()
        layout.addWidget(self._gl_widget, 1)
        layout.addWidget(toolbar)

    def _create_view_toolbar(self):
        """创建视角切换工具栏：透视/前视/俯视按钮 + 点数统计"""
        toolbar = QWidget()
        toolbar.setFixedHeight(28)
        toolbar.setStyleSheet(
            f"background-color: {COLOR_BG_PANEL}; border-top: 1px solid {COLOR_BORDER};"
        )
        tb_layout = QHBoxLayout(toolbar)
        tb_layout.setContentsMargins(4, 0, 4, 0)
        tb_layout.setSpacing(4)

        btn_style = (
            f"QPushButton {{"
            f"  background-color: transparent;"
            f"  color: {COLOR_NEON_CYAN};"
            f"  border: 1px solid {COLOR_BORDER};"
            f"  border-radius: 3px;"
            f"  padding: 1px 8px;"
            f"  font-size: 11px;"
            f"}}"
            f"QPushButton:hover {{"
            f"  background-color: {COLOR_BORDER};"
            f"}}"
            f"QPushButton:checked {{"
            f"  background-color: {COLOR_BORDER};"
            f"  border-color: {COLOR_NEON_CYAN};"
            f"}}"
        )

        views = [
            ("透视", LIDAR_VIEW_PERSPECTIVE),
            ("前视", LIDAR_VIEW_FORWARD),
            ("俯视", LIDAR_VIEW_TOPDOWN),
        ]

        self._view_buttons = []
        for label, view_id in views:
            btn = QPushButton(label)
            btn.setCheckable(True)
            btn.setStyleSheet(btn_style)
            btn.setFixedHeight(22)
            btn.clicked.connect(lambda checked, v=view_id: self._switch_view(v))
            tb_layout.addWidget(btn)
            self._view_buttons.append((btn, view_id))

        self._view_buttons[0][0].setChecked(True)

        tb_layout.addStretch()

        self._point_count_label = QPushButton("0 点")
        self._point_count_label.setStyleSheet(
            f"QPushButton {{"
            f"  background-color: transparent;"
            f"  color: {COLOR_NEON_YELLOW};"
            f"  border: none;"
            f"  font-size: 11px;"
            f"}}"
        )
        self._point_count_label.setEnabled(False)
        tb_layout.addWidget(self._point_count_label)

        return toolbar

    def _switch_view(self, view_id):
        """切换视角预设
        OpenGL坐标系: X=右(NED东), Y=前(NED北), Z=上
        pyqtgraph azimuth: 0=+X(东), 90=+Y(北=前), 180=-X, 270=-Y
        """
        if not self._gl_widget:
            return
        self._current_view = view_id

        for btn, vid in self._view_buttons:
            btn.setChecked(vid == view_id)

        if view_id == LIDAR_VIEW_PERSPECTIVE:
            # 透视视角：斜上方俯视，从东北方向看
            self._gl_widget.setCameraPosition(distance=80, elevation=30, azimuth=135)
        elif view_id == LIDAR_VIEW_FORWARD:
            # 前视视角：沿OpenGL +Y轴(NED北/前)平视
            self._gl_widget.setCameraPosition(distance=80, elevation=5, azimuth=90)
        elif view_id == LIDAR_VIEW_TOPDOWN:
            # 俯视视角：正上方俯视
            self._gl_widget.setCameraPosition(distance=100, elevation=90, azimuth=0)

    def update_points(self, lidar_data):
        """
        累积原始点云帧

        处理流程：
        1. 从lidar_data提取point_cloud一维数组
        2. reshape为Nx3矩阵（每行一个xyz点）
        3. 过滤全零点（无效回波）
        4. 传感器帧→全局NED帧坐标转换（使用pose四元数旋转+位置平移）
        5. 追加到30帧环形缓冲区

        关键修复：
        - point_cloud中的坐标是相对于LiDAR传感器自身的（传感器帧）
        - 必须通过pose中的四元数旋转矩阵+位置偏移转换到全局NED帧
        - 否则无人机移动/旋转后，所有帧的点云都堆积在原点，无法形成城市轮廓
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

                # 传感器帧→全局NED帧坐标转换
                # point_cloud坐标是相对于LiDAR传感器的，无人机移动时每帧从原点出发
                # 必须用pose旋转矩阵+位移转换到全局帧，不同帧的点云才能正确拼接
                pose = lidar_data.get("pose", {})
                orientation = pose.get("orientation", {})
                qw = orientation.get("w", 1.0)
                qx = orientation.get("x", 0.0)
                qy = orientation.get("y", 0.0)
                qz = orientation.get("z", 0.0)
                # 四元数→旋转矩阵（传感器帧→全局NED帧）
                R = np.array([
                    [1-2*(qy*qy+qz*qz), 2*(qx*qy-qz*qw), 2*(qx*qz+qy*qw)],
                    [2*(qx*qy+qz*qw), 1-2*(qx*qx+qz*qz), 2*(qy*qz-qx*qw)],
                    [2*(qx*qz-qy*qw), 2*(qy*qz+qx*qw), 1-2*(qx*qx+qy*qy)]
                ], dtype=np.float32)
                pts = (R @ pts.T).T                      # 旋转：传感器帧指向→全局帧指向
                position = pose.get("position", {})
                pts[:, 0] += position.get("x", 0.0)     # 平移：加上LiDAR全局NED位置
                pts[:, 1] += position.get("y", 0.0)
                pts[:, 2] += position.get("z", 0.0)

                with self._lock:
                    self._accum_buffer.append(pts)
                    while len(self._accum_buffer) > self.ACCUM_FRAMES:
                        self._accum_buffer.pop(0)

                    self._need_redraw = True
        except Exception:
            pass

    def clear_points(self):
        """清空点云缓冲区和渲染"""
        with self._lock:
            self._accum_buffer.clear()
        self._need_redraw = False
        if self._scatter:
            try:
                self._scatter.setData(pos=np.zeros((0, 3)))
            except Exception:
                pass

    def _get_plasma_cmap(self):
        """惰性加载plasma色图，避免启动时导入matplotlib拖慢速度"""
        if self._plasma_cmap is None:
            try:
                import matplotlib
                self._plasma_cmap = matplotlib.colormaps.get_cmap("plasma")
            except (AttributeError, ImportError):
                from matplotlib.cm import get_cmap
                self._plasma_cmap = get_cmap("plasma")
        return self._plasma_cmap

    def _plasma_colormap(self, intensity):
        """
        将[0,1]强度值映射为plasma色图RGB——与LidarDisplay完全一致

        LidarDisplay使用matplotlib的plasma色图：
        intensity → np.interp → plasma RGB三通道

        plasma色图效果：深蓝(低) → 紫色(中低) → 橙红(中高) → 亮黄(高)
        地面呈深蓝/紫色，建筑中部呈橙红色，高处呈亮黄色
        """
        plasma = self._get_plasma_cmap()
        mapped = plasma(intensity)
        return mapped[:, :3]

    def _redraw(self):
        """
        合并渲染——修复版：完整坐标系转换

        1. 合并30帧全局NED点云
        2. 随机降采样至200k点
        3. 完整NED→OpenGL坐标转换（XY交换+Z取反）
        4. Z轴归一化为[0,1]强度 → plasma色图着色
        5. 推入pyqtgraph GLScatterPlotItem渲染

        坐标系转换说明：
        ┌────────────┬───────────┬──────────┐
        │   坐标系    │    X      │    Y    │    Z      │
        ├────────────┼───────────┼─────────┼──────────┤
        │ NED(全局)  │ 北(前)    │ 东(右)  │ 下        │
        │ OpenGL     │ 右        │ 前      │ 上        │
        │ 转换公式   │ NED_Y     │ NED_X   │ -NED_Z   │
        └────────────┴───────────┴─────────┴──────────┘
        """
        if not self._scatter or not self._gl_widget:
            return

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

        # 完整NED→OpenGL坐标映射：XY交换 + Z取反
        # NED: X=北(前), Y=东(右), Z=下
        # OpenGL: X=右, Y=前, Z=上
        # 转换：gl_x = ned_y, gl_y = ned_x, gl_z = -ned_z
        ned_x = all_pts[:, 0]
        ned_y = all_pts[:, 1]
        ned_z = all_pts[:, 2]
        gl_pts = np.empty_like(all_pts)
        gl_pts[:, 0] = ned_y                     # NED东→OpenGL右
        gl_pts[:, 1] = ned_x                     # NED北→OpenGL前
        gl_pts[:, 2] = -ned_z                    # NED下→OpenGL上

        # Z轴归一化为[0,1]强度（OpenGL中：地面Z≈0→低处色，建筑Z大→高处色）
        z = gl_pts[:, 2]
        z_min, z_max = z.min(), z.max()
        if z_max - z_min < 1e-6:
            intensity = np.ones(len(gl_pts))
        else:
            intensity = (z - z_min) / (z_max - z_min)

        colors = self._plasma_colormap(intensity)

        try:
            self._scatter.setData(
                pos=gl_pts.astype(np.float32),
                color=colors.astype(np.float32),
                size=2,
                pxMode=True,
            )

            if hasattr(self, '_point_count_label'):
                self._point_count_label.setText(f"{len(gl_pts):,} 点")
        except Exception:
            pass

    def closeEvent(self, event):
        """停止定时器并清空渲染，防止关闭后仍在操作OpenGL上下文"""
        if hasattr(self, '_timer') and self._timer:
            self._timer.stop()
        if self._scatter:
            try:
                self._scatter.setData(pos=np.zeros((0, 3)))
            except Exception:
                pass

    def paintEvent(self, event):
        """pyqtgraph不可用时的降级绘制：显示安装提示"""
        if not self._has_gl:
            painter = QPainter(self)
            painter.fillRect(0, 0, self.width(), self.height(), QColor("#0d1117"))
            painter.setPen(QColor(COLOR_NEON_YELLOW))
            painter.setFont(QFont("Microsoft YaHei", 9))
            painter.drawText(self.rect(), Qt.AlignmentFlag.AlignCenter,
                           "请安装pyqtgraph:\npip install pyqtgraph")
            painter.end()
