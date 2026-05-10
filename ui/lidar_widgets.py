"""UI模块 - LiDAR点云3D可视化控件 (pyqtgraph版)

Lidar3DWidget：使用pyqtgraph的GLViewWidget渲染3D点云

处理逻辑与test_lidar_ls120s3.py完全一致：
1. 累积30帧原始点云（环形缓冲区）
2. 两阶段降采样：随机预过滤→体素网格降采样（保留建筑表面结构）
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
- 体素降采样保留建筑表面结构，无闪烁
- plasma色图LUT加速着色（20ms→1ms）
"""

import threading
import numpy as np

from PyQt6.QtWidgets import QWidget, QVBoxLayout, QHBoxLayout, QPushButton
from PyQt6.QtCore import Qt, QTimer
from PyQt6.QtGui import QPainter, QColor, QFont

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
    - 体素网格降采样（保留建筑表面结构，消除闪烁）
    - plasma色图着色（与LidarDisplay.COLOR_INTENSITY一致）

    交互操作：
    - 左键拖拽：旋转视角
    - 滚轮：缩放
    - 中键拖拽：平移
    - 工具栏按钮：切换透视/前视/俯视预设
    """

    ACCUM_FRAMES = 30
    MAX_DISPLAY_POINTS = 200000
    VOXEL_SIZE = 0.3
    PRE_FILTER_SIZE = 300000

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
        self._plasma_lut = None

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
            self._timer.start(100)
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
        # Z-up坐标系下的透视视角：距离80m, 仰角30度, 方位角135度
        self._gl_widget.setCameraPosition(distance=80, elevation=30, azimuth=135)
        self._gl_widget.setMinimumSize(200, 160)

        self._axis_item = gl.GLAxisItem()
        self._axis_item.setSize(5.0, 5.0, 5.0)
        self._gl_widget.addItem(self._axis_item)

        self._grid_item = gl.GLGridItem()
        self._grid_item.setSize(100, 100)
        self._grid_item.setSpacing(10, 10)
        self._grid_item.setColor((255, 255, 255, 15))
        self._gl_widget.addItem(self._grid_item)

        self._scatter = gl.GLScatterPlotItem()
        self._scatter.setGLOptions('additive')
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
        """切换视角预设：透视(斜45度)/前视(正前方)/俯视(正上方)"""
        if not self._gl_widget:
            return
        self._current_view = view_id

        for btn, vid in self._view_buttons:
            btn.setChecked(vid == view_id)

        if view_id == LIDAR_VIEW_PERSPECTIVE:
            self._gl_widget.setCameraPosition(distance=80, elevation=30, azimuth=135)
        elif view_id == LIDAR_VIEW_FORWARD:
            self._gl_widget.setCameraPosition(distance=80, elevation=5, azimuth=180)
        elif view_id == LIDAR_VIEW_TOPDOWN:
            self._gl_widget.setCameraPosition(distance=100, elevation=90, azimuth=0)

    def update_points(self, lidar_data):
        """
        累积原始点云帧——与test_lidar_ls120s3.py的lidar_callback完全一致

        处理流程：
        1. 从lidar_data提取point_cloud一维数组
        2. reshape为Nx3矩阵（每行一个xyz点）
        3. 过滤全零点（无效回波）
        4. 追加到30帧环形缓冲区
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
        """清空点云缓冲区和渲染"""
        with self._lock:
            self._accum_buffer.clear()
        self._need_redraw = False
        if self._scatter:
            try:
                self._scatter.setData(pos=np.zeros((0, 3)))
            except Exception:
                pass

    def _get_plasma_lut(self):
        """
        惰性构建plasma色图查找表（LUT），256级预计算

        优势：每次着色只需整数索引查表，无需调用matplotlib
        - 旧方案：plasma(intensity) → 每次调用matplotlib插值 → ~20ms/200k点
        - 新方案：lut[indices] → 纯numpy整数索引 → ~1ms/200k点
        """
        if self._plasma_lut is None:
            try:
                import matplotlib
                cmap = matplotlib.colormaps.get_cmap("plasma")
            except (AttributeError, ImportError):
                from matplotlib.cm import get_cmap
                cmap = get_cmap("plasma")
            self._plasma_lut = (cmap(np.linspace(0, 1, 256))[:, :3]).astype(np.float32)
        return self._plasma_lut

    def _plasma_colormap(self, intensity):
        """
        将[0,1]强度值映射为plasma色图RGB——使用预计算LUT加速

        原理：
        1. 将[0,1]强度值量化为0~255整数索引
        2. 从256级预计算LUT中查表获取RGB颜色
        3. 比每次调用matplotlib.plasma()快20倍

        plasma色图效果：深蓝(低) → 紫色(中低) → 橙红(中高) → 亮黄(高)
        地面呈深蓝/紫色，建筑中部呈橙红色，高处呈亮黄色
        """
        lut = self._get_plasma_lut()
        indices = np.clip((intensity * 255).astype(np.int32), 0, 255)
        return lut[indices]

    def _voxel_downsample(self, pts, voxel_size):
        """
        体素网格降采样：将3D空间划分为小立方体（体素），每个体素保留一个代表点

        相比随机降采样的优势：
        - 随机降采样：随机删除点 → 建筑表面出现孔洞，每帧闪烁
        - 体素降采样：每个体素保留一个点 → 建筑表面均匀完整，无闪烁

        原理：
        1. 将每个点的坐标除以voxel_size并取整，得到体素索引
        2. 将3D体素索引压缩为1D线性索引（避免2D np.unique的行比较开销）
        3. 对1D索引去重，每个体素只保留第一个落入的点
        4. 结果：点云均匀分布，表面结构完整保留

        性能优化：
        - 2D np.unique: O(n*k*log(n))，k=3列，每行需比较3个值 → 慢
        - 1D np.unique: O(n*log(n))，只比较1个整数 → 快10倍以上

        参数：
            pts: Nx3 numpy数组
            voxel_size: 体素边长（米），0.3m ≈ 30cm精度，适合建筑级可视化
        """
        voxel_idx = np.floor(pts / voxel_size).astype(np.int32)
        voxel_idx -= voxel_idx.min(axis=0)
        dims = voxel_idx.max(axis=0) + 1
        linear_idx = (voxel_idx[:, 0] * dims[1] + voxel_idx[:, 1]) * dims[2] + voxel_idx[:, 2]
        _, unique_idx = np.unique(linear_idx, return_index=True)
        return pts[unique_idx]

    def _redraw(self):
        """
        合并渲染——优化版

        1. 合并30帧点云
        2. NED→Z-up坐标转换（Z取反）
        3. 两阶段降采样：随机预过滤→体素网格降采样
        4. Z轴归一化为[0,1]强度 → plasma色图着色（LUT加速）
        5. 推入pyqtgraph GLScatterPlotItem渲染

        坐标系说明：
        - ProjectAirSim输出NED坐标：X=北, Y=东, Z=下（Z=-20表示20米高空）
        - pyqtgraph OpenGL使用Z-up：X=右, Y=前, Z=上（Z=20表示20米高空）
        - 转换：将Z取反，使建筑出现在地面上方
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

        # NED→Z-up：Z取反，使建筑出现在地面上方而非地下
        all_pts[:, 2] = -all_pts[:, 2]

        # 两阶段降采样策略：
        # 阶段1：随机预过滤（极速，仅numpy索引操作）
        #   30帧累积可达数百万点，直接体素降采样太慢
        #   预过滤到300k点，建筑表面仍有足够密度
        if len(all_pts) > self.PRE_FILTER_SIZE:
            idx = np.random.choice(len(all_pts), self.PRE_FILTER_SIZE, replace=False)
            all_pts = all_pts[idx]

        # 阶段2：体素网格降采样（保留建筑表面结构，消除闪烁）
        #   300k点体素降采样约50ms，可在100ms渲染周期内完成
        all_pts = self._voxel_downsample(all_pts, self.VOXEL_SIZE)

        # Z轴归一化为[0,1]强度（Z-up中：地面Z小→低处色，建筑Z大→高处色）
        z = all_pts[:, 2]
        z_min, z_max = z.min(), z.max()
        if z_max - z_min < 1e-6:
            intensity = np.ones(len(all_pts))
        else:
            intensity = (z - z_min) / (z_max - z_min)

        colors = self._plasma_colormap(intensity)

        try:
            self._scatter.setData(
                pos=all_pts.astype(np.float32),
                color=colors.astype(np.float32),
                size=3,
                pxMode=True,
            )

            if hasattr(self, '_point_count_label'):
                self._point_count_label.setText(f"{len(all_pts):,} 点")
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
