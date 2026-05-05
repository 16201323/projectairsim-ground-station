"""
传感器模块 - LiDAR回调处理器

本模块实现激光雷达的数据回调处理：
LidarCallback：处理LiDAR点云数据，支持3D可视化和数据保存

LiDAR传感器说明：
- 通过旋转激光束扫描周围环境，获取3D点云数据
- point_cloud字段为一维数组，每3个值为一组(x,y,z)坐标
- NED坐标系：X=北，Y=东，Z=下
"""

import threading
import numpy as np

from .base import SensorCallback, SensorType, SensorData
from typing import Dict, Any, Optional, Callable


class LidarCallback(SensorCallback):
    """
    LiDAR激光雷达回调处理器
    处理LiDAR点云数据，支持3D可视化和快照保存

    工作流程：
    1. 接收LiDAR点云数据（一维数组，每3个值为一组xyz坐标）
    2. 解析为Nx3的numpy数组
    3. 对UI回调数据进行降采样，减少传递到UI的数据量
    4. 缓存完整数据用于快照保存（不降采样）
    5. 计算点云统计信息（点数、距离范围等）

    参数：
        sensor_name: LiDAR名称（如"lidar1"）
        lidar_callback: 点云数据回调函数，格式：callback(lidar_data)
        recorder: DataRecorder实例，用于保存点云快照
    """

    # UI可视化最大显示点数
    # 从8000降至3000，减少UI线程渲染负担
    # 3000点足以呈现3D点云轮廓，过多点数导致matplotlib/open3d渲染卡顿
    MAX_UI_POINTS = 3000

    def __init__(self, sensor_name: str,
                 lidar_callback: Optional[Callable] = None,
                 recorder=None,
                 sensor_data_callback: Optional[Callable] = None):
        super().__init__(sensor_name, SensorType.LIDAR)
        self._lidar_callback = lidar_callback
        self._recorder = recorder
        self._sensor_data_callback = sensor_data_callback
        self._latest_lidar_data: Optional[Dict] = None
        self._lidar_lock = threading.Lock()

    def __call__(self, client, lidar_data) -> None:
        """
        LiDAR数据回调
        当LiDAR传感器数据更新时由AirSim客户端自动调用

        参数：
            client: AirSim客户端对象
            lidar_data: LiDAR传感器数据字典，包含point_cloud字段
        """
        try:
            if lidar_data is not None:
                with self._lidar_lock:
                    self._latest_lidar_data = lidar_data

                if "point_cloud" in lidar_data:
                    pts = np.array(lidar_data["point_cloud"])
                    if len(pts) > 0 and len(pts) % 3 == 0:
                        pts = pts.reshape(-1, 3)
                        dist_xy = np.sqrt(pts[:, 0] ** 2 + pts[:, 1] ** 2)
                        self._update_data({
                            "point_count": len(pts),
                            "dist_min": float(np.min(dist_xy)),
                            "dist_max": float(np.max(dist_xy)),
                            "z_min": float(np.min(pts[:, 2])),
                            "z_max": float(np.max(pts[:, 2])),
                        })
                        # 发送传感器数据到UI面板（节流控制）
                        if self._sensor_data_callback and self._latest_data and self._should_update_ui():
                            self._sensor_data_callback(self._latest_data)

                        # 发送点云数据到3D可视化（节流控制）
                        if self._lidar_callback and self._should_update_ui():
                            ui_data = self._downsample_for_ui(lidar_data, pts)
                            self._lidar_callback(ui_data)
        except Exception:
            pass

    def _downsample_for_ui(self, lidar_data: Dict, pts: np.ndarray) -> Dict:
        """
        为UI可视化降采样点云数据

        保留完整数据用于快照保存，只对发送到UI的数据降采样
        降采样策略：随机采样，保留空间分布均匀性

        参数：
            lidar_data: 原始LiDAR数据字典
            pts: 已解析的Nx3点云数组
        返回：
            降采样后的LiDAR数据字典（浅拷贝，仅point_cloud被替换）
        """
        n = len(pts)
        if n <= self.MAX_UI_POINTS:
            return lidar_data

        idx = np.random.choice(n, self.MAX_UI_POINTS, replace=False)
        sampled_pts = pts[idx]
        ui_data = dict(lidar_data)
        ui_data["point_cloud"] = sampled_pts.flatten().tolist()
        return ui_data

    def get_latest_lidar_data(self) -> Optional[Dict]:
        """
        获取最新的LiDAR数据（线程安全）

        返回：
            最新的LiDAR数据字典，如果没有数据则返回None
        """
        with self._lidar_lock:
            return self._latest_lidar_data

    def save_snapshot(self) -> bool:
        """
        保存LiDAR点云快照
        同时保存为NPY+PCD+LAS三种格式

        返回：
            True表示保存成功，False表示没有有效数据
        """
        if self._recorder is None:
            return False
        with self._lidar_lock:
            ld = self._latest_lidar_data
        if ld is not None:
            self._recorder.save_lidar_point_cloud(ld)
            return True
        return False

    def get_display_fields(self) -> Dict[str, str]:
        """获取UI显示字段：点数和距离范围"""
        if self._latest_data is None:
            return {"点数": "0", "距离范围": "N/A"}
        p = self._latest_data.payload
        return {
            "点数": str(p.get("point_count", 0)),
            "距离范围": f"{p.get('dist_min', 0):.1f}~{p.get('dist_max', 0):.1f}m",
        }
