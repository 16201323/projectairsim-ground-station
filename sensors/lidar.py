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

    MAX_UI_POINTS = 6000

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
        self._config: Dict[str, Any] = {}

    def __call__(self, client, lidar_data) -> None:
        """
        LiDAR数据回调

        性能优化：先检查节流，避免不必要的numpy数组操作
        LiDAR点云数据处理（reshape、距离计算等）是CPU密集型操作，
        在节流间隔内跳过可显著减少CPU占用
        """
        try:
            if lidar_data is not None:
                if not self._should_update_ui():
                    with self._lidar_lock:
                        self._latest_lidar_data = lidar_data
                    return

                with self._lidar_lock:
                    self._latest_lidar_data = lidar_data

                if "point_cloud" in lidar_data:
                    pts = np.array(lidar_data["point_cloud"])
                    if len(pts) > 0 and len(pts) % 3 == 0:
                        pts = pts.reshape(-1, 3)
                        dist_xy = np.sqrt(pts[:, 0] ** 2 + pts[:, 1] ** 2)
                        dist_3d = np.sqrt(pts[:, 0] ** 2 + pts[:, 1] ** 2 + pts[:, 2] ** 2)
                        self._update_data({
                            "point_count": len(pts),
                            "dist_min": float(np.min(dist_3d)),
                            "dist_max": float(np.max(dist_3d)),
                            "dist_xy_min": float(np.min(dist_xy)),
                            "dist_xy_max": float(np.max(dist_xy)),
                            "z_min": float(np.min(pts[:, 2])),
                            "z_max": float(np.max(pts[:, 2])),
                        })

                        if self._sensor_data_callback and self._latest_data:
                            self._sensor_data_callback(self._latest_data)

                        if self._lidar_callback:
                            ui_data = self._downsample_for_ui(lidar_data, pts)
                            self._lidar_callback(ui_data)
        except Exception:
            pass

    def _downsample_for_ui(self, lidar_data: Dict, pts: np.ndarray) -> Dict:
        n = len(pts)
        if n <= self.MAX_UI_POINTS:
            return lidar_data

        idx = np.random.choice(n, self.MAX_UI_POINTS, replace=False)
        sampled_pts = pts[idx]
        ui_data = dict(lidar_data)
        ui_data["point_cloud"] = sampled_pts.flatten().tolist()
        return ui_data

    def get_latest_lidar_data(self) -> Optional[Dict]:
        with self._lidar_lock:
            return self._latest_lidar_data

    def set_config(self, config: Dict[str, Any]):
        """
        设置激光雷达的固定配置参数（从JSONC配置文件读取）

        参数：
            config: 配置字典，包含以下字段：
                - number-of-channels: 线数
                - range: 测距范围(m)
                - points-per-second: 点频
                - horizontal-rotation-frequency: 旋转频率(Hz)
                - horizontal-fov-start-deg: 水平视场起始角(°)
                - horizontal-fov-end-deg: 水平视场结束角(°)
                - vertical-fov-upper-deg: 垂直视场上限(°)
                - vertical-fov-lower-deg: 垂直视场下限(°)
        """
        self._config = config

    def save_snapshot(self) -> bool:
        if self._recorder is None:
            return False
        with self._lidar_lock:
            ld = self._latest_lidar_data
        if ld is not None:
            self._recorder.save_lidar_point_cloud(ld)
            return True
        return False

    def get_display_fields(self) -> Dict[str, str]:
        if not self._config:
            return {"线数": "N/A", "测距范围": "N/A", "点频": "N/A",
                    "水平视场": "N/A", "垂直视场": "N/A", "旋转频率": "N/A"}
        channels = self._config.get("number-of-channels", 0)
        range_m = self._config.get("range", 0)
        pps = self._config.get("points-per-second", 0)
        hfov_start = self._config.get("horizontal-fov-start-deg", 0)
        hfov_end = self._config.get("horizontal-fov-end-deg", 0)
        vfov_upper = self._config.get("vertical-fov-upper-deg", 0)
        vfov_lower = self._config.get("vertical-fov-lower-deg", 0)
        hz = self._config.get("horizontal-rotation-frequency", 0)
        if pps >= 1000000:
            pps_str = f"{pps / 1000000:.1f}M/s"
        elif pps >= 1000:
            pps_str = f"{pps / 1000:.0f}K/s"
        else:
            pps_str = f"{pps}/s"
        return {
            "线数": str(channels),
            "测距范围": f"{range_m}m",
            "点频": pps_str,
            "水平视场": f"{hfov_start}°~{hfov_end}°",
            "垂直视场": f"{vfov_lower}°~{vfov_upper}°",
            "旋转频率": f"{hz}Hz",
        }
