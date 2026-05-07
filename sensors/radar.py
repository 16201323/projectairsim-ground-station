"""
传感器模块 - 毫米波雷达回调处理器

本模块实现毫米波雷达的数据回调处理：
RadarCallback：处理雷达检测目标和跟踪目标数据

毫米波雷达说明：
- 工作频段：通常为76-81GHz
- 探测距离：通常100-300米
- 输出数据：检测目标列表（radar_detections）和跟踪目标列表（radar_tracks）
- 检测目标：原始回波处理后的点目标，包含位置、速度、强度等
- 跟踪目标：经过跟踪滤波后的稳定目标，包含目标ID、状态等

ProjectAirSim中radar传感器的消息格式（C++端MSGPACK_DEFINE_MAP）：
- RadarDetectionMessage: time_stamp, radar_detections, pose
  - radar_detections: 检测目标列表，每个目标包含：
    - range: 距离（米，C++端已做ToMeters转换）
    - azimuth: 方位角（度！不是弧度！）
    - elevation: 仰角（度！不是弧度！）
    - velocity: 径向速度（米/秒）
    - rcs_sqm: 雷达截面积（平方米）
- RadarTrackMessage: time_stamp, radar_tracks, pose
  - radar_tracks: 跟踪目标列表，每个目标包含：
    - id: 目标ID
    - azimuth_est: 方位角估计（度）
    - elevation_est: 仰角估计（度）
    - range_est: 距离估计（米）
    - position_est: 位置估计（NED坐标，米）
    - velocity_est: 速度估计（NED坐标，米/秒）
    - accel_est: 加速度估计（NED坐标，米/秒²）
    - rcs_sqm: 雷达截面积（平方米）

重要角度单位说明：
C++端LoadRadarSettings()从JSONC读取FOV参数时，直接使用度数值，未做度→弧度转换。
因此GenerateFullFOVFrame()生成的波束方位角/仰角是度数，
检测结果Detection.azimuth/elevation也直接存储了这些度数值。
Python端收到的azimuth和elevation已经是度数，不需要math.degrees()转换！

注意：与distance-sensor不同，radar的range在C++端已经做了ToMeters()转换，
参考UnrealRadar.cpp:254：
  Detection.range = TransformUtils::ToMeters(RadarToHitPoint.Size());
所以Python端收到的range值已经是米为单位，不需要额外转换。
"""

import math
import threading
import numpy as np

from .base import SensorCallback, SensorType, SensorData
from typing import Dict, Any, Optional, Callable


class RadarCallback(SensorCallback):
    """
    毫米波雷达回调处理器
    处理雷达检测目标和跟踪目标数据

    工作流程：
    1. 接收雷达检测目标数据（radar_detections topic）
    2. 解析目标列表，提取位置、速度、强度等信息
    3. 通过radar_callback发送到UI进行可视化
    4. 计算目标统计信息（目标数量、最近距离等）

    参数：
        sensor_name: 雷达名称（如"Radar1"）
        radar_callback: 雷达数据回调函数，格式：callback(radar_data)
    """

    def __init__(self, sensor_name: str,
                 radar_callback: Optional[Callable] = None):
        super().__init__(sensor_name, SensorType.RADAR)
        self._radar_callback = radar_callback
        self._latest_detections: Optional[Dict] = None
        self._latest_tracks: Optional[Dict] = None
        self._data_lock = threading.Lock()

    def __call__(self, client, radar_data) -> None:
        """
        雷达数据回调（检测目标）
        当雷达传感器数据更新时由AirSim客户端自动调用

        参数：
            client: AirSim客户端对象
            radar_data: 雷达检测目标数据字典
        """
        try:
            if radar_data is not None:
                with self._data_lock:
                    self._latest_detections = radar_data
                # 解析检测目标统计（先解析，再发送SensorData到UI）
                self._parse_detections(radar_data)
                # 发送到UI（节流控制，避免频繁刷新）
                if self._radar_callback and self._latest_data and self._should_update_ui():
                    self._radar_callback(self._latest_data)
        except Exception:
            pass

    def on_tracks(self, client, tracks_data) -> None:
        """
        雷达跟踪目标回调
        当雷达跟踪目标数据更新时调用

        参数：
            client: AirSim客户端对象
            tracks_data: 雷达跟踪目标数据字典
        """
        try:
            if tracks_data is not None:
                with self._data_lock:
                    self._latest_tracks = tracks_data
        except Exception:
            pass

    def _parse_detections(self, radar_data: Dict):
        """
        解析雷达检测目标数据，提取统计信息

        ProjectAirSim C++端消息格式（MSGPACK_DEFINE_MAP）：
        - time_stamp: 时间戳
        - radar_detections: 检测目标列表（注意key是"radar_detections"不是"detections"）
        - pose: 传感器位姿

        每个检测目标的字段（RadarDetectionMsgpack MSGPACK_DEFINE_MAP）：
        - range: 距离（米，C++端已做ToMeters转换）
        - azimuth: 方位角（度！不是弧度！C++端LoadRadarSettings未做度→弧度转换）
        - elevation: 仰角（度！不是弧度！）
        - velocity: 径向速度（米/秒）
        - rcs_sqm: 雷达截面积（平方米）

        重要：azimuth和elevation已经是度数，不需要math.degrees()转换！
        如果误用math.degrees()会导致双重转换：
        例如 azimuth=-37.0° → math.degrees(-37.0) = -2,119.9°（错误！）
        正确做法：直接使用原始度数值

        参数：
            radar_data: 雷达检测目标数据
        """
        try:
            # 关键修复：C++端MSGPACK_DEFINE_MAP使用的key是"radar_detections"
            # 之前错误地使用了"detections"，导致永远获取不到目标列表
            detections = radar_data.get("radar_detections", [])
            num_targets = len(detections)
            min_range = float("inf")
            min_azimuth = 0.0
            min_elevation = 0.0
            min_velocity = 0.0

            for det in detections:
                # range字段已经是米（C++端做了ToMeters转换）
                r = det.get("range", 0.0)
                if 0 < r < min_range:
                    min_range = r
                    # 关键修复：azimuth和elevation已经是度数，不需要math.degrees()转换！
                    # C++端LoadRadarSettings从JSONC读取度数值时未做ToRadians转换，
                    # 所以Beam.azimuth/elevation存储的是度数而非弧度。
                    # Detection.azimuth = Beam.azimuth 直接存储了度数值。
                    # 之前错误地调用math.degrees()导致双重转换：
                    #   例如 azimuth=-37.0 → math.degrees(-37.0) = -2,119.9°（错误！）
                    min_azimuth = det.get("azimuth", 0.0)
                    min_elevation = det.get("elevation", 0.0)
                    min_velocity = det.get("velocity", 0.0)

            if min_range == float("inf"):
                min_range = 0.0

            self._update_data({
                "target_count": num_targets,
                "min_range": min_range,
                "min_azimuth": min_azimuth,
                "min_elevation": min_elevation,
                "min_velocity": min_velocity,
            })
        except Exception:
            pass

    def get_display_fields(self) -> Dict[str, str]:
        """获取UI显示字段：目标数量和最近目标距离"""
        if self._latest_data is None:
            return {"目标数": "0", "最近距离": "N/A"}
        p = self._latest_data.payload
        return {
            "目标数": str(p.get("target_count", 0)),
            "最近距离": f"{p.get('min_range', 0):.1f}m",
            "方位角": f"{p.get('min_azimuth', 0):.1f}°",
            "仰角": f"{p.get('min_elevation', 0):.1f}°",
        }
