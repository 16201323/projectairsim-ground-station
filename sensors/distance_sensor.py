"""
传感器模块 - 距离传感器回调处理器

本模块实现通用距离传感器的回调处理：
DistanceSensorCallback：处理distance-sensor类型传感器的距离数据

距离传感器说明：
- 测量传感器到最近障碍物的距离
- 可用于避障、定高、测距等场景
- ProjectAirSim中distance-sensor是通用类型
- 高度表（无线电/激光/超声波）也使用distance-sensor类型，但有专用回调类

ProjectAirSim中distance-sensor的topic：
- distance_sensor: 距离传感器数据
"""

import threading

from .base import SensorCallback, SensorType, SensorData
from typing import Dict, Any, Optional, Callable


class DistanceSensorCallback(SensorCallback):
    """
    通用距离传感器回调处理器
    处理distance-sensor类型的距离测量数据

    工作流程：
    1. 接收距离传感器数据
    2. 提取距离值和方向信息
    3. 通过callback发送到UI显示

    参数：
        sensor_name: 传感器名称（如"DistanceSensor1"）
        distance_callback: 距离数据回调函数
    """

    def __init__(self, sensor_name: str,
                 distance_callback: Optional[Callable] = None):
        super().__init__(sensor_name, SensorType.DISTANCE_SENSOR)
        self._distance_callback = distance_callback
        self._data_lock = threading.Lock()

    def __call__(self, client, distance_data) -> None:
        """
        距离传感器数据回调

        性能优化：先检查节流，避免不必要的数据解析

        参数：
            client: AirSim客户端对象
            distance_data: 距离传感器数据字典
        """
        try:
            if distance_data is not None:
                # 先检查节流，避免不必要的数据解析
                if not self._should_update_ui():
                    return
                # 解析距离数据
                distance_cm = distance_data.get("current_distance", 0.0)
                distance_m = distance_cm / 100.0
                self._update_data({
                    "distance": distance_m,
                })
                # 发送到UI
                if self._distance_callback and self._latest_data:
                    self._distance_callback(self._latest_data)
        except Exception:
            pass

    def get_display_fields(self) -> Dict[str, str]:
        """获取UI显示字段：距离值"""
        if self._latest_data is None:
            return {"距离": "N/A"}
        p = self._latest_data.payload
        return {
            "距离": f"{p.get('distance', 0):.2f}m",
        }
