"""
传感器模块 - 高度表回调处理器

本模块实现三种高度表的回调处理：
1. RadioAltimeterCallback：无线电高度表
2. LaserAltimeterCallback：激光高度表
3. UltrasonicAltimeterCallback：超声波高度表

三种高度表均使用distance-sensor类型模拟，朝下安装：
- 无线电高度表：测量范围大（0~500m），精度中等（±0.5m），受地表影响
- 激光高度表：测量范围中等（0~300m），精度高（±0.1m），受天气影响
- 超声波高度表：测量范围小（0~10m），精度低（±0.02m），仅低空有效

ProjectAirSim中distance-sensor的topic：
- distance_sensor: 距离传感器数据
"""

import threading

from .base import SensorCallback, SensorType, SensorData
from typing import Dict, Any, Optional, Callable


class AltimeterCallback(SensorCallback):
    """
    高度表回调处理器（基类）
    所有高度表（无线电/激光/超声波）共享此基类

    工作流程：
    1. 接收distance-sensor数据
    2. 提取距离值（即离地高度）
    3. 通过altimeter_callback发送到UI显示
    4. 根据传感器类型计算有效范围和精度

    参数：
        sensor_name: 高度表名称（如"RadioAltimeter"）
        sensor_type: 传感器类型枚举（RADIO_ALTIMETER/LASER_ALTIMETER/ULTRASONIC_ALTIMETER）
        altimeter_callback: 高度表数据回调函数
        max_range: 最大测量范围（米）
        accuracy: 测量精度（米）
    """

    def __init__(self, sensor_name: str, sensor_type: SensorType,
                 altimeter_callback: Optional[Callable] = None,
                 max_range: float = 500.0,
                 accuracy: float = 0.5):
        super().__init__(sensor_name, sensor_type)
        self._altimeter_callback = altimeter_callback
        self._max_range = max_range
        self._accuracy = accuracy
        self._data_lock = threading.Lock()

    def __call__(self, client, distance_data) -> None:
        """
        高度表数据回调
        当distance-sensor数据更新时由AirSim客户端自动调用

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
                # 解析高度数据
                self._parse_altitude(distance_data)
                # 发送到UI
                if self._altimeter_callback and self._latest_data:
                    self._altimeter_callback(self._latest_data)
        except Exception:
            pass

    def _parse_altitude(self, distance_data: Dict):
        """
        解析高度表数据

        ProjectAirSim的distance-sensor消息格式（C++端MSGPACK_DEFINE_MAP）：
        - time_stamp: 时间戳
        - current_distance: 当前测量距离（厘米！需要转换为米）
        - pose: 传感器位姿

        重要：单位转换问题！
        UnrealDistanceSensor.cpp中，Simulate()函数直接将FHitResult.Distance
        赋值给Distance变量，而FHitResult.Distance的单位是厘米（UE标准单位）。
        但该值在传入DistanceSensorMessage时没有经过ToMeters()转换，
        导致Python端收到的current_distance单位是厘米而非米。
        这是ProjectAirSim C++端的一个bug，我们在此做补偿转换。

        参考：
        - UnrealDistanceSensor.cpp:161 → Distance = HitInfo.Distance;（厘米）
        - UnrealDistanceSensor.cpp:65 → DistanceSensorMsg(CurSimTime, Distance, ...);（未转换）
        - UnrealTransforms.h:20 → 其他位置数据都经过ToMeters()转换，但Distance漏掉了

        参数：
            distance_data: 距离传感器原始数据
        """
        try:
            # 从消息中提取当前距离值
            # 字段名为current_distance（ProjectAirSim C++端定义）
            # 注意：该值单位为厘米（UE端bug），需要除以100转换为米
            distance_cm = distance_data.get("current_distance", 0.0)
            # 将厘米转换为米（补偿C++端缺失的ToMeters()转换）
            distance_m = distance_cm / 100.0
            # 检查是否在有效范围内
            # distance_m >= 0 表示有效测量（0表示在地面或极近距离）
            # distance_m <= max_range 表示未超出量程
            # distance_m < 0 通常表示无回波（超出最大量程）
            valid = 0 <= distance_m <= self._max_range
            self._update_data({
                "altitude": distance_m,
                "valid": valid,
                "max_range": self._max_range,
                "accuracy": self._accuracy,
            })
        except Exception:
            pass

    def get_display_fields(self) -> Dict[str, str]:
        """获取UI显示字段：高度值、量程和有效性"""
        if self._latest_data is None:
            return {"高度": "N/A", "量程": f"0~{self._max_range:.0f}m", "状态": "无效"}
        p = self._latest_data.payload
        alt = p.get("altitude", 0.0)
        valid = p.get("valid", False)
        return {
            "高度": f"{alt:.2f}m",
            "量程": f"0~{self._max_range:.0f}m",
            "状态": "有效" if valid else "超量程",
        }


class RadioAltimeterCallback(AltimeterCallback):
    """
    无线电高度表回调处理器
    使用无线电波测量离地高度

    特点：
    - 测量范围：0~500米
    - 精度：±0.5米
    - 工作原理：发射无线电波，测量反射波的时间差
    - 适用场景：中低空飞行的高度测量
    - 受地表材质影响，水面反射弱

    参数：
        sensor_name: 传感器名称
        altimeter_callback: 数据回调函数
    """

    def __init__(self, sensor_name: str,
                 altimeter_callback: Optional[Callable] = None):
        super().__init__(
            sensor_name=sensor_name,
            sensor_type=SensorType.RADIO_ALTIMETER,
            altimeter_callback=altimeter_callback,
            max_range=500.0,
            accuracy=0.5,
        )


class LaserAltimeterCallback(AltimeterCallback):
    """
    激光高度表回调处理器
    使用激光脉冲测量离地高度

    特点：
    - 测量范围：0~300米
    - 精度：±0.1米
    - 工作原理：发射激光脉冲，测量反射光的时间差
    - 适用场景：精确高度测量、地形测绘
    - 受天气影响，雨雾天气精度下降

    参数：
        sensor_name: 传感器名称
        altimeter_callback: 数据回调函数
    """

    def __init__(self, sensor_name: str,
                 altimeter_callback: Optional[Callable] = None):
        super().__init__(
            sensor_name=sensor_name,
            sensor_type=SensorType.LASER_ALTIMETER,
            altimeter_callback=altimeter_callback,
            max_range=300.0,
            accuracy=0.1,
        )


class UltrasonicAltimeterCallback(AltimeterCallback):
    """
    超声波高度表回调处理器
    使用超声波测量离地高度

    特点：
    - 测量范围：0~10米
    - 精度：±0.02米
    - 工作原理：发射超声波，测量反射波的时间差
    - 适用场景：低空悬停、起降阶段
    - 仅在低空有效，受温度和气流影响

    参数：
        sensor_name: 传感器名称
        altimeter_callback: 数据回调函数
    """

    def __init__(self, sensor_name: str,
                 altimeter_callback: Optional[Callable] = None):
        super().__init__(
            sensor_name=sensor_name,
            sensor_type=SensorType.ULTRASONIC_ALTIMETER,
            altimeter_callback=altimeter_callback,
            max_range=10.0,
            accuracy=0.02,
        )
