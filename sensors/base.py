"""
传感器模块 - 核心接口定义

本模块定义了传感器系统的核心抽象：
1. SensorType枚举：所有支持的传感器类型
2. SensorCallback基类：传感器回调的抽象接口
3. SensorData：传感器数据的统一封装

设计原则：
- 所有传感器回调类必须继承SensorCallback并实现__call__方法
- 传感器数据通过SensorData统一封装，包含类型、名称、时间戳和载荷
- 传感器类型通过SensorType枚举管理，便于扩展和类型安全
"""

from abc import ABC, abstractmethod
from enum import Enum
from dataclasses import dataclass, field
from typing import Any, Dict, Optional
import time


class SensorType(Enum):
    """
    传感器类型枚举
    每种传感器对应一个唯一的类型标识，用于工厂创建和数据路由

    分类：
    - 视觉传感器：CAMERA/DEPTH_CAMERA/STEREO_CAMERA
    - 激光传感器：LIDAR
    - 雷达传感器：RADAR
    - 导航传感器：IMU/GPS/MAGNETOMETER
    - 高度传感器：RADIO_ALTIMETER/LASER_ALTIMETER/ULTRASONIC_ALTIMETER
    - 大气传感器：BAROMETER/AIRSPEED
    - 距离传感器：DISTANCE_SENSOR
    """
    CAMERA = "camera"
    DEPTH_CAMERA = "depth_camera"
    STEREO_CAMERA = "stereo_camera"
    LIDAR = "lidar"
    RADAR = "radar"
    IMU = "imu"
    GPS = "gps"
    MAGNETOMETER = "magnetometer"
    RADIO_ALTIMETER = "radio_altimeter"
    LASER_ALTIMETER = "laser_altimeter"
    ULTRASONIC_ALTIMETER = "ultrasonic_altimeter"
    BAROMETER = "barometer"
    AIRSPEED = "airspeed"
    DISTANCE_SENSOR = "distance_sensor"
    BATTERY = "battery"


@dataclass
class SensorData:
    """
    传感器数据统一封装
    所有传感器回调产生的数据都通过此类封装，便于统一处理和传输

    属性：
        sensor_type: 传感器类型（SensorType枚举）
        sensor_name: 传感器名称（如"FrontCamera"、"IMU1"）
        timestamp: 数据时间戳（秒），默认为当前时间
        payload: 数据载荷（字典格式，包含传感器特定的数据字段）
    """
    sensor_type: SensorType
    sensor_name: str
    timestamp: float = field(default_factory=time.time)
    payload: Dict[str, Any] = field(default_factory=dict)


class SensorCallback(ABC):
    """
    传感器回调抽象基类
    所有传感器回调处理器必须继承此类并实现__call__方法

    设计原理：
    - 使用回调模式：当传感器数据更新时，AirSim客户端自动调用注册的回调函数
    - 每种传感器类型有独立的回调类，负责数据解析、UI更新和数据记录
    - 回调类通过信号（pyqtSignal）将数据传递到UI线程，实现线程安全

    子类必须实现：
    - __call__(self, client, data): 处理传感器数据的回调方法

    子类可选实现：
    - get_latest_data() -> SensorData: 获取最新的传感器数据
    - get_display_fields() -> dict: 获取用于UI显示的字段字典
    """

    def __init__(self, sensor_name: str, sensor_type: SensorType):
        """
        初始化传感器回调

        参数：
            sensor_name: 传感器名称（与JSONC配置中的id对应）
            sensor_type: 传感器类型（SensorType枚举）
        """
        self._sensor_name = sensor_name
        self._sensor_type = sensor_type
        self._latest_data: Optional[SensorData] = None
        self._data_count: int = 0
        # UI更新节流：避免传感器数据过于频繁地触发UI刷新导致卡顿
        # _last_ui_update_time：上次UI更新时间戳
        # _ui_throttle_interval：最小UI更新间隔（秒），默认0.2秒≈5Hz
        # 5Hz对地面站数值显示足够，人眼无法分辨更高频率的数值变化
        self._last_ui_update_time: float = 0.0
        self._ui_throttle_interval: float = 0.2

    def _should_update_ui(self) -> bool:
        """
        判断是否应该更新UI（节流控制）
        距离上次UI更新超过节流间隔时才允许更新

        返回：
            True = 允许更新UI，False = 跳过本次更新
        """
        now = time.time()
        if now - self._last_ui_update_time >= self._ui_throttle_interval:
            self._last_ui_update_time = now
            return True
        return False

    @property
    def sensor_name(self) -> str:
        """获取传感器名称"""
        return self._sensor_name

    @property
    def sensor_type(self) -> SensorType:
        """获取传感器类型"""
        return self._sensor_type

    @property
    def data_count(self) -> int:
        """获取已接收的数据帧数"""
        return self._data_count

    @abstractmethod
    def __call__(self, client, data) -> None:
        """
        传感器数据回调方法
        当传感器数据更新时由AirSim客户端自动调用

        参数：
            client: AirSim客户端对象
            data: 传感器原始数据（格式取决于传感器类型）
        """
        pass

    def get_latest_data(self) -> Optional[SensorData]:
        """
        获取最新的传感器数据

        返回：
            最新的SensorData对象，如果没有数据则返回None
        """
        return self._latest_data

    def get_display_fields(self) -> Dict[str, str]:
        """
        获取用于UI显示的字段字典
        子类可重写此方法以自定义显示内容

        返回：
            字典格式：{"显示标签": "显示值", ...}
        """
        if self._latest_data is None:
            return {}
        return {k: f"{v:.4f}" if isinstance(v, float) else str(v)
                for k, v in self._latest_data.payload.items()}

    def _update_data(self, payload: Dict[str, Any]) -> SensorData:
        """
        更新传感器数据（内部方法）
        创建新的SensorData并更新计数器

        参数：
            payload: 传感器数据载荷

        返回：
            新创建的SensorData对象
        """
        self._data_count += 1
        self._latest_data = SensorData(
            sensor_type=self._sensor_type,
            sensor_name=self._sensor_name,
            payload=payload,
        )
        return self._latest_data
