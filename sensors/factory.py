"""
传感器模块 - 传感器工厂

本模块实现传感器对象的创建工厂：
SensorFactory：根据传感器配置自动创建对应的回调处理器

工厂模式优势：
- 将对象创建逻辑与使用逻辑分离
- 新增传感器类型只需注册新的创建函数
- 统一管理传感器的创建参数和依赖注入
"""

from .base import SensorType, SensorCallback
from .camera import CameraCallback, DepthCameraCallback
from .stereo_camera import StereoCameraCallback
from .lidar import LidarCallback
from .radar import RadarCallback
from .imu import IMUCallback
from .gps import GPSCallback
from .altimeter import (
    RadioAltimeterCallback, LaserAltimeterCallback, UltrasonicAltimeterCallback
)
from .atmosphere import AtmosphereCallback
from .distance_sensor import DistanceSensorCallback
from typing import Dict, Any, Optional, Callable


class SensorFactory:
    """
    传感器工厂类
    根据传感器类型和配置创建对应的回调处理器

    使用方式：
    1. 调用create()方法，传入传感器类型、名称和配置
    2. 工厂自动选择对应的回调类并创建实例
    3. 返回的回调实例可直接用于AirSim客户端订阅

    注册机制：
    - _creators字典维护类型→创建函数的映射
    - 新增传感器类型时，只需在_creators中添加映射
    - 支持自定义创建函数，实现灵活的依赖注入
    """

    # 传感器类型→创建函数映射表
    # 每个创建函数签名：(sensor_name, config, callbacks) -> SensorCallback
    _creators = {
        SensorType.CAMERA: lambda name, cfg, cb: CameraCallback(
            sensor_name=name,
            camera_key=cfg.get("camera_key", name.lower()),
            frame_callback=cb.get("frame_callback"),
            recorder=cb.get("recorder"),
            sensor_data_callback=cb.get("sensor_data_callback"),
        ),
        SensorType.DEPTH_CAMERA: lambda name, cfg, cb: DepthCameraCallback(
            sensor_name=name,
            camera_key=cfg.get("camera_key", name.lower()),
            frame_callback=cb.get("frame_callback"),
            recorder=cb.get("recorder"),
        ),
        SensorType.STEREO_CAMERA: lambda name, cfg, cb: StereoCameraCallback(
            sensor_name=name,
            frame_callback=cb.get("frame_callback"),
            recorder=cb.get("recorder"),
            baseline=cfg.get("baseline", 0.12),
            compute_disparity=cfg.get("compute_disparity", False),
        ),
        SensorType.LIDAR: lambda name, cfg, cb: LidarCallback(
            sensor_name=name,
            lidar_callback=cb.get("lidar_callback"),
            recorder=cb.get("recorder"),
            sensor_data_callback=cb.get("sensor_data_callback"),
        ),
        SensorType.RADAR: lambda name, cfg, cb: RadarCallback(
            sensor_name=name,
            radar_callback=cb.get("radar_callback"),
        ),
        SensorType.IMU: lambda name, cfg, cb: IMUCallback(
            sensor_name=name,
            imu_callback=cb.get("imu_callback"),
        ),
        SensorType.GPS: lambda name, cfg, cb: GPSCallback(
            sensor_name=name,
            gps_callback=cb.get("gps_callback"),
        ),
        SensorType.RADIO_ALTIMETER: lambda name, cfg, cb: RadioAltimeterCallback(
            sensor_name=name,
            altimeter_callback=cb.get("altimeter_callback"),
        ),
        SensorType.LASER_ALTIMETER: lambda name, cfg, cb: LaserAltimeterCallback(
            sensor_name=name,
            altimeter_callback=cb.get("altimeter_callback"),
        ),
        SensorType.ULTRASONIC_ALTIMETER: lambda name, cfg, cb: UltrasonicAltimeterCallback(
            sensor_name=name,
            altimeter_callback=cb.get("altimeter_callback"),
        ),
        SensorType.BAROMETER: lambda name, cfg, cb: AtmosphereCallback(
            sensor_name=name,
            atmosphere_callback=cb.get("atmosphere_callback"),
        ),
        SensorType.DISTANCE_SENSOR: lambda name, cfg, cb: DistanceSensorCallback(
            sensor_name=name,
            distance_callback=cb.get("distance_callback"),
        ),
    }

    @classmethod
    def create(cls, sensor_type: SensorType, sensor_name: str,
               config: Optional[Dict[str, Any]] = None,
               callbacks: Optional[Dict[str, Callable]] = None) -> Optional[SensorCallback]:
        """
        创建传感器回调处理器

        参数：
            sensor_type: 传感器类型（SensorType枚举）
            sensor_name: 传感器名称（与JSONC配置中的id对应）
            config: 传感器配置字典（可选参数，如baseline、max_range等）
            callbacks: 回调函数字典（可选，如frame_callback、lidar_callback等）

        返回：
            创建的SensorCallback实例，如果类型不支持则返回None
        """
        if config is None:
            config = {}
        if callbacks is None:
            callbacks = {}

        creator = cls._creators.get(sensor_type)
        if creator is None:
            return None
        return creator(sensor_name, config, callbacks)

    @classmethod
    def register(cls, sensor_type: SensorType, creator_func: Callable):
        """
        注册新的传感器类型创建函数
        用于扩展工厂，支持自定义传感器类型

        参数：
            sensor_type: 传感器类型枚举值
            creator_func: 创建函数，签名：(name, config, callbacks) -> SensorCallback
        """
        cls._creators[sensor_type] = creator_func

    @classmethod
    def supported_types(cls) -> list:
        """获取所有支持的传感器类型列表"""
        return list(cls._creators.keys())
