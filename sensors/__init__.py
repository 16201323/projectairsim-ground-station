"""
传感器模块 - 包导出

本模块统一导出所有传感器相关的类，方便外部引用

使用方式：
    from sensors import SensorType, SensorCallback, SensorManager
    from sensors import CameraCallback, LidarCallback, IMUCallback
"""

from .base import SensorType, SensorCallback, SensorData
from .camera import CameraCallback, DepthCameraCallback
from .stereo_camera import StereoCameraCallback
from .lidar import LidarCallback
from .radar import RadarCallback
from .imu import IMUCallback
from .gps import GPSCallback
from .altimeter import (
    AltimeterCallback,
    RadioAltimeterCallback,
    LaserAltimeterCallback,
    UltrasonicAltimeterCallback,
)
from .atmosphere import AtmosphereCallback
from .distance_sensor import DistanceSensorCallback
from .factory import SensorFactory
from .manager import SensorManager

__all__ = [
    "SensorType",
    "SensorCallback",
    "SensorData",
    "CameraCallback",
    "DepthCameraCallback",
    "StereoCameraCallback",
    "LidarCallback",
    "RadarCallback",
    "IMUCallback",
    "GPSCallback",
    "AltimeterCallback",
    "RadioAltimeterCallback",
    "LaserAltimeterCallback",
    "UltrasonicAltimeterCallback",
    "AtmosphereCallback",
    "DistanceSensorCallback",
    "SensorFactory",
    "SensorManager",
]
