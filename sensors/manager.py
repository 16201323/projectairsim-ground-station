"""
传感器模块 - 传感器管理器

本模块实现传感器的统一管理和订阅：
SensorManager：管理所有传感器回调的创建、订阅和数据分发

核心功能：
1. 根据无人机配置自动创建传感器回调
2. 统一注册AirSim客户端订阅
3. 提供传感器数据的统一查询接口
4. 管理传感器生命周期（创建→订阅→销毁）
"""

import json
import threading
from typing import Dict, Any, Optional, Callable, List

from .base import SensorCallback, SensorType, SensorData
from .factory import SensorFactory
from .camera import CameraCallback
from .stereo_camera import StereoCameraCallback
from .radar import RadarCallback
from .atmosphere import AtmosphereCallback


class SensorManager:
    """
    传感器管理器
    统一管理所有传感器回调的创建、订阅和数据分发

    工作流程：
    1. 读取无人机传感器配置（drone.sensors）
    2. 根据配置创建对应的回调处理器
    3. 注册AirSim客户端订阅
    4. 提供数据查询接口供UI使用

    参数：
        client: AirSim客户端对象
        drone: 无人机对象
        recorder: DataRecorder实例
        log_func: 日志函数，格式：log_func(msg, level)
    """

    # JSONC传感器type字段 → SensorType枚举映射
    # 用于将配置文件中的传感器类型字符串转换为枚举值
    TYPE_MAP = {
        "camera": SensorType.CAMERA,
        "imu": SensorType.IMU,
        "gps": SensorType.GPS,
        "lidar": SensorType.LIDAR,
        "radar": SensorType.RADAR,
        "barometer": SensorType.BAROMETER,
        "airspeed": SensorType.AIRSPEED,
        "distance-sensor": SensorType.DISTANCE_SENSOR,
        "magnetometer": SensorType.MAGNETOMETER,
        "battery": SensorType.BATTERY,
    }

    # 特定传感器ID → 特定SensorType映射
    # 用于区分同类型但不同功能的传感器（如三种高度表）
    ID_TYPE_MAP = {
        "RadioAltimeter": SensorType.RADIO_ALTIMETER,
        "LaserAltimeter": SensorType.LASER_ALTIMETER,
        "UltrasonicAltimeter": SensorType.ULTRASONIC_ALTIMETER,
        "DownCamera": SensorType.CAMERA,
        "Chase": SensorType.CAMERA,
        "StereoLeft": SensorType.CAMERA,
        "StereoRight": SensorType.CAMERA,
    }

    CAMERA_KEY_MAP = {
        "DownCamera": "down",
        "Chase": "chase",
        "StereoLeft": "stereo_left",
        "StereoRight": "stereo_right",
    }

    def __init__(self, client, drone, recorder=None,
                 log_func: Optional[Callable] = None,
                 frame_callback: Optional[Callable] = None,
                 sim_config_path: Optional[str] = None,
                 robot_config: Optional[str] = None):
        """
        初始化传感器管理器

        参数：
            client: AirSim客户端对象
            drone: 无人机对象
            recorder: DataRecorder实例
            log_func: 日志函数
            frame_callback: 相机帧回调函数
            sim_config_path: 仿真配置文件目录路径（用于读取传感器配置）
            robot_config: 机器人配置文件名（如"robot_quadrotor_adv.jsonc"）
        """
        self._client = client
        self._drone = drone
        self._recorder = recorder
        self._log_func = log_func or (lambda msg, level="INFO": None)
        self._frame_callback = frame_callback
        self._sim_config_path = sim_config_path
        self._robot_config = robot_config
        # 传感器回调字典：{sensor_name: SensorCallback}
        self._sensors: Dict[str, SensorCallback] = {}
        # 双目相机组：{group_name: StereoCameraCallback}
        self._stereo_groups: Dict[str, StereoCameraCallback] = {}
        # 大气机组：{group_name: AtmosphereCallback}
        self._atmosphere_groups: Dict[str, AtmosphereCallback] = {}
        # 传感器数据信号回调（用于UI更新）
        self._sensor_data_callback: Optional[Callable] = None
        self._lock = threading.Lock()

    @property
    def sensors(self) -> Dict[str, SensorCallback]:
        """获取所有传感器回调字典"""
        return self._sensors

    def set_sensor_data_callback(self, callback: Callable):
        """
        设置传感器数据更新回调
        当任何传感器数据更新时调用此回调

        参数：
            callback: 回调函数，签名：callback(sensor_name: str, data: SensorData)
        """
        self._sensor_data_callback = callback

    def setup_all_sensors(self):
        """
        设置所有传感器订阅
        遍历无人机配置中的传感器列表，创建回调并注册订阅

        处理顺序：
        1. 先处理双目相机组（需要左右相机配对）
        2. 再处理大气机组（需要气压计+空速配对）
        3. 最后处理其他独立传感器
        """
        drone_sensors = self._drone.sensors
        self._log("开始设置传感器订阅...", "INFO")

        # 步骤1：识别双目相机组
        # 命名规则：StereoLeft/StereoRight为一组
        stereo_left_id = None
        stereo_right_id = None
        for sensor_id in drone_sensors:
            if sensor_id == "StereoLeft":
                stereo_left_id = sensor_id
            elif sensor_id == "StereoRight":
                stereo_right_id = sensor_id

        # 创建双目相机回调
        if stereo_left_id and stereo_right_id:
            stereo_callback = StereoCameraCallback(
                sensor_name="StereoCamera",
                frame_callback=self._frame_callback,
                recorder=self._recorder,
                baseline=0.12,
                compute_disparity=False,
            )
            # 设置传感器数据回调，用于UI面板显示基线距离和视差信息
            stereo_callback._stereo_data_callback = self._make_sensor_data_cb("StereoCamera")
            self._stereo_groups["StereoCamera"] = stereo_callback
            self._sensors["StereoCamera"] = stereo_callback
            self._log("双目相机组创建成功", "INFO")

        # 步骤2：识别大气机组
        # Barometer + Airspeed 组合
        has_barometer = "Barometer" in drone_sensors
        has_airspeed = "Airspeed" in drone_sensors
        if has_barometer or has_airspeed:
            atmo_callback = AtmosphereCallback(
                sensor_name="Atmosphere",
                atmosphere_callback=self._make_sensor_data_cb("Atmosphere"),
            )
            self._atmosphere_groups["Atmosphere"] = atmo_callback
            self._sensors["Atmosphere"] = atmo_callback
            self._log("大气机组创建成功", "INFO")

        # 步骤3：遍历所有传感器，创建回调并订阅
        for sensor_id, topics in drone_sensors.items():
            # 跳过已处理的传感器（双目相机和大气机的子传感器）
            if sensor_id in ("StereoLeft", "StereoRight"):
                self._subscribe_stereo(sensor_id, topics)
                continue
            if sensor_id in ("Barometer", "Airspeed"):
                self._subscribe_atmosphere(sensor_id, topics)
                continue

            # 创建独立传感器回调
            self._create_and_subscribe(sensor_id, topics)

        # 打印订阅摘要
        self._log(f"传感器订阅完成: 共{len(self._sensors)}个传感器/传感器组", "INFO")

        # 读取机器人配置文件，提取LiDAR固定配置参数
        self._load_lidar_config()

    def _create_and_subscribe(self, sensor_id: str, topics: Dict[str, str]):
        """
        创建独立传感器回调并注册订阅

        参数：
            sensor_id: 传感器ID（如"FrontCamera"、"IMU1"）
            topics: 传感器话题字典（如{"scene_camera": "/Drone1/sensors/FrontCamera/scene_camera"}）
        """
        # 确定传感器类型（传入topics用于类型推断）
        sensor_type = self._determine_sensor_type(sensor_id, topics)
        if sensor_type is None:
            self._log(f"未知传感器类型: {sensor_id}", "WARNING")
            return

        # 构建回调参数
        config = self._build_sensor_config(sensor_id, sensor_type)
        callbacks = self._build_callbacks(sensor_id, sensor_type)

        # 创建回调处理器
        callback = SensorFactory.create(sensor_type, sensor_id, config, callbacks)
        if callback is None:
            self._log(f"不支持的传感器类型: {sensor_type}", "WARNING")
            return

        self._sensors[sensor_id] = callback

        # 注册订阅
        self._subscribe_sensor(sensor_id, topics, callback)
        self._log(f"传感器订阅成功: {sensor_id} ({sensor_type.value})", "INFO")

    def _determine_sensor_type(self, sensor_id: str,
                               topics: Optional[Dict[str, str]] = None) -> Optional[SensorType]:
        """
        根据传感器ID和话题字典确定传感器类型
        推断优先级：
        1. ID_TYPE_MAP精确匹配（如RadioAltimeter→RADIO_ALTIMETER）
        2. 话题key推断（如包含"imu_kinematics"→IMU）
        3. 传感器ID关键词推断（如包含"altimeter"→高度表）

        参数：
            sensor_id: 传感器ID
            topics: 传感器话题字典（可选，用于辅助类型推断）

        返回：
            SensorType枚举值，如果无法确定则返回None
        """
        # 优先级1：ID_TYPE_MAP精确匹配
        if sensor_id in self.ID_TYPE_MAP:
            return self.ID_TYPE_MAP[sensor_id]

        # 优先级2：根据话题key推断传感器类型
        # drone.sensors中的topic key由drone.py根据sensor type生成
        # 例如：IMU→"imu_kinematics", GPS→"gps", LiDAR→"lidar"等
        if topics:
            for topic_key in topics.keys():
                topic_lower = topic_key.lower()
                if "scene_camera" in topic_lower or "depth_planar" in topic_lower:
                    return SensorType.CAMERA
                if "depth_camera" in topic_lower and "planar" not in topic_lower:
                    return SensorType.DEPTH_CAMERA
                if "segmentation" in topic_lower:
                    return SensorType.CAMERA
                if "imu" in topic_lower:
                    return SensorType.IMU
                if topic_lower == "gps":
                    return SensorType.GPS
                if "radar" in topic_lower:
                    return SensorType.RADAR
                if "lidar" in topic_lower:
                    return SensorType.LIDAR
                if topic_lower == "barometer":
                    return SensorType.BAROMETER
                if topic_lower == "airspeed":
                    return SensorType.AIRSPEED
                if "distance_sensor" in topic_lower:
                    return SensorType.DISTANCE_SENSOR
                if "magnetometer" in topic_lower:
                    return SensorType.MAGNETOMETER
                if "battery" in topic_lower:
                    return SensorType.BATTERY

        # 优先级3：根据传感器ID中的关键词推断
        id_lower = sensor_id.lower()
        if "altimeter" in id_lower or "radio_alt" in id_lower:
            return SensorType.RADIO_ALTIMETER
        if "laser_alt" in id_lower:
            return SensorType.LASER_ALTIMETER
        if "ultrasonic_alt" in id_lower:
            return SensorType.ULTRASONIC_ALTIMETER
        if "imu" in id_lower:
            return SensorType.IMU
        if "gps" in id_lower:
            return SensorType.GPS
        if "radar" in id_lower:
            return SensorType.RADAR
        if "lidar" in id_lower:
            return SensorType.LIDAR
        if "camera" in id_lower:
            return SensorType.CAMERA
        if "barometer" in id_lower or "baro" in id_lower:
            return SensorType.BAROMETER
        if "airspeed" in id_lower:
            return SensorType.AIRSPEED

        return None

    def _build_sensor_config(self, sensor_id: str, sensor_type: SensorType) -> Dict[str, Any]:
        """
        构建传感器配置字典

        参数：
            sensor_id: 传感器ID
            sensor_type: 传感器类型

        返回：
            配置字典
        """
        config = {}
        # 相机key映射
        if sensor_id in self.CAMERA_KEY_MAP:
            config["camera_key"] = self.CAMERA_KEY_MAP[sensor_id]
        return config

    def _build_callbacks(self, sensor_id: str, sensor_type: SensorType) -> Dict[str, Callable]:
        """
        构建回调函数字典

        参数：
            sensor_id: 传感器ID
            sensor_type: 传感器类型

        返回：
            回调函数字典
        """
        callbacks = {
            "recorder": self._recorder,
        }
        # 根据传感器类型注入不同的回调函数
        if sensor_type in (SensorType.CAMERA, SensorType.DEPTH_CAMERA):
            callbacks["frame_callback"] = self._frame_callback
            callbacks["sensor_data_callback"] = self._make_sensor_data_cb(sensor_id)
        elif sensor_type == SensorType.IMU:
            callbacks["imu_callback"] = self._make_sensor_data_cb(sensor_id)
        elif sensor_type == SensorType.GPS:
            callbacks["gps_callback"] = self._make_sensor_data_cb(sensor_id)
        elif sensor_type in (SensorType.RADIO_ALTIMETER, SensorType.LASER_ALTIMETER,
                             SensorType.ULTRASONIC_ALTIMETER):
            callbacks["altimeter_callback"] = self._make_sensor_data_cb(sensor_id)
        elif sensor_type == SensorType.RADAR:
            callbacks["radar_callback"] = self._make_sensor_data_cb(sensor_id)
        elif sensor_type == SensorType.LIDAR:
            callbacks["lidar_callback"] = self._make_sensor_data_cb(sensor_id)
            callbacks["sensor_data_callback"] = self._make_sensor_data_cb(sensor_id)
        return callbacks

    def _make_sensor_data_cb(self, sensor_id: str) -> Callable:
        """
        创建传感器数据更新回调（闭包）
        当传感器数据更新时，通知UI进行更新

        参数：
            sensor_id: 传感器ID

        返回：
            回调函数
        """
        def on_data(data):
            if self._sensor_data_callback:
                self._sensor_data_callback(sensor_id, data)
        return on_data

    def _subscribe_sensor(self, sensor_id: str, topics: Dict[str, str],
                          callback: SensorCallback):
        """
        注册AirSim客户端订阅

        参数：
            sensor_id: 传感器ID
            topics: 话题字典
            callback: 回调处理器
        """
        for topic_key, topic_path in topics.items():
            if "camera" in topic_key:
                if isinstance(callback, CameraCallback):
                    if "scene_camera" in topic_key:
                        self._client.subscribe(topic_path, callback)
                    elif "depth" in topic_key or "segmentation" in topic_key:
                        pass
            elif "imu" in topic_key:
                self._client.subscribe(topic_path, callback)
            elif "gps" in topic_key:
                self._client.subscribe(topic_path, callback)
            elif "distance_sensor" in topic_key:
                self._client.subscribe(topic_path, callback)
            elif "radar" in topic_key:
                if "detections" in topic_key:
                    self._client.subscribe(topic_path, callback)
                elif "tracks" in topic_key and isinstance(callback, RadarCallback):
                    self._client.subscribe(topic_path, callback.on_tracks)
            elif "lidar" in topic_key:
                self._client.subscribe(topic_path, callback)
            elif "barometer" in topic_key:
                self._client.subscribe(topic_path, callback)
            elif "airspeed" in topic_key:
                self._client.subscribe(topic_path, callback)
            else:
                self._client.subscribe(topic_path, callback)

    def _subscribe_stereo(self, sensor_id: str, topics: Dict[str, str]):
        """
        注册双目相机子传感器的订阅

        参数：
            sensor_id: 传感器ID（StereoLeft或StereoRight）
            topics: 话题字典
        """
        stereo_callback = self._stereo_groups.get("StereoCamera")
        if stereo_callback is None:
            return
        for topic_key, topic_path in topics.items():
            if "camera" in topic_key or "scene" in topic_key:
                if sensor_id == "StereoLeft":
                    self._client.subscribe(topic_path, stereo_callback.on_left_frame)
                elif sensor_id == "StereoRight":
                    self._client.subscribe(topic_path, stereo_callback.on_right_frame)

    def _subscribe_atmosphere(self, sensor_id: str, topics: Dict[str, str]):
        """
        注册大气机子传感器的订阅

        参数：
            sensor_id: 传感器ID（Barometer或Airspeed）
            topics: 话题字典
        """
        atmo_callback = self._atmosphere_groups.get("Atmosphere")
        if atmo_callback is None:
            return
        for topic_key, topic_path in topics.items():
            if sensor_id == "Barometer" or "barometer" in topic_key:
                self._client.subscribe(topic_path, atmo_callback.on_barometer)
            elif sensor_id == "Airspeed" or "airspeed" in topic_key:
                self._client.subscribe(topic_path, atmo_callback.on_airspeed)

    def get_sensor(self, sensor_name: str) -> Optional[SensorCallback]:
        """
        获取指定名称的传感器回调

        参数：
            sensor_name: 传感器名称

        返回：
            SensorCallback实例，如果不存在则返回None
        """
        return self._sensors.get(sensor_name)

    def get_all_display_fields(self) -> Dict[str, Dict[str, str]]:
        """
        获取所有传感器的UI显示字段

        返回：
            字典格式：{传感器名称: {显示标签: 显示值}}
        """
        result = {}
        for name, callback in self._sensors.items():
            result[name] = callback.get_display_fields()
        return result

    def get_camera_frame(self, camera_name: str):
        """
        获取指定相机的最新帧

        参数：
            camera_name: 相机名称

        返回：
            最新的BGR图像帧，如果不存在则返回None
        """
        callback = self._sensors.get(camera_name)
        if isinstance(callback, CameraCallback):
            return callback.get_latest_frame()
        return None

    def capture_stereo_photo(self) -> bool:
        """
        保存双目相机照片

        返回：
            True表示保存成功
        """
        for name, callback in self._sensors.items():
            if isinstance(callback, StereoCameraCallback):
                return callback.capture_stereo_photo()
        return False

    def _load_lidar_config(self):
        """
        从机器人JSONC配置文件中读取LiDAR固定配置参数
        并设置到对应的LidarCallback中
        """
        if not self._sim_config_path or not self._robot_config:
            return
        try:
            import os
            config_path = os.path.join(self._sim_config_path, self._robot_config)
            if not os.path.exists(config_path):
                self._log(f"机器人配置文件不存在: {config_path}", "WARNING")
                return
            with open(config_path, "r", encoding="utf-8") as f:
                content = f.read()
            lines = content.split("\n")
            clean_lines = []
            for line in lines:
                comment_idx = line.find("//")
                if comment_idx >= 0:
                    in_string = False
                    for ch in line[:comment_idx]:
                        if ch == '"':
                            in_string = not in_string
                    if not in_string:
                        line = line[:comment_idx]
                clean_lines.append(line)
            robot_data = json.loads("\n".join(clean_lines))
            sensors_list = robot_data.get("sensors", [])
            for sensor_cfg in sensors_list:
                if sensor_cfg.get("type") == "lidar":
                    sensor_id = sensor_cfg.get("id", "lidar1")
                    lidar_config = {
                        "number-of-channels": sensor_cfg.get("number-of-channels", 0),
                        "range": sensor_cfg.get("range", 0),
                        "points-per-second": sensor_cfg.get("points-per-second", 0),
                        "horizontal-rotation-frequency": sensor_cfg.get("horizontal-rotation-frequency", 0),
                        "horizontal-fov-start-deg": sensor_cfg.get("horizontal-fov-start-deg", 0),
                        "horizontal-fov-end-deg": sensor_cfg.get("horizontal-fov-end-deg", 0),
                        "vertical-fov-upper-deg": sensor_cfg.get("vertical-fov-upper-deg", 0),
                        "vertical-fov-lower-deg": sensor_cfg.get("vertical-fov-lower-deg", 0),
                    }
                    callback = self._sensors.get(sensor_id)
                    if callback is not None and hasattr(callback, 'set_config'):
                        callback.set_config(lidar_config)
                        self._log(f"LiDAR配置已加载: {sensor_id} ({lidar_config.get('number-of-channels')}线, {lidar_config.get('range')}m)", "INFO")
                        if self._sensor_data_callback:
                            from .base import SensorData
                            config_data = SensorData(
                                sensor_type=SensorType.LIDAR,
                                sensor_name=sensor_id,
                                payload=lidar_config,
                            )
                            self._sensor_data_callback(sensor_id, config_data)
        except Exception as e:
            self._log(f"读取LiDAR配置失败: {e}", "WARNING")

    def _log(self, msg: str, level: str = "INFO"):
        """发送日志消息"""
        self._log_func(msg, level)
