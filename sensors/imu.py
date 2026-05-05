"""
传感器模块 - IMU惯性测量单元回调处理器

本模块实现IMU传感器的数据回调处理：
IMUCallback：处理加速度和角速度数据

IMU传感器说明：
- 加速度计：测量三轴线加速度（m/s^2）
- 陀螺仪：测量三轴角速度（rad/s）
- 数据通过imu_kinematics话题发布
- 包含线加速度、角速度和姿态信息

ProjectAirSim中IMU传感器的topic：
- imu_kinematics: IMU运动学数据
"""

import threading
import math

from .base import SensorCallback, SensorType, SensorData
from typing import Dict, Any, Optional, Callable


class IMUCallback(SensorCallback):
    """
    IMU惯性测量单元回调处理器
    处理加速度计和陀螺仪数据

    工作流程：
    1. 接收IMU运动学数据（imu_kinematics）
    2. 提取线加速度、角速度和姿态信息
    3. 通过imu_callback发送到UI显示
    4. 计算姿态角（从四元数转换为欧拉角）

    参数：
        sensor_name: IMU名称（如"IMU1"）
        imu_callback: IMU数据回调函数，格式：callback(imu_data)
    """

    def __init__(self, sensor_name: str,
                 imu_callback: Optional[Callable] = None):
        super().__init__(sensor_name, SensorType.IMU)
        self._imu_callback = imu_callback
        self._data_lock = threading.Lock()

    def __call__(self, client, imu_data) -> None:
        """
        IMU数据回调
        当IMU传感器数据更新时由AirSim客户端自动调用

        参数：
            client: AirSim客户端对象
            imu_data: IMU运动学数据字典
        """
        try:
            if imu_data is not None:
                with self._data_lock:
                    self._latest_raw = imu_data
                # 解析IMU数据
                self._parse_imu_data(imu_data)
                # 发送到UI（节流控制，避免频繁刷新）
                if self._imu_callback and self._latest_data and self._should_update_ui():
                    self._imu_callback(self._latest_data)
        except Exception:
            pass

    def _parse_imu_data(self, imu_data: Dict):
        """
        解析IMU数据，提取加速度、角速度和姿态

        参数：
            imu_data: IMU原始数据字典
        """
        try:
            # 提取线加速度
            linear_accel = imu_data.get("linear_acceleration", {})
            ax = linear_accel.get("x", 0.0)
            ay = linear_accel.get("y", 0.0)
            az = linear_accel.get("z", 0.0)

            # 提取角速度
            angular_vel = imu_data.get("angular_velocity", {})
            wx = angular_vel.get("x", 0.0)
            wy = angular_vel.get("y", 0.0)
            wz = angular_vel.get("z", 0.0)

            # 提取姿态四元数并转换为欧拉角
            orientation = imu_data.get("orientation", {})
            qw = orientation.get("w", 1.0)
            qx = orientation.get("x", 0.0)
            qy = orientation.get("y", 0.0)
            qz = orientation.get("z", 0.0)
            roll, pitch, yaw = self._quaternion_to_euler(qw, qx, qy, qz)

            self._update_data({
                "ax": ax, "ay": ay, "az": az,
                "wx": wx, "wy": wy, "wz": wz,
                "roll": roll, "pitch": pitch, "yaw": yaw,
            })
        except Exception:
            pass

    @staticmethod
    def _quaternion_to_euler(w, x, y, z):
        """
        四元数转欧拉角（度）
        旋转顺序：ZYX（偏航→俯仰→滚转）

        参数：
            w, x, y, z: 四元数分量

        返回：
            (roll, pitch, yaw) 欧拉角，单位：度
        """
        # 滚转角（roll）
        sinr_cosp = 2 * (w * x + y * z)
        cosr_cosp = 1 - 2 * (x * x + y * y)
        roll = math.atan2(sinr_cosp, cosr_cosp)

        # 俯仰角（pitch）
        sinp = 2 * (w * y - z * x)
        sinp = max(-1.0, min(1.0, sinp))
        pitch = math.asin(sinp)

        # 偏航角（yaw）
        siny_cosp = 2 * (w * z + x * y)
        cosy_cosp = 1 - 2 * (y * y + z * z)
        yaw = math.atan2(siny_cosp, cosy_cosp)

        return math.degrees(roll), math.degrees(pitch), math.degrees(yaw)

    def get_display_fields(self) -> Dict[str, str]:
        """获取UI显示字段：姿态角、加速度、角速度"""
        if self._latest_data is None:
            return {
                "滚转角": "N/A", "俯仰角": "N/A", "偏航角": "N/A",
                "加速度X": "N/A", "加速度Y": "N/A", "加速度Z": "N/A",
            }
        p = self._latest_data.payload
        return {
            "滚转角": f"{p.get('roll', 0):.2f}°",
            "俯仰角": f"{p.get('pitch', 0):.2f}°",
            "偏航角": f"{p.get('yaw', 0):.2f}°",
            "加速度X": f"{p.get('ax', 0):.2f}m/s²",
            "加速度Y": f"{p.get('ay', 0):.2f}m/s²",
            "加速度Z": f"{p.get('az', 0):.2f}m/s²",
        }
