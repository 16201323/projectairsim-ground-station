"""
传感器模块 - 大气机回调处理器

本模块实现大气机（大气数据计算机）的回调处理：
AtmosphereCallback：组合处理气压计和空速传感器数据

大气机说明：
- 大气机是航空器上的综合大气数据系统
- 通过气压计获取气压高度和升降速率
- 通过空速管获取指示空速
- 组合输出：气压高度、指示空速、升降速率、外部温度等

ProjectAirSim中的传感器topic：
- barometer: 气压计数据（气压高度、气压值）
- airspeed: 空速传感器数据（指示空速、差压）
"""

import threading
import math

from .base import SensorCallback, SensorType, SensorData
from typing import Dict, Any, Optional, Callable


class AtmosphereCallback(SensorCallback):
    """
    大气机回调处理器
    组合处理气压计和空速传感器数据，提供完整的大气数据

    工作流程：
    1. 分别接收气压计和空速传感器数据
    2. 提取气压高度、指示空速等信息
    3. 通过atmosphere_callback发送到UI显示
    4. 计算综合大气参数

    参数：
        sensor_name: 大气机名称（如"Atmosphere"）
        atmosphere_callback: 大气数据回调函数
    """

    def __init__(self, sensor_name: str,
                 atmosphere_callback: Optional[Callable] = None):
        super().__init__(sensor_name, SensorType.BAROMETER)
        self._atmosphere_callback = atmosphere_callback
        self._data_lock = threading.Lock()
        # 缓存气压和空速数据
        self._barometer_data: Optional[Dict] = None
        self._airspeed_data: Optional[Dict] = None

    def on_barometer(self, client, baro_data) -> None:
        """
        气压计数据回调
        当气压计传感器数据更新时调用

        性能优化：先检查节流，避免不必要的数据解析
        """
        try:
            if baro_data is not None:
                # 先检查节流
                if not self._should_update_ui():
                    return
                # 解析气压数据
                self._parse_barometer(baro_data)
        except Exception:
            pass

    def on_airspeed(self, client, airspeed_data) -> None:
        """
        空速传感器数据回调
        当空速传感器数据更新时调用

        性能优化：先检查节流，避免不必要的数据解析
        """
        try:
            if airspeed_data is not None:
                # 先检查节流
                if not self._should_update_ui():
                    return
                # 解析空速数据
                self._parse_airspeed(airspeed_data)
        except Exception:
            pass

    def __call__(self, client, data) -> None:
        """
        大气机主回调（保留接口兼容性）
        实际使用中应分别调用on_barometer和on_airspeed

        参数：
            client: AirSim客户端对象
            data: 传感器数据
        """
        pass

    def _parse_barometer(self, baro_data: Dict):
        """
        解析气压计数据

        ProjectAirSim气压计订阅数据格式：
        - altitude：气压高度（米）
        - pressure：当前气压值（Pa）
        - qnh：修正海平面气压（Pa）

        参数：
            baro_data: 气压计原始数据
        """
        try:
            altitude = baro_data.get("altitude", 0.0)
            pressure = baro_data.get("pressure", 101325.0)
            qnh = baro_data.get("qnh", 101325.0)

            self._update_data({
                "baro_altitude": altitude,
                "pressure": pressure,
                "qnh": qnh,
                "airspeed": self._latest_data.payload.get("airspeed", 0.0) if self._latest_data else 0.0,
                "diff_pressure": self._latest_data.payload.get("diff_pressure", 0.0) if self._latest_data else 0.0,
            })

            # 发送到UI（外层on_barometer已做节流控制）
            if self._atmosphere_callback:
                self._atmosphere_callback(self._latest_data)
        except Exception:
            pass

    def _parse_airspeed(self, airspeed_data: Dict):
        """
        解析空速传感器数据

        ProjectAirSim空速传感器订阅数据格式：
        - diff_pressure：差压（Pa），皮托管与静压孔的压差
        - 没有airspeed字段！需要从差压计算指示空速

        指示空速计算公式（伯努利方程）：
        IAS = sqrt(2 * diff_pressure / rho)
        其中 rho = 1.225 kg/m³（标准海平面空气密度）

        参数：
            airspeed_data: 空速原始数据
        """
        try:
            diff_pressure = airspeed_data.get("diff_pressure", 0.0)

            # 从差压计算指示空速（伯努利方程）
            # IAS = sqrt(2 * q / rho)，q=差压，rho=空气密度
            RHO = 1.225  # 标准海平面空气密度 kg/m³
            if diff_pressure > 0:
                airspeed = math.sqrt(2.0 * diff_pressure / RHO)
            else:
                airspeed = 0.0

            # 更新数据（保留气压数据）
            old_payload = self._latest_data.payload if self._latest_data else {}
            self._update_data({
                "baro_altitude": old_payload.get("baro_altitude", 0.0),
                "pressure": old_payload.get("pressure", 101325.0),
                "qnh": old_payload.get("qnh", 101325.0),
                "airspeed": airspeed,
                "diff_pressure": diff_pressure,
            })

            # 发送到UI（外层on_airspeed已做节流控制）
            if self._atmosphere_callback:
                self._atmosphere_callback(self._latest_data)
        except Exception:
            pass

    def get_display_fields(self) -> Dict[str, str]:
        """获取UI显示字段：气压高度、指示空速、气压、QNH、差压"""
        if self._latest_data is None:
            return {
                "气高": "N/A", "空速": "N/A",
                "气压": "N/A", "QNH": "N/A", "差压": "N/A",
            }
        p = self._latest_data.payload
        pressure_pa = p.get('pressure', 101325.0)
        pressure_hpa = pressure_pa / 100.0
        qnh_pa = p.get('qnh', 101325.0)
        qnh_hpa = qnh_pa / 100.0
        return {
            "气高": f"{p.get('baro_altitude', 0):.1f}m",
            "空速": f"{p.get('airspeed', 0):.2f}m/s",
            "气压": f"{pressure_hpa:.2f}hPa",
            "QNH": f"{qnh_hpa:.2f}hPa",
            "差压": f"{p.get('diff_pressure', 0):.1f}Pa",
        }
