"""
传感器模块 - GPS回调处理器

本模块实现GPS全球定位系统的数据回调处理：
GPSCallback：处理GPS位置、速度和时间数据

GPS传感器说明：
- 提供全球定位信息（经度、纬度、海拔）
- 提供速度信息（北向、东向、垂直速度）
- 提供时间信息和卫星状态
- 精度受卫星几何分布和信号质量影响

ProjectAirSim中GPS传感器的topic：
- gps: GPS定位数据
"""

import threading

from .base import SensorCallback, SensorType, SensorData
from typing import Dict, Any, Optional, Callable


class GPSCallback(SensorCallback):
    """
    GPS全球定位系统回调处理器
    处理GPS位置、速度和时间数据

    工作流程：
    1. 接收GPS定位数据
    2. 提取经纬度、海拔、速度等信息
    3. 通过gps_callback发送到UI显示
    4. 计算定位精度和速度信息

    参数：
        sensor_name: GPS名称（如"GPS"）
        gps_callback: GPS数据回调函数，格式：callback(gps_data)
    """

    def __init__(self, sensor_name: str,
                 gps_callback: Optional[Callable] = None):
        super().__init__(sensor_name, SensorType.GPS)
        self._gps_callback = gps_callback
        self._data_lock = threading.Lock()
        # GPS坐标补偿偏移（UDP模式下飞控经纬度与场景原点的差值）
        self._gps_lat_offset = 0.0
        self._gps_lon_offset = 0.0

    def set_geo_offset(self, lat_offset: float, lon_offset: float):
        """设置GPS坐标补偿偏移（由control_thread在UDP首包后调用）"""
        self._gps_lat_offset = lat_offset
        self._gps_lon_offset = lon_offset

    def __call__(self, client, gps_data) -> None:
        """
        GPS数据回调
        当GPS传感器数据更新时由AirSim客户端自动调用

        性能优化：始终解析数据更新_latest_data，仅UI回调做节流

        参数：
            client: AirSim客户端对象
            gps_data: GPS定位数据字典
        """
        try:
            if gps_data is not None:
                # 始终解析数据，确保_latest_data为最新值
                # 外部模块（如NavUDPSender）以100Hz读取_latest_data，必须始终最新
                self._parse_gps_data(gps_data)
                # 仅UI回调做节流，避免5Hz以上的信号emit拖慢UI线程
                if self._should_update_ui():
                    if self._gps_callback and self._latest_data:
                        self._gps_callback(self._latest_data)
        except Exception:
            pass

    def _parse_gps_data(self, gps_data: Dict):
        """
        解析GPS数据，提取位置和速度信息

        ProjectAirSim GPS订阅数据格式（与service API不同）：
        - latitude/longitude/altitude：顶层字段，不在geo_point子字典中
        - velocity：包含x(北向)/y(东向)/z(下向)的速度字典
        - time_utc_millis：UTC时间（毫秒），不是time_utc
        - eph/epv：水平/垂直精度估计
        - fix_type：定位类型（0=无定位, 2=2D, 3=3D）
        - position_cov_type：协方差类型
        - 无num_satellites字段（ProjectAirSim不提供卫星数量）

        参数：
            gps_data: GPS原始数据字典
        """
        try:
            # 提取位置信息
            # ProjectAirSim GPS数据中经纬度是顶层字段，不在geo_point中
            latitude = gps_data.get("latitude", 0.0) + self._gps_lat_offset
            longitude = gps_data.get("longitude", 0.0) + self._gps_lon_offset
            altitude = gps_data.get("altitude", 0.0)

            # 提取速度信息
            # velocity字典：x=北向速度, y=东向速度, z=下向速度（NED坐标系）
            velocity = gps_data.get("velocity", {})
            vn = velocity.get("x", 0.0)
            ve = velocity.get("y", 0.0)
            vd = velocity.get("z", 0.0)

            # 计算水平速度
            import math
            speed = math.sqrt(vn ** 2 + ve ** 2)

            # 提取时间信息
            # ProjectAirSim使用time_utc_millis（毫秒），不是time_utc
            time_utc_millis = gps_data.get("time_utc_millis", 0)

            # 提取精度信息
            eph = gps_data.get("eph", 0.0)
            epv = gps_data.get("epv", 0.0)

            # 提取定位类型
            # fix_type: 0=无定位, 2=2D定位, 3=3D定位
            fix_type = gps_data.get("fix_type", 0)

            self._update_data({
                "latitude": latitude,
                "longitude": longitude,
                "altitude": altitude,
                "vn": vn, "ve": ve, "vd": vd,
                "speed": speed,
                "time_utc_millis": time_utc_millis,
                "eph": eph,
                "epv": epv,
                "fix_type": fix_type,
            })
        except Exception:
            pass

    def get_display_fields(self) -> Dict[str, str]:
        """获取UI显示字段：经纬度、海拔、速度、定位类型"""
        if self._latest_data is None:
            return {
                "纬度": "N/A", "经度": "N/A", "海拔": "N/A",
                "地速": "N/A", "定位": "N/A",
            }
        p = self._latest_data.payload
        fix_map = {0: "无定位", 2: "2D", 3: "3D"}
        fix_str = fix_map.get(p.get("fix_type", 0), "未知")
        return {
            "纬度": f"{p.get('latitude', 0):.6f}°",
            "经度": f"{p.get('longitude', 0):.6f}°",
            "海拔": f"{p.get('altitude', 0):.1f}m",
            "地速": f"{p.get('speed', 0):.2f}m/s",
            "定位": fix_str,
        }
