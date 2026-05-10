"""
传感器模块 - 相机回调处理器

本模块实现普通相机和深度相机的数据回调处理：
1. CameraCallback：普通可见光相机（image-type=0）
2. DepthCameraCallback：深度相机（image-type=1/2/4）

功能：
- 解码压缩图像为OpenCV格式
- 缓存最新帧供UI定时拉取（避免跨线程信号风暴）
- 写入视频录像文件
- 拍照功能

性能优化要点：
- 相机回调仅缓存帧，不主动emit信号到UI
- UI通过定时器（QTimer）主动拉取最新帧，控制刷新频率
- 这避免了每帧都通过pyqtSignal跨线程传递2.7MB图像的开销
- 原先4个相机×20fps=80次/秒的跨线程emit，优化后降为0
"""

import threading
import time
import numpy as np
from projectairsim.utils import unpack_image

from .base import SensorCallback, SensorType, SensorData
from typing import Dict, Any, Optional, Callable


class CameraCallback(SensorCallback):
    """
    普通相机回调处理器
    处理可见光相机（image-type=0）的图像数据

    工作流程（优化后）：
    1. 接收AirSim相机消息（压缩图像数据）
    2. 解码为OpenCV BGR格式
    3. 缓存最新帧（线程安全），不主动emit信号
    4. UI通过定时器主动调用get_latest_frame()拉取最新帧
    5. 通过recorder写入视频录像
    6. 拍照时从缓存获取最新帧

    性能优化说明：
    - 原方案：每帧通过frame_callback→pyqtSignal.emit()传递到UI
      4个相机×20fps=80次/秒跨线程信号，每次传递2.7MB图像
    - 新方案：回调仅缓存帧，UI以固定频率（如15fps）主动拉取
      跨线程信号传递降为0，UI刷新频率可控

    参数：
        sensor_name: 相机名称（如"FrontCamera"、"DownCamera"）
        camera_key: 相机标识键（如"front"、"down"、"chase"）
        frame_callback: 帧数据回调函数（保留兼容，但不再用于高频emit）
        recorder: DataRecorder实例，用于录像和拍照
        sensor_type: 传感器类型，默认为CAMERA
        sensor_data_callback: 传感器数据UI回调（节流控制）
    """

    def __init__(self, sensor_name: str, camera_key: str,
                 frame_callback: Optional[Callable] = None,
                 recorder=None,
                 sensor_type: SensorType = SensorType.CAMERA,
                 sensor_data_callback: Optional[Callable] = None):
        super().__init__(sensor_name, sensor_type)
        self._camera_key = camera_key
        self._frame_callback = frame_callback
        self._recorder = recorder
        self._sensor_data_callback = sensor_data_callback
        self._latest_frame: Optional[np.ndarray] = None
        self._frame_lock = threading.Lock()
        # 帧节流：控制录像写入频率，避免磁盘IO成为瓶颈
        # 录像帧率设为15fps，足够流畅且减少磁盘写入压力
        self._last_record_time: float = 0.0
        self._record_interval: float = 1.0 / 15.0
        # 帧更新标志：用于UI拉取时判断是否有新帧
        self._frame_updated: bool = False

    def __call__(self, client, image_msg) -> None:
        """
        相机数据回调
        当相机传感器数据更新时由AirSim客户端自动调用

        优化策略：
        1. 仅缓存最新帧，不主动emit信号到UI（避免跨线程信号风暴）
        2. 录像写入降频到15fps（原每帧写入，磁盘IO压力大）
        3. 传感器面板数据更新走节流通道（0.2秒间隔）

        参数：
            client: AirSim客户端对象
            image_msg: 相机图像消息字典，包含压缩的图像数据
        """
        try:
            if image_msg and "data" in image_msg and len(image_msg["data"]) > 0:
                frame = unpack_image(image_msg)
                if frame is not None:
                    # 缓存最新帧（UI通过定时器主动拉取，不再通过信号推送）
                    with self._frame_lock:
                        self._latest_frame = frame
                        self._frame_updated = True

                    if self._frame_callback:
                        self._frame_callback(self._camera_key, frame)

                    # 录像写入降频：15fps，减少磁盘IO压力
                    now = time.time()
                    if self._recorder and (now - self._last_record_time >= self._record_interval):
                        self._last_record_time = now
                        self._recorder.write_video_frame(self._camera_key, frame)

                    # 更新传感器数据（分辨率、帧数等，用于传感器面板显示）
                    h, w = frame.shape[:2]
                    self._update_data({
                        "width": w,
                        "height": h,
                        "channels": frame.shape[2] if len(frame.shape) == 3 else 1,
                    })
                    # 传感器面板数据更新（已有节流控制，0.2秒间隔）
                    if self._sensor_data_callback and self._latest_data and self._should_update_ui():
                        self._sensor_data_callback(self._latest_data)
        except Exception:
            pass

    def get_latest_frame(self) -> Optional[np.ndarray]:
        """
        获取最新的相机帧（线程安全，零拷贝优化）

        优化说明：
        - 返回帧的引用而非拷贝，减少大图像的内存拷贝开销
        - 调用方（VideoWidget）会在paintEvent中做BGR→RGB转换和缩放
        - 由于UI定时器控制拉取频率（15fps），不存在竞争问题

        返回：
            最新的BGR图像帧，如果没有数据则返回None
        """
        with self._frame_lock:
            return self._latest_frame

    def consume_frame(self) -> Optional[np.ndarray]:
        """
        消费最新帧（拉取模式专用）
        返回最新帧并清除更新标志，避免UI重复处理同一帧

        返回：
            最新的BGR图像帧（如果有新帧），否则返回None
        """
        with self._frame_lock:
            if self._frame_updated:
                self._frame_updated = False
                return self._latest_frame
            return None

    def get_display_fields(self) -> Dict[str, str]:
        """
        获取UI显示字段
        显示相机的分辨率和帧数信息
        """
        if self._latest_data is None:
            return {"分辨率": "N/A", "帧数": "0"}
        p = self._latest_data.payload
        return {
            "分辨率": f"{p.get('width', 0)}x{p.get('height', 0)}",
            "帧数": str(self._data_count),
        }


class DepthCameraCallback(SensorCallback):
    """
    深度相机回调处理器
    处理深度相机（image-type=1/2/4）的深度图像数据

    深度图像特点：
    - pixels-as-float=true，像素值为浮点型
    - 每个像素值表示该像素到相机的距离（米）
    - image-type=1: 平面深度图（正交投影）
    - image-type=2: 透视深度图（透视投影）
    - image-type=4: 深度可视化图（伪彩色显示）

    参数：
        sensor_name: 深度相机名称（如"FrontDepthCamera"）
        camera_key: 相机标识键（如"front_depth"）
        frame_callback: 帧数据回调函数
        recorder: DataRecorder实例
    """

    def __init__(self, sensor_name: str, camera_key: str,
                 frame_callback: Optional[Callable] = None,
                 recorder=None):
        super().__init__(sensor_name, SensorType.DEPTH_CAMERA)
        self._camera_key = camera_key
        self._frame_callback = frame_callback
        self._recorder = recorder
        self._latest_frame: Optional[np.ndarray] = None
        self._frame_lock = threading.Lock()

    def __call__(self, client, image_msg) -> None:
        """
        深度相机数据回调

        参数：
            client: AirSim客户端对象
            image_msg: 深度图像消息字典
        """
        try:
            if image_msg and "data" in image_msg and len(image_msg["data"]) > 0:
                frame = unpack_image(image_msg)
                if frame is not None:
                    with self._frame_lock:
                        self._latest_frame = frame
                    # 计算深度统计信息
                    if len(frame.shape) == 2:
                        depth_min = float(np.min(frame))
                        depth_max = float(np.max(frame))
                        depth_mean = float(np.mean(frame))
                    elif len(frame.shape) == 3 and frame.shape[2] == 1:
                        depth_min = float(np.min(frame))
                        depth_max = float(np.max(frame))
                        depth_mean = float(np.mean(frame))
                    else:
                        depth_min = 0.0
                        depth_max = 0.0
                        depth_mean = 0.0
                    self._update_data({
                        "depth_min": depth_min,
                        "depth_max": depth_max,
                        "depth_mean": depth_mean,
                        "width": frame.shape[1],
                        "height": frame.shape[0],
                    })
        except Exception:
            pass

    def get_latest_frame(self) -> Optional[np.ndarray]:
        """获取最新的深度帧（线程安全）"""
        with self._frame_lock:
            return self._latest_frame.copy() if self._latest_frame is not None else None

    def get_display_fields(self) -> Dict[str, str]:
        """获取UI显示字段：深度范围和平均深度"""
        if self._latest_data is None:
            return {"最近距离": "N/A", "最远距离": "N/A", "平均深度": "N/A"}
        p = self._latest_data.payload
        return {
            "最近距离": f"{p.get('depth_min', 0):.2f}m",
            "最远距离": f"{p.get('depth_max', 0):.2f}m",
            "平均深度": f"{p.get('depth_mean', 0):.2f}m",
        }
