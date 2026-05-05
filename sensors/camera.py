"""
传感器模块 - 相机回调处理器

本模块实现普通相机和深度相机的数据回调处理：
1. CameraCallback：普通可见光相机（image-type=0）
2. DepthCameraCallback：深度相机（image-type=1/2/4）

功能：
- 解码压缩图像为OpenCV格式
- 发送帧到UI进行显示
- 写入视频录像文件
- 缓存最新帧用于拍照功能
"""

import threading
import numpy as np
from projectairsim.utils import unpack_image

from .base import SensorCallback, SensorType, SensorData
from typing import Dict, Any, Optional, Callable


class CameraCallback(SensorCallback):
    """
    普通相机回调处理器
    处理可见光相机（image-type=0）的图像数据

    工作流程：
    1. 接收AirSim相机消息（压缩图像数据）
    2. 解码为OpenCV BGR格式
    3. 通过frame_callback发送到UI显示
    4. 通过recorder写入视频录像
    5. 缓存最新帧用于拍照

    参数：
        sensor_name: 相机名称（如"FrontCamera"、"DownCamera"）
        camera_key: 相机标识键（如"front"、"down"、"chase"）
        frame_callback: 帧数据回调函数，格式：callback(camera_key, frame)
        recorder: DataRecorder实例，用于录像和拍照
        sensor_type: 传感器类型，默认为CAMERA
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

    def __call__(self, client, image_msg) -> None:
        """
        相机数据回调
        当相机传感器数据更新时由AirSim客户端自动调用

        参数：
            client: AirSim客户端对象
            image_msg: 相机图像消息字典，包含压缩的图像数据
        """
        try:
            if image_msg and "data" in image_msg and len(image_msg["data"]) > 0:
                frame = unpack_image(image_msg)
                if frame is not None:
                    # 发送到UI显示
                    if self._frame_callback:
                        self._frame_callback(self._camera_key, frame)
                    # 写入视频录像
                    if self._recorder:
                        self._recorder.write_video_frame(self._camera_key, frame)
                    # 缓存最新帧
                    with self._frame_lock:
                        self._latest_frame = frame
                    # 更新传感器数据
                    h, w = frame.shape[:2]
                    self._update_data({
                        "width": w,
                        "height": h,
                        "channels": frame.shape[2] if len(frame.shape) == 3 else 1,
                    })
                    # 通知UI更新传感器面板
                    if self._sensor_data_callback and self._latest_data:
                        self._sensor_data_callback(self._latest_data)
        except Exception:
            pass

    def get_latest_frame(self) -> Optional[np.ndarray]:
        """
        获取最新的相机帧（线程安全）

        返回：
            最新的BGR图像帧，如果没有数据则返回None
        """
        with self._frame_lock:
            return self._latest_frame.copy() if self._latest_frame is not None else None

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
