"""传感器模块 - 双目相机回调处理器

本模块实现双目相机的数据回调处理：
StereoCameraCallback：管理左右两个相机的数据同步和视差图计算

双目相机原理：
- 左右两个相机平行安装，间距为基线距离(baseline)
- 通过左右图像的视差(disparity)计算深度信息
- 深度 = 基线距离 × 焦距 / 视差

本实现中，双目相机由两个独立的camera传感器组成：
- LeftCamera：左相机（偏左安装）
- RightCamera：右相机（偏右安装）
- 回调处理器分别接收左右相机数据，并可选计算视差图

性能优化要点：
- 左右相机帧仅缓存，不主动emit信号到UI
- UI通过定时器主动拉取get_latest_left_frame()/get_latest_right_frame()
- 录像写入降频到15fps，减少磁盘IO压力
"""

import threading
import time
import numpy as np
from projectairsim.utils import unpack_image

from .base import SensorCallback, SensorType, SensorData
from typing import Dict, Any, Optional, Callable


class StereoCameraCallback(SensorCallback):
    """
    双目相机回调处理器
    管理左右两个相机的数据接收、同步和视差图计算

    工作流程：
    1. 分别接收左右相机的图像数据
    2. 缓存左右帧到独立缓冲区
    3. 可选：使用SGBM算法计算视差图
    4. 通过回调发送帧到UI显示
    5. 支持同时保存左右相机照片

    参数：
        sensor_name: 双目相机组名称（如"StereoCamera"）
        frame_callback: 帧数据回调函数，格式：callback(camera_key, frame)
        recorder: DataRecorder实例
        baseline: 双目基线距离（米），默认0.12m
        compute_disparity: 是否计算视差图，默认False（计算量大）
    """

    def __init__(self, sensor_name: str,
                 frame_callback: Optional[Callable] = None,
                 recorder=None,
                 baseline: float = 0.12,
                 compute_disparity: bool = False):
        super().__init__(sensor_name, SensorType.STEREO_CAMERA)
        self._frame_callback = frame_callback
        self._recorder = recorder
        self._baseline = baseline
        self._compute_disparity = compute_disparity
        # 左右相机帧缓存
        self._left_frame: Optional[np.ndarray] = None
        self._right_frame: Optional[np.ndarray] = None
        self._disparity_map: Optional[np.ndarray] = None
        # 左右相机分辨率缓存
        self._left_resolution: str = "N/A"
        self._right_resolution: str = "N/A"
        self._left_lock = threading.Lock()
        self._right_lock = threading.Lock()
        # 录像降频：15fps，减少磁盘IO压力
        self._last_record_time_left: float = 0.0
        self._last_record_time_right: float = 0.0
        self._record_interval: float = 1.0 / 15.0
        # SGBM匹配器（延迟初始化，需要cv2）
        self._matcher = None
        if compute_disparity:
            self._init_matcher()

    def _init_matcher(self):
        """
        初始化SGBM视差匹配器
        SGBM（Semi-Global Block Matching）是OpenCV中的半全局匹配算法
        相比BM算法，SGBM在纹理较少区域和边界处效果更好
        """
        try:
            import cv2
            self._matcher = cv2.StereoSGBM_create(
                minDisparity=0,
                numDisparities=64,
                blockSize=11,
                P1=8 * 3 * 11 ** 2,
                P2=32 * 3 * 11 ** 2,
                disp12MaxDiff=1,
                uniquenessRatio=10,
                speckleWindowSize=100,
                speckleRange=32,
            )
        except ImportError:
            self._matcher = None

    def on_left_frame(self, client, image_msg) -> None:
        """
        左相机数据回调
        当左相机传感器数据更新时调用

        优化策略：
        - 仅缓存帧，不主动emit信号到UI（UI通过定时器拉取）
        - 录像写入降频到15fps

        参数：
            client: AirSim客户端对象
            image_msg: 左相机图像消息字典
        """
        try:
            # 从消息顶层提取分辨率（unpack_image源码确认width/height是顶层字段）
            w = image_msg.get("width", 0) if image_msg else 0
            h = image_msg.get("height", 0) if image_msg else 0
            if w > 0 and h > 0:
                res_str = f"{w}x{h}"
                if self._left_resolution != res_str:
                    self._left_resolution = res_str

            if image_msg and "data" in image_msg and len(image_msg["data"]) > 0:
                frame = unpack_image(image_msg)
                if frame is not None:
                    # 缓存最新帧（UI通过定时器主动拉取）
                    with self._left_lock:
                        self._left_frame = frame

                    if self._frame_callback:
                        self._frame_callback("stereo_left", frame)

                    # 录像降频：15fps
                    now = time.time()
                    if self._recorder and (now - self._last_record_time_left >= self._record_interval):
                        self._last_record_time_left = now
                        self._recorder.write_video_frame("stereo_left", frame)
                    # 更新基本传感器数据（基线距离等）
                    if self._latest_data is None:
                        self._update_data({"baseline": self._baseline})
                        if self._should_update_ui():
                            self._notify_sensor_data()
                    # 尝试计算视差图
                    self._try_compute_disparity()
        except Exception:
            pass

    def on_right_frame(self, client, image_msg) -> None:
        """
        右相机数据回调
        当右相机传感器数据更新时调用

        优化策略：
        - 仅缓存帧，不主动emit信号到UI（UI通过定时器拉取）
        - 录像写入降频到15fps

        参数：
            client: AirSim客户端对象
            image_msg: 右相机图像消息字典
        """
        try:
            # 从消息顶层提取分辨率（unpack_image源码确认width/height是顶层字段）
            w = image_msg.get("width", 0) if image_msg else 0
            h = image_msg.get("height", 0) if image_msg else 0
            if w > 0 and h > 0:
                res_str = f"{w}x{h}"
                if self._right_resolution != res_str:
                    self._right_resolution = res_str

            if image_msg and "data" in image_msg and len(image_msg["data"]) > 0:
                frame = unpack_image(image_msg)
                if frame is not None:
                    # 缓存最新帧（UI通过定时器主动拉取）
                    with self._right_lock:
                        self._right_frame = frame

                    if self._frame_callback:
                        self._frame_callback("stereo_right", frame)

                    # 录像降频：15fps
                    now = time.time()
                    if self._recorder and (now - self._last_record_time_right >= self._record_interval):
                        self._last_record_time_right = now
                        self._recorder.write_video_frame("stereo_right", frame)
                    # 尝试计算视差图
                    self._try_compute_disparity()
        except Exception:
            pass

    def _notify_sensor_data(self):
        """
        通知UI更新传感器面板数据
        通过_stereo_data_callback将SensorData传递到UI
        """
        if hasattr(self, '_stereo_data_callback') and self._stereo_data_callback:
            if self._latest_data:
                self._stereo_data_callback(self._latest_data)

    def __call__(self, client, data) -> None:
        """
        双目相机主回调（保留接口兼容性）
        实际使用中应分别调用on_left_frame和on_right_frame

        参数：
            client: AirSim客户端对象
            data: 传感器数据
        """
        pass

    def _try_compute_disparity(self):
        """
        尝试计算视差图
        当左右帧都有效时，使用SGBM算法计算视差图
        视差图可用于后续的深度估计
        """
        if self._matcher is None:
            return
        try:
            import cv2
            with self._left_lock, self._right_lock:
                left = self._left_frame
                right = self._right_frame
            if left is None or right is None:
                return
            if left.shape != right.shape:
                return
            # 转换为灰度图
            if len(left.shape) == 3:
                left_gray = cv2.cvtColor(left, cv2.COLOR_BGR2GRAY)
                right_gray = cv2.cvtColor(right, cv2.COLOR_BGR2GRAY)
            else:
                left_gray = left
                right_gray = right
            # 计算视差
            disparity = self._matcher.compute(left_gray, right_gray)
            disparity = disparity.astype(np.float32) / 16.0
            self._disparity_map = disparity
            # 更新传感器数据
            valid = disparity[disparity > 0]
            if len(valid) > 0:
                self._update_data({
                    "baseline": self._baseline,
                    "disparity_min": float(np.min(valid)),
                    "disparity_max": float(np.max(valid)),
                    "disparity_mean": float(np.mean(valid)),
                })
                # 通知UI更新传感器面板
                self._notify_sensor_data()
        except Exception:
            pass

    def get_disparity(self) -> Optional[np.ndarray]:
        """
        获取最新的视差图

        返回：
            视差图（浮点型，单位：像素），如果没有则返回None
        """
        return self._disparity_map

    def get_latest_left_frame(self) -> Optional[np.ndarray]:
        """
        获取最新的左相机帧（拉取模式，线程安全）

        返回：
            最新的BGR图像帧，如果没有数据则返回None
        """
        with self._left_lock:
            return self._left_frame

    def get_latest_right_frame(self) -> Optional[np.ndarray]:
        """
        获取最新的右相机帧（拉取模式，线程安全）

        返回：
            最新的BGR图像帧，如果没有数据则返回None
        """
        with self._right_lock:
            return self._right_frame

    def get_depth_map(self, focal_length: float) -> Optional[np.ndarray]:
        """
        从视差图计算深度图
        深度 = 基线距离 × 焦距 / 视差

        参数：
            focal_length: 相机焦距（像素）

        返回：
            深度图（浮点型，单位：米），如果没有视差图则返回None
        """
        if self._disparity_map is None:
            return None
        disparity = np.where(self._disparity_map > 0, self._disparity_map, 0.001)
        depth = (self._baseline * focal_length) / disparity
        return depth

    def capture_stereo_photo(self) -> bool:
        """
        同时保存左右相机照片

        返回：
            True表示保存成功，False表示没有有效帧
        """
        if self._recorder is None:
            return False
        with self._left_lock:
            left = self._left_frame.copy() if self._left_frame is not None else None
        with self._right_lock:
            right = self._right_frame.copy() if self._right_frame is not None else None
        if left is not None and right is not None:
            self._recorder.save_photo("stereo_left", left)
            self._recorder.save_photo("stereo_right", right)
            return True
        return False

    def get_display_fields(self) -> Dict[str, str]:
        """获取UI显示字段：基线距离、视差、左右相机分辨率"""
        fields = {
            "基线": f"{self._baseline:.3f}m",
            "左相机": self._left_resolution,
            "右相机": self._right_resolution,
        }
        if self._latest_data is not None:
            p = self._latest_data.payload
            fields["视差"] = f"{p.get('disparity_min', 0):.1f}~{p.get('disparity_max', 0):.1f}px"
        else:
            fields["视差"] = "N/A"
        return fields
