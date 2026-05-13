"""
核心模块 - 激光雷达点云UDP分包发送器

本模块实现将LiDAR点云数据以PointCloudMsg二进制格式通过UDP分包发送到指定目标：
LidarUdpSender：独立daemon线程读取最新点云数据，NED→UE5坐标转换后打包发送

PointCloudMsg协议格式（小端序）：
┌──────────┬───────────────┬────────┬──────────────────────────────────┐
│   偏移    │     字段       │  类型  │            说明                  │
├──────────┼───────────────┼────────┼──────────────────────────────────┤
│  Header (44 bytes)                                                    │
│  0        │ seq           │ UINT32 │ 帧序号，递增                      │
│  4        │ stamp_sec     │ UINT32 │ 时间戳秒部分                      │
│  8        │ stamp_nsec    │ UINT32 │ 时间戳纳秒部分                    │
│  12       │ frame_id      │ char[32]│ 坐标系名称，如"lidar"            │
├──────────┼───────────────┼────────┼──────────────────────────────────┤
│  Info (13 bytes)                                                      │
│  44       │ height        │ UINT32 │ 点云高度（无序点云=1）            │
│  48       │ width         │ UINT32 │ 点云宽度（=点数）                 │
│  52       │ points_count  │ UINT32 │ 本包点数                          │
│  56       │ valid         │ UINT8  │ 数据有效标志（1=有效）            │
├──────────┼───────────────┼────────┼──────────────────────────────────┤
│  PointXYZIRGB (21 bytes each)                                         │
│  57+21*i  │ time          │ FLOAT32│ 点的时间偏移                      │
│  61+21*i  │ x             │ FLOAT32│ X坐标（UE5: 前）                  │
│  65+21*i  │ y             │ FLOAT32│ Y坐标（UE5: 右）                  │
│  69+21*i  │ z             │ FLOAT32│ Z坐标（UE5: 上）                  │
│  73+21*i  │ reflectivity  │ UINT8  │ 反射强度（0~255）                 │
│  74+21*i  │ rgb_type      │ INT32  │ 标签/类别（默认0）                │
└──────────┴───────────────┴────────┴──────────────────────────────────┘

坐标系转换说明：
- ProjectAirSim LiDAR数据为NED坐标系：X=北, Y=东, Z=下
- 发送到310P需转为UE5坐标系：X=前(北), Y=右(东), Z=上
- 转换规则：ue5_x = ned_x, ue5_y = ned_y, ue5_z = -ned_z

分包策略：
- UDP单包最大负载65000字节
- 每包最多约3092个点（(65000-44-13)/21 ≈ 3092）
- 超过3092点的帧自动分包发送，每包独立带Header+Info
"""

import socket
import struct
import threading
import time
from typing import Optional

import numpy as np

from .constants import LIDAR_UDP_TARGET_IP, LIDAR_UDP_TARGET_PORT


class LidarUdpSender:
    """
    激光雷达点云UDP分包发送器

    独立daemon线程读取LidarCallback最新点云数据，
    NED→UE5坐标转换后按PointCloudMsg格式打包，通过UDP分包发送

    设计原则（与NavUDPSender一致）：
    - 独立线程：不占用UI/控制线程，不影响现有功能
    - 无锁读取：仅读取latest_lidar_data引用（GIL原子操作），不加锁
    - 无新订阅：复用现有LidarCallback回调数据，零额外网络开销
    - UDP无阻塞：sendto()无连接、无重传、无阻塞风险
    """

    # 全局配置：目标IP、目标端口
    TARGET_IP = LIDAR_UDP_TARGET_IP
    TARGET_PORT = LIDAR_UDP_TARGET_PORT

    # PointCloudMsg协议常量
    HEADER_SIZE = 44       # Header字节数：seq(4) + stamp_sec(4) + stamp_nsec(4) + frame_id(32)
    INFO_SIZE = 13         # Info字节数：height(4) + width(4) + points_count(4) + valid(1)
    POINT_SIZE = 21        # 每个点字节数：time(4) + x(4) + y(4) + z(4) + reflectivity(1) + rgb_type(4)
    MAX_UDP_PAYLOAD = 65000  # UDP单包最大负载字节数
    # 每包最大点数：(65000 - 44 - 13) / 21 ≈ 3092
    MAX_POINTS_PER_PACKET = (MAX_UDP_PAYLOAD - HEADER_SIZE - INFO_SIZE) // POINT_SIZE

    # numpy结构化数组dtype，用于高效打包点云数据（小端序）
    POINT_DTYPE = np.dtype([
        ('time', '<f4'),
        ('x', '<f4'),
        ('y', '<f4'),
        ('z', '<f4'),
        ('reflectivity', 'u1'),
        ('rgb_type', '<i4'),
    ])

    def __init__(self, lidar_callback, log_func=None):
        """
        初始化激光雷达点云UDP发送器

        参数：
            lidar_callback: LidarCallback实例，用于读取最新点云数据
            log_func: 日志函数，格式：log_func(msg, level)
        """
        self._lidar_callback = lidar_callback
        self._log_func = log_func or (lambda msg, level="INFO": None)

        self._socket = None
        self._running = False
        self._thread = None
        self._stop_event = threading.Event()
        self._send_count = 0
        self._udp_seq = 0

    @property
    def is_running(self) -> bool:
        """发送器是否正在运行"""
        return self._running

    @property
    def send_count(self) -> int:
        """已发送的帧数量"""
        return self._send_count

    def start(self):
        """启动UDP发送线程"""
        if self._running:
            return
        try:
            self._socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            # 设置4MB发送缓冲区，适配大点云帧（75000点×21字节≈1.5MB）
            self._socket.setsockopt(socket.SOL_SOCKET, socket.SO_SNDBUF, 4 * 1024 * 1024)
            self._running = True
            self._stop_event.clear()
            self._send_count = 0
            self._udp_seq = 0
            self._thread = threading.Thread(
                target=self._send_loop,
                name="LidarUDP",
                daemon=True,
            )
            self._thread.start()
            self._log_func(
                f"激光点云UDP发送已启动 → {self.TARGET_IP}:{self.TARGET_PORT}",
                "INFO",
            )
        except Exception as e:
            self._running = False
            self._log_func(f"激光点云UDP发送启动失败: {e}", "ERROR")

    def stop(self):
        """停止UDP发送线程"""
        if not self._running:
            return
        self._running = False
        self._stop_event.set()
        if self._thread and self._thread.is_alive():
            self._thread.join(timeout=2.0)
        if self._socket:
            self._socket.close()
            self._socket = None
        self._log_func(
            f"激光点云UDP发送已停止，共发送{self._send_count}帧", "INFO"
        )

    def _send_loop(self):
        """
        发送线程主循环

        有新点云数据时立即打包发送，无数据时短暂休眠避免CPU空转
        与NavUDPSender的定时发送不同，点云发送采用数据驱动模式：
        LiDAR传感器回调频率约10~20Hz，发送线程跟随数据到达节奏
        """
        last_data_id = None
        while not self._stop_event.is_set():
            try:
                # 读取最新点云数据
                lidar_data = None
                if self._lidar_callback:
                    lidar_data = self._lidar_callback.get_latest_lidar_data()

                # 检查是否有新数据（通过对象引用判断，避免重复发送同一帧）
                if lidar_data is None or lidar_data is last_data_id:
                    self._stop_event.wait(0.005)  # 无新数据时休眠5ms
                    continue

                last_data_id = lidar_data

                # 解析点云数据并打包发送
                if "point_cloud" in lidar_data:
                    pc = lidar_data["point_cloud"]
                    if pc and len(pc) > 0 and len(pc) % 3 == 0:
                        pts = np.array(pc, dtype=np.float32).reshape(-1, 3)
                        # 过滤零点（无效点）
                        mask = ~np.all(pts == 0, axis=1)
                        pts = pts[mask]
                        if len(pts) > 0:
                            # NED→UE5坐标转换
                            # NED: X=北, Y=东, Z=下
                            # UE5: X=前(北), Y=右(东), Z=上
                            ue5_pts = np.empty_like(pts)
                            ue5_pts[:, 0] = pts[:, 0]   # 北→前
                            ue5_pts[:, 1] = pts[:, 1]   # 东→右
                            ue5_pts[:, 2] = -pts[:, 2]  # 下→上（取反）
                            # 分包发送
                            self._send_pointcloud(ue5_pts)
                            self._send_count += 1

            except Exception:
                pass
            self._stop_event.wait(0.001)

    def _send_pointcloud(self, pts: np.ndarray):
        """
        按PointCloudMsg格式打包并通过UDP分包发送

        参数：
            pts: Nx3的numpy数组，UE5坐标系下的点云坐标
        """
        total_points = len(pts)
        max_points = self.MAX_POINTS_PER_PACKET

        if total_points <= max_points:
            # 单包发送
            packet = self._pack_pointcloud_msg(pts, self._udp_seq)
            if packet:
                self._socket.sendto(packet, (self.TARGET_IP, self.TARGET_PORT))
        else:
            # 分包发送：每包最多max_points个点
            for start_idx in range(0, total_points, max_points):
                end_idx = min(start_idx + max_points, total_points)
                chunk = pts[start_idx:end_idx]
                packet = self._pack_pointcloud_msg(chunk, self._udp_seq)
                if packet:
                    self._socket.sendto(packet, (self.TARGET_IP, self.TARGET_PORT))
                # 每发10个包短暂休眠1ms，防止网络拥塞
                if (start_idx // max_points + 1) % 10 == 0:
                    time.sleep(0.001)

        self._udp_seq += 1

    def _pack_pointcloud_msg(self, pts: np.ndarray, seq: int,
                             frame_id: str = "lidar") -> Optional[bytes]:
        """
        打包PointCloudMsg二进制格式

        Header (44 bytes):
            uint32 seq, uint32 stamp_sec, uint32 stamp_nsec, char[32] frame_id
        Info (13 bytes):
            uint32 height, uint32 width, uint32 points_count, uint8 valid
        Points (21 bytes each):
            float32 time, float32 x, float32 y, float32 z,
            uint8 reflectivity, int32 rgb_type

        参数：
            pts: Nx3的numpy数组（UE5坐标系）
            seq: 帧序号
            frame_id: 坐标系名称

        返回：
            打包后的字节流，失败返回None
        """
        n = len(pts)
        if n == 0:
            return None

        # 时间戳
        now = time.time()
        stamp_sec = int(now)
        stamp_nsec = int((now - stamp_sec) * 1e9)

        # 打包Header (44 bytes)
        frame_id_bytes = frame_id.encode('utf-8')[:32].ljust(32, b'\x00')
        header = struct.pack("<III", seq, stamp_sec, stamp_nsec) + frame_id_bytes

        # 打包Info (13 bytes)
        info = struct.pack("<IIIB", 1, n, n, 1)

        # 打包点云数据 (21 bytes/point)，使用numpy结构化数组高效打包
        points = np.zeros(n, dtype=self.POINT_DTYPE)
        points['time'] = np.arange(n, dtype=np.float32) * 0.0001  # 时间偏移
        points['x'] = pts[:, 0].astype(np.float32)
        points['y'] = pts[:, 1].astype(np.float32)
        points['z'] = pts[:, 2].astype(np.float32)
        points['reflectivity'] = 0   # 无反射强度数据，默认0
        points['rgb_type'] = 0       # 默认标签

        return header + info + points.tobytes()
