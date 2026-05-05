"""
核心模块 - 数据记录管理器

本模块实现所有传感器数据和日志的本地保存：
DataRecorder：统一管理视频录像、拍照、LiDAR点云、控制日志

保存内容包括：
1. 视频录像：前视/下视相机的持续录像（AVI格式，XVID编码）
2. 拍照图片：手动触发的相机快照（JPG格式）
3. LiDAR点云：点云数据快照（NPY+PCD+LAS三种格式）
4. 控制日志：所有控制指令的时间序列记录（CSV格式）

目录结构：
mine/drone_data/{timestamp}/
├── images/          # 拍照图片
├── videos/          # 视频录像
└── lidar/           # LiDAR点云
mine/drone_data/logs/{timestamp}/
└── control_log.csv  # 控制指令日志
"""

import csv
import os
import struct
import threading
from datetime import datetime
from pathlib import Path

import cv2
import numpy as np

from .constants import (
    DATA_BASE_DIR, LOG_SAVE_DIR, CAMERA_WIDTH, CAMERA_HEIGHT, VIDEO_FPS
)


class DataRecorder:
    """
    数据记录管理器：管理所有传感器数据和日志的本地保存

    保存内容包括：
    1. 视频录像：前视/下视相机的持续录像（AVI格式，XVID编码）
    2. 拍照图片：手动触发的相机快照（JPG格式）
    3. LiDAR点云：点云数据快照（NPY+PCD+LAS三种格式）
    4. 控制日志：所有控制指令的时间序列记录（CSV格式）

    目录结构：
    mine/drone_data/{timestamp}/
    ├── images/          # 拍照图片
    ├── videos/          # 视频录像
    └── lidar/           # LiDAR点云
    mine/drone_data/logs/{timestamp}/
    └── control_log.csv  # 控制指令日志
    """

    def __init__(self, base_dir=DATA_BASE_DIR, log_dir=LOG_SAVE_DIR):
        """
        初始化数据记录管理器

        参数：
            base_dir: 数据保存根目录
            log_dir: 日志保存路径
        """
        # 创建以时间戳命名的会话目录，每次飞行生成独立目录
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        self.session_dir = os.path.join(base_dir, timestamp)
        self.images_dir = os.path.join(self.session_dir, "images")    # 拍照图片目录
        self.videos_dir = os.path.join(self.session_dir, "videos")    # 视频录像目录
        self.lidar_dir = os.path.join(self.session_dir, "lidar")      # LiDAR点云目录
        self.logs_dir = os.path.join(log_dir, timestamp)              # 控制日志目录
        for dir_path in [self.images_dir, self.videos_dir, self.lidar_dir, self.logs_dir]:
            os.makedirs(dir_path, exist_ok=True)
        # 录像写入器（前视/下视相机各一个）
        self.front_video_writer = None
        self.down_video_writer = None

        # 控制日志CSV文件路径，记录每次控制指令的详细信息
        self.control_log_path = os.path.join(self.logs_dir, "control_log.csv")

        # 保持CSV文件句柄打开，避免每次写入都open/close
        self._log_file = open(self.control_log_path, "w", encoding="utf-8")
        self._log_file.write("timestamp,mode,vx,vy,vz,yaw_rate,pos_x,pos_y,pos_z\n")
        self._log_file.flush()

        # 线程锁，保护日志文件的并发写入
        self.log_lock = threading.Lock()

        # 拍照计数器，用于生成递增的文件名
        self.front_photo_count = 0
        self.down_photo_count = 0
        self.chase_photo_count = 0

        # LiDAR保存计数器，用于生成递增的文件名
        self.lidar_save_count = 0

    def init_video_writers(self):
        """
        初始化视频写入器
        使用XVID编码器，960x540分辨率，30fps帧率
        前视和下视相机各创建一个独立的录像文件
        如果写入器创建失败则置为None，避免后续写入报错
        """
        fourcc = cv2.VideoWriter_fourcc(*"XVID")
        front_path = os.path.join(self.videos_dir, f"front_{datetime.now().strftime('%H%M%S')}.avi")
        self.front_video_writer = cv2.VideoWriter(front_path, fourcc, VIDEO_FPS, (CAMERA_WIDTH, CAMERA_HEIGHT))
        if not self.front_video_writer.isOpened():
            self.front_video_writer = None
        down_path = os.path.join(self.videos_dir, f"down_{datetime.now().strftime('%H%M%S')}.avi")
        self.down_video_writer = cv2.VideoWriter(down_path, fourcc, VIDEO_FPS, (CAMERA_WIDTH, CAMERA_HEIGHT))
        if not self.down_video_writer.isOpened():
            self.down_video_writer = None

    def write_video_frame(self, camera_name, frame):
        """
        写入视频帧到录像文件
        在传感器回调中持续调用，实现持续录像功能

        参数：
            camera_name: 相机名称 ("front" 前视 或 "down" 下视)
            frame: OpenCV格式的BGR图像帧
        """
        if frame is None:
            return
        try:
            h, w = frame.shape[:2]
            if w != CAMERA_WIDTH or h != CAMERA_HEIGHT:
                frame = cv2.resize(frame, (CAMERA_WIDTH, CAMERA_HEIGHT))
            if camera_name == "front" and self.front_video_writer and self.front_video_writer.isOpened():
                self.front_video_writer.write(frame)
            elif camera_name == "down" and self.down_video_writer and self.down_video_writer.isOpened():
                self.down_video_writer.write(frame)
        except Exception:
            pass

    def save_photo(self, camera_name, frame):
        """
        保存拍照图片到磁盘
        文件名格式：{相机}_{序号}_{时间戳}.jpg

        参数：
            camera_name: 相机名称 ("front" 前视, "down" 下视, "chase" 第三人称)
            frame: OpenCV格式的BGR图像帧
        """
        if frame is None:
            return
        try:
            ts = datetime.now().strftime("%Y%m%d_%H%M%S_%f")
            if camera_name == "front":
                self.front_photo_count += 1
                fn = f"front_{self.front_photo_count:04d}_{ts}.jpg"
            elif camera_name == "down":
                self.down_photo_count += 1
                fn = f"down_{self.down_photo_count:04d}_{ts}.jpg"
            else:
                self.chase_photo_count += 1
                fn = f"chase_{self.chase_photo_count:04d}_{ts}.jpg"
            cv2.imwrite(os.path.join(self.images_dir, fn), frame)
        except Exception:
            pass

    def save_lidar_point_cloud(self, lidar_data):
        """
        保存LiDAR点云数据到磁盘
        同时保存为三种格式，满足不同使用场景：
        - NPY：NumPy原始数据格式，方便Python直接读取和处理
        - PCD：PCL标准格式，可被CloudCompare、PCL、Open3D等工具读取
        - LAS：测绘行业标准格式，可被LAStools、QGIS、ArcGIS等工具读取

        参数：
            lidar_data: LiDAR传感器数据字典，包含"point_cloud"字段
                        point_cloud为一维数组，每3个值为一个点的(x,y,z)坐标
        """
        if lidar_data is None:
            return
        try:
            self.lidar_save_count += 1
            ts = datetime.now().strftime("%Y%m%d_%H%M%S_%f")
            base = f"lidar_{self.lidar_save_count:04d}_{ts}"
            if "point_cloud" in lidar_data:
                pts = np.array(lidar_data["point_cloud"])
                if len(pts) > 0 and len(pts) % 3 == 0:
                    pts = pts.reshape(-1, 3)
                    np.save(os.path.join(self.lidar_dir, f"{base}.npy"), pts)
                    self._save_pcd(os.path.join(self.lidar_dir, f"{base}.pcd"), pts)
                    self._save_las(os.path.join(self.lidar_dir, f"{base}.las"), pts)
        except Exception:
            pass

    def _save_pcd(self, filepath, points):
        """
        保存点云为PCD（Point Cloud Data）格式
        PCD是PCL（Point Cloud Library）的标准点云格式
        采用ASCII模式，兼容性最好，可被CloudCompare、PCL、Open3D等工具读取

        PCD文件结构：
        - 头部：版本号、字段定义、点数等元信息
        - 数据：每行一个点的xyz坐标，空格分隔

        参数：
            filepath: 保存文件路径
            points: Nx3的numpy数组，每行为(x, y, z)坐标
        """
        n = len(points)
        with open(filepath, "w", encoding="utf-8") as f:
            f.write("# .PCD v0.7 - Point Cloud Data file format\nVERSION 0.7\n")
            f.write("FIELDS x y z\nSIZE 4 4 4\nTYPE F F F\nCOUNT 1 1 1\n")
            f.write(f"WIDTH {n}\nHEIGHT 1\nVIEWPOINT 0 0 0 1 0 0 0\n")
            f.write(f"POINTS {n}\nDATA ascii\n")
            np.savetxt(f, points, fmt="%.6f", delimiter=" ")

    def _save_las(self, filepath, points):
        """
        保存点云为LAS（LASer）格式
        LAS是ASPRS（美国摄影测量与遥感学会）制定的测绘行业标准点云格式
        采用LAS 1.2版本，点数据记录格式0（仅XYZ坐标+基本属性）
        可被CloudCompare、LAStools、QGIS、ArcGIS等专业工具读取

        LAS文件结构：
        - 公共头部（227字节）：文件签名、版本、点数、缩放因子、偏移量等
        - 点数据记录：每点20字节，包含XYZ整数坐标、强度、回波、分类等

        坐标存储原理：
        LAS使用"整数+缩放因子+偏移量"方式存储坐标，保证精度
        实际坐标 = 整数值 × 缩放因子 + 偏移量
        缩放因子0.0001表示0.1毫米精度

        参数：
            filepath: 保存文件路径
            points: Nx3的numpy数组，每行为(x, y, z)坐标
        """
        n = len(points)
        # 计算坐标范围，用于LAS头部的偏移量和极值字段
        x_min, y_min, z_min = float(np.min(points[:,0])), float(np.min(points[:,1])), float(np.min(points[:,2]))
        x_max, y_max, z_max = float(np.max(points[:,0])), float(np.max(points[:,1])), float(np.max(points[:,2]))
        # 缩放因子：0.0001米精度（0.1毫米），将浮点坐标转换为整数存储
        scale = 0.0001
        # 构建LAS 1.2公共头部块（固定227字节）
        hdr = bytearray(227)
        # 文件签名 "LASF"（4字节），LAS格式的魔数标识
        hdr[0:4] = b"LASF"
        # 文件源ID（2字节）：数据来源标识，0表示默认
        struct.pack_into("<H", hdr, 4, 0)
        # 全局编码（2字节）：坐标系信息，0表示未指定
        struct.pack_into("<H", hdr, 6, 0)
        # 项目ID GUID（16字节）：唯一标识符，0表示未指定
        struct.pack_into("<I", hdr, 8, 0)
        for off in [12, 14, 16, 18]:
            struct.pack_into("<H", hdr, off, 0)
        # 版本号（2字节）：主版本1，次版本2，即LAS 1.2
        hdr[20], hdr[21] = 1, 2
        # 系统标识符（32字节）：生成此文件的系统名称
        hdr[22:54] = b"ProjectAirSim".ljust(32, b"\x00")
        # 生成软件标识符（32字节）：生成此文件的软件名称
        hdr[54:86] = b"ProjectAirSim LiDAR".ljust(32, b"\x00")
        # 文件创建日期（4字节）：年份+年积日（一年中的第几天）
        now = datetime.now()
        struct.pack_into("<H", hdr, 86, now.year)
        struct.pack_into("<H", hdr, 88, now.timetuple().tm_yday)
        # 头部大小（2字节）：227字节（LAS 1.2格式0的固定头部大小）
        struct.pack_into("<H", hdr, 90, 227)
        # 点数据偏移量（4字节）：点数据开始位置，等于头部大小
        struct.pack_into("<I", hdr, 92, 227)
        # 变长记录数量（4字节）：0表示无额外元数据记录
        struct.pack_into("<I", hdr, 96, 0)
        # 点数据记录格式（1字节）：格式0，最基础的点记录格式（仅XYZ+基本属性）
        hdr[100] = 0
        # 点数据记录长度（2字节）：格式0每条记录20字节
        struct.pack_into("<H", hdr, 101, 20)
        # 点数量（4字节）：总点数
        struct.pack_into("<I", hdr, 103, n)
        # 各回波点数（5×4字节）：格式0只有1个回波，全部点数放在第1个回波
        struct.pack_into("<I", hdr, 107, n)
        for off in [111, 115, 119, 123]:
            struct.pack_into("<I", hdr, off, 0)
        # X/Y/Z缩放因子（各8字节双精度浮点）：将整数坐标还原为浮点坐标
        for off in [131, 139, 147]:
            struct.pack_into("<d", hdr, off, scale)
        # X/Y/Z偏移量（各8字节双精度浮点）：坐标基准值
        struct.pack_into("<d", hdr, 155, x_min)
        struct.pack_into("<d", hdr, 163, y_min)
        struct.pack_into("<d", hdr, 171, z_min)
        # X/Y/Z最大值（各8字节双精度浮点）：坐标范围上界
        struct.pack_into("<d", hdr, 179, x_max)
        struct.pack_into("<d", hdr, 187, y_max)
        struct.pack_into("<d", hdr, 195, z_max)
        # X/Y/Z最小值（各8字节双精度浮点）：坐标范围下界
        struct.pack_into("<d", hdr, 203, x_min)
        struct.pack_into("<d", hdr, 211, y_min)
        struct.pack_into("<d", hdr, 219, z_min)
        # 构建点数据记录（每点20字节，numpy向量化）
        # 格式0记录结构：X(4) + Y(4) + Z(4) + 强度(2) + 回波信息(1) + 分类(1) + 扫描角(1) + 源ID(1) + 用户数据(2)
        xyz_int = np.round((points - np.array([x_min, y_min, z_min])) / scale).astype(np.int32)
        recs = bytearray(n * 20)
        for i in range(n):
            o = i * 20
            struct.pack_into("<i", recs, o, xyz_int[i, 0])
            struct.pack_into("<i", recs, o + 4, xyz_int[i, 1])
            struct.pack_into("<i", recs, o + 8, xyz_int[i, 2])
            struct.pack_into("<H", recs, o + 12, 0)
            struct.pack_into("<B", recs, o + 14, 0)
            struct.pack_into("<B", recs, o + 15, 0)
            struct.pack_into("<b", recs, o + 16, 0)
            struct.pack_into("<B", recs, o + 17, 0)
            struct.pack_into("<H", recs, o + 18, 0)
        with open(filepath, "wb") as f:
            f.write(hdr)
            f.write(recs)

    def log_control_command(self, mode, vx, vy, vz, yaw_rate, pos):
        """
        记录控制指令日志到CSV文件
        每次控制指令执行时调用，记录完整的时间序列数据
        使用线程锁保护并发写入安全

        参数：
            mode: 控制模式 ("manual" 键盘手动 或 "udp" UDP自动)
            vx: X方向速度（米/秒，北向为正）
            vy: Y方向速度（米/秒，东向为正）
            vz: Z方向速度（米/秒，下向为正）
            yaw_rate: 偏航角速度（度/秒，右转为正）
            pos: 位置字典 {"x": 北, "y": 东, "z": 下}
        """
        try:
            with self.log_lock:
                ts = datetime.now().isoformat()
                self._log_file.write(f"{ts},{mode},{vx},{vy},{vz},{yaw_rate},"
                                     f"{pos.get('x',0)},{pos.get('y',0)},{pos.get('z',0)}\n")
                self._log_file.flush()
        except Exception:
            pass

    def release(self):
        """
        释放所有资源，关闭文件和写入器
        在控制线程退出时调用，确保所有数据正确写入磁盘
        包括：关闭视频写入器、关闭日志文件句柄
        """
        if self.front_video_writer:
            self.front_video_writer.release()
            self.front_video_writer = None
        if self.down_video_writer:
            self.down_video_writer.release()
            self.down_video_writer = None
        if self._log_file:
            self._log_file.close()
            self._log_file = None
