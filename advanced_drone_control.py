"""
高级无人机控制脚本
基于Project AirSim平台，支持多种无人机型号、传感器数据采集、
键盘/UDP双模式控制、录像拍照、LiDAR点云可视化等功能。

功能概述：
1. 支持四旋翼/六旋翼/倾斜旋翼(VTOL)三种无人机型号
2. 前视+下视双相机，支持实时显示、持续录像、手动拍照
3. LiDAR点云实时可视化
4. 键盘手动控制 / UDP自动控制双模式
5. 飞行速度可调
6. 倾斜旋翼VTOL模式切换
7. 传感器数据、图像、点云、控制日志本地保存
"""

import argparse
import asyncio
import json
import os
import socket
import struct
import time
import threading
from datetime import datetime
from pathlib import Path

import cv2
import numpy as np

import projectairsim
from projectairsim import Drone, World
from projectairsim.utils import projectairsim_log, unpack_image
from projectairsim.image_utils import ImageDisplay
from projectairsim.lidar_utils import LidarDisplay
from projectairsim.types import ImageType

try:
    import keyboard
except ImportError:
    print("错误：未安装keyboard库，请执行 pip install keyboard")
    exit(1)


# ==============================================================================
# 常量定义
# ==============================================================================

# 无人机型号映射：用户输入 -> (配置文件名, 显示名称, 是否支持VTOL)
DRONE_MODELS = {
    "1": ("robot_quadrotor_adv.jsonc", "四旋翼", False),
    "2": ("robot_hexarotor_adv.jsonc", "六旋翼", False),
    "3": ("robot_quadtiltrotor_adv.jsonc", "倾斜旋翼(VTOL)", True),
}

# 控制模式映射
CONTROL_MODES = {
    "1": "手动控制",
    "2": "UDP自动控制",
}

# 键盘控制参数
DEFAULT_SPEED = 5.0          # 默认飞行速度（米/秒）
DEFAULT_YAW_SPEED = 20.0     # 默认偏航速度（度/秒）
SPEED_STEP = 1.0             # 速度调节步长（米/秒）
MIN_SPEED = 1.0              # 最小飞行速度
MAX_SPEED = 20.0             # 最大飞行速度
CONTROL_DURATION = 0.1       # 控制指令持续时间（秒）

# UDP参数
UDP_DEFAULT_IP = "127.0.0.1"  # 默认UDP监听IP
UDP_DEFAULT_PORT = 9876        # 默认UDP监听端口
UDP_BUFFER_SIZE = 1024         # UDP接收缓冲区大小
UDP_RECV_TIMEOUT = 0.1         # UDP接收超时时间（秒）
UDP_HOVER_TIMEOUT = 2.0        # UDP指令超时时间（秒），超时后自动悬停

# 相机参数
CAMERA_WIDTH = 640             # 相机图像宽度
CAMERA_HEIGHT = 360            # 相机图像高度
VIDEO_FPS = 30                 # 录像帧率

# 数据保存路径
DATA_BASE_DIR = "mine/drone_data"   # 数据保存根目录
LOG_SAVE_DIR = "mine/drone_data/logs"  # 日志保存路径


# ==============================================================================
# 配置管理器
# ==============================================================================

class ConfigManager:
    """
    配置管理器：负责管理场景配置文件的动态生成
    根据用户选择的无人机型号，动态修改场景配置中的robot-config字段
    """

    def __init__(self, sim_config_path):
        """
        初始化配置管理器

        参数：
            sim_config_path: 仿真配置文件目录路径
        """
        self.sim_config_path = sim_config_path
        self.scene_template_path = os.path.join(sim_config_path, "scene_adv_drone.jsonc")

    def generate_scene_config(self, robot_config_file):
        """
        根据无人机型号生成场景配置文件

        参数：
            robot_config_file: 机器人配置文件名

        返回：
            生成的场景配置文件路径
        """
        # 读取模板场景配置
        with open(self.scene_template_path, "r", encoding="utf-8") as f:
            content = f.read()

        # 移除JSONC注释（简单处理：逐行移除//注释）
        lines = content.split("\n")
        clean_lines = []
        for line in lines:
            # 移除行内注释（但不移除字符串内的//）
            comment_idx = line.find("//")
            if comment_idx >= 0:
                # 检查//是否在字符串内
                in_string = False
                for i, ch in enumerate(line[:comment_idx]):
                    if ch == '"':
                        in_string = not in_string
                if not in_string:
                    line = line[:comment_idx]
            clean_lines.append(line)

        scene_data = json.loads("\n".join(clean_lines))

        # 修改机器人配置引用
        for actor in scene_data.get("actors", []):
            if actor.get("type") == "robot":
                actor["robot-config"] = robot_config_file

        # 生成临时场景配置文件
        # 使用ensure_ascii=True避免中文在GBK编码系统上解码失败
        temp_scene_path = os.path.join(self.sim_config_path, "_scene_adv_drone_temp.jsonc")
        with open(temp_scene_path, "w", encoding="utf-8") as f:
            json.dump(scene_data, f, indent=2, ensure_ascii=True)

        return temp_scene_path


# ==============================================================================
# 数据记录管理器
# ==============================================================================

class DataRecorder:
    """
    数据记录管理器：负责管理所有传感器数据和日志的本地保存
    包括：录像文件、拍照图片、LiDAR点云、控制指令日志
    """

    def __init__(self, base_dir=DATA_BASE_DIR, log_dir=LOG_SAVE_DIR):
        """
        初始化数据记录管理器

        参数：
            base_dir: 数据保存根目录
            log_dir: 日志保存路径
        """
        # 创建以时间戳命名的会话目录
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        self.session_dir = os.path.join(base_dir, timestamp)
        self.images_dir = os.path.join(self.session_dir, "images")
        self.videos_dir = os.path.join(self.session_dir, "videos")
        self.lidar_dir = os.path.join(self.session_dir, "lidar")
        self.logs_dir = os.path.join(log_dir, timestamp)

        # 创建所有目录
        for dir_path in [self.images_dir, self.videos_dir,
                         self.lidar_dir, self.logs_dir]:
            os.makedirs(dir_path, exist_ok=True)

        # 录像写入器
        self.front_video_writer = None
        self.down_video_writer = None

        # 控制日志文件
        self.control_log_file = None
        self.control_log_path = os.path.join(self.logs_dir, "control_log.csv")

        # 写入CSV表头
        with open(self.control_log_path, "w", encoding="utf-8") as f:
            f.write("timestamp,mode,vx,vy,vz,yaw_rate,pos_x,pos_y,pos_z\n")

        # 线程锁
        self.log_lock = threading.Lock()

        # 拍照计数器
        self.front_photo_count = 0
        self.down_photo_count = 0

        # LiDAR保存计数器
        self.lidar_save_count = 0

        projectairsim_log().info(f"数据保存目录: {self.session_dir}")

    def init_video_writers(self):
        """
        初始化视频写入器
        使用XVID编码器，640x360分辨率，30fps
        """
        fourcc = cv2.VideoWriter_fourcc(*"XVID")

        front_video_path = os.path.join(
            self.videos_dir,
            f"front_camera_{datetime.now().strftime('%H%M%S')}.avi"
        )
        self.front_video_writer = cv2.VideoWriter(
            front_video_path, fourcc, VIDEO_FPS, (CAMERA_WIDTH, CAMERA_HEIGHT)
        )

        down_video_path = os.path.join(
            self.videos_dir,
            f"down_camera_{datetime.now().strftime('%H%M%S')}.avi"
        )
        self.down_video_writer = cv2.VideoWriter(
            down_video_path, fourcc, VIDEO_FPS, (CAMERA_WIDTH, CAMERA_HEIGHT)
        )

        projectairsim_log().info(f"前视录像: {front_video_path}")
        projectairsim_log().info(f"下视录像: {down_video_path}")

    def write_video_frame(self, camera_name, frame):
        """
        写入视频帧

        参数：
            camera_name: 相机名称 ("front" 或 "down")
            frame: OpenCV格式的图像帧
        """
        if frame is None:
            return

        try:
            if camera_name == "front" and self.front_video_writer is not None:
                if self.front_video_writer.isOpened():
                    self.front_video_writer.write(frame)
            elif camera_name == "down" and self.down_video_writer is not None:
                if self.down_video_writer.isOpened():
                    self.down_video_writer.write(frame)
        except Exception as e:
            projectairsim_log().warning(f"写入视频帧失败({camera_name}): {e}")

    def save_photo(self, camera_name, frame):
        """
        保存拍照图片

        参数：
            camera_name: 相机名称 ("front" 或 "down")
            frame: OpenCV格式的图像帧
        """
        if frame is None:
            return

        try:
            timestamp = datetime.now().strftime("%Y%m%d_%H%M%S_%f")
            if camera_name == "front":
                self.front_photo_count += 1
                filename = f"front_photo_{self.front_photo_count:04d}_{timestamp}.jpg"
            else:
                self.down_photo_count += 1
                filename = f"down_photo_{self.down_photo_count:04d}_{timestamp}.jpg"

            filepath = os.path.join(self.images_dir, filename)
            cv2.imwrite(filepath, frame)
            projectairsim_log().info(f"拍照保存: {filepath}")
        except Exception as e:
            projectairsim_log().warning(f"保存拍照失败({camera_name}): {e}")

    def save_lidar_point_cloud(self, lidar_data):
        """
        保存LiDAR点云数据
        同时保存为NPY（原始数据）、PCD（标准点云格式）、LAS（测绘标准格式）

        参数：
            lidar_data: LiDAR传感器数据字典
        """
        if lidar_data is None:
            return

        try:
            self.lidar_save_count += 1
            timestamp = datetime.now().strftime("%Y%m%d_%H%M%S_%f")
            base_filename = f"lidar_{self.lidar_save_count:04d}_{timestamp}"

            if "point_cloud" in lidar_data:
                points = np.array(lidar_data["point_cloud"])
                if len(points) > 0 and len(points) % 3 == 0:
                    points = points.reshape(-1, 3)

                    # 保存NPY格式（原始数据，方便Python读取）
                    npy_path = os.path.join(self.lidar_dir, f"{base_filename}.npy")
                    np.save(npy_path, points)

                    # 保存PCD格式（Point Cloud Data，PCL库标准格式）
                    pcd_path = os.path.join(self.lidar_dir, f"{base_filename}.pcd")
                    self._save_pcd(pcd_path, points)

                    # 保存LAS格式（测绘行业标准格式）
                    las_path = os.path.join(self.lidar_dir, f"{base_filename}.las")
                    self._save_las(las_path, points)

                    projectairsim_log().info(
                        f"LiDAR点云保存: NPY={npy_path}, PCD={pcd_path}, LAS={las_path}"
                    )
        except Exception as e:
            projectairsim_log().warning(f"保存LiDAR点云失败: {e}")

    def _save_pcd(self, filepath, points):
        """
        保存点云为PCD（Point Cloud Data）格式
        PCD是PCL（Point Cloud Library）的标准格式
        采用ASCII模式，兼容性最好，可被CloudCompare、PCL、Open3D等工具读取

        参数：
            filepath: 保存文件路径
            points: Nx3的numpy数组，每行为(x, y, z)
        """
        num_points = len(points)
        with open(filepath, "w", encoding="utf-8") as f:
            f.write("# .PCD v0.7 - Point Cloud Data file format\n")
            f.write("VERSION 0.7\n")
            f.write("FIELDS x y z\n")
            f.write("SIZE 4 4 4\n")
            f.write("TYPE F F F\n")
            f.write("COUNT 1 1 1\n")
            f.write(f"WIDTH {num_points}\n")
            f.write("HEIGHT 1\n")
            f.write("VIEWPOINT 0 0 0 1 0 0 0\n")
            f.write(f"POINTS {num_points}\n")
            f.write("DATA ascii\n")
            for i in range(num_points):
                f.write(f"{points[i, 0]:.6f} {points[i, 1]:.6f} {points[i, 2]:.6f}\n")

    def _save_las(self, filepath, points):
        """
        保存点云为LAS（LASer）格式
        LAS是测绘行业的标准点云格式（ASPRS标准）
        采用LAS 1.2版本，点数据记录格式0（仅XYZ坐标）
        可被CloudCompare、LAStools、QGIS、ArcGIS等工具读取

        参数：
            filepath: 保存文件路径
            points: Nx3的numpy数组，每行为(x, y, z)
        """
        num_points = len(points)

        # 计算缩放参数和偏移量
        # LAS使用整数存储+缩放因子来保证精度
        x_min = float(np.min(points[:, 0]))
        y_min = float(np.min(points[:, 1]))
        z_min = float(np.min(points[:, 2]))
        x_max = float(np.max(points[:, 0]))
        y_max = float(np.max(points[:, 1]))
        z_max = float(np.max(points[:, 2]))

        # 缩放因子：0.0001米精度（0.1毫米）
        scale = 0.0001
        x_offset = x_min
        y_offset = y_min
        z_offset = z_min

        # 构建LAS 1.2文件头（227字节）
        header = bytearray(227)

        # 文件签名 "LASF"
        header[0:4] = b"LASF"
        # 文件源ID
        struct.pack_into("<H", header, 4, 0)
        # 全局编码
        struct.pack_into("<H", header, 6, 0)
        # 项目ID (GUID)
        struct.pack_into("<I", header, 8, 0)
        struct.pack_into("<H", header, 12, 0)
        struct.pack_into("<H", header, 14, 0)
        struct.pack_into("<H", header, 16, 0)
        struct.pack_into("<H", header, 18, 0)
        # 版本号 1.2
        header[20] = 1
        header[21] = 2
        # 系统标识符
        header[22:54] = b"ProjectAirSim".ljust(32, b"\x00")
        # 生成软件标识符
        header[54:86] = b"ProjectAirSim LiDAR".ljust(32, b"\x00")
        # 文件创建日/年
        now = datetime.now()
        struct.pack_into("<H", header, 86, now.year)
        struct.pack_into("<H", header, 88, now.timetuple().tm_yday)
        # 头部大小
        struct.pack_into("<H", header, 90, 227)
        # 点数据偏移量（头部大小）
        struct.pack_into("<I", header, 92, 227)
        # 变长记录数量
        struct.pack_into("<I", header, 96, 0)
        # 点数据记录格式（格式0：仅XYZ+强度+回波+分类）
        header[100] = 0
        # 点数据记录长度（格式0为20字节）
        struct.pack_into("<H", header, 101, 20)
        # 点数量
        struct.pack_into("<I", header, 103, num_points)
        # 点数量（5个回波）
        struct.pack_into("<I", header, 107, num_points)
        struct.pack_into("<I", header, 111, 0)
        struct.pack_into("<I", header, 115, 0)
        struct.pack_into("<I", header, 119, 0)
        struct.pack_into("<I", header, 123, 0)
        # X/Y/Z缩放因子
        struct.pack_into("<d", header, 131, scale)
        struct.pack_into("<d", header, 139, scale)
        struct.pack_into("<d", header, 147, scale)
        # X/Y/Z偏移量
        struct.pack_into("<d", header, 155, x_offset)
        struct.pack_into("<d", header, 163, y_offset)
        struct.pack_into("<d", header, 171, z_offset)
        # X/Y/Z最大值
        struct.pack_into("<d", header, 179, x_max)
        struct.pack_into("<d", header, 187, y_max)
        struct.pack_into("<d", header, 195, z_max)
        # X/Y/Z最小值
        struct.pack_into("<d", header, 203, x_min)
        struct.pack_into("<d", header, 211, y_min)
        struct.pack_into("<d", header, 219, z_min)

        # 构建点数据记录
        point_records = bytearray(num_points * 20)
        for i in range(num_points):
            offset = i * 20
            # X/Y/Z坐标（整数，需乘以1/scale + offset）
            x_int = int(round((points[i, 0] - x_offset) / scale))
            y_int = int(round((points[i, 1] - y_offset) / scale))
            z_int = int(round((points[i, 2] - z_offset) / scale))
            struct.pack_into("<i", point_records, offset, x_int)
            struct.pack_into("<i", point_records, offset + 4, y_int)
            struct.pack_into("<i", point_records, offset + 8, z_int)
            # 强度（0）
            struct.pack_into("<H", point_records, offset + 12, 0)
            # 回波信息+分类（0）
            struct.pack_into("<B", point_records, offset + 14, 0)
            # 分类（0=未分类）
            struct.pack_into("<B", point_records, offset + 15, 0)
            # 扫描角度（0）
            struct.pack_into("<b", point_records, offset + 16, 0)
            # 文件源ID（0）
            struct.pack_into("<B", point_records, offset + 17, 0)
            # 用户数据（0）
            struct.pack_into("<H", point_records, offset + 18, 0)

        with open(filepath, "wb") as f:
            f.write(header)
            f.write(point_records)

    def log_control_command(self, mode, vx, vy, vz, yaw_rate, pos):
        """
        记录控制指令日志

        参数：
            mode: 控制模式 ("manual" 或 "udp")
            vx: X方向速度
            vy: Y方向速度
            vz: Z方向速度
            yaw_rate: 偏航角速度
            pos: 位置字典 {"x": ..., "y": ..., "z": ...}
        """
        try:
            with self.log_lock:
                timestamp = datetime.now().isoformat()
                with open(self.control_log_path, "a", encoding="utf-8") as f:
                    f.write(
                        f"{timestamp},{mode},{vx},{vy},{vz},{yaw_rate},"
                        f"{pos.get('x', 0)},{pos.get('y', 0)},{pos.get('z', 0)}\n"
                    )
        except Exception as e:
            projectairsim_log().warning(f"记录控制日志失败: {e}")

    def release(self):
        """
        释放所有资源，关闭文件和写入器
        """
        if self.front_video_writer is not None:
            self.front_video_writer.release()
            self.front_video_writer = None

        if self.down_video_writer is not None:
            self.down_video_writer.release()
            self.down_video_writer = None

        projectairsim_log().info("数据记录器已释放")


# ==============================================================================
# UDP通信管理器
# ==============================================================================

class UDPManager:
    """
    UDP通信管理器：负责接收外部控制指令
    接收格式为JSON字符串，包含位置和姿态信息
    每100ms周期接收一次控制指令
    支持超时检测，超过指定时间未收到指令时标记为超时状态
    """

    def __init__(self, ip=UDP_DEFAULT_IP, port=UDP_DEFAULT_PORT):
        """
        初始化UDP管理器

        参数：
            ip: 监听IP地址
            port: 监听端口号
        """
        self.ip = ip
        self.port = port
        self.socket = None
        self.running = False
        self.latest_command = None
        self.lock = threading.Lock()
        self.last_command_time = time.time()
        self.hover_triggered = False

    def start(self):
        """
        启动UDP监听
        创建UDP套接字并绑定到指定地址和端口
        """
        try:
            self.socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            self.socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            self.socket.bind((self.ip, self.port))
            self.socket.settimeout(UDP_RECV_TIMEOUT)
            self.running = True
            projectairsim_log().info(f"UDP监听已启动: {self.ip}:{self.port}")
        except Exception as e:
            projectairsim_log().error(f"UDP启动失败: {e}")
            self.running = False

    def receive_command(self):
        """
        接收UDP控制指令
        非阻塞方式接收，超时返回None
        收到指令时更新最后接收时间戳

        返回：
            解析后的控制指令字典，或None
        """
        if not self.running or self.socket is None:
            return None

        try:
            data, addr = self.socket.recvfrom(UDP_BUFFER_SIZE)
            command = json.loads(data.decode("utf-8"))

            with self.lock:
                self.latest_command = command
                self.last_command_time = time.time()
                self.hover_triggered = False

            return command
        except socket.timeout:
            return None
        except json.JSONDecodeError as e:
            projectairsim_log().warning(f"UDP数据解析失败: {e}")
            return None
        except Exception as e:
            return None

    def get_latest_command(self):
        """
        获取最新的UDP控制指令

        返回：
            最新的控制指令字典，或None
        """
        with self.lock:
            return self.latest_command

    def is_command_timeout(self):
        """
        检测是否超时未收到UDP控制指令

        返回：
            True表示已超时，False表示未超时
        """
        with self.lock:
            elapsed = time.time() - self.last_command_time
            return elapsed > UDP_HOVER_TIMEOUT

    def should_trigger_hover(self):
        """
        判断是否需要触发自动悬停
        仅在首次超时时触发，避免重复触发

        返回：
            True表示需要触发悬停，False表示不需要
        """
        with self.lock:
            if self.hover_triggered:
                return False
            elapsed = time.time() - self.last_command_time
            if elapsed > UDP_HOVER_TIMEOUT:
                self.hover_triggered = True
                return True
            return False

    def stop(self):
        """
        停止UDP监听，关闭套接字
        """
        self.running = False
        if self.socket is not None:
            self.socket.close()
            self.socket = None
        projectairsim_log().info("UDP监听已停止")


# ==============================================================================
# 传感器管理器
# ==============================================================================

class SensorManager:
    """
    传感器管理器：负责相机和LiDAR传感器的数据订阅、处理和显示
    """

    def __init__(self, client, drone, image_display, lidar_display, data_recorder):
        """
        初始化传感器管理器

        参数：
            client: ProjectAirSim客户端
            drone: 无人机对象
            image_display: 图像显示对象
            lidar_display: LiDAR显示对象
            data_recorder: 数据记录器
        """
        self.client = client
        self.drone = drone
        self.image_display = image_display
        self.lidar_display = lidar_display
        self.data_recorder = data_recorder

        # 缓存最新帧数据，用于录像和拍照
        self.latest_front_frame = None
        self.latest_down_frame = None
        self.front_lock = threading.Lock()
        self.down_lock = threading.Lock()

        # 缓存最新LiDAR数据，用于点云快照保存
        self.latest_lidar_data = None
        self.lidar_lock = threading.Lock()

    def _sensor_topic_exists(self, sensor_name, topic_key):
        """
        检查传感器主题是否存在

        参数：
            sensor_name: 传感器名称（如"Chase"、"FrontCamera"）
            topic_key: 主题键名（如"scene_camera"、"lidar"）

        返回：
            True如果主题存在，False否则
        """
        if sensor_name not in self.drone.sensors:
            projectairsim_log().warning(f"传感器 '{sensor_name}' 不存在于当前无人机配置中")
            return False
        if topic_key not in self.drone.sensors[sensor_name]:
            projectairsim_log().warning(f"传感器 '{sensor_name}' 没有主题 '{topic_key}'，可能capture-enabled为false")
            return False
        return True

    def setup_subscriptions(self):
        """
        设置所有传感器的订阅
        包括：追踪相机、前视相机、下视相机、LiDAR
        订阅前会检查传感器主题是否存在，避免KeyError
        """
        # 订阅追踪相机（用于全局视角显示）
        if self._sensor_topic_exists("Chase", "scene_camera"):
            chase_cam_window = "ChaseCam"
            self.image_display.add_chase_cam(chase_cam_window)
            self.client.subscribe(
                self.drone.sensors["Chase"]["scene_camera"],
                lambda _, chase: self.image_display.receive(chase, chase_cam_window),
            )
        else:
            projectairsim_log().warning("追踪相机不可用，跳过订阅")

        # 订阅前视相机
        if self._sensor_topic_exists("FrontCamera", "scene_camera"):
            front_cam_window = "FrontCamera"
            self.image_display.add_image(front_cam_window, subwin_idx=0)
            self.client.subscribe(
                self.drone.sensors["FrontCamera"]["scene_camera"],
                lambda _, img: self._on_front_camera_data(img, front_cam_window),
            )
        else:
            projectairsim_log().warning("前视相机不可用，跳过订阅")

        # 订阅下视相机
        if self._sensor_topic_exists("DownCamera", "scene_camera"):
            down_cam_window = "DownCamera"
            self.image_display.add_image(down_cam_window, subwin_idx=1)
            self.client.subscribe(
                self.drone.sensors["DownCamera"]["scene_camera"],
                lambda _, img: self._on_down_camera_data(img, down_cam_window),
            )
        else:
            projectairsim_log().warning("下视相机不可用，跳过订阅")

        # 订阅LiDAR
        if self._sensor_topic_exists("lidar1", "lidar"):
            self.client.subscribe(
                self.drone.sensors["lidar1"]["lidar"],
                lambda _, lidar: self._on_lidar_data(lidar),
            )
        else:
            projectairsim_log().warning("LiDAR传感器不可用，跳过订阅")

        projectairsim_log().info("传感器订阅已设置完成")

    def _on_front_camera_data(self, image_msg, window_name):
        """
        前视相机数据回调函数
        同时用于图像显示和录像帧缓存

        参数：
            image_msg: 图像消息
            window_name: 显示窗口名称
        """
        # 传递给图像显示
        self.image_display.receive(image_msg, window_name)

        # 解包图像用于录像
        try:
            if image_msg and "data" in image_msg and len(image_msg["data"]) > 0:
                frame = unpack_image(image_msg)
                with self.front_lock:
                    self.latest_front_frame = frame
                # 写入录像
                self.data_recorder.write_video_frame("front", frame)
        except Exception as e:
            pass

    def _on_down_camera_data(self, image_msg, window_name):
        """
        下视相机数据回调函数
        同时用于图像显示和录像帧缓存

        参数：
            image_msg: 图像消息
            window_name: 显示窗口名称
        """
        # 传递给图像显示
        self.image_display.receive(image_msg, window_name)

        # 解包图像用于录像
        try:
            if image_msg and "data" in image_msg and len(image_msg["data"]) > 0:
                frame = unpack_image(image_msg)
                with self.down_lock:
                    self.latest_down_frame = frame
                # 写入录像
                self.data_recorder.write_video_frame("down", frame)
        except Exception as e:
            pass

    def capture_front_photo(self):
        """
        前视相机手动拍照
        从缓存中获取最新帧并保存
        """
        with self.front_lock:
            frame = self.latest_front_frame.copy() if self.latest_front_frame is not None else None
        if frame is not None:
            self.data_recorder.save_photo("front", frame)
        else:
            projectairsim_log().warning("前视相机无可用帧，拍照失败")

    def capture_down_photo(self):
        """
        下视相机手动拍照
        从缓存中获取最新帧并保存
        """
        with self.down_lock:
            frame = self.latest_down_frame.copy() if self.latest_down_frame is not None else None
        if frame is not None:
            self.data_recorder.save_photo("down", frame)
        else:
            projectairsim_log().warning("下视相机无可用帧，拍照失败")

    def _on_lidar_data(self, lidar_data):
        """
        LiDAR数据回调函数
        同时用于点云可视化和数据缓存

        参数：
            lidar_data: LiDAR数据消息
        """
        # 传递给LiDAR显示
        self.lidar_display.receive(lidar_data)

        # 缓存最新LiDAR数据，用于点云快照保存
        if lidar_data is not None:
            with self.lidar_lock:
                self.latest_lidar_data = lidar_data

    def save_lidar_snapshot(self):
        """
        保存LiDAR点云快照
        从订阅回调缓存中获取最新LiDAR数据并保存
        """
        with self.lidar_lock:
            lidar_data = self.latest_lidar_data
        if lidar_data is not None:
            self.data_recorder.save_lidar_point_cloud(lidar_data)
        else:
            projectairsim_log().warning("LiDAR无可用数据，点云快照保存失败")


# ==============================================================================
# 无人机控制器
# ==============================================================================

class DroneController:
    """
    无人机控制器：负责键盘手动控制和UDP自动控制
    """

    def __init__(self, drone, data_recorder, is_vtol=False):
        """
        初始化无人机控制器

        参数：
            drone: 无人机对象
            data_recorder: 数据记录器
            is_vtol: 是否为VTOL（倾斜旋翼）机型
        """
        self.drone = drone
        self.data_recorder = data_recorder
        self.is_vtol = is_vtol

        # 飞行参数
        self.speed = DEFAULT_SPEED
        self.yaw_speed = DEFAULT_YAW_SPEED
        self.duration = CONTROL_DURATION

        # VTOL模式状态
        self.is_fixed_wing = False

        # 运行状态
        self.running = True
        self.is_flying = False

    async def takeoff(self):
        """
        解锁无人机并起飞到默认高度
        """
        projectairsim_log().info("正在解锁无人机...")
        self.drone.enable_api_control()
        self.drone.arm()

        projectairsim_log().info("正在起飞...")
        await self.drone.takeoff_async()
        self.is_flying = True
        projectairsim_log().info("起飞完成")

    async def land(self):
        """
        着陆无人机
        带超时保护，避免连接关闭时无限等待
        """
        projectairsim_log().info("正在着陆...")
        try:
            land_task = await self.drone.land_async()
            await asyncio.wait_for(land_task, timeout=30.0)
            self.drone.disarm()
            self.is_flying = False
            projectairsim_log().info("着陆完成")
        except asyncio.TimeoutError:
            projectairsim_log().warning("着陆超时，强制标记为已着陆")
            self.is_flying = False
        except Exception as e:
            projectairsim_log().warning(f"着陆过程中出现异常: {e}")
            self.is_flying = False

    async def toggle_vtol_mode(self):
        """
        切换VTOL模式（多旋翼 <-> 固定翼）
        仅对倾斜旋翼机型有效
        """
        if not self.is_vtol:
            projectairsim_log().warning("当前机型不支持VTOL模式切换")
            return

        if self.is_fixed_wing:
            projectairsim_log().info("切换到多旋翼模式...")
            await self.drone.set_vtol_mode_async(Drone.VTOLMode.Multirotor)
            self.is_fixed_wing = False
            projectairsim_log().info("已切换到多旋翼模式")
        else:
            projectairsim_log().info("切换到固定翼模式...")
            await self.drone.set_vtol_mode_async(Drone.VTOLMode.FixedWing)
            self.is_fixed_wing = True
            projectairsim_log().info("已切换到固定翼模式")

    def get_current_position(self):
        """
        获取无人机当前位置（NED坐标系）

        返回：
            位置字典 {"x": ..., "y": ..., "z": ...}
        """
        try:
            kinematics = self.drone.get_ground_truth_kinematics()
            return kinematics["pose"]["position"]
        except Exception:
            return {"x": 0, "y": 0, "z": 0}

    async def process_keyboard_input(self):
        """
        处理键盘输入，生成速度控制指令
        读取键盘状态并执行对应的飞行动作
        """
        vx, vy, vz, yaw_rate = 0, 0, 0, 0

        # 前后控制（W/S键）
        if keyboard.is_pressed("w"):
            vx = self.speed
        elif keyboard.is_pressed("s"):
            vx = -self.speed

        # 左右控制（A/D键）
        if keyboard.is_pressed("a"):
            vy = -self.speed
        elif keyboard.is_pressed("d"):
            vy = self.speed

        # 上下控制（上/下箭头键）
        if keyboard.is_pressed("up"):
            vz = -self.speed
        elif keyboard.is_pressed("down"):
            vz = self.speed

        # 偏航控制（左/右箭头键）
        if keyboard.is_pressed("left"):
            yaw_rate = -self.yaw_speed
        elif keyboard.is_pressed("right"):
            yaw_rate = self.yaw_speed

        # 速度调节（+/-键）
        if keyboard.is_pressed("+") or keyboard.is_pressed("="):
            self.speed = min(self.speed + SPEED_STEP, MAX_SPEED)
            projectairsim_log().info(f"飞行速度: {self.speed:.1f} m/s")
            await asyncio.sleep(0.2)
        elif keyboard.is_pressed("-"):
            self.speed = max(self.speed - SPEED_STEP, MIN_SPEED)
            projectairsim_log().info(f"飞行速度: {self.speed:.1f} m/s")
            await asyncio.sleep(0.2)

        # 执行移动指令
        if vx != 0 or vy != 0 or vz != 0:
            await self.drone.move_by_velocity_body_frame_async(
                vx, vy, vz, self.duration
            )
        if yaw_rate != 0:
            await self.drone.rotate_by_yaw_rate_async(yaw_rate, self.duration)

        # 记录控制日志
        if vx != 0 or vy != 0 or vz != 0 or yaw_rate != 0:
            pos = self.get_current_position()
            self.data_recorder.log_control_command(
                "manual", vx, vy, vz, yaw_rate, pos
            )
    # ==============================================================================
    # 相应UDP控制指令
    # ==============================================================================
    async def process_udp_command(self, command):
        """
        处理UDP控制指令
        解析JSON格式的控制指令并执行飞行动作

        指令格式示例：
        {
            "position": {"x": 10.0, "y": 5.0, "z": -3.0},
            "velocity": {"vx": 2.0, "vy": 1.0, "vz": 0.0},
            "attitude": {"roll": 0.0, "pitch": 0.0, "yaw": 45.0},
            "yaw_rate": 0.0
        }

        参数：
            command: 解析后的控制指令字典
        """
        if command is None:
            return

        try:
            # 优先使用速度控制
            if "velocity" in command:
                vel = command["velocity"]
                vx = vel.get("vx", 0)
                vy = vel.get("vy", 0)
                vz = vel.get("vz", 0)
                await self.drone.move_by_velocity_async(
                    v_north=vx, v_east=vy, v_down=vz, duration=self.duration
                )

            # 如果有位置指令，使用move_to_position
            elif "position" in command:
                pos = command["position"]
                x = pos.get("x", 0)
                y = pos.get("y", 0)
                z = pos.get("z", 0)
                await self.drone.move_to_position_async(x, y, z, self.speed)

            # 如果有偏航角速度指令
            if "yaw_rate" in command:
                yaw_rate = command["yaw_rate"]
                if yaw_rate != 0:
                    await self.drone.rotate_by_yaw_rate_async(
                        yaw_rate, self.duration
                    )

            # 记录控制日志
            current_pos = self.get_current_position()
            vel = command.get("velocity", {"vx": 0, "vy": 0, "vz": 0})
            self.data_recorder.log_control_command(
                "udp",
                vel.get("vx", 0),
                vel.get("vy", 0),
                vel.get("vz", 0),
                command.get("yaw_rate", 0),
                current_pos,
            )

        except Exception as e:
            projectairsim_log().warning(f"处理UDP指令失败: {e}")


# ==============================================================================
# 用户交互界面
# ==============================================================================

def print_banner():
    """
    打印程序启动横幅
    """
    print("=" * 60)
    print("       Project AirSim 高级无人机控制系统")
    print("=" * 60)
    print()


def select_drone_model():
    """
    让用户选择无人机型号

    返回：
        (配置文件名, 显示名称, 是否支持VTOL) 的元组
    """
    print("请选择无人机型号：")
    print("  1 - 四旋翼（灵活机动，适合近距作业）")
    print("  2 - 六旋翼（载重更大，冗余度更高）")
    print("  3 - 倾斜旋翼VTOL（支持固定翼/多旋翼切换，适合长距飞行）")
    print()

    while True:
        choice = input("请输入选项编号 (1/2/3): ").strip()
        if choice in DRONE_MODELS:
            config_file, display_name, is_vtol = DRONE_MODELS[choice]
            print(f"已选择: {display_name}")
            return config_file, display_name, is_vtol
        print("无效输入，请重新选择")


def select_control_mode():
    """
    让用户选择控制模式

    返回：
        控制模式字符串
    """
    print()
    print("请选择控制模式：")
    print("  1 - 手动控制（键盘WASD+方向键控制）")
    print("  2 - UDP自动控制（接收外部UDP指令控制）")
    print()

    while True:
        choice = input("请输入选项编号 (1/2): ").strip()
        if choice in CONTROL_MODES:
            mode_name = CONTROL_MODES[choice]
            print(f"已选择: {mode_name}")
            return mode_name
        print("无效输入，请重新选择")


def print_help(is_vtol=False):
    """
    打印控制帮助信息

    参数：
        is_vtol: 是否为VTOL机型
    """
    print()
    print("-" * 50)
    print("  键盘控制说明")
    print("-" * 50)
    print("  W/S        : 前进/后退")
    print("  A/D        : 左移/右移")
    print("  上/下箭头   : 上升/下降")
    print("  左/右箭头   : 左转/右转")
    print("  +/-        : 加速/减速")
    print("  F          : 前视相机拍照")
    print("  G          : 下视相机拍照")
    print("  L          : 保存LiDAR点云快照(NPY+PCD+LAS)")
    print("  T          : 着陆")
    if is_vtol:
        print("  V          : 切换VTOL模式(多旋翼/固定翼)")
    print("  Q          : 退出程序")
    print("-" * 50)
    print()


# ==============================================================================
# 主函数
# ==============================================================================

async def main():
    """
    主函数：高级无人机控制系统入口
    流程：选择型号 -> 选择模式 -> 连接仿真 -> 起飞 -> 控制循环 -> 着陆 -> 退出
    """
    # 打印启动横幅
    print_banner()

    # 用户选择无人机型号
    robot_config, drone_model_name, is_vtol = select_drone_model()

    # 用户选择控制模式
    control_mode = select_control_mode()

    # 解析命令行参数
    parser = argparse.ArgumentParser(
        description="Project AirSim 高级无人机控制系统"
    )
    parser.add_argument(
        "--address",
        type=str,
        default="127.0.0.1",
        help="Project AirSim仿真器IP地址",
    )
    parser.add_argument(
        "--simconfigpath",
        type=str,
        default=os.path.join(os.path.dirname(__file__), "sim_config"),
        help="仿真配置文件目录路径",
    )
    parser.add_argument(
        "--topicsport",
        type=int,
        default=8989,
        help="Topic通信端口",
    )
    parser.add_argument(
        "--servicesport",
        type=int,
        default=8990,
        help="Service通信端口",
    )
    parser.add_argument(
        "--udp-ip",
        type=str,
        default=UDP_DEFAULT_IP,
        help="UDP监听IP地址",
    )
    parser.add_argument(
        "--udp-port",
        type=int,
        default=UDP_DEFAULT_PORT,
        help="UDP监听端口号",
    )
    args = parser.parse_args()

    # 初始化配置管理器，根据用户选择生成场景配置
    config_manager = ConfigManager(args.simconfigpath)
    scene_config_file = config_manager.generate_scene_config(robot_config)
    projectairsim_log().info(f"场景配置: {scene_config_file}")
    projectairsim_log().info(f"无人机型号: {drone_model_name}")
    projectairsim_log().info(f"控制模式: {control_mode}")

    # 初始化数据记录器
    data_recorder = DataRecorder()

    # 初始化图像显示
    image_display = ImageDisplay(num_subwin=3)

    # 初始化LiDAR显示
    lidar_subwin = image_display.get_subwin_info(2)
    lidar_display = LidarDisplay(
        x=lidar_subwin["x"],
        y=lidar_subwin["y"] + 30,
        view=LidarDisplay.VIEW_PERSPECTIVE,
    )

    # 初始化UDP管理器（仅UDP模式需要）
    udp_manager = None
    if control_mode == "UDP自动控制":
        udp_manager = UDPManager(args.udp_ip, args.udp_port)

    # 创建Project AirSim客户端
    client = projectairsim.ProjectAirSimClient(
        address=args.address,
        port_topics=args.topicsport,
        port_services=args.servicesport,
    )

    drone = None
    sensor_manager = None
    drone_controller = None

    try:
        # 连接到仿真环境
        projectairsim_log().info("正在连接仿真环境...")
        try:
            client.connect()
        except Exception as conn_err:
            print()
            print("=" * 60)
            print("  仿真环境连接失败！")
            print("=" * 60)
            print(f"  错误信息: {conn_err}")
            print()
            print("  可能的原因：")
            print("    1. Project AirSim仿真器未启动")
            print("    2. 仿真器IP地址或端口不正确")
            print("    3. 仿真器正在加载场景，请稍后重试")
            print()
            print("  解决方法：")
            print("    1. 先启动Project AirSim仿真器，等待场景加载完成")
            print("    2. 检查--address参数是否与仿真器IP一致")
            print("    3. 检查--topicsport和--servicesport是否与仿真器一致")
            print("=" * 60)
            return

        # 加载场景
        try:
            scene_filename = os.path.basename(scene_config_file)
            world = World(
                client=client,
                scene_config_name=scene_filename,
                sim_config_path=args.simconfigpath,
                delay_after_load_sec=2,
            )
            projectairsim_log().info("场景加载完成")
        except Exception as scene_err:
            print()
            print("=" * 60)
            print("  场景加载失败！")
            print("=" * 60)
            print(f"  错误信息: {scene_err}")
            print()
            print("  可能的原因：")
            print("    1. 配置文件不存在或格式错误")
            print(f"    2. 场景配置: {scene_config_file}")
            print("    3. 无人机配置文件缺失")
            print()
            print("  解决方法：")
            print("    1. 检查mine/sim_config/目录下配置文件是否完整")
            print("    2. 确认仿真器已正确加载Unreal场景")
            print("=" * 60)
            client.disconnect()
            return

        # 创建无人机对象
        drone = Drone(client, world, "Drone1")
        projectairsim_log().info("无人机对象创建完成")

        # 初始化传感器管理器
        sensor_manager = SensorManager(
            client, drone, image_display, lidar_display, data_recorder
        )

        # 初始化无人机控制器
        drone_controller = DroneController(drone, data_recorder, is_vtol)

        # 设置传感器订阅
        sensor_manager.setup_subscriptions()

        # 启动图像和LiDAR显示
        image_display.start()
        lidar_display.start()

        # 初始化录像写入器
        data_recorder.init_video_writers()

        # 启动UDP监听（仅UDP模式）
        if udp_manager is not None:
            udp_manager.start()

        # 起飞
        await drone_controller.takeoff()

        # 打印控制帮助
        print_help(is_vtol)

        # 主控制循环
        projectairsim_log().info("进入控制循环，按Q退出")
        while drone_controller.running:
            # 处理键盘输入
            if keyboard.is_pressed("q"):
                projectairsim_log().info("收到退出指令")
                drone_controller.running = False
                break

            if keyboard.is_pressed("t"):
                await drone_controller.land()
                drone_controller.running = False
                break

            # 前视相机拍照
            if keyboard.is_pressed("f"):
                sensor_manager.capture_front_photo()
                await asyncio.sleep(0.3)

            # 下视相机拍照
            if keyboard.is_pressed("g"):
                sensor_manager.capture_down_photo()
                await asyncio.sleep(0.3)

            # LiDAR点云快照保存
            if keyboard.is_pressed("l"):
                sensor_manager.save_lidar_snapshot()
                await asyncio.sleep(0.3)

            # VTOL模式切换
            if keyboard.is_pressed("v") and is_vtol:
                await drone_controller.toggle_vtol_mode()
                await asyncio.sleep(0.5)

            # 根据控制模式执行控制
            if control_mode == "手动控制":
                await drone_controller.process_keyboard_input()
            elif control_mode == "UDP自动控制":
                if udp_manager is not None:
                    command = udp_manager.receive_command()
                    if command is not None:
                        await drone_controller.process_udp_command(command)
                    else:
                        # 超时未收到UDP指令，触发自动悬停
                        if udp_manager.should_trigger_hover():
                            projectairsim_log().warning(
                                f"UDP指令超时({UDP_HOVER_TIMEOUT}秒)，自动悬停"
                            )
                            print(f"\n[警告] UDP指令超时({UDP_HOVER_TIMEOUT}秒)，已自动悬停！")
                            await drone.move_by_velocity_async(
                                v_north=0, v_east=0, v_down=0, duration=1.0
                            )
                        elif udp_manager.is_command_timeout():
                            # 持续超时状态，保持悬停
                            await drone.move_by_velocity_async(
                                v_north=0, v_east=0, v_down=0, duration=0.1
                            )
                        else:
                            await asyncio.sleep(0.05)

            # 控制循环间隔
            await asyncio.sleep(0.01)

    except Exception as err:
        projectairsim_log().error(f"程序异常: {err}", exc_info=True)

    finally:
        # 清理资源
        projectairsim_log().info("正在清理资源...")

        # 如果仍在飞行，执行着陆（带超时保护）
        if drone_controller and drone_controller.is_flying:
            try:
                await asyncio.wait_for(drone_controller.land(), timeout=30.0)
            except asyncio.TimeoutError:
                projectairsim_log().warning("着陆超时，继续清理")
            except Exception as e:
                projectairsim_log().warning(f"着陆异常（可忽略）: {e}")

        # 禁用API控制
        if drone is not None:
            try:
                drone.disable_api_control()
            except Exception:
                pass

        # 停止UDP监听
        if udp_manager is not None:
            udp_manager.stop()

        # 停止显示
        image_display.stop()
        lidar_display.stop()

        # 释放数据记录器
        data_recorder.release()

        # 断开仿真连接（先取消所有挂起的异步任务）
        try:
            pending = asyncio.all_tasks(asyncio.get_event_loop())
            for task in pending:
                if task is not asyncio.current_task():
                    task.cancel()
        except Exception:
            pass

        client.disconnect()

        # 删除临时场景配置文件
        try:
            temp_scene = os.path.join(args.simconfigpath, "_scene_adv_drone_temp.jsonc")
            if os.path.exists(temp_scene):
                os.remove(temp_scene)
        except Exception:
            pass

        projectairsim_log().info("程序已退出")


# ==============================================================================
# 程序入口
# ==============================================================================

if __name__ == "__main__":
    asyncio.run(main())
