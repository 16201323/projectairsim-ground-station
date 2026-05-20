"""
核心模块 - 无人机控制线程

本模块实现无人机控制的完整生命周期：
DroneControlThread：在独立线程中运行异步控制循环

控制流程：
1. 初始化配置管理器，生成场景配置
2. 连接仿真环境（带友好提示）
3. 加载场景，创建无人机对象
4. 设置传感器订阅（通过SensorManager）
5. 解锁、起飞
6. 进入主控制循环（键盘/UDP模式）
7. 退出时着陆、清理资源
"""

import asyncio
import math
import os
import threading
import time
import traceback
from datetime import datetime

import cv2
import numpy as np

import projectairsim
from projectairsim import Drone, World
from projectairsim.types import Pose, Quaternion, Vector3
from projectairsim.utils import unpack_image, geo_to_ned_coordinates

from PyQt6.QtCore import QThread, pyqtSignal

from .config_manager import ConfigManager
from .data_recorder import DataRecorder
from .udp_manager import UDPManager
from .constants import (
    DRONE_MODELS, DEFAULT_SPEED, DEFAULT_YAW_SPEED,
    SPEED_STEP, MIN_SPEED, MAX_SPEED, CONTROL_DURATION,
    UDP_DEFAULT_IP, UDP_DEFAULT_PORT, UDP_MULTICAST_ADDR,
    CAMERA_WIDTH, CAMERA_HEIGHT, VIDEO_FPS,
)

from .nav_udp_sender import NavUDPSender
from .lidar_udp_sender import LidarUdpSender

from sensors import SensorData, SensorManager


class DroneControlThread(QThread):
    """
    无人机控制线程：在独立线程中运行异步控制循环

    设计原理：
    - PyQt6的UI运行在主线程，耗时操作不能阻塞主线程
    - 无人机控制涉及异步IO（网络通信、传感器订阅），需要在独立线程中运行
    - 通过Qt信号（pyqtSignal）与主界面通信，实现线程安全的数据传递

    信号说明：
    - log_signal(str, str): 日志信号，参数为(消息内容, 日志级别)
    - status_signal(str, str): 状态更新信号，参数为(状态项, 状态值)
    - udp_param_signal(dict): UDP参数更新信号，参数为指令字典
    - frame_signal(object): 视频帧信号，参数为(相机名, 图像帧)元组
    - sensor_data_signal(str, object): 传感器数据信号，参数为(传感器名, SensorData)
    - finished_signal(str): 线程结束信号，参数为结束原因

    控制流程：
    1. 初始化配置管理器，生成场景配置
    2. 连接仿真环境（带友好提示）
    3. 加载场景，创建无人机对象
    4. 设置传感器订阅（相机）
    5. 解锁、起飞
    6. 进入主控制循环（键盘/UDP模式）
    7. 退出时着陆、清理资源
    """

    log_signal = pyqtSignal(str, str)
    status_signal = pyqtSignal(str, str)
    udp_param_signal = pyqtSignal(dict)
    frame_signal = pyqtSignal(object)
    sensor_data_signal = pyqtSignal(str, object)
    finished_signal = pyqtSignal(str)

    def __init__(self, robot_config, drone_model_name, is_vtol,
                 control_mode, sim_config_path, address="127.0.0.1",
                 topicsport=8989, servicesport=8990,
                 udp_ip=UDP_DEFAULT_IP, udp_port=UDP_DEFAULT_PORT,
                 udp_multicast_addr=None):
        """
        初始化无人机控制线程

        参数：
            robot_config: 机器人配置文件名（如"robot_quadrotor_adv.jsonc"）
            drone_model_name: 无人机型号显示名称（如"四旋翼"）
            is_vtol: 是否支持VTOL模式切换
            control_mode: 控制模式（"键盘控制" 或 "UDP自动控制"）
            sim_config_path: 仿真配置文件目录路径
            address: 仿真器IP地址（默认127.0.0.1）
            topicsport: 话题端口（默认8989）
            servicesport: 服务端口（默认8990）
            udp_ip: UDP本机网络接口IP（如192.168.1.5，用于指定接收组播的网卡）
            udp_port: UDP监听端口（默认15610）
            udp_multicast_addr: UDP组播组地址（如224.0.0.25），None表示单播模式
        """
        super().__init__()
        self.robot_config = robot_config
        self.drone_model_name = drone_model_name
        self.is_vtol = is_vtol
        self.control_mode = control_mode
        self.sim_config_path = sim_config_path
        self.address = address
        self.topicsport = topicsport
        self.servicesport = servicesport
        self.udp_ip = udp_ip
        self.udp_port = udp_port
        self.udp_multicast_addr = udp_multicast_addr
        self.running = False
        self._stop_requested = False
        self._land_requested = False
        self._takeoff_requested = False
        self._photo_front = False
        self._photo_down = False
        self._photo_chase = False
        self._photo_stereo_left = False
        self._photo_stereo_right = False
        self._vtol_toggle = False
        self._speed_up = False
        self._speed_down = False
        self._chase_gimbal_requested = False
        self._chase_gimbal_roll = 0.0
        self._chase_gimbal_pitch = -15.0
        self._chase_gimbal_yaw = 0.0
        self._drone_obj = None
        # 键盘控制状态：当前按下的方向键对应的速度分量（-1/0/1）
        self.key_vx = 0       # 前后方向（W/S键，1=前进，-1=后退）
        self.key_vy = 0       # 左右方向（D/A键，1=右移，-1=左移）
        self.key_vz = 0       # 上下方向（↑/↓键，-1=上升，1=下降）
        self.key_yaw = 0      # 偏航方向（←/→键，-1=左转，1=右转）
        self.speed = DEFAULT_SPEED       # 当前飞行速度（米/秒）
        self.yaw_speed = DEFAULT_YAW_SPEED  # 当前偏航速度（度/秒）
        # 缓存的最新相机帧，用于拍照功能
        self._latest_stereo_left_frame = None  # 最新双目左相机帧
        self._latest_down_frame = None    # 最新下视相机帧
        self._latest_chase_frame = None   # 最新第三人称相机帧
        self._latest_stereo_right_frame = None # 最新双目右相机帧
        # 帧数据的线程锁，保护并发访问
        self._frame_lock = threading.Lock()
        self._lidar_lock = threading.Lock()
        self._latest_lidar_data = None
        # 位置缓存与计数器（主循环中使用）
        self._pos_update_counter = 0
        self._cached_pos = {"x": 0, "y": 0, "z": 0}
        self._cached_lat = 0
        self._cached_lon = 0
        self._cached_alt = 0
        self._cached_agl = 0
        self._cached_yaw_rad = 0.0
        # UDP控制状态缓存
        self._udp_packet_count = 0
        self._home_geo_point = {}
        self._udp_pos_calibrated = False
        self._udp_pos_offset_x = 0.0
        self._udp_pos_offset_y = 0.0
        self._udp_pos_offset_z = 0.0
        self._last_vel_cmd_time = 0.0
        self._smooth_vn = 0.0
        self._smooth_ve = 0.0
        self._smooth_vd = 0.0
        self._nav_udp_sender = None
        self._lidar_udp_sender = None
        # 飞控断开检测：超过1秒无数据打印一次警告（仅打印一次）
        self._last_udp_recv_time = 0.0
        self._udp_disconnect_warned = False