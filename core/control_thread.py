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
    UDP_DEFAULT_IP, UDP_DEFAULT_PORT, UDP_MULTICAST_ADDR, UDP_HOME_GEO_POINT,
    CAMERA_WIDTH, CAMERA_HEIGHT, VIDEO_FPS,
)

from .nav_udp_sender import NavUDPSender
from .lidar_udp_sender import LidarUdpSender

from sensors import SensorData, SensorManager
