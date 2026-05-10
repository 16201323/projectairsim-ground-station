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
        self._smooth_yr = 0.0

    def request_stop(self):
        """请求停止控制线程（安全退出）"""
        self._stop_requested = True

    def request_land(self):
        """请求着陆（触发着陆流程）"""
        self._land_requested = True

    def request_takeoff(self):
        """请求再次起飞（着陆后使用）"""
        self._takeoff_requested = True

    def request_photo_front(self):
        """请求前视相机拍照"""
        self._photo_front = True

    def request_photo_down(self):
        """请求下视相机拍照"""
        self._photo_down = True

    def request_photo_chase(self):
        """请求第三人称相机拍照"""
        self._photo_chase = True

    def request_photo_stereo_left(self):
        """请求双目左相机拍照"""
        self._photo_stereo_left = True

    def request_photo_stereo_right(self):
        """请求双目右相机拍照"""
        self._photo_stereo_right = True

    def request_vtol_toggle(self):
        """请求切换VTOL模式（多旋翼↔固定翼）"""
        self._vtol_toggle = True

    def request_speed_up(self):
        """请求加速（增加飞行速度）"""
        self._speed_up = True

    def request_speed_down(self):
        """请求减速（降低飞行速度）"""
        self._speed_down = True

    def request_set_chase_gimbal(self, roll, pitch, yaw):
        """请求设置追踪相机云台角度（度）"""
        self._chase_gimbal_roll = roll
        self._chase_gimbal_pitch = pitch
        self._chase_gimbal_yaw = yaw
        self._chase_gimbal_requested = True

    def get_latest_lidar_data(self):
        with self._lidar_lock:
            return self._latest_lidar_data

    def update_keyboard(self, vx, vy, vz, yaw):
        """
        更新键盘控制状态（由主界面定时调用）

        参数：
            vx: 前后方向分量（-1/0/1）
            vy: 左右方向分量（-1/0/1）
            vz: 上下方向分量（-1/0/1）
            yaw: 偏航方向分量（-1/0/1）
        """
        self.key_vx = vx
        self.key_vy = vy
        self.key_vz = vz
        self.key_yaw = yaw

    def _log(self, msg, level="INFO"):
        """
        发送日志消息到主界面

        参数：
            msg: 日志消息内容
            level: 日志级别（"INFO"/"WARNING"/"ERROR"）
        """
        ts = datetime.now().strftime("%H:%M:%S.%f")[:-3]
        self.log_signal.emit(f"[{ts}] {msg}", level)

    def run(self):
        """
        线程入口方法（QThread.run的重写）
        创建新的事件循环并运行异步主控制流程
        异常时发送错误日志，结束时发送finished信号
        """
        try:
            asyncio.run(self._async_main())
        except Exception as e:
            self._log(f"控制线程异常: {e}", "ERROR")
        finally:
            self.finished_signal.emit("stopped")

    async def _async_main(self):
        """
        异步主控制流程
        完整的无人机控制生命周期：连接→加载→起飞→控制→着陆→清理

        状态转换：
        空闲 → 连接中 → 已连接 → 起飞中 → 飞行中 → 着陆中 → 空闲

        异常处理策略：
        - 连接失败：友好提示，不崩溃
        - 场景加载失败：断开连接，友好提示
        - 控制异常：记录日志，尝试着陆
        - 着陆超时：30秒超时保护
        """
        # 步骤1：初始化配置管理器，根据无人机型号生成场景配置
        self._log("初始化配置管理器...", "INFO")
        config_mgr = ConfigManager(self.sim_config_path)
        # UDP模式下，将home-geo-point设置为外部程序对应的地理位置
        # 确保NED坐标转换结果在仿真世界范围内
        custom_home_geo = None
        if self.control_mode == "UDP自动控制":
            custom_home_geo = UDP_HOME_GEO_POINT
        scene_config = config_mgr.generate_scene_config(self.robot_config, custom_home_geo)
        self._log(f"场景配置: {scene_config}", "INFO")

        # 初始化数据记录管理器，创建会话目录和日志文件
        data_recorder = DataRecorder()
        self._log(f"数据保存目录: {data_recorder.session_dir}", "INFO")

        # 如果是UDP模式，创建UDP管理器（稍后启动）
        udp_manager = None
        if self.control_mode == "UDP自动控制":
            udp_manager = UDPManager(self.udp_ip, self.udp_port, self.udp_multicast_addr)

        # 创建AirSim客户端，指定连接参数
        client = projectairsim.ProjectAirSimClient(
            address=self.address,
            port_topics=self.topicsport,
            port_services=self.servicesport,
        )

        drone = None
        is_flying = False
        is_fixed_wing = False   # VTOL模式标记：False=多旋翼，True=固定翼

        try:
            # 步骤2：连接仿真环境（带友好错误提示）
            self._log("正在连接仿真环境...", "INFO")
            self.status_signal.emit("connection", "connecting")
            try:
                client.connect()
            except Exception as conn_err:
                # 连接失败时提供友好的错误提示，而不是直接崩溃
                self._log("仿真环境连接失败！", "ERROR")
                self._log(f"错误: {conn_err}", "ERROR")
                self._log("请检查：1.仿真器是否启动 2.IP/端口是否正确 3.场景是否加载完成", "ERROR")
                self.status_signal.emit("connection", "disconnected")
                return

            self._log("仿真环境连接成功", "INFO")
            self.status_signal.emit("connection", "connected")

            # 步骤3：加载场景（使用生成的临时配置文件）
            self._log("正在加载场景...", "INFO")
            try:
                scene_fn = os.path.basename(scene_config)
                world = World(client=client, scene_config_name=scene_fn,
                              sim_config_path=self.sim_config_path, delay_after_load_sec=2)
                self._log("场景加载完成", "INFO")
            except Exception as scene_err:
                self._log(f"场景加载失败: {scene_err}", "ERROR")
                client.disconnect()
                self.status_signal.emit("connection", "disconnected")
                return

            # 步骤4：创建无人机对象
            drone = Drone(client, world, "Drone1")
            self._drone_obj = drone
            self._log(f"无人机创建完成: {self.drone_model_name}", "INFO")

            # 保存home_geo_point用于UDP位置控制坐标转换
            self._home_geo_point = drone.home_geo_point
            self._world = world

            # 步骤5：设置传感器订阅（相机）
            self._setup_sensors(client, drone, data_recorder)

            # 初始化视频录像写入器
            # 录像功能暂时禁用（用户要求取消实时录像，后续可能恢复）
            # data_recorder.init_video_writers()

            # 启动UDP监听（仅UDP模式）
            if udp_manager:
                udp_manager.start()
                if self.udp_multicast_addr:
                    self._log(f"UDP组播监听已启动: {self.udp_multicast_addr}:{self.udp_port} (接口: {self.udp_ip})", "INFO")
                else:
                    self._log(f"UDP监听已启动: {self.udp_ip}:{self.udp_port}", "INFO")

            # 步骤6：解锁并起飞
            self._log("正在解锁无人机...", "INFO")
            drone.enable_api_control()   # 启用API控制权
            drone.arm()                   # 解锁电机
            self.status_signal.emit("flight", "taking_off")

            # 检测无人机是否在地面以下，如果是则执行快速地下救援
            # DynamicCity等UE5原生城市环境可能将地面建在很高的UE世界坐标处
            # 简化为2步快速救援，减少启动延迟：
            # 1.探测地面高度 2.set_pose直接传送到安全高度
            rescue_success = False
            try:
                kin = drone.get_ground_truth_kinematics()
                pos = kin["pose"]["position"]
                drone_z = pos["z"]
                drone_x = pos["x"]
                drone_y = pos["y"]

                # 快速探测地面高度：仅在3个关键位置尝试
                # 减少探测次数，加快启动速度
                ground_z = None
                for px, py in [(drone_x, drone_y), (0.0, 0.0), (100.0, 100.0)]:
                    try:
                        gz = world.get_surface_elevation_at_point(px, py)
                        if gz is not None:
                            ground_z = gz
                            self._log(f"地面探测成功: ({px:.0f},{py:.0f}) Z={gz:.1f}", "INFO")
                            break
                    except Exception:
                        continue

                if ground_z is None:
                    ground_z = 0.0
                    self._log("地面探测失败，使用默认值0.0", "WARNING")

                agl = -(drone_z - ground_z)
                self._log(f"初始位置: x={drone_x:.1f} y={drone_y:.1f} z={drone_z:.1f} "
                          f"地面={ground_z:.1f} 离地={agl:.1f}m", "INFO")

                if agl < 1.0:
                    # 无人机在地面以下，直接传送到地面以上30m安全高度
                    safe_z = ground_z - 30.0
                    self._log(f"⚠️ 无人机在地面以下，传送到安全高度 z={safe_z:.1f}", "WARNING")
                    try:
                        rescue_pose = Pose({
                            "translation": Vector3({"x": drone_x, "y": drone_y, "z": safe_z}),
                            "rotation": Quaternion({"w": 1.0, "x": 0.0, "y": 0.0, "z": 0.0}),
                            "frame_id": "DEFAULT_ID",
                        })
                        drone.set_pose(rescue_pose, reset_kinematics=True)
                        await asyncio.sleep(0.5)
                        rescue_success = True
                        self._log("✅ 传送成功", "INFO")
                    except Exception as pose_err:
                        # set_pose失败，回退到move_to_position爬升
                        self._log(f"set_pose失败: {pose_err}，回退到爬升方式", "WARNING")
                        try:
                            await drone.move_to_position_async(drone_x, drone_y, safe_z, 5.0)
                            await asyncio.sleep(1.0)
                            rescue_success = True
                            self._log("爬升完成", "INFO")
                        except Exception as move_err:
                            self._log(f"爬升也失败: {move_err}", "ERROR")
                else:
                    self._log(f"✅ 无人机初始位置正常，离地{agl:.1f}m", "INFO")
                    rescue_success = True

            except Exception as e:
                self._log(f"地面检测异常(继续起飞): {e}", "WARNING")

            self._log("正在起飞...", "INFO")
            await drone.takeoff_async()   # 异步起飞，等待完成
            is_flying = True
            self.status_signal.emit("flight", "flying")
            self._log("起飞完成，进入控制循环", "INFO")
            # 仅手动模式下显示键盘控制说明
            if self.control_mode == "键盘控制":
                self._log("键盘控制: W/S前后 A/D左右 ↑↓升降 ←→偏转 +/-速度 F/G拍照 L点云 T着陆 Q退出", "INFO")

            self.running = True

            # 启动独立的位置更新异步任务（键盘控制模式下以2Hz低频更新位置）
            # 与主控制循环并行运行，不阻塞飞控指令发送
            pos_update_task = asyncio.create_task(
                self._position_update_loop(drone, world))

            # 步骤7：主控制循环（10ms周期≈100Hz）
            # 与advanced_drone_control.py保持一致的循环频率
            # 10ms循环确保飞控指令发送频率足够高，无人机运动平滑
            # 之前30ms循环导致飞控指令间隔过大，无人机运动出现明显顿挫
            # 关键优化：循环内仅做飞控指令发送，RPC调用移到独立定时器
            while not self._stop_requested:
                # 内层循环：飞行控制
                while not self._stop_requested and not self._land_requested:
                    await asyncio.sleep(0.01)

                    # ---- 处理拍照请求 ----
                    if self._photo_front:
                        self._photo_front = False
                        with self._frame_lock:
                            f = self._latest_stereo_left_frame
                        if f is not None:
                            data_recorder.save_photo("stereo_left", f)
                            self._log("双目左相机拍照已保存", "INFO")
                    if self._photo_down:
                        self._photo_down = False
                        with self._frame_lock:
                            f = self._latest_down_frame
                        if f is not None:
                            data_recorder.save_photo("down", f)
                            self._log("下视拍照已保存", "INFO")
                    if self._photo_chase:
                        self._photo_chase = False
                        with self._frame_lock:
                            f = self._latest_chase_frame
                        if f is not None:
                            data_recorder.save_photo("chase", f)
                            self._log("第三人称拍照已保存", "INFO")
                    if self._photo_stereo_left:
                        self._photo_stereo_left = False
                        with self._frame_lock:
                            f = self._latest_stereo_left_frame
                        if f is not None:
                            data_recorder.save_photo("stereo_left", f)
                            self._log("双目左相机拍照已保存", "INFO")
                    if self._photo_stereo_right:
                        self._photo_stereo_right = False
                        with self._frame_lock:
                            f = self._latest_stereo_right_frame
                        if f is not None:
                            data_recorder.save_photo("stereo_right", f)
                            self._log("双目右相机拍照已保存", "INFO")

                    # ---- 处理VTOL模式切换请求（仅倾斜旋翼支持）----
                    if self._vtol_toggle and self.is_vtol:
                        self._vtol_toggle = False
                        if is_fixed_wing:
                            # 固定翼→多旋翼：悬停模式
                            await drone.set_vtol_mode_async(Drone.VTOLMode.Multirotor)
                            is_fixed_wing = False
                            self._log("已切换到多旋翼模式", "INFO")
                            self.status_signal.emit("vtol", "multirotor")
                        else:
                            # 多旋翼→固定翼：前飞模式
                            await drone.set_vtol_mode_async(Drone.VTOLMode.FixedWing)
                            is_fixed_wing = True
                            self._log("已切换到固定翼模式", "INFO")
                            self.status_signal.emit("vtol", "fixedwing")

                    # ---- 处理速度调节请求 ----
                    if self._speed_up:
                        self._speed_up = False
                        self.speed = min(self.speed + SPEED_STEP, MAX_SPEED)
                        self._log(f"飞行速度: {self.speed:.1f} m/s", "INFO")
                        self.status_signal.emit("speed", f"{self.speed:.1f}")
                    if self._speed_down:
                        self._speed_down = False
                        self.speed = max(self.speed - SPEED_STEP, MIN_SPEED)
                        self._log(f"飞行速度: {self.speed:.1f} m/s", "INFO")
                        self.status_signal.emit("speed", f"{self.speed:.1f}")

                    # ---- 处理追踪相机云台视角切换请求 ----
                    if self._chase_gimbal_requested and self._drone_obj is not None:
                        self._chase_gimbal_requested = False
                        try:
                            from projectairsim.types import Pose, Vector3, Quaternion
                            from projectairsim.utils import rpy_to_quaternion
                            import math
                            roll_rad = math.radians(self._chase_gimbal_roll)
                            pitch_rad = math.radians(self._chase_gimbal_pitch)
                            yaw_rad = math.radians(self._chase_gimbal_yaw)
                            w_val, x_val, y_val, z_val = rpy_to_quaternion(roll_rad, pitch_rad, yaw_rad)
                            chase_pose = Pose({
                                "translation": Vector3({"x": -4.0, "y": 0.0, "z": -1.2}),
                                "rotation": Quaternion({"w": w_val, "x": x_val, "y": y_val, "z": z_val})
                            })
                            self._drone_obj.set_camera_pose("Chase", chase_pose)
                            self._log(f"追踪相机云台: R={self._chase_gimbal_roll:.0f}° P={self._chase_gimbal_pitch:.0f}° Y={self._chase_gimbal_yaw:.0f}°", "INFO")
                        except Exception as e:
                            self._log(f"云台设置失败: {e}", "WARNING")

                    # ---- 获取当前位置信息 ----
                    # 核心优化：键盘控制模式下完全跳过RPC调用
                    # advanced_drone_control.py在键盘控制时不做任何周期性RPC调用
                    # RPC调用（get_ground_truth_kinematics等）是同步网络请求，
                    # 即使异步化也会占用事件循环时间，导致10ms控制循环被拉长
                    # 键盘控制模式下，位置信息仅用于日志和状态显示，不影响飞控
                    # 位置更新改为独立低频异步任务（_position_update_loop）
                    if self.control_mode == "UDP自动控制":
                        self._pos_update_counter += 1
                        pos = self._cached_pos
                        if self._pos_update_counter % 10 == 1:
                            try:
                                loop = asyncio.get_event_loop()
                                kin = await loop.run_in_executor(None, drone.get_ground_truth_kinematics)
                                pos = kin["pose"]["position"]
                                self._cached_pos = pos
                                orientation = kin.get("pose", {}).get("orientation", {})
                                qw = orientation.get("w", 1.0)
                                qx = orientation.get("x", 0.0)
                                qy = orientation.get("y", 0.0)
                                qz = orientation.get("z", 0.0)
                                siny_cosp = 2 * (qw * qz + qx * qy)
                                cosy_cosp = 1 - 2 * (qy * qy + qz * qz)
                                self._cached_yaw_rad = math.atan2(siny_cosp, cosy_cosp)
                            except Exception:
                                pos = self._cached_pos
                            if self._pos_update_counter % 20 == 1:
                                try:
                                    loop = asyncio.get_event_loop()
                                    geo = await loop.run_in_executor(None, drone.get_ground_truth_geo_location)
                                    self._cached_lat = geo.get("latitude", 0)
                                    self._cached_lon = geo.get("longitude", 0)
                                    self._cached_alt = geo.get("altitude", 0)
                                except Exception:
                                    pass
                                if self._pos_update_counter % 60 == 1:
                                    try:
                                        loop = asyncio.get_event_loop()
                                        ground_z = await loop.run_in_executor(
                                            None, lambda: world.get_surface_elevation_at_point(pos["x"], pos["y"]))
                                        self._cached_agl = -(pos["z"] - ground_z)
                                    except Exception:
                                        self._cached_agl = self._cached_agl
                    else:
                        # 键盘控制模式：使用缓存位置，不做RPC调用
                        pos = self._cached_pos
                    lat = self._cached_lat
                    lon = self._cached_lon
                    alt = self._cached_alt
                    agl = self._cached_agl

                    # ---- 控制逻辑分支：键盘控制 / UDP自动控制 ----
                    if self.control_mode == "键盘控制":
                        # 键盘控制模式：根据按键状态计算速度并执行移动
                        # 速度 = 方向分量(-1/0/1) × 飞行速度
                        vx = self.key_vx * self.speed
                        vy = self.key_vy * self.speed
                        vz = self.key_vz * self.speed
                        yr = self.key_yaw * self.yaw_speed
                        # 仅在有移动指令时发送控制命令
                        if vx != 0 or vy != 0 or vz != 0:
                            await drone.move_by_velocity_body_frame_async(vx, vy, vz, CONTROL_DURATION)
                        if yr != 0:
                            await drone.rotate_by_yaw_rate_async(yr, CONTROL_DURATION)
                        # 记录控制指令日志
                        if vx != 0 or vy != 0 or vz != 0 or yr != 0:
                            data_recorder.log_control_command("manual", vx, vy, vz, yr, pos)

                    elif self.control_mode == "UDP自动控制":
                        if udp_manager:
                            loop = asyncio.get_event_loop()
                            cmd = await loop.run_in_executor(None, udp_manager.receive_command)
                            if cmd is not None:
                                self.udp_param_signal.emit(cmd)
                                try:
                                    await self._process_udp(drone, cmd, data_recorder, pos)
                                except Exception as e:
                                    self._log(f"执行UDP指令异常: {e}", "ERROR")

                # ---- 着陆处理（着陆按钮触发后执行）----
                if self._land_requested and is_flying and drone:
                    self._log(f"着陆条件满足: _land_requested={self._land_requested}, is_flying={is_flying}", "INFO")
                    self._log("正在着陆...", "INFO")
                    self.status_signal.emit("flight", "landing")
                    try:
                        lt = await drone.land_async()
                        await asyncio.wait_for(lt, timeout=60.0)
                        is_flying = False
                        self._log("着陆完成，按↑键或点击启动可再次起飞", "INFO")
                        self.status_signal.emit("flight", "landed")
                    except asyncio.TimeoutError:
                        self._log("着陆超时，继续等待", "WARNING")
                    except Exception as e:
                        self._log(f"着陆异常: {e}", "WARNING")
                    self._land_requested = False
                elif self._land_requested and not is_flying:
                    self._log(f"着陆请求已忽略：_land_requested={self._land_requested}, is_flying={is_flying}", "WARNING")
                    self._land_requested = False

                    # 着陆后等待：按↑键或点击启动可再次起飞，或点击退出断开连接
                    self._log("等待指令：按↑键/点击启动再次起飞，或点击退出断开连接", "INFO")
                    while not self._stop_requested and not self._takeoff_requested:
                        await asyncio.sleep(0.05)

                    # 再次起飞
                    if self._takeoff_requested and not self._stop_requested:
                        self._takeoff_requested = False
                        try:
                            await drone.takeoff_async()
                            is_flying = True
                            self._log("再次起飞成功", "INFO")
                            self.status_signal.emit("flight", "flying")
                        except Exception as e:
                            self._log(f"起飞失败: {e}", "ERROR")

        except Exception as e:
            self._log(f"控制异常: {e}", "ERROR")

        finally:
            # 步骤8：资源清理（退出时执行）
            # 着陆逻辑已在主循环中处理，此处仅做资源释放

            # 取消位置更新异步任务
            try:
                pos_update_task.cancel()
            except Exception:
                pass

            # 禁用API控制权
            if drone:
                try:
                    drone.disable_api_control()
                except Exception:
                    pass

            # 停止UDP监听
            if udp_manager:
                udp_manager.stop()
                self._log("UDP监听已停止", "INFO")

            # 释放数据记录器资源（关闭视频写入器等）
            data_recorder.release()

            # 断开仿真环境连接
            try:
                client.disconnect()
            except Exception:
                pass

            # 删除临时场景配置文件
            try:
                tmp = os.path.join(self.sim_config_path, "_scene_adv_drone_temp.jsonc")
                if os.path.exists(tmp):
                    os.remove(tmp)
            except Exception:
                pass

            # 更新状态并通知主界面
            self.running = False
            self.status_signal.emit("connection", "disconnected")
            self.status_signal.emit("flight", "idle")
            self._log("控制线程已退出", "INFO")

    async def _position_update_loop(self, drone, world):
        """
        独立的位置更新异步任务
        以2Hz低频更新无人机位置信息，供UI状态显示使用

        设计原理：
        - 主控制循环（10ms）仅做飞控指令发送，不做任何RPC调用
        - 位置信息（kinematics/geo/elevation）通过此独立任务以2Hz更新
        - 2Hz对地面站数值显示已足够，且不会阻塞主控制循环
        - 与advanced_drone_control.py的设计理念一致：
          键盘控制时不在主循环中做RPC调用
        """
        while not self._stop_requested:
            try:
                await asyncio.sleep(0.5)  # 2Hz更新频率
                loop = asyncio.get_event_loop()
                kin = await loop.run_in_executor(None, drone.get_ground_truth_kinematics)
                pos = kin["pose"]["position"]
                self._cached_pos = pos
                # 缓存偏航角
                orientation = kin.get("pose", {}).get("orientation", {})
                qw = orientation.get("w", 1.0)
                qx = orientation.get("x", 0.0)
                qy = orientation.get("y", 0.0)
                qz = orientation.get("z", 0.0)
                siny_cosp = 2 * (qw * qz + qx * qy)
                cosy_cosp = 1 - 2 * (qy * qy + qz * qz)
                self._cached_yaw_rad = math.atan2(siny_cosp, cosy_cosp)
                # 更新经纬度（每2次循环≈1Hz）
                self._pos_update_counter += 1
                if self._pos_update_counter % 2 == 1:
                    try:
                        geo = await loop.run_in_executor(None, drone.get_ground_truth_geo_location)
                        self._cached_lat = geo.get("latitude", 0)
                        self._cached_lon = geo.get("longitude", 0)
                        self._cached_alt = geo.get("altitude", 0)
                    except Exception:
                        pass
                # 更新地面高度（每6次循环≈0.33Hz）
                if self._pos_update_counter % 6 == 1:
                    try:
                        ground_z = await loop.run_in_executor(
                            None, lambda: world.get_surface_elevation_at_point(pos["x"], pos["y"]))
                        self._cached_agl = -(pos["z"] - ground_z)
                    except Exception:
                        pass
                # 发送位置状态到UI
                self.status_signal.emit("position", f"{pos.get('x',0):.1f},{pos.get('y',0):.1f},{pos.get('z',0):.1f}")
            except asyncio.CancelledError:
                break
            except Exception:
                pass

    def _geo_to_ned(self, lat, lon, abs_alt):
        """经纬高转NED坐标（优先使用SDK，失败时回退到简易计算）"""
        try:
            coords = geo_to_ned_coordinates(self._home_geo_point, [lat, lon, abs_alt])
            return coords[0], coords[1], coords[2]
        except Exception:
            home_lat = self._home_geo_point.get("latitude", 47.0)
            home_lon = self._home_geo_point.get("longitude", -122.0)
            meters_per_deg_lat = 111320.0
            meters_per_deg_lon = 111320.0 * math.cos(math.radians(home_lat))
            return ((lat - home_lat) * meters_per_deg_lat,
                    (lon - home_lon) * meters_per_deg_lon,
                    -abs_alt)

    async def _process_udp(self, drone, cmd, data_recorder, pos):
        """
        处理UDP控制指令（ModelOutputStruct结构体解析后的字典）

        两阶段控制策略：
        - 前3包：闪现模式，用set_pose直接将无人机定位到UDP数据指定的位置和姿态
        - 第4包起：速度跟踪模式，机体速度转NED + 自动校准 + P校正 + 偏航速率

        处理流程：
        1. 从结构体中提取机体速度（Vx/Vy/Hdot）、偏航角速率（R）、位置（lon/lat/alt）
        2. 前3包：经纬高转NED → 欧拉角转四元数 → set_pose闪现到位
        3. 第4包起：机体速度旋转到NED → 首次校准位置偏移 → P校正 → move_by_velocity_async

        参数：
            drone: 无人机对象
            cmd: UDP指令字典，包含ModelOutputStruct所有字段
            data_recorder: 数据记录器
            pos: 当前位置字典
        """
        try:
            self._udp_packet_count += 1
            pkt = self._udp_packet_count

            if pkt <= 3:
                await self._process_udp_teleport(drone, cmd, data_recorder, pos, pkt)
            else:
                await self._process_udp_track(drone, cmd, data_recorder, pos, pkt)
        except Exception as e:
            traceback.print_exc()
            self._log(f"处理UDP指令失败: {e}\n{traceback.format_exc()}", "WARNING")

    async def _process_udp_teleport(self, drone, cmd, data_recorder, pos, pkt):
        """前3包闪现：直接从UDP经纬高+姿态设置无人机位姿"""
        lon = cmd.get("lon", 0)
        lat = cmd.get("lat", 0)
        alt = cmd.get("alt", 0)
        phi = cmd.get("phi", 0)
        theta = cmd.get("theta", 0)
        psi = cmd.get("psi", 0)

        home_geo = self._home_geo_point
        home_altitude = home_geo.get("altitude", 0)
        abs_alt = alt + home_altitude

        target_x, target_y, target_z = self._geo_to_ned(lat, lon, abs_alt)

        phi_rad = math.radians(phi)
        theta_rad = math.radians(theta)
        psi_rad = math.radians(psi)

        cy = math.cos(psi_rad / 2)
        sy = math.sin(psi_rad / 2)
        cp = math.cos(theta_rad / 2)
        sp = math.sin(theta_rad / 2)
        cr = math.cos(phi_rad / 2)
        sr = math.sin(phi_rad / 2)

        qw = cr * cp * cy + sr * sp * sy
        qx = sr * cp * cy - cr * sp * sy
        qy = cr * sp * cy + sr * cp * sy
        qz = cr * cp * sy - sr * sp * cy

        pose = Pose({
            "translation": Vector3({"x": target_x, "y": target_y, "z": target_z}),
            "rotation": Quaternion({"w": qw, "x": qx, "y": qy, "z": qz}),
            "frame_id": "DEFAULT_ID",
        })

        drone.set_pose(pose, reset_kinematics=True)

        data_recorder.log_control_command(
            "udp", cmd.get("vn", 0), cmd.get("ve", 0), cmd.get("Hdot", 0),
            cmd.get("psi", 0), pos)

    async def _process_udp_track(self, drone, cmd, data_recorder, pos, pkt):
        """第4包起速度跟踪：机体速度转NED + 自动位置校准 + P校正 + 偏航速率
        50ms速率限制 + EMA平滑，防止高频指令风暴导致飞控抖动"""

        now = time.monotonic()
        if now - self._last_vel_cmd_time < 0.05:
            return
        self._last_vel_cmd_time = now

        Vx = cmd.get("Vx", 0)
        Vy = cmd.get("Vy", 0)
        Hdot = cmd.get("Hdot", 0)
        R = cmd.get("R", 0)

        yaw = self._cached_yaw_rad
        cy = math.cos(yaw)
        sy = math.sin(yaw)
        vn_body = Vx * cy - Vy * sy
        ve_body = Vx * sy + Vy * cy

        lon = cmd.get("lon", 0)
        lat = cmd.get("lat", 0)
        alt = cmd.get("alt", 0)

        home_geo = self._home_geo_point
        home_altitude = home_geo.get("altitude", 0)
        abs_alt = alt + home_altitude

        raw_x, raw_y, raw_z = self._geo_to_ned(lat, lon, abs_alt)

        if not self._udp_pos_calibrated:
            self._udp_pos_offset_x = pos.get("x", 0) - raw_x
            self._udp_pos_offset_y = pos.get("y", 0) - raw_y
            self._udp_pos_offset_z = pos.get("z", 0) - raw_z
            self._udp_pos_calibrated = True
            self._log(f"UDP位置校准完成: 偏移=({self._udp_pos_offset_x:.1f}, "
                      f"{self._udp_pos_offset_y:.1f}, {self._udp_pos_offset_z:.1f})m", "INFO")

        target_x = raw_x + self._udp_pos_offset_x
        target_y = raw_y + self._udp_pos_offset_y
        target_z = raw_z + self._udp_pos_offset_z

        Kp = 0.5
        MAX_POS_CORRECTION = 5.0

        delta_x = target_x - pos.get("x", 0)
        delta_y = target_y - pos.get("y", 0)
        delta_z = target_z - pos.get("z", 0)

        corr_x = max(-MAX_POS_CORRECTION, min(MAX_POS_CORRECTION, Kp * delta_x))
        corr_y = max(-MAX_POS_CORRECTION, min(MAX_POS_CORRECTION, Kp * delta_y))
        corr_z = max(-MAX_POS_CORRECTION, min(MAX_POS_CORRECTION, Kp * delta_z))

        v_north = vn_body + corr_x
        v_east = ve_body + corr_y
        v_down = -Hdot + corr_z
        yaw_rate_rad = math.radians(R)

        alpha = 0.4
        smooth_vn = self._smooth_vn
        smooth_ve = self._smooth_ve
        smooth_vd = self._smooth_vd
        smooth_yr = self._smooth_yr
        self._smooth_vn = alpha * v_north + (1 - alpha) * smooth_vn
        self._smooth_ve = alpha * v_east + (1 - alpha) * smooth_ve
        self._smooth_vd = alpha * v_down + (1 - alpha) * smooth_vd
        self._smooth_yr = alpha * yaw_rate_rad + (1 - alpha) * smooth_yr

        await drone.move_by_velocity_async(
            self._smooth_vn, self._smooth_ve, self._smooth_vd, 0.1,
            yaw_is_rate=True,
            yaw=self._smooth_yr,
        )

        data_recorder.log_control_command(
            "udp", vn_body, ve_body, Hdot, cmd.get("psi", 0), pos)

    def _setup_sensors(self, client, drone, data_recorder):
        """
        设置传感器订阅
        使用SensorManager统一管理所有传感器的创建和订阅

        性能优化说明：
        - 相机帧不再通过frame_signal主动推送到UI（避免跨线程信号风暴）
        - 改为UI通过定时器主动拉取SensorManager中的缓存帧
        - 传感器数据（IMU/GPS/高度表等）仍通过sensor_data_signal传递
          但已有节流控制（0.2秒间隔），不会造成性能问题

        SensorManager工作流程：
        1. 读取无人机传感器配置（drone.sensors）
        2. 根据配置自动创建对应的回调处理器
        3. 注册AirSim客户端订阅
        4. 通过信号将传感器数据传递到UI

        参数：
            client: AirSim客户端
            drone: 无人机对象
            data_recorder: 数据记录器
        """
        # 相机帧回调：仅缓存帧用于拍照，不再emit信号到UI
        # UI通过定时器主动调用SensorManager.get_camera_frame()拉取最新帧
        def on_frame(camera_key, frame):
            """相机帧回调：仅缓存最新帧用于拍照功能"""
            with self._frame_lock:
                if camera_key == "stereo_left":
                    self._latest_stereo_left_frame = frame
                elif camera_key == "down":
                    self._latest_down_frame = frame
                elif camera_key == "chase":
                    self._latest_chase_frame = frame
                elif camera_key == "stereo_right":
                    self._latest_stereo_right_frame = frame

        def on_sensor_data(sensor_name, data):
            """传感器数据更新回调：发送到UI传感器面板"""
            if isinstance(data, SensorData):
                self.sensor_data_signal.emit(sensor_name, data)

        # 创建SensorManager
        self._sensor_manager = SensorManager(
            client=client,
            drone=drone,
            recorder=data_recorder,
            log_func=self._log,
            frame_callback=on_frame,
            sim_config_path=self.sim_config_path,
            robot_config=self.robot_config,
        )
        self._sensor_manager.set_sensor_data_callback(on_sensor_data)

        # 设置所有传感器订阅
        self._sensor_manager.setup_all_sensors()

        if "lidar1" in drone.sensors:
            lidar_topic = drone.sensors["lidar1"]["lidar"]
            def on_lidar(_, data):
                with self._lidar_lock:
                    self._latest_lidar_data = data
            client.subscribe(lidar_topic, on_lidar)
            self._log("LiDAR点云订阅已设置", "INFO")

    def _on_camera(self, image_msg, camera_name, data_recorder):
        """
        相机数据回调函数（兼容旧逻辑，新传感器通过SensorManager管理）
        当传感器数据更新时由AirSim客户端自动调用

        参数：
            image_msg: 相机图像消息字典
            camera_name: 相机名称
            data_recorder: 数据记录器
        """
        try:
            if image_msg and "data" in image_msg and len(image_msg["data"]) > 0:
                frame = unpack_image(image_msg)
                self.frame_signal.emit((camera_name, frame))
                data_recorder.write_video_frame(camera_name, frame)
                with self._frame_lock:
                    if camera_name == "stereo_left":
                        self._latest_stereo_left_frame = frame
                    elif camera_name == "down":
                        self._latest_down_frame = frame
                    elif camera_name == "chase":
                        self._latest_chase_frame = frame
                    elif camera_name == "stereo_right":
                        self._latest_stereo_right_frame = frame
        except Exception:
            pass
