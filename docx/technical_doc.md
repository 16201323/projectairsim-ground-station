# 高级无人机地面站控制系统 — 项目框架与技术详解说明书

---

## 第一章 项目总览

### 1.1 项目定位

本项目是一套基于 **Microsoft Project AirSim** 仿真平台构建的 **高级无人机地面站控制系统**，提供从仿真连接、飞行控制、传感器数据采集到可视化展示的全链路解决方案。系统以 Python 为开发语言，采用模块化分层架构，同时提供 **GUI 图形界面版** 和 **CLI 命令行版** 两种交互形态。

### 1.2 核心能力

| 能力域 | 具体功能 |
|--------|----------|
| 多机型支持 | 四旋翼（Quadrotor-X）、六旋翼（Hexarotor-X）、倾斜旋翼VTOL（Quad-Tiltrotor） |
| 双模式控制 | 键盘手动控制（机体坐标系）、UDP自动控制（世界坐标系/闪现模式） |
| 多传感器采集 | 相机（前视/下视/追踪/双目/深度）、LiDAR、IMU、GPS、高度表（无线电/激光/超声波）、气压计、空速计、毫米波雷达 |
| 视频流处理 | 实时视频显示、XVID持续录像、手动拍照（JPG） |
| 点云处理 | LiDAR点云2D俯视图、3D立体可视化、NPY/PCD/LAS三格式保存 |
| 双目视觉 | 左右相机同步、SGBM视差图计算、深度图生成 |
| 数据持久化 | 时间戳会话目录、CSV控制日志、多格式数据导出 |
| 安全保护 | UDP超时自动悬停、异常捕获与恢复、退出后可重新启动 |

### 1.3 技术栈全景

| 层次 | 技术 | 用途 |
|------|------|------|
| 仿真平台 | Project AirSim (UE5) | 3D物理仿真与渲染后端 |
| 通信协议 | NNG (nanomsg-next-gen) | 客户端-服务端双通道TCP通信 |
| 序列化 | MessagePack + JSON | 传感器数据与控制消息编码 |
| 编程语言 | Python 3.11+ | 主开发语言 |
| 异步框架 | asyncio | 飞行控制命令异步执行 |
| GUI框架 | PyQt6 | 深色霓虹科幻风格界面 |
| 系统键盘 | pynput | 跨窗口系统级键盘监听 |
| 图像处理 | OpenCV (cv2) | 图像解码、录像编码、图像变换 |
| 数值计算 | NumPy | 点云矩阵运算、结构体解析 |
| 3D可视化 | Matplotlib (mplot3d) | LiDAR点云3D渲染（嵌入Qt） |
| 点云格式 | Open3D (可选) | PCD/LAS点云文件处理 |
| 网络通信 | socket (UDP) | 外部自动驾驶指令接收 |
| 配置格式 | JSONC (JSON with Comments) | 场景与机器人配置文件 |

---

## 第二章 系统架构

### 2.1 宏观架构：三层体系

本项目运行于 Project AirSim 的三层架构之上：

```
┌──────────────────────────────────────────────────────────────────────┐
│                        用户交互层 (Client)                           │
│  ┌─────────────────────────┐  ┌──────────────────────────────────┐  │
│  │   GUI 地面站 (PyQt6)     │  │   CLI 命令行版 (OpenCV/Open3D)   │  │
│  │   drone_ground_station   │  │   advanced_drone_control         │  │
│  └────────────┬────────────┘  └─────────────┬────────────────────┘  │
│               │                              │                       │
│  ┌────────────┴──────────────────────────────┴────────────────────┐ │
│  │              Python 客户端库 (projectairsim)                    │ │
│  │   ProjectAirSimClient / World / Drone / Sensor APIs            │ │
│  └────────────────────────────┬───────────────────────────────────┘ │
├───────────────────────────────┼─────────────────────────────────────┤
│           NNG 通信层          │                                     │
│   TCP:8989 (Pub/Sub)          │  TCP:8990 (Req/Rep)                │
│   传感器数据流                │  控制命令 / 同步查询                 │
├───────────────────────────────┴─────────────────────────────────────┤
│                     仿真服务层 (SimServer)                           │
│  ┌──────────┐  ┌──────────┐  ┌──────────┐  ┌──────────────────┐   │
│  │ FastPhysics│  │ SimpleFlight│  │ Sensor   │  │ UE5 Rendering   │   │
│  │ 物理引擎   │  │ 飞行控制器  │  │ 传感器仿真│  │ 渲染引擎        │   │
│  └──────────┘  └──────────┘  └──────────┘  └──────────────────┘   │
├─────────────────────────────────────────────────────────────────────┤
│                   UE5 插件层 (ProjectAirSim Plugin)                  │
│   AProjectAirSimGameMode → AUnrealSimLoader → AUnrealRobot/Sensor   │
├─────────────────────────────────────────────────────────────────────┤
│                   Unreal Engine 5 渲染层                             │
│   Nanite / Lumen / PixelStreaming (h.264 WebRTC)                   │
└─────────────────────────────────────────────────────────────────────┘
```

### 2.2 客户端内部架构：模块分层

```
drone_ground_station.py (GUI主入口)
│
├── core/                          ← 核心业务层
│   ├── constants.py               ← 全局常量中心
│   ├── config_manager.py          ← 配置动态生成
│   ├── control_thread.py          ← 异步控制线程（核心调度器）
│   ├── data_recorder.py           ← 数据持久化管理
│   └── udp_manager.py             ← UDP外部通信
│
├── sensors/                       ← 传感器抽象层
│   ├── base.py                    ← SensorType枚举 + SensorCallback基类 + SensorData封装
│   ├── factory.py                 ← SensorFactory工厂模式
│   ├── manager.py                 ← SensorManager统一调度
│   ├── camera.py                  ← CameraCallback / DepthCameraCallback
│   ├── stereo_camera.py           ← StereoCameraCallback (SGBM视差)
│   ├── lidar.py                   ← LidarCallback (点云解析+保存)
│   ├── imu.py                     ← IMUCallback (四元数→欧拉角)
│   ├── gps.py                     ← GPSCallback
│   ├── altimeter.py               ← Radio/Laser/UltrasonicAltimeterCallback
│   ├── atmosphere.py              ← AtmosphereCallback (气压+空速组合)
│   ├── distance_sensor.py         ← DistanceSensorCallback
│   └── radar.py                   ← RadarCallback (检测+跟踪)
│
├── ui/                            ← 界面表现层
│   ├── widgets.py                 ← NeonLabel / StatusIndicator
│   ├── video_widget.py            ← VideoWidget (BGR→RGB→Qt渲染)
│   ├── lidar_widgets.py           ← Lidar2DWidget / Lidar3DWidget
│   └── sensor_panel.py            ← SensorPanel (动态传感器面板)
│
└── sim_config/                    ← 配置文件层
    ├── scene_*.jsonc              ← 场景配置（5个）
    └── robot_*.jsonc              ← 机器人配置（6个）
```

### 2.3 数据流全景

```
                          ┌─────────────────────┐
                          │   UE5 仿真服务端      │
                          └──────┬────────┬──────┘
                   NNG Pub/Sub   │        │  NNG Req/Rep
                  (TCP:8989)     │        │  (TCP:8990)
                                 │        │
┌────────────────────────────────┼────────┼────────────────────────────┐
│                                ▼        ▼                            │
│  ┌──────────────────────────────────────────────────────────────┐   │
│  │                   DroneControlThread                          │   │
│  │  ┌─────────────┐  ┌──────────────┐  ┌────────────────────┐  │   │
│  │  │ SensorManager│  │  UDPManager   │  │  DataRecorder      │  │   │
│  │  │  ┌────────┐ │  │  (UDP Socket) │  │  ┌──────────────┐  │  │   │
│  │  │  │Factory │ │  │               │  │  │ VideoWriters │  │  │   │
│  │  │  └───┬────┘ │  │  224字节      │  │  │ PhotoSaver   │  │  │   │
│  │  │      │      │  │  ModelOutput  │  │  │ LidarSaver   │  │  │   │
│  │  │  ┌───┴────┐ │  │  Struct解析   │  │  │ CSVLogger    │  │  │   │
│  │  │  │Callbacks│ │  │               │  │  └──────────────┘  │  │   │
│  │  │  │(11种)  │ │  └───────┬───────┘  └────────────────────┘  │   │
│  │  │  └────────┘ │          │                                   │   │
│  │  └──────┬───────┘          │                                   │   │
│  │         │                  │                                   │   │
│  │    Qt Signals         UDP Command                             │   │
│  │  ┌──────┴──────┐   ┌──────┴───────┐                          │   │
│  │  │frame_signal │   │_process_udp()│                          │   │
│  │  │lidar_signal │   │ LLA→NED坐标  │                          │   │
│  │  │sensor_signal│   │ 欧拉→四元数   │                          │   │
│  │  │udp_param_   │   │ set_pose闪现  │                          │   │
│  │  │signal      │   └──────────────┘                          │   │
│  │  │log_signal  │                                             │   │
│  │  │status_signal│                                            │   │
│  │  └──────┬──────┘                                             │   │
│  └─────────┼────────────────────────────────────────────────────┘   │
│            │                                                        │
│  ┌─────────┴────────────────────────────────────────────────────┐   │
│  │                GroundStationWindow (PyQt6)                    │   │
│  │  ┌──────────┐ ┌──────────┐ ┌──────────┐ ┌────────────────┐  │   │
│  │  │VideoWidget│ │Lidar2D/3D│ │SensorPanel│ │ UDP参数面板    │  │   │
│  │  └──────────┘ └──────────┘ └──────────┘ └────────────────┘  │   │
│  └──────────────────────────────────────────────────────────────┘   │
└─────────────────────────────────────────────────────────────────────┘
```

### 2.4 通信协议详解

#### 2.4.1 NNG 双通道架构

| 通道 | 端口 | 模式 | 序列化 | 用途 |
|------|------|------|--------|------|
| Topics | 8989 | Pub/Sub | MessagePack | 传感器数据流、状态更新、系统事件 |
| Services | 8990 | Req/Rep | MessagePack | 控制命令、同步查询、场景管理 |

#### 2.4.2 UDP 外部控制协议

UDP模式接收 **224字节 ModelOutputStruct** 结构体（28个double，8字节/个）：

| 偏移 | 字段 | 说明 |
|------|------|------|
| 0 | P | 总压 |
| 8 | R | 动压 |
| 16 | Q | 俯仰角速率 |
| 24 | Ax, Ay, Az | 三轴加速度 |
| 48 | Vx, Vy, Vz | 三轴速度 |
| 72 | Nx, Ny, Nz | 三轴推力方向 |
| 96 | theta, phi, psi | 欧拉角（俯仰/滚转/偏航） |
| 120 | alt | 海拔高度 |
| 128 | height | 离地高度 |
| 136 | Vi | 指示空速 |
| 144 | Vt | 真空速 |
| 152 | alpha, beta | 攻角/侧滑角 |
| 168 | vn, ve | 北向/东向速度 |
| 184 | Hdot | 垂直速度 |
| 192 | Vd | 下向速度 |
| 200 | lon, lat | 经度/纬度 |
| 216 | track | 航迹角 |

**闪现控制流程**：UDP数据 → 经纬高转NED坐标 → 欧拉角转四元数 → `set_pose()` 直接设置位姿

---

## 第三章 核心模块详解

### 3.1 constants.py — 全局常量中心

**设计理念**：集中管理所有可配置参数，避免魔法数字散落在代码各处，实现"一处修改，全局生效"。

| 常量分组 | 关键常量 | 默认值 | 说明 |
|----------|---------|--------|------|
| 无人机型号 | `DRONE_MODELS` | 3种机型 | 编号→(配置文件, 名称, 是否VTOL)映射 |
| 键盘控制 | `DEFAULT_SPEED` | 5.0 m/s | 默认飞行速度 |
| | `DEFAULT_YAW_SPEED` | 20.0 °/s | 默认偏航速率 |
| | `SPEED_STEP` | 1.0 m/s | 速度调节步长 |
| | `CONTROL_DURATION` | 0.1 s | 控制指令持续时间 |
| UDP通信 | `UDP_DEFAULT_IP` | 192.168.1.5 | UDP监听地址 |
| | `UDP_DEFAULT_PORT` | 15610 | UDP监听端口 |
| | `UDP_MULTICAST_ADDR` | 224.0.0.25 | 组播地址 |
| 相机参数 | `CAMERA_WIDTH/HEIGHT` | 1280×720 | 视频分辨率 |
| | `VIDEO_FPS` | 30 | 录像帧率 |
| UI配色 | `COLOR_BG_MAIN` | #0a0a1a | 主背景色 |
| | `COLOR_NEON_CYAN` | #00ffff | 霓虹青色 |
| | `COLOR_NEON_GREEN` | #00ff88 | 霓虹绿色 |
| 窗口尺寸 | `WINDOW_WIDTH/HEIGHT` | 1400×900 | 主窗口大小 |

### 3.2 config_manager.py — 配置动态生成器

**设计思路**：Project AirSim 的场景配置在 `World` 加载时一次性读取，无法运行时动态修改无人机型号。采用"模板+动态生成"策略：维护模板场景配置，根据用户选择修改 `robot-config` 字段后生成临时文件。

**工作流程**：

```
scene_adv_drone.jsonc (模板)
        │
        ▼  读取并移除JSONC注释
    JSON对象
        │
        ▼  替换 actors[0]["robot-config"]
    修改后JSON
        │
        ▼  可选：更新 home-geo-point
    最终JSON
        │
        ▼  写入临时文件
    /tmp/scene_xxx.json
        │
        ▼  World(client, temp_file_path)
    加载场景
```

**关键方法**：

| 方法 | 功能 |
|------|------|
| `generate_scene_config(robot_config_file, home_geo_point)` | 读取模板→替换配置→生成临时文件→返回文件路径 |

### 3.3 control_thread.py — 异步控制线程（核心调度器）

**设计思路**：将所有异步仿真操作封装在 `QThread` 中，通过 Qt 信号与主界面通信，避免阻塞 GUI 事件循环。线程内部创建独立的 `asyncio` 事件循环运行异步控制流程。

**类结构**：

```python
class DroneControlThread(QThread):
    # 7个Qt信号
    log_signal = Signal(str, str)          # (消息, 级别)
    status_signal = Signal(str, str)       # (状态名, 值)
    udp_param_signal = Signal(dict)        # UDP参数字典
    frame_signal = Signal(object)          # 视频帧
    lidar_signal = Signal(object)          # LiDAR数据
    sensor_data_signal = Signal(str, object)  # (传感器名, 数据)
    finished_signal = Signal(str)          # 结束消息
```

**异步主流程** (`_async_main`)：

```
connect() → World加载 → enable_api_control → arm → takeoff
    │
    ▼ 进入控制循环
    ┌──────────────────────────────────────────────┐
    │  while not stopped:                          │
    │    if 键盘模式:                               │
    │      move_by_velocity_body_frame_async()     │
    │    elif UDP模式:                              │
    │      _process_udp() → set_pose() 闪现        │
    │    await asyncio.sleep(CONTROL_DURATION)     │
    └──────────────────────────────────────────────┘
    │
    ▼ 退出循环
land → disconnect → cleanup
```

**请求方法模式**：所有外部操作通过"请求标志+异步检查"模式实现线程安全：

```python
def request_takeoff(self):
    self._takeoff_requested = True

# 在异步循环中检查
if self._takeoff_requested:
    self._takeoff_requested = False
    await drone.takeoff_async()
```

**VTOL模式切换**：

```python
def request_vtol_toggle(self):
    self._vtol_toggle_requested = True

# 异步循环中
if self._vtol_toggle_requested:
    current = drone.get_vtol_mode()
    new_mode = Drone.VTOLMode.FixedWing if current == Drone.VTOLMode.Multirotor else Drone.VTOLMode.Multirotor
    drone.set_vtol_mode(new_mode)
```

### 3.4 data_recorder.py — 数据持久化管理器

**会话目录结构**：

```
drone_data/
└── 20260428_124258/                    ← 时间戳会话目录
    ├── videos/
    │   ├── front_124301.avi            ← XVID编码前视录像
    │   └── down_124301.avi             ← XVID编码下视录像
    ├── images/                         ← 手动拍照 (JPG)
    ├── lidar/                          ← LiDAR快照 (NPY+PCD+LAS)
    └── logs/
        └── control_log.csv             ← 控制日志
```

**点云三格式保存**：

| 格式 | 文件 | 特点 |
|------|------|------|
| NPY | `lidar_xxx.npy` | NumPy二进制，最快读写，Python原生 |
| PCD | `lidar_xxx.pcd` | ASCII文本，Point Cloud Data标准格式，兼容PCL/Open3D |
| LAS | `lidar_xxx.las` | LAS 1.2二进制，测绘行业标准，格式0（20字节/点） |

**LAS 1.2 二进制格式**（20字节/点）：

| 偏移 | 类型 | 字段 |
|------|------|------|
| 0-3 | int32 | X坐标（缩放整数） |
| 4-7 | int32 | Y坐标 |
| 8-11 | int32 | Z坐标 |
| 12 | uint8 | 强度 |
| 13 | uint8 | 回波信息 |
| 14 | uint8 | 分类 |
| 15 | int8 | 扫描角度 |
| 16 | uint8 | 点源ID |
| 17-19 | uint8[3] | RGB颜色 |

**CSV控制日志格式**：

```csv
timestamp,mode,vx,vy,vz,yaw_rate,pos_x,pos_y,pos_z
2026-04-28T12:42:58.123456,keyboard,5.0,0.0,0.0,0.0,-1.0,8.0,-4.0
2026-04-28T12:42:58.223456,udp,2.0,1.0,0.0,0.0,-0.5,8.5,-4.0
```

### 3.5 udp_manager.py — UDP通信管理器

**设计思路**：非阻塞UDP套接字接收外部控制指令，支持单播和组播两种模式。自动检测组播地址（224.x.x.x~239.x.x.x）并加入组播组。

**核心方法**：

| 方法 | 功能 |
|------|------|
| `start()` | 创建UDP套接字，绑定端口，可选加入组播组 |
| `receive_command()` | 非阻塞接收，100ms超时，返回解析后的字典 |
| `_parse_struct(data)` | 解析224字节ModelOutputStruct为28个字段的字典 |
| `stop()` | 关闭套接字 |

**组播支持**：

```python
if multicast_addr:
    if sys.platform == 'win32':
        sock.setsockopt(socket.IPPROTO_IP, socket.IP_ADD_MEMBERSHIP,
                        struct.pack('4s4s', inet_aton(multicast_addr), inet_aton(ip)))
    else:
        sock.setsockopt(socket.IPPROTO_IP, socket.IP_ADD_MEMBERSHIP,
                        struct.pack('4sl', inet_aton(multicast_addr), INADDR_ANY))
```

---

## 第四章 传感器系统详解

### 4.1 传感器架构设计

传感器系统采用 **工厂模式 + 回调模式 + 管理器模式** 三层架构：

```
SensorManager (统一调度)
    │
    ├── TYPE_MAP: JSONC type → SensorType枚举
    ├── ID_TYPE_MAP: 传感器ID → SensorType (精确匹配)
    ├── CAMERA_KEY_MAP: 相机ID → camera_key
    │
    ├── setup_all_sensors()
    │   ├── 1. 识别双目相机组 → StereoCameraCallback
    │   ├── 2. 识别大气机组 → AtmosphereCallback
    │   └── 3. 独立传感器 → SensorFactory.create()
    │
    └── SensorFactory (工厂模式)
        ├── _creators: SensorType → lambda创建函数
        └── create(sensor_type, sensor_name, config) → SensorCallback
```

**传感器类型推断三级策略**：

```
1. ID精确匹配 (ID_TYPE_MAP)
   "RadioAltimeter" → RADIO_ALTIMETER
   "LaserAltimeter" → LASER_ALTIMETER
   "UltrasonicAltimeter" → ULTRASONIC_ALTIMETER
   "Barometer" → BAROMETER
   "Airspeed" → AIRSPEED
   │
2. 话题key匹配
   "scene_camera" → CAMERA
   "depth_camera" → DEPTH_CAMERA
   "lidar" → LIDAR
   │
3. ID关键词匹配
   "IMU" in sensor_id → IMU
   "GPS" in sensor_id → GPS
   "Radar" in sensor_id → RADAR
```

### 4.2 SensorType 枚举与 SensorData 封装

**SensorType**（15种）：

| 枚举值 | 说明 | 回调类 |
|--------|------|--------|
| CAMERA | 可见光相机 | CameraCallback |
| DEPTH_CAMERA | 深度相机 | DepthCameraCallback |
| STEREO_CAMERA | 双目相机 | StereoCameraCallback |
| LIDAR | 激光雷达 | LidarCallback |
| RADAR | 毫米波雷达 | RadarCallback |
| IMU | 惯性测量单元 | IMUCallback |
| GPS | 全球定位系统 | GPSCallback |
| MAGNETOMETER | 磁力计 | — |
| RADIO_ALTIMETER | 无线电高度表 | RadioAltimeterCallback |
| LASER_ALTIMETER | 激光高度表 | LaserAltimeterCallback |
| ULTRASONIC_ALTIMETER | 超声波高度表 | UltrasonicAltimeterCallback |
| BAROMETER | 气压计 | AtmosphereCallback (组合) |
| AIRSPEED | 空速计 | AtmosphereCallback (组合) |
| DISTANCE_SENSOR | 通用距离传感器 | DistanceSensorCallback |
| BATTERY | 电池 | — |

**SensorData** 数据封装（dataclass）：

```python
@dataclass
class SensorData:
    sensor_type: SensorType
    sensor_name: str
    timestamp: float
    payload: dict
```

### 4.3 SensorCallback 基类

```python
class SensorCallback(ABC):
    def __init__(self, sensor_name, sensor_type):
        self._sensor_name = sensor_name
        self._sensor_type = sensor_type
        self._latest_data = None
        self._lock = threading.Lock()

    @abstractmethod
    def __call__(self, client, data):
        """子类必须实现：处理传感器原始数据"""
        pass

    def get_latest_data(self) -> SensorData:
        """线程安全获取最新数据"""

    def get_display_fields(self) -> dict:
        """返回UI显示字段字典"""

    def _update_data(self, payload: dict):
        """更新数据并创建SensorData"""
```

### 4.4 各传感器回调详解

#### 4.4.1 CameraCallback — 可见光相机

**数据处理流程**：

```
AirSim压缩图像数据
    │
    ▼ cv2.imdecode(np.frombuffer(data, np.uint8), cv2.IMREAD_COLOR)
BGR NumPy数组
    │
    ├──→ frame_signal → VideoWidget (UI显示)
    ├──→ DataRecorder.write_video_frame() (录像)
    ├──→ 缓存最新帧 (手动拍照用)
    └──→ _update_data() (传感器面板)
```

**显示字段**：分辨率、帧计数

#### 4.4.2 DepthCameraCallback — 深度相机

**数据处理**：浮点像素深度图 → 计算深度统计（最小/最大/平均深度）

**显示字段**：最近距离、最远距离、平均深度

#### 4.4.3 StereoCameraCallback — 双目相机

**核心算法 — SGBM视差图计算**：

```
左相机帧 ──→ 灰度化 ──→ ┐
                        ├──→ cv2.StereoSGBM_create().compute() ──→ 视差图
右相机帧 ──→ 灰度化 ──→ ┘
```

**深度图生成**：

```
depth = (baseline × focal_length) / disparity
```

**SGBM参数**：

| 参数 | 值 | 说明 |
|------|-----|------|
| minDisparity | 0 | 最小视差 |
| numDisparities | 64 | 视差搜索范围（必须为16的倍数） |
| blockSize | 5 | 匹配块大小 |
| P1 | 8×3×5² | 视差平滑惩罚（相邻像素视差变化±1） |
| P2 | 32×3×5² | 视差平滑惩罚（相邻像素视差变化>±1） |

**显示字段**：基线距离、视差范围、平均视差

#### 4.4.4 LidarCallback — 激光雷达

**数据处理**：

```
一维浮点数组 → reshape(-1, 3) → Nx3点云矩阵 (X, Y, Z)
    │
    ├──→ lidar_signal → Lidar2D/3DWidget (可视化)
    ├──→ 缓存最新数据 (快照保存用)
    └──→ 计算统计 (点数、距离范围)
```

**三格式保存**：
- NPY：`np.save()` 直接保存Nx3矩阵
- PCD：ASCII模式，逐行写入 `x y z` 坐标
- LAS：LAS 1.2二进制格式，含文件头（227字节）+ 点数据（20字节/点）

**显示字段**：点数、距离范围

#### 4.4.5 IMUCallback — 惯性测量单元

**核心算法 — 四元数转欧拉角**（ZYX旋转顺序）：

```python
def _quaternion_to_euler(w, x, y, z):
    # Roll (X轴旋转)
    sinr_cosp = 2 * (w * x + y * z)
    cosr_cosp = 1 - 2 * (x * x + y * y)
    roll = math.atan2(sinr_cosp, cosr_cosp)

    # Pitch (Y轴旋转)
    sinp = 2 * (w * y - z * x)
    sinp = max(-1.0, min(1.0, sinp))  # 限幅防NaN
    pitch = math.asin(sinp)

    # Yaw (Z轴旋转)
    siny_cosp = 2 * (w * z + x * y)
    cosy_cosp = 1 - 2 * (y * y + z * z)
    yaw = math.atan2(siny_cosp, cosy_cosp)

    return roll, pitch, yaw
```

**显示字段**：滚转角、俯仰角、偏航角、加速度X/Y/Z

#### 4.4.6 GPSCallback — 全球定位系统

**数据处理**：提取经纬度、海拔、地速、卫星数

**显示字段**：纬度、经度、海拔、地速、卫星数

#### 4.4.7 AltimeterCallbacks — 高度表组

三种高度表均基于 `distance-sensor` 类型模拟，朝下安装：

| 类型 | 量程 | 精度 | 用途 |
|------|------|------|------|
| RadioAltimeterCallback | 0~500m | ±0.5m | 中高空离地高度 |
| LaserAltimeterCallback | 0~300m | ±0.1m | 精密地形测量 |
| UltrasonicAltimeterCallback | 0~10m | ±0.02m | 近地精确测高/着陆 |

**显示字段**：高度值、状态（有效/超量程）

#### 4.4.8 AtmosphereCallback — 大气机组

**组合处理**：同时管理气压计和空速传感器，两个回调分别更新 payload 中的对应字段：

```python
class AtmosphereCallback(SensorCallback):
    def on_barometer(self, client, baro_data):
        # 更新气压高度和气压值，保留空速数据

    def on_airspeed(self, client, airspeed_data):
        # 更新指示空速，保留气压数据
```

**显示字段**：气压高度、指示空速、气压

#### 4.4.9 RadarCallback — 毫米波雷达

**双回调机制**：

| 回调 | 数据 | 处理 |
|------|------|------|
| `__call__()` | radar_detections | 检测目标列表 |
| `on_tracks()` | radar_tracks | 跟踪目标列表 |

**检测目标解析**：提取目标数量、最近距离、方位角、仰角

**显示字段**：目标数、最近距离、方位角、仰角

#### 4.4.10 DistanceSensorCallback — 通用距离传感器

**数据处理**：提取距离值和有效范围

**显示字段**：距离

---

## 第五章 界面系统详解

### 5.1 GroundStationWindow — 主窗口

**布局结构**：

```
┌──────────────────────────────────────────────────────────────┐
│  [LOGO] 高级无人机地面站   [起飞][着陆][退出]  ●状态  🕐时钟  │ ← 顶部标题栏
├──────────┬───────────────────────────────────────────────────┤
│ 控制面板  │                                                   │
│ ──────── │                                                   │
│ 机型选择  │              视频显示区域                          │
│ ○四旋翼   │         (VideoWidget × 3路切换)                   │
│ ○六旋翼   │                                                   │
│ ○VTOL    │                                                   │
│ ──────── │                                                   │
│ 控制模式  │                                                   │
│ ○键盘     │                                                   │
│ ○UDP     │                                                   │
│ ──────── │                                                   │
│ 速度: 5.0 │                                                   │
│ [+][-]   │                                                   │
│ ──────── │                                                   │
│ 传感器面板 │                                                   │
│ (SensorPanel)│                                                │
├──────────┴──────────────┬────────────────────────────────────┤
│  实时流动日志            │  LiDAR 3D  │  LiDAR 2D            │ ← 底部面板
│  (颜色区分日志)          │  (3D点云)  │  (俯视图)            │  (280px高)
└─────────────────────────┴────────────┴───────────────────────┘
```

**UDP模式参数面板**（12个参数实时显示）：

| 参数 | 说明 |
|------|------|
| 纬度/经度 | GPS定位 |
| 海拔/离地高度 | 垂直位置 |
| 滚转/俯仰/偏航 | 姿态角 |
| 北向/东向速度 | 水平速度 |
| 垂直速度 | 升降速率 |
| 真空速/指示空速 | 速度信息 |

### 5.2 VideoWidget — 视频显示组件

**渲染流程**：

```
BGR NumPy数组 (OpenCV格式)
    │
    ▼ cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
RGB数组
    │
    ▼ QImage(data, width, height, bytes_per_line, QImage.Format_RGB888)
QImage
    │
    ▼ scaled(widget_size, Qt.KeepAspectRatio, Qt.SmoothTransformation)
缩放QImage
    │
    ▼ QPainter.drawPixmap() 居中绘制
显示
```

**无信号状态**：显示 "NO SIGNAL" 文字

### 5.3 Lidar2DWidget — LiDAR点云2D俯视图

**绘制流程**：

```
1. 背景网格 (10m间隔，灰色线条)
2. 十字线 (中心原点标记)
3. 无人机标记 (中心十字+圆圈)
4. 点云渲染 (颜色随距离变化)
   ├── 近距离 (< 30m): 绿色 #00ff88
   ├── 中距离 (30~70m): 黄色 #ffff00
   └── 远距离 (> 70m): 红色 #ff4444
5. 标题文字
```

**坐标系**：X轴朝上（北），Y轴朝右（东），与NED坐标系的X(北)-Y(东)对应

### 5.4 Lidar3DWidget — LiDAR点云3D立体视图

**核心特性**：

| 特性 | 实现 |
|------|------|
| 多帧累积 | 保留最近6帧点云，构建稠密3D地图 |
| 高度着色 | 蓝→绿→黄→红 热力图风格 |
| 深度着色 | 远处点变暗（alpha衰减） |
| 固定视角 | 30°俯瞰，-45°方位角 |
| 性能限制 | 最大显示30000点 |
| 渲染引擎 | matplotlib mplot3d 嵌入Qt |

**高度着色算法**：

```python
normalized_z = (z - z_min) / (z_max - z_min + 1e-6)
if normalized_z < 0.25:
    color = (0, normalized_z*4, 1)          # 蓝→青
elif normalized_z < 0.5:
    color = (0, 1, 1-(normalized_z-0.25)*4) # 青→绿
elif normalized_z < 0.75:
    color = ((normalized_z-0.5)*4, 1, 0)    # 绿→黄
else:
    color = (1, 1-(normalized_z-0.75)*4, 0) # 黄→红
```

### 5.5 SensorPanel — 动态传感器数据面板

**设计思路**：根据实际连接的传感器动态创建显示分组，无需硬编码UI布局。

**核心映射表**：

```python
SENSOR_NAME_MAP = {
    "IMU1": "IMU", "GPS": "GPS", "lidar1": "LiDAR",
    "FrontCamera": "前视相机", "DownCamera": "下视相机",
    "Chase": "追踪相机", "RadioAltimeter": "无线电高度表",
    ...
}

SENSOR_DISPLAY_ORDER = [
    "IMU1", "GPS", "lidar1", "FrontCamera", "DownCamera",
    "Chase", "RadioAltimeter", "LaserAltimeter", "UltrasonicAltimeter",
    "Barometer", "Airspeed", "Radar1", "StereoLeft"
]
```

**更新机制**：接收 `sensor_data_signal` → 查找或创建 SensorGroupBox → 更新 SensorDataLabel

### 5.6 NeonLabel & StatusIndicator — 霓虹风格组件

**NeonLabel**：自定义颜色和字号的发光标签，通过 `QGraphicsDropShadowEffect` 实现霓虹光晕效果。

**StatusIndicator**：圆形发光状态指示灯，三层绘制：

```
1. 外层径向渐变光晕 (半透明扩散，半径=指示灯1.5倍)
2. 内层径向渐变圆形 (高光→主色→暗边)
3. 右侧状态文字标签
```

---

## 第六章 配置文件体系

### 6.1 配置文件总览

| 配置文件 | 类型 | 物理引擎 | 机型 | 传感器数 | 用途 |
|----------|------|----------|------|----------|------|
| robot_quadrotor_adv.jsonc | 机器人 | fast-physics | 四旋翼X | 14 | 高级四旋翼（全传感器） |
| robot_hexarotor_adv.jsonc | 机器人 | fast-physics | 六旋翼X | 6 | 高级六旋翼 |
| robot_quadtiltrotor_adv.jsonc | 机器人 | fast-physics | VTOL | 6 | 倾斜旋翼飞行器 |
| robot_hexarotor_fastphysics.jsonc | 机器人 | fast-physics | 六旋翼Plus | 6 | 基础六旋翼 |
| robot_quadrotor_camera.jsonc | 机器人 | fast-physics | 四旋翼X | 6 | 相机四旋翼 |
| robot_computer_vision.jsonc | 机器人 | non-physics | 静态平台 | 1 | 计算机视觉模式 |
| scene_adv_drone.jsonc | 场景 | — | — | — | 高级无人机场景 |
| scene_basic_drone.jsonc | 场景 | — | — | — | 基础四旋翼场景 |
| scene_basic_hexarotor.jsonc | 场景 | — | — | — | 基础六旋翼场景 |
| scene_camera_drone.jsonc | 场景 | — | — | — | 相机无人机场景 |
| scene_computer_vision.jsonc | 场景 | — | — | — | 计算机视觉场景 |

### 6.2 机器人配置结构

```jsonc
{
  "physics-type": "fast-physics | non-physics | unreal-physics",
  "links": [        // 刚体链接数组
    {
      "name": "Frame",
      "inertial": { "mass": 1.0, "inertia": {...}, "aerodynamics": {...} },
      "collision": { "restitution": 0.1, "friction": 0.5 },
      "visual": { "geometry": { "type": "unreal_mesh", "name": "..." } }
    }
  ],
  "joints": [       // 关节数组
    { "id": "...", "type": "fixed|continuous|revolute", "parent-link": "...", "child-link": "...", "axis": "0 0 1" }
  ],
  "controller": {   // 飞行控制器
    "id": "Simple_Flight_Controller",
    "airframe-setup": "quadrotor-x | hexarotor-x | hexarotor-plus | vtol-quad-tiltrotor",
    "type": "simple-flight-api",
    "simple-flight-api-settings": { "actuator-order": [...], "parameters": {...} }
  },
  "actuators": [    // 执行器数组
    { "type": "rotor", "rotor-settings": { "turning-direction", "coeff-of-thrust", "coeff-of-torque", "max-rpm" } },
    { "type": "tilt", "tilt-settings": { "angle-min": 0, "angle-max": 1.57 } },
    { "type": "lift-drag-control-surface", ... }
  ],
  "sensors": [      // 传感器数组
    { "id": "...", "type": "camera|imu|lidar|gps|barometer|distance-sensor|radar|airspeed|magnetometer", ... }
  ]
}
```

### 6.3 场景配置结构

```jsonc
{
  "id": "SceneAdvDrone",
  "actors": [
    {
      "type": "robot",
      "name": "Drone1",
      "origin": { "xyz": "-1.0 8.0 -4.0", "rpy-deg": "0 0 -45" },
      "robot-config": "robot_quadrotor_adv.jsonc"
    }
  ],
  "clock": {
    "type": "steppable | real-time",
    "step-ns": 3000000,
    "real-time-update-rate": 3000000,
    "pause-on-start": false
  },
  "home-geo-point": { "latitude": 47.641468, "longitude": -122.140165, "altitude": 122.0 },
  "segmentation": { "initialize-ids": true, "ignore-existing": false, "use-owner-name": true },
  "scene-type": "UnrealNative"
}
```

### 6.4 三种机型配置对比

| 参数 | 四旋翼Adv | 六旋翼Adv | 倾斜旋翼VTOL |
|------|-----------|-----------|--------------|
| 机身质量 | 1.0 kg | 1.0 kg | 10.0 kg |
| 机身尺寸 | 0.18×0.11×0.04 m | 0.18×0.11×0.04 m | 3.2×3.2×2.68 m |
| 阻力系数 | 0.325 | 0.325 | 0.04 |
| 布局 | quadrotor-x | hexarotor-x | vtol-quad-tiltrotor |
| 螺旋桨数 | 4 | 6 | 4 |
| 执行器数 | 4 (rotor) | 6 (rotor) | 11 (4rotor+4tilt+2aileron+1elevator) |
| 推力系数 | 0.109919 | 0.109919 | 0.109919 |
| 最大转速 | 6396.667 RPM | 6396.667 RPM | 6396.667 RPM |
| 传感器数 | 14 | 6 | 6 |
| 前视相机 | ✓ 640×360 | ✓ 640×360 | ✓ 640×360 |
| 下视相机 | ✓ 1280×720 | ✓ 1280×720 | ✓ 1280×720 |
| LiDAR | ✓ 128ch/300m/前方30° | ✓ 128ch/300m/360° | ✓ 128ch/300m/360° |
| 双目相机 | ✓ 640×480 | ✗ | ✗ |
| 高度表×3 | ✓ | ✗ | ✗ |
| 气压+空速 | ✓ | ✗ | ✗ |
| 毫米波雷达 | ✓ 200m | ✗ | ✗ |
| GPS | ✓ 启用 | ✗ 禁用 | ✗ 禁用 |

---

## 第七章 两个版本对比

### 7.1 GUI版 vs CLI版

| 维度 | GUI版 (drone_ground_station.py) | CLI版 (advanced_drone_control.py) |
|------|--------------------------------|----------------------------------|
| 代码量 | ~1259行（主入口）+ 模块化 | ~1438行（单文件内嵌所有类） |
| 界面 | PyQt6深色霓虹科幻风格 | OpenCV窗口 + 终端输出 |
| 键盘监听 | pynput（系统级，UE窗口可用） | keyboard库（需管理员权限） |
| 传感器 | 11种回调（工厂模式可扩展） | 仅相机+LiDAR |
| UDP协议 | ModelOutputStruct结构体（闪现） | JSON格式（速度/位置控制） |
| 数据保存 | NPY+PCD+LAS+AVI+JPG+CSV | NPY+AVI+JPG+CSV |
| 点云可视化 | 2D俯视+3D立体（Qt内嵌） | Open3D独立窗口 |
| 传感器面板 | 动态SensorPanel | 无 |
| UDP参数显示 | 12参数实时面板 | 终端打印 |
| 退出重启 | 支持 | 不支持 |
| 架构 | 模块化（core/sensors/ui） | 单文件内嵌 |

### 7.2 独立工具脚本

| 脚本 | 功能 | 用途 |
|------|------|------|
| keyboard_control_hexarotor.py | 最简六旋翼键盘控制 | 参考实现/快速验证 |
| udp_auto_nav_sender.py | 航点自动飞行UDP发送器 | 模拟自动驾驶航点导航 |
| udp_test_sender.py | 预定义动作序列UDP发送器 | 验证UDP闪现控制功能 |

**udp_auto_nav_sender.py 航点列表**：

| 航点 | 位置(NED) | 动作 |
|------|-----------|------|
| 0 | (0, 0, -5) | 起点悬停 |
| 1 | (50, 0, -5) | 向北50m |
| 2 | (50, 50, -5) | 向东50m |
| 3 | (0, 50, -5) | 向南50m |
| 4 | (-25, -25, -8) | 向西南+上升 |
| 5 | (0, 0, -5) | 返回起点 |

---

## 第八章 关键技术实现

### 8.1 异步控制与GUI线程协作

**核心挑战**：PyQt6的GUI事件循环与asyncio的事件循环需要共存。

**解决方案**：将所有异步仿真操作放入 `QThread`，线程内部创建独立的 `asyncio` 事件循环：

```python
class DroneControlThread(QThread):
    def run(self):
        self._loop = asyncio.new_event_loop()
        asyncio.set_event_loop(self._loop)
        try:
            self._loop.run_until_complete(self._async_main())
        finally:
            self._loop.close()
```

**线程间通信**：通过Qt信号（Signal/Slot机制），确保跨线程安全：

```python
# 控制线程中发射信号
self.log_signal.emit("起飞成功", "INFO")

# 主窗口中接收信号
self.control_thread.log_signal.connect(self.on_log_message)
```

### 8.2 系统级键盘监听

**核心挑战**：当UE5仿真窗口获得焦点时，Python进程无法接收键盘事件。

**解决方案**：使用 `pynput` 库的 `Listener` 实现系统级键盘监听，无论哪个窗口在前台都能响应：

```python
from pynput.keyboard import Listener, Key

def on_press(key):
    if key == Key.up:
        self.control_thread.update_keyboard(vx=self.speed, vy=0, vz=0, yaw=0)

def on_release(key):
    if key == Key.up:
        self.control_thread.update_keyboard(vx=0, vy=0, vz=0, yaw=0)

listener = Listener(on_press=on_press, on_release=on_release)
listener.start()
```

### 8.3 UDP闪现控制

**核心挑战**：外部自动驾驶系统发送的是经纬高坐标和欧拉角，需要转换为NED坐标和四元数才能设置位姿。

**转换流程**：

```
UDP 224字节结构体
    │
    ▼ _parse_struct()
28个double字段字典
    │
    ▼ 经纬高 → NED
home_geo_point作为参考点
    │
    │  Δnorth = (lat - home_lat) × 111319.488
    │  Δeast = (lon - home_lon) × 111319.488 × cos(home_lat)
    │  Δdown = -(alt - home_alt)
    │
    ▼ 欧拉角 → 四元数
    │  q = Rz(ψ) × Ry(θ) × Rx(φ)
    │
    ▼ drone.set_pose(Pose(position, orientation))
闪现到位姿
```

### 8.4 UDP超时自动悬停保护

**设计思路**：当UDP数据源中断时，无人机应自动悬停而非继续执行最后的指令。

```python
# 在控制循环中
if control_mode == "udp":
    udp_data = self.udp_manager.receive_command()
    if udp_data:
        self._last_udp_time = time.time()
        self._process_udp(udp_data)
    elif time.time() - self._last_udp_time > UDP_TIMEOUT:
        # 超时，自动悬停
        await drone.hover_async()
```

### 8.5 相机图像类型体系

| image-type | 名称 | 像素格式 | 用途 |
|------------|------|----------|------|
| 0 | Scene | uint8 BGR | 可见光RGB场景 |
| 1 | DepthPlanar | float32 | 深度平面图（正交投影） |
| 2 | Depth | float32 | 深度透视图 |
| 3 | Segmentation | uint8 BGR | 语义分割（颜色编码类别） |
| 4 | DepthVis | uint8 BGR | 深度可视化（近=深色，远=浅色） |
| 5 | DisparityNormalized | float32 | 归一化视差图 |
| 6 | SurfaceNormals | uint8 BGR | 表面法线图 |

---

## 第九章 Project AirSim 上游框架

### 9.1 Project AirSim 概述

Project AirSim 是由原 Microsoft AirSim 团队成员组成的 IAMAI Simulations 公司继续开发的仿真平台，基于 Unreal Engine 5，支持自定义物理引擎、控制器、执行器和传感器。采用 MIT 开源许可。

### 9.2 Python 客户端库核心API

#### 9.2.1 ProjectAirSimClient — 连接管理

```python
client = ProjectAirSimClient()
client.connect()                    # 连接仿真后端
client.subscribe(topic, callback)   # 订阅传感器话题
client.publish(topic, message)      # 发布消息
client.request(request_data)        # 同步请求
client.disconnect()                 # 断开连接
```

#### 9.2.2 World — 场景管理

```python
world = World(client, "scene_adv_drone.jsonc", delay_after_load_sec=2)
world.pause()                       # 暂停仿真
world.resume()                      # 恢复仿真
world.list_actors()                 # 列出场景角色
```

#### 9.2.3 Drone — 无人机控制

```python
drone = Drone(client, world, "Drone1")
drone.enable_api_control()          # 启用API控制
drone.arm()                         # 解锁电机
await drone.takeoff_async()         # 起飞
await drone.land_async()            # 着陆
await drone.move_by_velocity_async(v_north, v_east, v_down, duration)
await drone.move_by_velocity_body_frame_async(vx, vy, vz, duration)
await drone.move_to_position_async(x, y, z, speed)
drone.set_pose(pose)               # 直接设置位姿
drone.set_vtol_mode(mode)          # VTOL模式切换
drone.hover_async()                # 悬停
```

### 9.3 坐标系约定

| 坐标系 | X轴 | Y轴 | Z轴 | 用途 |
|--------|-----|-----|-----|------|
| NED (世界) | 北 | 东 | 下 | 位置、速度、GPS |
| 机体坐标系 | 前 | 右 | 下 | 键盘控制 |
| FRD (机体) | 前 | 右 | 下 | IMU数据 |

**单位约定**：SI单位（米、弧度、m/s、rad/s），除非另有说明。

---

## 第十章 开发与扩展指南

### 10.1 新增无人机型号

1. 在 `sim_config/` 中创建机器人配置文件（如 `robot_octocoptor_adv.jsonc`）
2. 在 `core/constants.py` 的 `DRONE_MODELS` 字典中添加条目
3. 在 `ui/` 的机型选择UI中添加选项

### 10.2 新增传感器类型

1. 在 `sensors/base.py` 的 `SensorType` 枚举中添加新类型
2. 创建新的回调类（继承 `SensorCallback`），实现 `__call__` 和 `get_display_fields`
3. 在 `sensors/factory.py` 的 `_creators` 映射表中注册
4. 在 `sensors/manager.py` 的 `TYPE_MAP` / `ID_TYPE_MAP` 中添加映射
5. 在 `ui/sensor_panel.py` 的 `SENSOR_NAME_MAP` 和 `SENSOR_FIELDS_MAP` 中添加显示配置
6. 在机器人配置文件的 `sensors` 数组中添加传感器条目

### 10.3 新增数据保存格式

在 `core/data_recorder.py` 中添加对应的保存方法，在 `sensors/lidar.py` 的 `save_snapshot()` 中调用。

### 10.4 修改控制参数

| 参数 | 位置 | 默认值 | 说明 |
|------|------|--------|------|
| 飞行速度 | `constants.DEFAULT_SPEED` | 5.0 m/s | 键盘控制默认速度 |
| 偏航速率 | `constants.DEFAULT_YAW_SPEED` | 20.0 °/s | 键盘控制默认偏航速率 |
| 速度范围 | `constants.MIN_SPEED/MAX_SPEED` | 1~20 m/s | 速度调节范围 |
| 控制周期 | `constants.CONTROL_DURATION` | 0.1 s | 控制指令持续时间 |
| UDP端口 | `constants.UDP_DEFAULT_PORT` | 15610 | UDP监听端口 |
| 视频分辨率 | `constants.CAMERA_WIDTH/HEIGHT` | 1280×720 | 录像分辨率 |
| 录像帧率 | `constants.VIDEO_FPS` | 30 fps | 录像帧率 |

### 10.5 性能优化建议

1. **降低传感器频率**：增大 `capture-interval` 减少CPU和带宽占用
2. **关闭未使用的图像类型**：将 `capture-enabled` 设为 `false`
3. **启用压缩传输**：将 `compress` 设为 `true`（增加CPU解码开销但降低带宽）
4. **调整LiDAR参数**：降低 `points-per-second` 或 `number-of-channels`
5. **录像降帧**：在 `write_video_frame` 中添加帧计数器，隔帧写入
6. **3D点云降采样**：Lidar3DWidget已限制最大30000点

---

## 第十一章 故障排除

| 问题 | 原因 | 解决方案 |
|------|------|---------|
| Actor Drone1 not found | 场景配置中无人机名称不匹配 | 确认scene配置的actor name与代码中引用一致 |
| ServiceMethod not supported | 使用了非fast-physics模型 | 确保physics-type为"fast-physics" |
| 相机无图像 | 传感器未启用或订阅路径错误 | 检查enabled=true和sensors路径 |
| LiDAR无数据 | LiDAR传感器配置错误 | 检查lidar1的enabled和参数 |
| 键盘无响应(GUI版) | pynput监听器未启动 | 检查listener.start()是否执行 |
| 键盘无响应(CLI版) | keyboard库权限不足 | 以管理员权限运行 |
| UDP收不到数据 | 防火墙阻止或端口冲突 | 检查防火墙设置和端口占用 |
| 录像文件为空 | VideoWriter未正确初始化 | 检查fourcc编码器和分辨率匹配 |
| LiDAR 3D显示空白 | matplotlib未安装或版本不兼容 | pip install matplotlib>=3.5 |
| 双目视差图全黑 | 左右相机未同步或基线距离为0 | 检查StereoCamera的左右相机配置 |
| VTOL切换失败 | 前飞速度不足 | 保持一定前飞速度后切换固定翼模式 |
| UDP闪现位置偏移 | home-geo-point与实际不符 | 校准UDP_HOME_GEO_POINT常量 |
| 传感器面板无数据 | SensorManager未正确识别传感器类型 | 检查TYPE_MAP和ID_TYPE_MAP映射 |

---

## 附录A 文件清单与代码统计

| 文件 | 行数 | 职责 |
|------|------|------|
| drone_ground_station.py | ~1259 | GUI主入口，GroundStationWindow |
| advanced_drone_control.py | ~1438 | CLI主入口，内嵌所有类 |
| keyboard_control_hexarotor.py | ~223 | 最简六旋翼键盘控制示例 |
| core/constants.py | ~93 | 全局常量定义 |
| core/config_manager.py | ~95 | 配置动态生成 |
| core/control_thread.py | ~729 | 异步控制线程（核心调度器） |
| core/data_recorder.py | ~349 | 数据持久化管理 |
| core/udp_manager.py | ~178 | UDP通信管理 |
| sensors/base.py | ~168 | SensorType/SensorCallback/SensorData |
| sensors/factory.py | ~148 | SensorFactory工厂 |
| sensors/manager.py | ~500 | SensorManager统一调度 |
| sensors/camera.py | ~195 | Camera/DepthCamera回调 |
| sensors/stereo_camera.py | ~262 | StereoCamera回调(SGBM) |
| sensors/lidar.py | ~120 | Lidar回调(点云+保存) |
| sensors/imu.py | ~147 | IMU回调(四元数→欧拉角) |
| sensors/gps.py | ~124 | GPS回调 |
| sensors/altimeter.py | ~188 | 三种高度表回调 |
| sensors/atmosphere.py | ~167 | 大气机组回调(气压+空速) |
| sensors/distance_sensor.py | ~78 | 通用距离传感器回调 |
| sensors/radar.py | ~130 | 毫米波雷达回调 |
| ui/widgets.py | ~96 | NeonLabel/StatusIndicator |
| ui/video_widget.py | ~79 | VideoWidget |
| ui/lidar_widgets.py | ~356 | Lidar2D/Lidar3D组件 |
| ui/sensor_panel.py | ~376 | SensorPanel动态面板 |
| udp_auto_nav_sender.py | ~289 | UDP航点自动导航发送器 |
| udp_test_sender.py | ~257 | UDP测试动作发送器 |
| **总计** | **~7254** | — |

## 附录B 依赖清单

| 包名 | 版本要求 | 用途 |
|------|---------|------|
| projectairsim | ≥0.1.1 | AirSim Python客户端库 |
| PyQt6 | ≥6.0 | GUI框架 |
| pynput | ≥1.7 | 系统级键盘监听 |
| opencv-python | ≥4.2 | 图像处理 |
| numpy | ≥1.21 | 数值计算 |
| matplotlib | ≥3.5 | 3D点云可视化 |
| open3d | ≥0.16 (可选) | 点云文件处理 |
| pynng | ≥0.5 | NNG通信 |
| msgpack | ≥1.0 | 消息序列化 |
| commentjson | ≥0.9 | JSONC配置解析 |

## 附录C 设计模式索引

| 设计模式 | 应用位置 | 说明 |
|----------|---------|------|
| 工厂模式 | SensorFactory | 根据SensorType创建对应回调实例 |
| 观察者模式 | Qt Signal/Slot | 控制线程与UI的解耦通信 |
| 回调模式 | SensorCallback | 传感器数据的统一处理接口 |
| 模板方法模式 | SensorCallback基类 | 定义get_display_fields等通用方法 |
| 策略模式 | 键盘/UDP双模式控制 | 运行时切换控制策略 |
| 单例模式 | constants.py | 全局唯一常量定义 |
| 组合模式 | AtmosphereCallback | 组合气压计+空速传感器 |
| 代理模式 | DroneControlThread | QThread作为异步操作的代理 |
