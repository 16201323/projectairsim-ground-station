# ProjectAirSim 无人机地面站

基于 [ProjectAirSim](https://github.com/iamaisim/ProjectAirSim) 的无人机地面控制系统，提供多传感器实时监控、3D LiDAR点云可视化、键盘/UDP飞行控制等功能。

## 功能特性

### 传感器数据采集

- **IMU**：滚转/俯仰/偏航角、三轴加速度
- **GPS**：经纬度、海拔、地速、定位状态
- **高度表**：无线电(0.5~500m)、激光(0.2~300m)、超声波(0.02~10m)
- **大气机**：气压高度、指示空速、气压、QNH、差压
- **激光雷达**：128线3D点云，150m测距，300万点/秒
- **毫米波雷达**：UCM241模拟，方位角±60°，仰角-30°~+10°

### 多相机实时画面

- 追踪相机（第三人称视角，1280×720）
- 双目相机（左/右立体视觉）
- 下视相机（RGB + 深度图 + 语义分割）

### LiDAR 3D 点云可视化

- **plasma 彩色色图着色**：深蓝(地面) → 紫色 → 橙红 → 亮黄(高处)
- **30帧累积 + 200k降采样**：保留建筑整体形态
- **鼠标交互**：左键旋转 / 滚轮缩放 / 中键平移
- **视角预设**：透视 / 前视 / 俯视 一键切换
- **坐标轴 + 参考网格**：5m XYZ坐标轴 + 100×100网格
- **跨平台**：基于 pyqtgraph OpenGL，Win11 + Ubuntu 24.04 均可运行

### 飞行控制

- **键盘控制**：WASD/方向键控制无人机移动，支持起飞/降落/悬停
- **UDP外部控制**：支持外部程序通过UDP组播发送控制指令
- **VTOL模式切换**：倾斜旋翼无人机支持多旋翼/固定翼模式切换

## 系统要求

| 项目 | 要求 |
|------|------|
| 操作系统 | Windows 10+ / Ubuntu 22.04+ |
| Python | 3.10+ |
| ProjectAirSim | v0.1.1+ |
| UE环境 | DynamicCity 或其他城市环境 |

## 快速开始

### 1. 安装依赖

```bash
python3 -m venv venv
source venv/bin/activate  # Linux
# venv\Scripts\activate   # Windows

pip install -r requirements.txt
```

### 2. 启动仿真环境

启动 ProjectAirSim 城市环境（如 DynamicCity），等待场景加载完成。

### 3. 运行地面站

```bash
# Linux
bash run.sh

# 或手动运行
source venv/bin/activate
python main.py
```

## 操作说明

### 键盘控制

| 按键 | 功能 |
|------|------|
| W / S | 前进 / 后退 |
| A / D | 左移 / 右移 |
| ↑ / ↓ | 上升 / 下降 |
| ← / → | 左转 / 右转 |
| Space | 紧急悬停 |
| T / L | 起飞 / 降落 |
| Q | 退出程序 |
| +/- | 调整飞行速度 |

### 相机切换

| 按键 | 功能 |
|------|------|
| 1 | 追踪相机（第三人称） |
| 2 | 双目左相机 |
| 3 | 双目右相机 |
| 4 | 下视深度相机 |
| C | 拍照（当前相机） |
| F | 双目左相机拍照 |

## 项目结构

```
├── main.py                  # 主程序入口（PyQt6地面站窗口）
├── core/
│   ├── constants.py         # 全局常量（颜色、尺寸、无人机型号、视角预设）
│   ├── control_thread.py    # 飞行控制线程（传感器订阅+飞行指令）
│   ├── config_manager.py    # 仿真配置管理（动态生成场景JSONC）
│   ├── data_recorder.py     # 数据记录器（传感器快照+飞行日志）
│   └── udp_manager.py       # UDP通信管理（组播接收+指令解析）
├── sensors/
│   ├── base.py              # 传感器回调基类（节流+统计）
│   ├── factory.py           # 传感器工厂（按类型创建回调实例）
│   ├── manager.py           # 传感器管理器（统一订阅+数据分发）
│   ├── imu.py               # IMU惯性测量单元
│   ├── gps.py               # GPS全球定位
│   ├── altimeter.py         # 高度表（无线电/激光/超声波）
│   ├── atmosphere.py        # 大气机
│   ├── lidar.py             # 激光雷达（降采样+统计+快照保存）
│   ├── radar.py             # 毫米波雷达
│   ├── camera.py            # 相机（帧缓存+截图）
│   └── stereo_camera.py     # 双目相机（左右帧同步）
├── ui/
│   ├── sensor_panel.py      # 传感器仪表盘面板（双列卡片布局）
│   ├── video_widget.py      # 视频显示控件（QPainter高性能渲染）
│   ├── lidar_widgets.py     # LiDAR 3D点云可视化（pyqtgraph OpenGL）
│   └── widgets.py           # 通用UI控件（霓虹标签/状态指示灯）
├── sim_config/
│   ├── scene_adv_drone.jsonc       # 场景模板（动态替换机器人配置）
│   ├── robot_quadrotor_adv.jsonc   # 四旋翼配置（14传感器）
│   ├── robot_hexarotor_adv.jsonc   # 六旋翼配置
│   ├── robot_quadtiltrotor_adv.jsonc # VTOL倾斜旋翼配置
│   └── ...                         # 其他配置文件
├── requirements.txt         # Python依赖
└── run.sh                   # Linux启动脚本
```

## 传感器配置

| 传感器 | ID | 说明 |
|--------|-----|------|
| IMU | IMU1 | 惯性测量单元，输出滚转/俯仰/偏航角 |
| GPS | GPS | 全球定位，输出经纬度和海拔 |
| 无线电高度表 | RadioAltimeter | 测量对地高度，量程0.5~500m |
| 激光高度表 | LaserAltimeter | 激光测距，量程0.2~300m |
| 超声波高度表 | UltrasonicAltimeter | 超声波测距，量程0.02~10m |
| 大气机 | Barometer | 输出气压高度、指示空速、气压、QNH、差压 |
| 激光雷达 | lidar1 | 128线3D点云，150m测距，300万点/秒，水平FOV 120°，垂直FOV 25° |
| 毫米波雷达 | Radar1 | 目标检测，方位角±60°，仰角-30°~+10° |
| 双目相机 | StereoLeft/StereoRight | 左目120°FOV / 右目30°FOV |
| 下视相机 | DownCamera | RGB + 平面深度 + 语义分割 |
| 追踪相机 | Chase | 第三人称追踪视角（1280×720） |

## 技术要点

- **NED坐标系**：ProjectAirSim 使用北-东-地坐标系（Z向下为正），LiDAR点云渲染时自动转换为Z-up
- **单位转换**：C++端距离传感器输出单位为厘米，Python端已做米转换补偿
- **UI节流**：传感器回调使用0.2秒节流间隔，避免高频更新导致界面卡顿
- **拉取模式**：相机帧和LiDAR数据由UI定时器主动拉取（15fps/5Hz），避免跨线程信号开销
- **LiDAR渲染**：pyqtgraph GLViewWidget 原生嵌入Qt，跨平台兼容，鼠标交互原生支持
- **plasma着色**：Z轴归一化强度映射为matplotlib plasma色图，与LidarDisplay.COLOR_INTENSITY一致
- **Ctrl+C处理**：注册SIGINT默认处理，捕获KeyboardInterrupt优雅退出

## 依赖说明

| 依赖 | 版本 | 用途 |
|------|------|------|
| PyQt6 | >=6.4.0 | GUI框架 |
| pyqtgraph | >=0.13.0 | 3D点云可视化（OpenGL加速） |
| PyOpenGL | >=3.1.0 | pyqtgraph OpenGL后端 |
| opencv-python | >=4.7.0 | 图像处理 |
| numpy | >=1.24.0 | 数值计算 |
| pynput | >=1.7.6 | 系统级键盘监听 |

## 许可证

本项目仅供学习和研究使用。
