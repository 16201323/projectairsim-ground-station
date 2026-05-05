# ProjectAirSim 无人机地面站

基于 [ProjectAirSim](https://github.com/iamaisim/ProjectAirSim) 的无人机地面控制系统，提供传感器数据实时采集、3D可视化、飞行控制等功能。

## 功能特性

- **多传感器数据采集**：IMU、GPS、高度表（无线电/激光/超声波）、大气机、激光雷达、毫米波雷达
- **多相机实时画面**：双目相机、前视相机、下视相机、第三人称追踪相机
- **3D点云可视化**：LiDAR点云实时渲染，支持多帧累积和高度着色
- **键盘飞行控制**：WASD/方向键控制无人机移动，支持起飞/降落/悬停
- **UDP外部控制**：支持外部程序通过UDP协议发送控制指令
- **数据记录**：传感器数据和飞行日志自动保存
- **景德镇坐标定位**：默认出生点设置为中国景德镇市

## 系统要求

- Ubuntu 22.04+ / Windows 10+
- Python 3.10+
- ProjectAirSim v0.1.1+
- Unreal Engine 城市环境（如 DynamicCity）

## 快速开始

### 1. 安装依赖

```bash
# 创建虚拟环境
python3 -m venv venv
source venv/bin/activate  # Linux
# venv\Scripts\activate   # Windows

# 安装依赖包
pip install -r requirements.txt
```

### 2. 启动仿真环境

先启动 ProjectAirSim 城市环境（如 DynamicCity），等待场景加载完成。

### 3. 运行地面站

```bash
# Linux
bash run.sh

# 或手动运行
source venv/bin/activate
python main.py
```

## 操作说明

| 按键 | 功能 |
|------|------|
| W/S | 前进/后退 |
| A/D | 左移/右移 |
| 方向键↑↓ | 上升/下降 |
| 方向键←→ | 左转/右转 |
| Space | 紧急悬停 |
| T | 起飞 |
| L | 降落 |

## 项目结构

```
├── main.py                  # 主程序入口
├── core/
│   ├── constants.py         # 全局常量（坐标、端口等）
│   ├── control_thread.py    # 飞行控制线程
│   ├── config_manager.py    # 仿真配置管理
│   ├── data_recorder.py     # 数据记录器
│   └── udp_manager.py       # UDP通信管理
├── sensors/
│   ├── base.py              # 传感器基类
│   ├── factory.py           # 传感器工厂
│   ├── manager.py           # 传感器管理器
│   ├── imu.py               # IMU惯性测量单元
│   ├── gps.py               # GPS全球定位
│   ├── altimeter.py         # 高度表（无线电/激光/超声波）
│   ├── atmosphere.py        # 大气机
│   ├── lidar.py             # 激光雷达
│   ├── radar.py             # 毫米波雷达
│   ├── camera.py            # 相机
│   └── stereo_camera.py     # 双目相机
├── ui/
│   ├── sensor_panel.py      # 传感器参数面板
│   ├── video_widget.py      # 视频显示控件
│   ├── lidar_widgets.py     # LiDAR点云可视化
│   └── widgets.py           # 通用UI控件
├── sim_config/
│   ├── robot_quadrotor_adv.jsonc  # 四旋翼无人机配置
│   ├── scene_adv_drone.jsonc      # 仿真场景配置
│   └── ...                        # 其他配置文件
├── requirements.txt         # Python依赖
└── run.sh                   # Linux启动脚本
```

## 传感器配置

| 传感器 | ID | 说明 |
|--------|-----|------|
| IMU | IMU1 | 惯性测量单元，输出滚转/俯仰/偏航角 |
| GPS | GPS | 全球定位，输出经纬度和海拔 |
| 无线电高度表 | RadioAltimeter | 测量对地高度，量程0~50m |
| 激光高度表 | LaserAltimeter | 激光测距，量程0~100m |
| 超声波高度表 | UltrasonicAltimeter | 超声波测距，量程0~10m |
| 大气机 | Atmosphere | 输出气压/温度/风速/空速 |
| 激光雷达 | lidar1 | 3D点云扫描，50000点/秒 |
| 毫米波雷达 | Radar1 | 目标检测，方位角±60°，仰角-30°~+10° |
| 双目相机 | StereoCamera | 左右目立体视觉 |
| 前视相机 | FrontCamera | 机头前方视角 |
| 下视相机 | DownCamera | 机腹下方视角 |
| 追踪相机 | Chase | 第三人称追踪视角（1280×720） |

## 技术要点

- **单位转换**：ProjectAirSim C++端距离传感器输出单位为厘米（UE标准），Python端已做米转换补偿
- **雷达数据**：C++端方位角/仰角单位为度（非弧度），Python端直接使用无需二次转换
- **NED坐标系**：所有3D坐标使用北-东-地（NED）坐标系
- **UI节流**：传感器回调使用节流机制，避免高频更新导致界面卡顿

## 许可证

本项目仅供学习和研究使用。
