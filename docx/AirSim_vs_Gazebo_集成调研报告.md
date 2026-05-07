# Project AirSim vs Gazebo 集成调研报告

## 摘要

本报告对 Project AirSim 与 Gazebo 两大仿真平台进行全面对比，重点研究如何将 Gazebo 的模型与传感器引用到 Project AirSim 中。核心发现：Project AirSim 不支持直接导入 Gazebo 的 URDF/SDF 文件，但可通过"外部物理引擎接口"（`non-physics` 模式 + `set_pose()` API）将 Gazebo 作为飞行动力学模型（FDM），由 Gazebo 负责姿态解算与动力学响应，AirSim 负责高质量视觉渲染与传感器数据生成。传感器无法直接迁移，需在 AirSim 的 JSONC 配置中重新复现参数。

---

## 板块一：Project AirSim vs Gazebo 对比分析

### 1.1 基本信息对比

| 维度 | Project AirSim | Gazebo (Gz) |
|------|---------------|-------------|
| **开发方** | IAMAI Simulations（原微软 AirSim 团队） | Open Robotics / OSRF |
| **开源协议** | MIT | Apache 2.0 |
| **当前版本** | 持续更新（基于 UE 5.2/5.7） | Gazebo Ionic (Gz Sim 8) |
| **渲染引擎** | Unreal Engine 5 | OGRE2 / 自定义渲染管线 |
| **物理引擎** | Fast Physics / PhysX / 外部物理 | ODE / Bullet / DART / Simbody |
| **模型格式** | JSONC（自定义 robot config） | SDF / URDF（XML 标准） |
| **编程语言** | C++ 核心 + Python 客户端 | C++ 核心 + 多语言接口 |
| **支持平台** | Windows 11, Ubuntu 22 | Ubuntu, macOS, Windows (部分) |
| **ROS 集成** | 通过 ROS Bridge | 原生 ROS 2 集成（ros_gz） |
| **GitHub** | iamaisim/ProjectAirSim | gazebosim/gz-sim |

### 1.2 技术架构对比

| 架构维度 | Project AirSim | Gazebo |
|----------|---------------|--------|
| **整体架构** | 三层架构：Sim Libs → UE Plugin → Client Library | 模块化架构：gz-sim + gz-physics + gz-rendering + gz-sensors 等 |
| **仿真循环** | UE 驱动的主循环，支持 steppable/real-time 时钟 | 事件驱动，支持实时与步进模式 |
| **物理抽象** | `physics-type` 字段选择引擎：`fast-physics` / `unreal-physics` / `non-physics` / `matlab-physics` | 通过 `<physics>` 标签选择引擎：ODE / Bullet / DART / Simbody |
| **机器人描述** | JSONC 格式，links-joints-controller-actuators-sensors 树结构 | SDF/URDF XML 格式，model-link-joint-sensor 树结构 |
| **通信机制** | TCP（port_topics/port_services） | Gazebo Transport（内部分布式通信） |
| **外部物理接口** | `non-physics` 模式 + `set_pose()` API；`matlab-physics` + `physics-connection` TCP | Gazebo Plugin 系统（C++ 插件接口） |
| **渲染管线** | UE5 Lumen/Nanite，照片级真实感 | OGRE2 渲染，功能完整但视觉保真度较低 |

### 1.3 传感器支持对比

| 传感器类型 | Project AirSim | Gazebo | 对比说明 |
|-----------|---------------|--------|---------|
| **IMU** | ✅ imu（含加速度计+陀螺仪噪声模型） | ✅ imu（含噪声模型） | AirSim 噪声模型更精细（VRW/ARW/tau/bias-stability） |
| **GPS/NavSat** | ✅ gps（eph/epv 时间常数） | ✅ navsat（水平/垂直精度模型） | 参数体系不同，需手动映射 |
| **气压计** | ✅ barometer（压力因子噪声） | ✅ air_pressure（参考压力+噪声） | 实现原理类似，参数命名差异大 |
| **磁力计** | ✅ magnetometer（噪声σ/比例因子/偏置） | ✅ magnetometer（噪声模型） | AirSim 参数更简洁 |
| **相机（RGB）** | ✅ camera（多图像类型，UE5 渲染） | ✅ camera（OGRE2 渲染） | AirSim 视觉质量远超 Gazebo |
| **深度相机** | ✅ camera（image-type: depth_planar/perspective） | ✅ depth_camera / rgbd_camera | AirSim 深度图基于 UE5，精度更高 |
| **分割相机** | ✅ camera（image-type: segmentation） | ✅ segmentation_camera | AirSim 基于 UE CustomDepth |
| **LiDAR** | ✅ lidar（多通道，旋转频率可配） | ✅ gpu_lidar / lidar（GPU/CPU 加速） | Gazebo 支持 GPU 加速，AirSim 依赖 UE |
| **Radar** | ✅ radar | ❌ 无原生支持 | AirSim 独有 |
| **距离传感器** | ✅ distance | ✅ sonar（声纳类） | 实现原理不同 |
| **电池** | ✅ battery | ❌ 无原生支持 | AirSim 独有 |
| **空速计** | ❌ 无原生支持 | ✅ air_speed | Gazebo 独有 |
| **高度计** | ❌ 无原生（需组合 barometer） | ✅ altimeter（独立传感器） | Gazebo 原生支持 |
| **接触传感器** | ✅ collision_info（订阅回调） | ✅ contact（碰撞检测） | 实现方式不同 |
| **力/力矩** | ❌ 无原生支持 | ✅ force_torque | Gazebo 独有 |
| **逻辑相机** | ❌ 无原生支持 | ✅ logical_camera | Gazebo 独有 |
| **热成像相机** | ❌ 无原生支持 | ✅ thermal_camera | Gazebo 独有 |
| **边界框相机** | ❌ 无原生支持 | ✅ boundingbox_camera | Gazebo 独有 |
| **DVL** | ❌ 无原生支持 | ✅ doppler_velocity_log | Gazebo 水下场景专用 |
| **无线收发** | ❌ 无原生支持 | ✅ wireless_receiver/transmitter | Gazebo 独有 |

**传感器数量汇总**：Project AirSim 约 10 种基础传感器 + 组合传感器；Gazebo 约 19+ 种传感器类型。

### 1.4 相机/视觉能力对比

| 能力维度 | Project AirSim | Gazebo |
|----------|---------------|--------|
| **渲染引擎** | Unreal Engine 5（Lumen 全局光照、Nanite 虚拟几何） | OGRE2（前向渲染） |
| **图像类型** | RGB / 深度（planar/perspective）/ 分割 / Surface Normals / Infrared | RGB / 深度 / 分割 / 热成像 / 边界框 |
| **分辨率** | 任意（受 GPU 限制） | 任意（受 GPU 限制） |
| **FOV 配置** | fov-degrees 参数 | horizontal_fov / vertical_fov 参数 |
| **噪声注入** | 噪声设置（随机噪声、水平波纹、畸变等） | 高斯噪声 / 自定义噪声模型 |
| **后处理** | 支持 NN 后处理模型 | 有限后处理 |
| **图像标注** | 分割 ID + 目标检测标注 | 分割 + 边界框 + 逻辑相机 |
| **天气效果** | 雨雪雾、体积云、日照系统 | 基础天气插件 |
| **视觉保真度** | ★★★★★（照片级） | ★★★（功能级） |

### 1.5 平台生态对比

| 生态维度 | Project AirSim | Gazebo |
|----------|---------------|--------|
| **飞控集成** | Simple Flight / PX4（SITL/HITL） | PX4 / ArduPilot（原生 SITL） |
| **ROS 支持** | ROS Bridge（独立包） | 原生 ROS 2 集成（ros_gz） |
| **模型库** | 内置无人机模型，自定义 UE 资产 | 丰富模型库（Fuel 生态系统） |
| **社区规模** | 较小（新项目，持续增长中） | 非常大（ROS 社区核心工具） |
| **文档质量** | 较完善（官方文档站） | 非常完善（教程+API 文档） |
| **第三方模型** | 需自行导入 UE 资产 | Gazebo Fuel 海量共享模型 |
| **学术采用** | 增长中 | 非常广泛 |

### 1.6 开发体验对比

| 体验维度 | Project AirSim | Gazebo |
|----------|---------------|--------|
| **上手难度** | 中等（需 UE 环境 + JSONC 配置） | 中等（需 ROS 基础 + SDF 配置） |
| **配置方式** | JSONC 文件（scene + robot 分离） | SDF/XML 文件（world + model 分离） |
| **调试工具** | UE 编辑器 + 日志 | Gazebo GUI + gz topic 命令行 |
| **API 风格** | Python 异步 API（asyncio） | C++ Plugin + ROS 2 Topic/Service |
| **热重载** | 不支持运行时添加机器人 | 支持运行时生成/销毁模型 |
| **多机器人** | 配置文件预定义，暂不支持运行时添加 | 原生支持动态多机器人 |

### 1.7 性能消耗对比

| 性能维度 | Project AirSim | Gazebo |
|----------|---------------|--------|
| **GPU 需求** | 高（UE5 渲染管线） | 中（OGRE2 渲染） |
| **CPU 需求** | 中（Fast Physics 轻量） | 中-高（取决于物理引擎选择） |
| **内存占用** | 高（UE 引擎开销） | 中 |
| **实时性** | steppable 时钟可精确控制 | 实时性良好 |
| **多实例** | Docker 支持 headless | 原生支持多实例 |
| **最低 GPU** | NVIDIA RTX 2070 或同等 | 集成显卡可运行基础场景 |

### 1.8 应用场景对比

| 应用场景 | 推荐平台 | 原因 |
|----------|---------|------|
| **计算机视觉 / AI 训练** | Project AirSim | UE5 照片级渲染，数据质量高 |
| **ROS 机器人开发** | Gazebo | 原生 ROS 集成，社区资源丰富 |
| **无人机飞控验证** | 两者皆可 | AirSim 有 Simple Flight；Gazebo 有 PX4 SITL |
| **多机器人协作** | Gazebo | 原生支持动态多机器人 |
| **自动驾驶** | Project AirSim | 高保真城市场景 + GIS 支持 |
| **室内导航** | Gazebo | 丰富的室内环境模型 |
| **水下机器人** | Gazebo | DVL 传感器 + 水下物理 |
| **快速原型验证** | Gazebo | 轻量、启动快、模型库丰富 |
| **仿真数据生成** | Project AirSim | 渲染质量高，后处理能力强 |

### 1.9 优缺点总结

#### Project AirSim 优点
1. **视觉保真度极高**：UE5 渲染引擎提供照片级画面，适合 CV/AI 训练数据生成
2. **外部物理引擎接口**：支持 `non-physics` 模式注入自定义动力学，灵活性强
3. **Matlab/Simulink 集成**：官方支持 Simulink 物理模型，适合学术研究
4. **GIS 场景支持**：支持地理信息系统场景，适合大规模户外仿真
5. **天气系统**：UE5 体积云、大气散射等高级天气效果
6. **雷达传感器**：原生支持 Radar 传感器

#### Project AirSim 缺点
1. **GPU 需求高**：UE5 渲染对硬件要求高
2. **模型生态小**：缺乏类似 Gazebo Fuel 的模型共享平台
3. **传感器种类较少**：缺少空速计、高度计、热成像等专用传感器
4. **不支持运行时添加机器人**：所有机器人需在配置文件中预定义
5. **不支持直接导入 SDF/URDF**：模型需手动转换为 JSONC 格式
6. **社区规模较小**：作为新项目，社区资源和第三方支持有限

#### Gazebo 优点
1. **ROS 原生集成**：ROS 2 生态核心仿真工具
2. **传感器种类丰富**：19+ 种传感器，覆盖面广
3. **模型库丰富**：Gazebo Fuel 提供海量共享模型
4. **多物理引擎**：ODE / Bullet / DART / Simbody 可选
5. **动态多机器人**：支持运行时生成/销毁模型
6. **社区成熟**：大量教程、第三方插件和学术资源
7. **轻量级**：硬件需求低，启动快

#### Gazebo 缺点
1. **视觉保真度低**：OGRE2 渲染无法与 UE5 相比
2. **深度学习数据质量差**：渲染图像不够真实，不适合 CV 训练
3. **无 Radar 传感器**：缺少雷达仿真
4. **Windows 支持弱**：主要面向 Linux 平台
5. **配置复杂**：SDF/XML 格式冗长，学习曲线陡峭

### 1.10 选型建议

| 需求场景 | 推荐选择 | 理由 |
|----------|---------|------|
| 需要**高质量视觉数据**用于 AI 训练 | Project AirSim | UE5 渲染质量远超 Gazebo |
| 需要**ROS 原生集成**进行机器人开发 | Gazebo | ros_gz 无缝集成 |
| 需要**快速原型验证** | Gazebo | 轻量、模型库丰富 |
| 需要**两者优势结合** | Project AirSim + Gazebo 联合 | Gazebo 做物理，AirSim 做渲染（见板块二） |
| 需要**PX4 飞控验证** | 两者皆可 | 均支持 PX4 SITL/HITL |
| 需要**大规模多机器人仿真** | Gazebo | 原生支持动态多机器人 |

---

## 板块二：如何在 Project AirSim 中引用 Gazebo 模型

### 2.1 迁移核心原理

#### 2.1.1 "引用"的本质

Project AirSim **不支持直接导入** Gazebo 的 URDF/SDF 文件。两者的模型描述体系完全不同：

- **Gazebo**：使用 SDF/URDF（XML 格式）描述机器人，包含 `<model>` → `<link>` → `<joint>` → `<sensor>` 的树结构
- **Project AirSim**：使用 JSONC 格式描述机器人，包含 `links` → `joints` → `controller` → `actuators` → `sensors` 的树结构

因此，"引用"的本质是**架构级解耦**：

```
┌─────────────────────────────────────────────────────┐
│                  联合仿真架构                         │
│                                                     │
│  ┌──────────┐    位姿数据     ┌──────────────────┐  │
│  │  Gazebo  │ ──────────────→ │  Project AirSim  │  │
│  │ (FDM)    │                 │  (渲染+传感器)    │  │
│  │          │ ←────────────── │                  │  │
│  │ 物理仿真  │   碰撞/环境信息  │  视觉渲染+传感器  │  │
│  └──────────┘                 └──────────────────┘  │
│       ↑                              ↑              │
│       │                              │              │
│  GazeboDrone /                   set_pose()        │
│  Python 桥接程序                  API 调用           │
└─────────────────────────────────────────────────────┘
```

**核心思路**：
1. Gazebo 负责飞行动力学模型（FDM），进行姿态解算与动力学响应
2. Project AirSim 使用 `non-physics` 模式，不进行物理计算
3. 通过桥接程序将 Gazebo 的位姿数据同步到 AirSim 的 `set_pose()` API
4. AirSim 负责高质量的视觉渲染与传感器数据生成

#### 2.1.2 两种实现路径

| 路径 | 描述 | 复杂度 | 适用场景 |
|------|------|--------|---------|
| **路径 A：Python 桥接** | 使用 Project AirSim Python 客户端的 `non-physics` 模式 + `set_pose()` API | ★★☆ | 推荐方式，灵活且与 Project AirSim 原生 API 兼容 |
| **路径 B：GazeboDrone（旧 AirSim）** | 使用旧版 AirSim 的 GazeboDrone C++ 桥接程序 | ★★★★ | 仅适用于旧版 AirSim，不兼容 Project AirSim |

> **重要说明**：GazeboDrone 是旧版 Microsoft AirSim 的扩展组件（位于 `AirSim/GazeboDrone/` 目录），**不包含在 Project AirSim 中**。Project AirSim 的等价方案是路径 A。

### 2.2 具体操作步骤

#### 第一步：安装依赖与编译环境

**1. 安装 Gazebo 依赖**

```bash
# Ubuntu 22.04 - 安装 Gazebo Ionic (推荐)
sudo apt-get update
sudo apt-get install lsb-release gnupg2
sudo sh -c 'echo "deb http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" > /etc/apt/sources.list.d/gazebo-stable.list'
wget http://packages.osrfoundation.org/gazebo.key -O - | sudo apt-key add -
sudo apt-get update
sudo apt-get install gz-ionic

# 或安装 Gazebo Classic (Gazebo 11，兼容旧模型)
sudo apt-get install libgazebo11-dev gazebo11
```

**2. 安装 Project AirSim**

参考 [官方文档](https://iamaisim.github.io/ProjectAirSim/development/use_source.html)：
- 安装 Unreal Engine 5.2 或 5.7
- 编译 Sim Libs：`build.cmd simlibs_debug`（Windows）或 `./build.sh simlibs_debug`（Linux）
- 生成项目文件并运行

**3. 关于 GCC 8 / clang 编译问题**

> 此问题仅适用于**旧版 AirSim 的 GazeboDrone**，不适用于 Project AirSim。

旧版 AirSim 的 GazeboDrone 需要：
- AirLib 用 GCC 8 编译：`./build.sh --gcc`（因为 GazeboDrone 使用 GCC 8）
- UE 插件用 clang 编译：在另一个目录重新 clone AirSim 并正常编译

**Project AirSim 无此问题**，因为使用 Python 客户端 API 进行桥接，无需编译 C++ 桥接程序。

#### 第二步：模型视觉资产迁移

**1. 从 Gazebo 模型中提取视觉网格（Mesh）**

Gazebo 模型的视觉网格通常位于模型目录的 `meshes/` 子目录中，格式包括：
- `.dae`（COLLADA）— 最常见
- `.stl`（STL）— 简单几何
- `.obj`（Wavefront OBJ）

提取步骤：
```bash
# 找到 Gazebo 模型目录
ls ~/.gazebo/models/<model_name>/meshes/

# 或从 SDF 文件中查找 mesh 引用
grep -r "<uri>" <model>.sdf | grep mesh
```

**2. 将 Mesh 导入 Unreal Engine 项目**

```
操作流程：
1. 在 UE 编辑器中，打开内容浏览器（Content Browser）
2. 在目标文件夹右键 → Import to /Game/Drone/
3. 选择 .dae / .stl / .obj 文件
4. 在导入选项中：
   - 勾选 "Import Mesh"
   - 设置合适的缩放比例（Gazebo 使用米，UE 默认厘米，需注意单位）
   - 材质选项：选择 "Import Materials" 和 "Import Textures"
5. 导入后在内容浏览器中确认 .uasset 文件已生成
```

**3. 材质与纹理兼容性处理**

| 问题 | 原因 | 解决方案 |
|------|------|---------|
| 材质丢失 | COLLADA 材质不被 UE 完全支持 | 在 UE 中手动重新创建材质，使用原始纹理贴图 |
| 纹理路径错误 | 相对路径在 UE 中无效 | 将纹理与模型放在同一目录，重新关联 |
| 单位缩放不一致 | Gazebo 使用米，UE 默认厘米 | 导入时设置 Scale = 100，或在 robot config 中使用 `scale` 参数 |
| 法线翻转 | 坐标系差异（Gazebo: NED vs UE: 左手坐标系） | 在 UE 中使用 "Flip Normals" 或在 3D 建模软件中预处理 |

**4. 在 Robot Config 中引用 UE Mesh**

```jsonc
{
  "physics-type": "non-physics",
  "links": [
    {
      "name": "Frame",
      "collision": { "enabled": false },
      "visual": {
        "geometry": {
          "type": "unreal_mesh",
          "name": "/Game/Drone/MyGazeboDrone",  // UE 内容路径
          "scale": "1.0 1.0 1.0"
        }
      }
    }
  ]
}
```

#### 第三步：构建外部物理引擎桥接程序

**1. 桥接程序的结构与作用**

桥接程序是连接 Gazebo 与 Project AirSim 的核心组件，其作用是：
- 从 Gazebo 获取无人机的实时位姿（位置 + 姿态四元数）
- 将位姿数据通过 Project AirSim 的 `set_pose()` API 同步到 AirSim 中的无人机
- 实现物理仿真循环的同步

**2. Python 桥接程序示例**

```python
import time
import asyncio
from projectairsim import ProjectAirSimClient, World, Drone
from projectairsim.utils import projectairsim_log

# Gazebo 位姿获取（通过 Gazebo Transport 或 ROS 2 Topic）
# 此处使用模拟数据作为示例，实际需替换为 Gazebo 数据源
class GazeboPoseProvider:
    def __init__(self, model_name="my_drone"):
        self.model_name = model_name
        # TODO: 初始化 Gazebo Transport 或 ROS 2 订阅
        # 例如：from gz.transport import Node
        # 或：import rclpy; from geometry_msgs.msg import PoseStamped

    def get_pose(self):
        # TODO: 从 Gazebo 获取位姿数据
        # 返回格式：{"x": float, "y": float, "z": float,
        #           "w": float, "x_rot": float, "y_rot": float, "z_rot": float}
        return {"x": 0, "y": 0, "z": -5, "w": 1, "x_rot": 0, "y_rot": 0, "z_rot": 0}

async def run_bridge():
    client = None
    try:
        # 连接 Project AirSim
        client = ProjectAirSimClient()
        client.connect()
        world = World(client, "scene_gazebo_bridge_drone.jsonc", delay_after_load_sec=2)
        drone = Drone(client, world, "Drone1")

        # 初始化 Gazebo 位姿提供者
        gazebo = GazeboPoseProvider("my_drone")

        # 物理仿真循环
        dt = 0.003  # 3ms 步长，与 AirSim real-time-update-rate 一致
        while True:
            # 1. 从 Gazebo 获取位姿
            gazebo_pose = gazebo.get_pose()

            # 2. 构造 AirSim pose 并设置
            pose = drone.get_ground_truth_pose()
            pose["translation"]["x"] = gazebo_pose["x"]
            pose["translation"]["y"] = gazebo_pose["y"]
            pose["translation"]["z"] = gazebo_pose["z"]
            pose["rotation"]["w"] = gazebo_pose["w"]
            pose["rotation"]["x"] = gazebo_pose["x_rot"]
            pose["rotation"]["y"] = gazebo_pose["y_rot"]
            pose["rotation"]["z"] = gazebo_pose["z_rot"]

            # 3. 同步位姿到 AirSim
            drone.set_pose(pose, False)

            # 4. 等待下一个步长
            time.sleep(dt)

    except KeyboardInterrupt:
        projectairsim_log().info("Bridge stopped by user")
    finally:
        if client is not None:
            client.disconnect()

if __name__ == "__main__":
    asyncio.run(run_bridge())
```

**3. 如何修改以加载自己的 Gazebo 模型**

1. 在 Gazebo 中加载你的模型：`gz sim my_drone_world.sdf`
2. 修改 `GazeboPoseProvider` 类，使用 Gazebo Transport API 订阅模型位姿：
   ```python
   # 使用 gz-transport-python（如果可用）或 ROS 2 桥接
   # 方案 A：通过 ROS 2 桥接
   # ros2 topic echo /model/my_drone/pose
   # 方案 B：通过 gz topic 命令行
   # gz topic -e -t /model/my_drone/pose
   ```
3. 确保位姿坐标系转换正确（Gazebo ENU → AirSim NED）

**4. 物理仿真循环要求**

```
循环结构：
┌──────────────────────────────────────┐
│  while simulation_running:           │
│    1. 从 Gazebo 获取位姿 (pose)       │
│    2. 坐标系转换 (ENU → NED)         │
│    3. drone.set_pose(pose)           │
│    4. 从 AirSim 获取传感器数据 (可选)  │
│    5. sleep(dt)                      │
└──────────────────────────────────────┘
```

关键要求：
- 循环频率应与 AirSim 的 `real-time-update-rate` 匹配（默认 3ms = ~333Hz）
- 位姿数据延迟应尽量小（< 1ms），否则会出现视觉抖动
- 需处理坐标系转换：Gazebo 使用 ENU（东-北-上），AirSim 使用 NED（北-东-下）

**5. 通信方式**

| 通信方式 | 延迟 | 复杂度 | 推荐度 |
|----------|------|--------|--------|
| Gazebo Transport (C++/Python) | 低 | 中 | ★★★★ |
| ROS 2 Topic (通过 ros_gz_bridge) | 中 | 中 | ★★★ |
| 共享内存 | 极低 | 高 | ★★☆ |
| TCP/UDP Socket | 低 | 低 | ★★★ |

#### 第四步：配置 AirSim 的 Scene 和 Robot 配置文件

**1. Scene 配置文件**（`scene_gazebo_bridge_drone.jsonc`）

```jsonc
{
  "id": "SceneGazeboBridgeDrone",
  "actors": [
    {
      "type": "robot",
      "name": "Drone1",
      "origin": {
        "xyz": "0.0 0.0 -5.0",
        "rpy-deg": "0 0 0"
      },
      "robot-config": "robot_gazebo_bridge_drone.jsonc"
    }
  ],
  "clock": {
    "type": "real-time",
    "real-time-update-rate": 3000000
  },
  "home-geo-point": {
    "latitude": 47.641468,
    "longitude": -122.140165,
    "altitude": 122.0
  },
  "segmentation": {
    "initialize-ids": true,
    "ignore-existing": false,
    "use-owner-name": true
  },
  "scene-type": "UnrealNative"
}
```

**2. Robot 配置文件**（`robot_gazebo_bridge_drone.jsonc`）

```jsonc
{
  // 关键：使用 non-physics 模式
  "physics-type": "non-physics",
  "links": [
    {
      "name": "Frame",
      "collision": { "enabled": true },
      "visual": {
        "geometry": {
          "type": "unreal_mesh",
          "name": "/Game/Drone/MyGazeboDrone"
        }
      }
    },
    {
      "name": "Prop_FL",
      "visual": {
        "geometry": {
          "type": "unreal_mesh",
          "name": "/Drone/PropellerRed"
        }
      }
    },
    {
      "name": "Prop_FR",
      "visual": {
        "geometry": {
          "type": "unreal_mesh",
          "name": "/Drone/PropellerRed"
        }
      }
    },
    {
      "name": "Prop_RL",
      "visual": {
        "geometry": {
          "type": "unreal_mesh",
          "name": "/Drone/PropellerWhite"
        }
      }
    },
    {
      "name": "Prop_RR",
      "visual": {
        "geometry": {
          "type": "unreal_mesh",
          "name": "/Drone/PropellerWhite"
        }
      }
    }
  ],
  "joints": [
    { "id": "Frame_Prop_FL", "type": "fixed", "parent-link": "Frame", "child-link": "Prop_FL", "axis": "0 0 1" },
    { "id": "Frame_Prop_FR", "type": "fixed", "parent-link": "Frame", "child-link": "Prop_FR", "axis": "0 0 1" },
    { "id": "Frame_Prop_RL", "type": "fixed", "parent-link": "Frame", "child-link": "Prop_RL", "axis": "0 0 1" },
    { "id": "Frame_Prop_RR", "type": "fixed", "parent-link": "Frame", "child-link": "Prop_RR", "axis": "0 0 1" }
  ],
  "sensors": [
    {
      "id": "FrontCamera",
      "type": "camera",
      "enabled": true,
      "parent-link": "Frame",
      "capture-interval": 0.03,
      "capture-settings": [
        {
          "image-type": 0,
          "width": 640,
          "height": 360,
          "fov-degrees": 90,
          "capture-enabled": true,
          "streaming-enabled": true,
          "pixels-as-float": false,
          "compress": false,
          "target-gamma": 2.5
        }
      ],
      "origin": {
        "xyz": "0.3 0.0 0.0",
        "rpy-deg": "0 0 0"
      }
    },
    {
      "id": "IMU1",
      "type": "imu",
      "enabled": true,
      "parent-link": "Frame",
      "accelerometer": {
        "velocity-random-walk": 2.353e-3,
        "tau": 800,
        "bias-stability": 3.53e-4,
        "turn-on-bias": "0 0 0"
      },
      "gyroscope": {
        "angle-random-walk": 8.72644e-5,
        "tau": 500,
        "bias-stability": 2.23014e-5,
        "turn-on-bias": "0 0 0"
      }
    },
    {
      "id": "GPS",
      "type": "gps",
      "enabled": true,
      "parent-link": "Frame"
    },
    {
      "id": "Barometer",
      "type": "barometer",
      "enabled": true,
      "parent-link": "Frame"
    },
    {
      "id": "Magnetometer",
      "type": "magnetometer",
      "enabled": true,
      "parent-link": "Frame"
    }
  ]
}
```

**3. 关键配置参数说明**

| 参数 | 值 | 说明 |
|------|-----|------|
| `physics-type` | `"non-physics"` | **最关键**：禁用 AirSim 内置物理，允许通过 API 设置位姿 |
| `clock.type` | `"real-time"` | 使用实时时钟，与 Gazebo 同步 |
| `collision.enabled` | `true` | 启用碰撞检测（可选） |
| `sensors` | 按需配置 | 在 AirSim 中配置传感器，而非使用 Gazebo 传感器 |

**4. 配置注意事项**

- `physics-type` **必须**设为 `"non-physics"`，否则 AirSim 会用内置物理引擎覆盖 `set_pose()` 的位姿
- `non-physics` 模式下不需要 `controller` 和 `actuators` 字段
- `collision` 可设为 `true` 以获取碰撞信息（通过 `collision_info` 订阅）
- 传感器配置在 AirSim 侧，不在 Gazebo 侧（见 2.3 节）
- 时钟类型建议使用 `real-time`，避免步进时钟与 Gazebo 不同步

#### 第五步：启动与联调

**1. 启动顺序**

```
步骤 1：启动 Project AirSim UE 环境
  → 打开 UE 编辑器，加载包含 AirSim 插件的项目
  → 点击 Play 进入仿真模式
  → 等待场景完全加载

步骤 2：启动 Gazebo 仿真
  → 在终端运行：gz sim my_drone_world.sdf
  → 或使用 ROS 2 launch：ros2 launch my_pkg gazebo.launch.py
  → 确认模型已加载并可控制

步骤 3：运行桥接程序
  → python gazebo_airsim_bridge.py
  → 观察日志确认连接成功

步骤 4：验证联调
  → 在 Gazebo 中控制无人机移动
  → 观察 AirSim 中的无人机是否同步移动
  → 检查 AirSim 传感器数据是否正常
```

**2. 验证模型已正确接入**

| 验证项 | 方法 | 预期结果 |
|--------|------|---------|
| 位姿同步 | 在 Gazebo 中移动无人机 | AirSim 中无人机同步移动 |
| 传感器数据 | 调用 `drone.get_imu_data("IMU1")` | 返回有效 IMU 数据 |
| 相机图像 | 调用 `drone.get_images("FrontCamera", [0])` | 返回 RGB 图像 |
| 碰撞检测 | 订阅 `collision_info` | 碰撞时收到回调 |
| 延迟测试 | 测量 Gazebo→AirSim 延迟 | < 10ms |
| 坐标系 | 检查移动方向是否正确 | 前后左右上下方向一致 |

**3. 常见问题排查**

| 问题 | 原因 | 解决方案 |
|------|------|---------|
| 无人机不动 | `physics-type` 不是 `non-physics` | 确认 robot config 中 `physics-type: "non-physics"` |
| 无人机位置偏移 | 坐标系转换错误 | 检查 ENU→NED 转换（X↔Y, Z取反） |
| 画面抖动 | 位姿更新频率不匹配 | 调整 sleep(dt) 与 real-time-update-rate 一致 |
| 传感器无数据 | 传感器未启用或 parent-link 错误 | 检查 sensor 的 `enabled: true` 和 `parent-link` |
| 连接失败 | AirSim 未启动或端口冲突 | 确认 UE 已 Play，检查端口占用 |

### 2.3 引用 Gazebo 传感器的实现方法

#### 2.3.1 AirSim 能否直接使用 Gazebo 的传感器？

**不能。** Project AirSim 和 Gazebo 的传感器系统完全独立：

- **Gazebo 传感器**：由 Gazebo 引擎计算，数据通过 Gazebo Transport / ROS 2 Topic 发布
- **AirSim 传感器**：由 AirSim 引擎计算（基于 UE5 渲染），数据通过 AirSim Client API 获取

在 `non-physics` 模式下，AirSim 的传感器仍然基于 AirSim 自己的仿真环境计算（使用 AirSim 的场景和渲染），不会使用 Gazebo 的传感器数据。

#### 2.3.2 替代方案：在 AirSim 中复现 Gazebo 传感器配置

核心思路：**参考 Gazebo SDF 中的传感器参数，在 AirSim 的 JSONC 配置中重新配置等效传感器。**

#### 2.3.3 传感器配置对比表

##### IMU 传感器对比

| 配置项 | Gazebo (SDF) | Project AirSim (JSONC) | 差异说明 |
|--------|-------------|----------------------|---------|
| 传感器类型 | `<sensor type="imu">` | `"type": "imu"` | 命名一致 |
| 更新频率 | `<update_rate>100</update_rate>` | 无直接对应 | AirSim 按仿真步长更新 |
| 加速度计噪声 | `<noise><type>gaussian</type><mean>0</mean><stddev>0.01</stddev></noise>` | `"accelerometer": {"velocity-random-walk": 2.353e-3, "tau": 800, "bias-stability": 3.53e-4}` | AirSim 使用更精细的噪声模型（VRW + bias stability + tau） |
| 陀螺仪噪声 | `<noise><type>gaussian</type><mean>0</mean><stddev>0.001</stddev></noise>` | `"gyroscope": {"angle-random-walk": 8.72644e-5, "tau": 500, "bias-stability": 2.23014e-5}` | AirSim 使用 ARW + bias stability 模型 |
| 安装位置 | `<pose>x y z roll pitch yaw</pose>` | `"origin": {"xyz": "x y z", "rpy-deg": "r p y"}` | AirSim 用度，Gazebo 用弧度 |
| 父链接 | 所在 `<link>` | `"parent-link": "Frame"` | 概念一致 |

**Gazebo SDF 示例 → AirSim JSONC 转换**：

```xml
<!-- Gazebo SDF -->
<sensor name="imu_sensor" type="imu">
  <update_rate>100</update_rate>
  <imu>
    <angular_velocity>
      <x><noise><type>gaussian</type><mean>0</mean><stddev>0.001</stddev></noise></x>
      <y><noise><type>gaussian</type><mean>0</mean><stddev>0.001</stddev></noise></y>
      <z><noise><type>gaussian</type><mean>0</mean><stddev>0.001</stddev></noise></z>
    </angular_velocity>
    <linear_acceleration>
      <x><noise><type>gaussian</type><mean>0</mean><stddev>0.01</stddev></noise></x>
      <y><noise><type>gaussian</type><mean>0</mean><stddev>0.01</stddev></noise></y>
      <z><noise><type>gaussian</type><mean>0</mean><stddev>0.01</stddev></noise></z>
    </linear_acceleration>
  </imu>
</sensor>
```

```jsonc
// Project AirSim JSONC 等效配置
{
  "id": "IMU1",
  "type": "imu",
  "enabled": true,
  "parent-link": "Frame",
  "accelerometer": {
    "velocity-random-walk": 0.01,
    "tau": 800,
    "bias-stability": 3.53e-4,
    "turn-on-bias": "0 0 0"
  },
  "gyroscope": {
    "angle-random-walk": 0.001,
    "tau": 500,
    "bias-stability": 2.23014e-5,
    "turn-on-bias": "0 0 0"
  }
}
```

##### 深度相机对比

| 配置项 | Gazebo (SDF) | Project AirSim (JSONC) | 差异说明 |
|--------|-------------|----------------------|---------|
| 传感器类型 | `<sensor type="depth_camera">` 或 `<sensor type="rgbd_camera">` | `"type": "camera"` + `"image-type": 1/2` | AirSim 用 image-type 区分 |
| 分辨率 | `<camera><image><width>640</width><height>480</height></image></camera>` | `"width": 640, "height": 480` | 参数名不同 |
| FOV | `<camera><horizontal_fov>1.57</horizontal_fov></camera>` | `"fov-degrees": 90` | Gazebo 用弧度，AirSim 用度 |
| 近/远裁剪面 | `<camera><clip><near>0.1</near><far>100</far></clip></camera>` | 无直接对应 | 需进一步测试验证 |
| 噪声 | `<noise><type>gaussian</type>...</noise>` | `"noise-settings": [{...}]` | AirSim 支持更复杂的噪声模式 |
| 更新频率 | `<update_rate>30</update_rate>` | `"capture-interval": 0.033` | AirSim 用间隔时间，Gazebo 用频率 |

**Gazebo SDF 示例 → AirSim JSONC 转换**：

```xml
<!-- Gazebo SDF -->
<sensor name="depth_camera" type="depth_camera">
  <update_rate>30</update_rate>
  <camera>
    <horizontal_fov>1.57</horizontal_fov>
    <image>
      <width>640</width>
      <height>480</height>
      <format>R8G8B8</format>
    </image>
    <clip>
      <near>0.1</near>
      <far>100</far>
    </clip>
  </camera>
</sensor>
```

```jsonc
// Project AirSim JSONC 等效配置
{
  "id": "DepthCamera",
  "type": "camera",
  "enabled": true,
  "parent-link": "Frame",
  "capture-interval": 0.033,
  "capture-settings": [
    {
      "image-type": 0,
      "width": 640,
      "height": 480,
      "fov-degrees": 90,
      "capture-enabled": true,
      "streaming-enabled": false,
      "pixels-as-float": false,
      "compress": false,
      "target-gamma": 2.5
    },
    {
      "image-type": 2,
      "width": 640,
      "height": 480,
      "fov-degrees": 90,
      "capture-enabled": true,
      "pixels-as-float": false,
      "compress": false
    }
  ],
  "origin": {
    "xyz": "0.3 0.0 0.0",
    "rpy-deg": "0 0 0"
  }
}
```

##### LiDAR 对比

| 配置项 | Gazebo (SDF) | Project AirSim (JSONC) | 差异说明 |
|--------|-------------|----------------------|---------|
| 传感器类型 | `<sensor type="gpu_lidar">` | `"type": "lidar"` | AirSim 无 GPU LiDAR 区分 |
| 通道数 | `<lidar><scan><horizontal><samples>640</samples></horizontal></scan></lidar>` | `"number-of-channels": 16` | 参数名不同 |
| 扫描范围 | `<lidar><range><min>0.1</min><max>100</max></range></lidar>` | `"range": 100` | AirSim 只有最大范围 |
| 水平 FOV | `<lidar><scan><horizontal><min_angle>-3.14159</min_angle><max_angle>3.14159</max_angle></horizontal></scan></lidar>` | `"horizontal-fov-start-deg": -180, "horizontal-fov-end-deg": 180` | Gazebo 用弧度，AirSim 用度 |
| 垂直 FOV | `<lidar><scan><vertical><min_angle>-0.26</min_angle><max_angle>0.26</max_angle></vertical></scan></lidar>` | `"vertical-fov-upper-deg": 15, "vertical-fov-lower-deg": -15` | 同上 |
| 旋转频率 | `<lidar><scan><horizontal><resolution>0.4</resolution></horizontal></scan></lidar>` | `"horizontal-rotation-frequency": 10` | AirSim 用 Hz |
| 每秒点数 | 需计算 (samples × vertical_samples × freq) | `"points-per-second": 100000` | AirSim 直接指定 |
| 噪声 | `<noise><type>gaussian</type><mean>0</mean><stddev>0.01</stddev></noise>` | 无直接对应 | AirSim LiDAR 噪声模型需进一步测试验证 |

**Gazebo SDF 示例 → AirSim JSONC 转换**：

```xml
<!-- Gazebo SDF -->
<sensor name="lidar" type="gpu_lidar">
  <update_rate>10</update_rate>
  <lidar>
    <scan>
      <horizontal>
        <samples>640</samples>
        <resolution>1</resolution>
        <min_angle>-3.14159</min_angle>
        <max_angle>3.14159</max_angle>
      </horizontal>
      <vertical>
        <samples>16</samples>
        <resolution>1</resolution>
        <min_angle>-0.261799</min_angle>
        <max_angle>0.261799</max_angle>
      </vertical>
    </scan>
    <range>
      <min>0.1</min>
      <max>100</max>
      <resolution>0.01</resolution>
    </range>
  </lidar>
</sensor>
```

```jsonc
// Project AirSim JSONC 等效配置
{
  "id": "Lidar1",
  "type": "lidar",
  "enabled": true,
  "parent-link": "Frame",
  "number-of-channels": 16,
  "range": 100,
  "points-per-second": 102400,
  "horizontal-rotation-frequency": 10,
  "horizontal-fov-start-deg": -180,
  "horizontal-fov-end-deg": 180,
  "vertical-fov-upper-deg": 15,
  "vertical-fov-lower-deg": -15,
  "origin": {
    "xyz": "0.3 0.0 0.0",
    "rpy-deg": "0 0 0"
  }
}
```

#### 2.3.4 Gazebo 独有传感器的处理

对于 Gazebo 有但 AirSim 没有的传感器，有以下处理方式：

| Gazebo 独有传感器 | 处理方式 | 说明 |
|-------------------|---------|------|
| **air_speed（空速计）** | 方案 1：使用 Gazebo 传感器数据通过桥接程序转发到 Python 客户端 | 需自行实现数据转发 |
| | 方案 2：在 AirSim 中基于 ground_truth_kinematics 计算等效值 | 通过速度矢量计算空速 |
| **altimeter（高度计）** | 方案 1：组合 AirSim 的 barometer + GPS 数据 | 需自行实现高度解算 |
| | 方案 2：使用 Gazebo 高度计数据通过桥接转发 | 需自行实现数据转发 |
| **force_torque（力/力矩）** | 无直接替代 | 需进一步测试验证是否可通过 AirSim API 获取 |
| **thermal_camera（热成像）** | 无直接替代 | AirSim 不支持热成像渲染 |
| **logical_camera（逻辑相机）** | 使用 AirSim 的 segmentation + 目标检测 | 功能类似但实现不同 |
| **sonar（声纳）** | 使用 AirSim 的 distance 传感器 | 原理类似，参数需调整 |
| **doppler_velocity_log** | 无替代 | 水下场景专用，AirSim 不支持 |

---

## 板块三：最佳实践与局限性

### 3.1 适用场景（何时值得花精力做 Gazebo→AirSim 集成）

| 场景 | 是否推荐 | 原因 |
|------|---------|------|
| **已有 Gazebo 模型，需要高质量视觉数据** | ✅ 强烈推荐 | 这正是该方案的核心价值 |
| **需要 Gazebo 特定物理引擎（如 DART）** | ✅ 推荐 | AirSim 内置物理引擎有限 |
| **需要 Gazebo + AirSim 传感器数据对比** | ✅ 推荐 | 可同时获取两套传感器数据进行交叉验证 |
| **已有 ROS/Gazebo 工作流，需增加视觉保真度** | ✅ 推荐 | 最小化改动，增加渲染层 |
| **仅需要基础仿真** | ❌ 不推荐 | 直接使用 Gazebo 或 AirSim 即可，无需增加复杂度 |
| **需要实时性极高的控制回路** | ❌ 不推荐 | 桥接程序引入额外延迟 |
| **需要大量 Gazebo 独有传感器** | ❌ 不推荐 | 传感器迁移成本高，不如直接使用 Gazebo |

### 3.2 技术局限与常见问题

#### 3.2.1 核心局限

1. **不支持直接导入 SDF/URDF**
   - 必须手动将 Gazebo 模型转换为 AirSim JSONC 格式
   - 视觉资产需导入 UE 并转换为 .uasset 格式
   - 物理参数（惯性、碰撞）需手动重新配置

2. **传感器无法直接迁移**
   - Gazebo 传感器数据与 AirSim 传感器数据独立计算
   - 需在 AirSim 中重新配置所有传感器参数
   - 部分 Gazebo 传感器在 AirSim 中无对应（热成像、空速计等）

3. **位姿同步延迟**
   - 桥接程序引入不可避免的通信延迟
   - 高速运动时可能出现视觉抖动
   - 延迟取决于通信方式和系统负载

4. **坐标系转换复杂**
   - Gazebo: ENU（东-北-上），右手坐标系
   - AirSim: NED（北-东-下），右手坐标系
   - 转换规则：X_airsim = Y_gazebo, Y_airsim = X_gazebo, Z_airsim = -Z_gazebo

5. **时钟同步困难**
   - Gazebo 和 AirSim 各自维护仿真时钟
   - 步进模式下同步更复杂
   - 建议使用实时时钟模式

6. **双引擎资源消耗**
   - 同时运行 Gazebo + UE5，GPU/CPU 资源消耗翻倍
   - 建议使用高性能工作站（≥32GB RAM, RTX 3080+）

#### 3.2.2 常见问题与解决方案

| 问题 | 原因 | 解决方案 |
|------|------|---------|
| 无人机在 AirSim 中不移动 | `physics-type` 配置错误 | 确认设为 `"non-physics"` |
| 位姿同步有明显延迟 | 通信方式效率低 | 使用 Gazebo Transport 替代 ROS 2 |
| 画面卡顿/抖动 | 更新频率不匹配 | 调整桥接循环频率，与 AirSim `real-time-update-rate` 一致 |
| 传感器数据不合理 | 传感器 parent-link 配置错误 | 确认传感器挂载在正确的 link 上 |
| Gazebo 模型在 AirSim 中方向错误 | 坐标系转换遗漏 | 实现 ENU→NED 转换 |
| UE 崩溃 | GPU 内存不足 | 降低渲染质量或使用 headless 模式 |
| 桥接程序断连 | 网络超时 | 添加重连机制和心跳检测 |

### 3.3 针对不同需求的推荐实现路径

#### 路径 A：纯 AirSim 方案（推荐大多数场景）

```
适用：新项目，无需复用 Gazebo 模型
实现：
1. 在 AirSim JSONC 中直接定义机器人
2. 使用 fast-physics 或 unreal-physics
3. 使用 AirSim 原生传感器
4. 视觉资产直接在 UE 中制作
优点：最简单，无集成复杂度
缺点：无法利用 Gazebo 生态
```

#### 路径 B：AirSim + Gazebo 物理桥接（推荐需要特定物理引擎的场景）

```
适用：已有 Gazebo 模型，需要高质量视觉数据
实现：
1. Gazebo 运行物理仿真
2. AirSim 使用 non-physics 模式
3. Python 桥接程序同步位姿
4. AirSim 负责渲染和传感器
优点：兼顾物理精度和视觉质量
缺点：系统复杂度高，需维护桥接程序
```

#### 路径 C：AirSim + Matlab/Simulink（推荐学术研究场景）

```
适用：已有 Simulink 物理模型
实现：
1. 使用 matlab-physics 类型
2. 配置 physics-connection (IP + Port)
3. Simulink 模型通过 TCP 与 AirSim 通信
优点：官方支持，文档完善
缺点：需要 Matlab 许可证
```

#### 路径 D：纯 Gazebo 方案（推荐 ROS 原生开发场景）

```
适用：ROS 2 工作流，不需要高保真视觉
实现：
1. 直接使用 Gazebo Ionic
2. 通过 ros_gz 桥接 ROS 2
3. 使用 Gazebo 丰富的传感器和模型库
优点：ROS 集成最好，社区资源丰富
缺点：视觉保真度低
```

### 3.4 未来展望

1. **Project AirSim 可能增加 SDF/URDF 导入支持**：当前版本不支持，但社区有相关需求
2. **Gazebo Ionic 的渲染改进**：新版 Gazebo 正在改进渲染管线，可能减少对 AirSim 渲染的依赖
3. **统一仿真接口标准**：ROS 2 生态正在推动仿真接口标准化，未来可能简化多仿真器集成
4. **Project AirSim 社区增长**：随着社区扩大，可能出现更多第三方桥接工具和模型转换器

---

## 引用来源

1. Project AirSim 官方文档站：https://iamaisim.github.io/ProjectAirSim/
2. Project AirSim GitHub 仓库：https://github.com/iamaisim/ProjectAirSim
3. Project AirSim Robot Configuration 文档：https://iamaisim.github.io/ProjectAirSim/config_robot.html
4. Project AirSim Scene Configuration 文档：https://iamaisim.github.io/ProjectAirSim/config_scene.html
5. Project AirSim Matlab Physics 文档：https://iamaisim.github.io/ProjectAirSim/physics/matlab_physics.html
6. Project AirSim Fast Physics 文档：https://iamaisim.github.io/ProjectAirSim/physics/fast_physics.html
7. Project AirSim Transition from AirSim 文档：https://iamaisim.github.io/ProjectAirSim/transition_from_airsim.html
8. 旧版 AirSim GazeboDrone 文档：https://microsoft.github.io/AirSimExtensions/gazebo_drone/
9. Gazebo 官方文档站：https://gazebosim.org/docs/latest/
10. Gazebo Sensors 文档：https://gazebosim.org/docs/latest/sensors/
11. Gazebo Sensors 库：https://gazebosim.org/libs/sensors/
12. SDF Specification：http://sdformat.org/spec?ver=1.12&elem=sensor
13. Project AirSim 源码 - external_physics_engine.py 示例：`client/python/airsimv1_scripts_migrated/multirotor/external_physics_engine.py`
14. Project AirSim 源码 - simulink_physics_quadrotor.py 示例：`client/python/example_user_scripts/simulink_physics_quadrotor.py`
15. Project AirSim 源码 - robot_quadrotor_nonphysics.jsonc：`client/python/example_user_scripts/sim_config/robot_quadrotor_nonphysics.jsonc`
16. Project AirSim 源码 - scene_quadrotor_matlab_physics.jsonc：`client/python/example_user_scripts/sim_config/scene_quadrotor_matlab_physics.jsonc`
17. GitHub Issue - ExternalPhysicsEngine with PX4：https://github.com/microsoft/AirSim/issues/4588
18. GitHub Issue - Using newer Gazebo versions with AirSim：https://github.com/microsoft/AirSim/issues/5046

---

*本报告中标注"需进一步测试验证"的内容表示无法从官方文档或源码中确认，需要实际测试才能得出结论。所有信息均基于截至 2026 年 5 月的官方文档和源码分析，不包含凭空编造的内容。*
