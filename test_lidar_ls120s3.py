"""
LS120S3 LiDAR 独立测试脚本

模拟镭神 LS120S3 型号激光雷达，使用 Open3D 实时显示 3D 点云。

LS120S3 核心参数：
  - 水平视场角：120° (-60° ~ +60°)
  - 垂直视场角：25° (+12.5° ~ -12.5°)
  - 角度分辨率：0.05° × 0.04°
  - 测距范围：0.1 ~ 200m (10%反射率)
  - 测距精度：±2mm (150m量程)
  - 点云密度：320万点/秒
  - 帧率：10Hz
  - 线数：120线
  - 有效点频：300万点/秒

使用方法：
  python test_lidar_ls120s3.py

操作说明：
  - Open3D 窗口支持鼠标旋转/缩放/平移
  - 控制台实时显示点云统计信息
  - 按 Ctrl+C 终止脚本
"""

import asyncio
import signal
import sys
import time
import threading
import numpy as np

from projectairsim import ProjectAirSimClient, Drone, World
from projectairsim.utils import projectairsim_log
from projectairsim.lidar_utils import LidarDisplay

SCENE_CONFIG = "scene_lidar_ls120s3.jsonc"
DRONE_NAME = "Drone1"

ACCUM_FRAMES = 100
_accumulated_points = []
_accum_lock = threading.Lock()
_frame_count = 0
_start_time = None
_stats_lock = threading.Lock()


def lidar_callback(_, lidar_data):
    global _frame_count, _start_time, _accumulated_points

    if lidar_data is None:
        return

    pc = lidar_data.get("point_cloud", [])
    if not pc:
        return

    pts = np.array(pc, dtype=np.float32).reshape(-1, 3)

    with _accum_lock:
        _accumulated_points.append(pts)
        while len(_accumulated_points) > ACCUM_FRAMES:
            _accumulated_points.pop(0)

    with _stats_lock:
        if _start_time is None:
            _start_time = time.time()
        _frame_count += 1

        elapsed = time.time() - _start_time
        if elapsed < 1.0:
            return

        n_points = len(pc) // 3
        fps = _frame_count / elapsed
        total_points = sum(len(p) for p in _accumulated_points)

        _frame_count = 0
        _start_time = time.time()

        projectairsim_log().info(
            f"[LS120S3] 当前帧: {n_points:,}点 | 累积 {len(_accumulated_points)}帧/{total_points:,}点 | 帧率: {fps:.1f} Hz"
        )


def get_accumulated_cloud():
    with _accum_lock:
        if not _accumulated_points:
            return None
        all_pts = np.concatenate(_accumulated_points, axis=0)

    if len(all_pts) > 200000:
        idx = np.random.choice(len(all_pts), 200000, replace=False)
        all_pts = all_pts[idx]

    z = all_pts[:, 2]
    z_min, z_max = z.min(), z.max()
    if z_max - z_min < 1e-6:
        intensity = np.ones(len(all_pts))
    else:
        intensity = (z - z_min) / (z_max - z_min)

    return {
        "point_cloud": all_pts.flatten().tolist(),
        "intensity_cloud": intensity.tolist(),
    }


async def display_updater(interval=0.2):
    await asyncio.sleep(2.0)
    while True:
        await asyncio.sleep(interval)
        cloud = get_accumulated_cloud()
        if cloud:
            lidar_display.receive(cloud)


async def main():
    global lidar_display

    client = ProjectAirSimClient()

    lidar_display = LidarDisplay(
        win_name="LS120S3 LiDAR - 3D Point Cloud (累积显示)",
        color_mode=LidarDisplay.COLOR_INTENSITY,
        width=1280,
        height=720,
        x=100,
        y=100,
        view=LidarDisplay.VIEW_PERSPECTIVE,
        zoom=0.2,
        coordinate_axes_size=5.0,
    )

    try:
        client.connect()

        projectairsim_log().info(f"加载场景: {SCENE_CONFIG}")
        world = World(client, SCENE_CONFIG, delay_after_load_sec=2)

        drone = Drone(client, world, DRONE_NAME)

        client.subscribe(
            drone.sensors["lidar1"]["lidar"],
            lidar_callback,
        )

        lidar_display.start()

        drone.enable_api_control()
        drone.arm()

        projectairsim_log().info("无人机起飞至 20m 高度...")
        move_task = await drone.move_by_velocity_async(
            v_north=0.0, v_east=0.0, v_down=-3.0, duration=7.0
        )
        await move_task

        projectairsim_log().info("以 2m/s 慢速向前飞行，LiDAR 3D 点云累积显示中...")
        projectairsim_log().info("在 Open3D 窗口中可鼠标交互旋转/缩放")
        projectairsim_log().info("按 Ctrl+C 退出")
        move_forward_task = await drone.move_by_velocity_async(
            v_north=2.0, v_east=0.0, v_down=0.0, duration=120.0
        )

        updater_task = asyncio.create_task(display_updater(0.2))

        try:
            while True:
                await asyncio.sleep(1.0)
        except (KeyboardInterrupt, asyncio.CancelledError):
            pass

        updater_task.cancel()
        try:
            await updater_task
        except asyncio.CancelledError:
            pass

        try:
            move_forward_task.cancel()
            await move_forward_task
        except (asyncio.CancelledError, Exception):
            pass

        projectairsim_log().info("降落...")
        move_task = await drone.move_by_velocity_async(
            v_north=0.0, v_east=0.0, v_down=2.0, duration=10.0
        )
        await move_task

        drone.disarm()
        drone.disable_api_control()

    except Exception as err:
        projectairsim_log().error(f"异常: {err}", exc_info=True)
    finally:
        client.disconnect()
        lidar_display.stop()
        projectairsim_log().info("已断开连接，测试结束")


if __name__ == "__main__":
    asyncio.run(main())
