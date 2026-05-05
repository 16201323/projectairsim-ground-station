import argparse
import asyncio
import time
import projectairsim
from projectairsim import Drone, World
import keyboard

# --- 无人机控制函数 (Drone Control Functions) ---

async def takeoff(drone):
    """Arms the drone and takes off to a default altitude."""
    # 解锁无人机并起飞至默认高度 (Arms the drone and takes off to a default altitude)
    print("Arming the hexarotor...")
    drone.arm()
    print("Taking off...")
    await drone.takeoff_async()
    time.sleep(1)


async def land(drone):
    """Lands the drone."""
    # 降落无人机 (Lands the drone)
    print("Landing...")
    await drone.land_async()
    print("Disarming the hexarotor...")
    drone.disarm()

# --- 主控制循环 (Main Control Loop) ---

async def run_keyboard_control(drone):
    """
    Controls the hexarotor using keyboard inputs.
    # 使用键盘控制六旋翼无人机 (Controls the hexarotor using keyboard inputs)

    Args:
        drone: The Drone object. # 无人机对象 (The Drone object)
    """

    # 启用API控制 (Enable API control)
    drone.enable_api_control()

    # 起飞 (Takeoff)
    await takeoff(drone)

    # 速度设置 (Speed settings)
    speed = 5  # m/s - 前进/后退/左移/右移的速度 (m/s - speed for forward/backward/left/right movement)
    yaw_speed = 20  # degrees/s - 偏航角速度 (degrees/s - yaw rotation speed)
    duration = 0.1  # seconds - 控制指令持续时间 (seconds - duration of control command)

    print("\n--- Keyboard Control ---")
    print("W/S: Pitch (Forward/Backward)")
    print("A/D: Roll (Left/Right)")
    print("Up/Down Arrows: Throttle (Altitude)")
    print("Left/Right Arrows: Yaw (Rotation)")
    print("L: Land")
    print("Q: Quit")
    print("--------------------")

    keep_running = True

    while keep_running:
        # 重置速度分量 (Reset velocity components)
        vx, vy, vz, yaw_rate = 0, 0, 0, 0

        # 俯仰控制 (Pitch control)
        if keyboard.is_pressed('w'):
            vx = speed

        elif keyboard.is_pressed('s'):
            vx = -speed

        # 横滚控制 (Roll control)
        if keyboard.is_pressed('a'):
            vy = -speed

        elif keyboard.is_pressed('d'):
            vy = speed

        # 油门控制 (Throttle control)
        if keyboard.is_pressed('up'):
            vz = -speed  # Negative Z is up - 负Z方向为上升 (Negative Z is up)

        elif keyboard.is_pressed('down'):
            vz = speed

        # 偏航控制 (Yaw control)
        if keyboard.is_pressed('left'):
            yaw_rate = -yaw_speed

        elif keyboard.is_pressed('right'):
            yaw_rate = yaw_speed

        # 降落并退出 (Land and exit)
        if keyboard.is_pressed('l'):
            await land(drone)
            keep_running = False

        # 退出程序 (Quit program)
        if keyboard.is_pressed('q'):
            keep_running = False

        # 以机体坐标系移动无人机 (Move the drone in its body frame)
        # vx, vy, vz 现在被解释为相对于无人机的前后/左右/上下移动
        # (vx, vy, vz are now interpreted as forward/backward, right/left, up/down relative to the drone)
        if vx != 0 or vy != 0 or vz != 0:
            await drone.move_by_velocity_body_frame_async(vx, vy, vz, duration)
        if yaw_rate != 0:
            await drone.rotate_by_yaw_rate_async(yaw_rate, duration)
        await asyncio.sleep(0.01)

# --- 主程序入口 (Main Execution) ---

async def main():
    # 命令行参数解析器 (Command line argument parser)
    parser = argparse.ArgumentParser(
        description="Example of using keyboard to control a hexarotor in Project AirSim."
        # 使用键盘控制六旋翼无人机的示例 (Example of using keyboard to control a hexarotor in Project AirSim)
    )

    # 服务器IP地址参数 (Server IP address argument)
    parser.add_argument(
        "--address",
        help=("the IP address of the host running Project AirSim"),
        # 运行Project AirSim的主机IP地址 (the IP address of the host running Project AirSim)
        type=str,
        default="127.0.0.1",
    )

    # 场景配置文件参数 (Scene config file argument)
    parser.add_argument(
        "--sceneconfigfile",
        help=(
            'the Project AirSim scene config file to load, defaults to "scene_basic_hexarotor.jsonc"'
        ),
        # 要加载的Project AirSim场景配置文件，默认为"scene_basic_hexarotor.jsonc"
        # (the Project AirSim scene config file to load, defaults to "scene_basic_hexarotor.jsonc")

        type=str,
        default="scene_basic_hexarotor.jsonc",
    )

    # 配置文件目录参数 (Config directory argument)
    parser.add_argument(
        "--simconfigpath",
        help=(
            'the directory containing Project AirSim config files, defaults to "sim_config"'
        ),
        # 包含Project AirSim配置文件的目录，默认为"sim_config"
        # (the directory containing Project AirSim config files, defaults to "sim_config")
        type=str,
        default="sim_config/",
    )

    # Topic发布-订阅端口参数 (Topic pub-sub port argument)
    parser.add_argument(
        "--topicsport",
        help=(
            "the TCP/IP port of Project AirSim's topic pub-sub client connection "
            '(see the Project AirSim command line switch "-topicsport")'
        ),
        # Project AirSim的topic发布-订阅客户端连接的TCP/IP端口
        # (the TCP/IP port of Project AirSim's topic pub-sub client connection)
        type=int,
        default=8989,
    )

    # 服务端口参数 (Services port argument)
    parser.add_argument(
        "--servicesport",
        help=(
            "the TCP/IP port of Project AirSim's services client connection "
            '(see the Project AirSim command line switch "-servicessport")'
        ),
        # Project AirSim的服务客户端连接的TCP/IP端口
        # (the TCP/IP port of Project AirSim's services client connection)
        type=int,
        default=8990,
    )

    args = parser.parse_args()

    # 创建Project AirSim客户端 (Create Project AirSim client)
    client = projectairsim.ProjectAirSimClient(
        address=args.address,
        port_topics=args.topicsport,
        port_services=args.servicesport,
    )

    drone = None
    try:
        # 连接到服务器 (Connect to server)
        client.connect()

        # 创建World对象并加载场景 (Create World object and load scene)
        world = projectairsim.World(
            client=client,
            scene_config_name=args.sceneconfigfile,
            sim_config_path=args.simconfigpath,
        )

        # 创建无人机对象 (Create drone object)
        drone = Drone(client, world, "Hexarotor1")

        # 运行键盘控制 (Run keyboard control)
        await run_keyboard_control(drone)

    except Exception as e:
        print(f"An error occurred: {e}")
        # 发生错误时打印错误信息 (Print error message when exception occurs)

    finally:
        # 清理工作：锁定无人机、禁用API控制、断开连接
        # (Cleanup: disarm drone, disable API control, disconnect)
        if drone:
            drone.disarm()
            drone.disable_api_control()
        client.disconnect()

        print("Cleaned up and disconnected.")
        # 清理完成并已断开连接 (Cleaned up and disconnected)

if __name__ == "__main__":
    asyncio.run(main())
