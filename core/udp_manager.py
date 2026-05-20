"""
核心模块 - UDP通信管理器

本模块实现外部UDP控制指令的接收和解析：
UDPManager：接收ModelOutputStruct结构体数据，提供速度和位置信息用于无人机飞行控制
"""

import socket
import struct
import sys
import threading
import time

from .constants import (
    UDP_DEFAULT_IP, UDP_DEFAULT_PORT, UDP_MULTICAST_ADDR,
    UDP_BUFFER_SIZE, UDP_RECV_TIMEOUT
)


class UDPManager:
    """
    UDP通信管理器：接收外部ModelOutputStruct结构体数据，提供速度和位置信息

    工作原理：
    1. 在指定IP和端口上创建UDP套接字并监听
    2. 非阻塞方式接收ModelOutputStruct结构体数据
    3. 解析结构体中的位置（经纬高）、姿态（欧拉角）、速度等字段
    4. 提供解析后的数据供控制线程使用（速度跟踪+位置P校正模式）

    ModelOutputStruct结构体协议格式（共224字节，28个double）：
    ┌────────┬────────┬────────┬────────┬────────┬────────┬────────┬────────┬────────┬────────┐
    │   P    │   R    │   Q    │   Ax   │   Ay   │   Az   │   Vx   │   Vy   │   Vz   │   Nx   │
    │ 8字节  │ 8字节  │ 8字节  │ 8字节  │ 8字节  │ 8字节  │ 8字节  │ 8字节  │ 8字节  │ 8字节  │
    │ double │ double │ double │ double │ double │ double │ double │ double │ double │ double │
    ├────────┼────────┼────────┼────────┼────────┼────────┼────────┼────────┼────────┼────────┤
    │   Ny   │   Nz   │ theta  │  phi   │  psi   │  alt   │ height │   Vi   │   Vt   │ alpha  │
    │ 8字节  │ 8字节  │ 8字节  │ 8字节  │ 8字节  │ 8字节  │ 8字节  │ 8字节  │ 8字节  │ 8字节  │
    │ double │ double │ double │ double │ double │ double │ double │ double │ double │ double │
    ├────────┼────────┼────────┼────────┼────────┼────────┼────────┼────────┼────────┼────────┤
    │  beta  │   vn   │   ve   │  Hdot  │   Vd   │  lon   │  lat   │ track  │        │        │
    │ 8字节  │ 8字节  │ 8字节  │ 8字节  │ 8字节  │ 8字节  │ 8字节  │ 8字节  │        │        │
    │ double │ double │ double │ double │ double │ double │ double │ double │        │        │
    └────────┴────────┴────────┴────────┴────────┴────────┴────────┴────────┴────────┘

    struct格式: "<28d" (小端序，28个double)
    字段说明：
    P: 滚转角速率(deg/s)    R: 偏航角速率(deg/s)    Q: 俯仰角速率(deg/s)
    Ax/Ay/Az: X/Y/Z轴线加速度(m/s^2)
    Vx/Vy/Vz: X/Y/Z轴速度(m/s)
    Nx/Ny/Nz: 纵向/侧向/法向过载(m/s^2)
    theta: 俯仰角(deg)      phi: 滚转角(deg)        psi: 偏航角(deg)
    alt: 相对高度(m,向上)    height: 绝对高度(m,向上)
    Vi: 指示空速(m/s)       Vt: 真空速(m/s)
    alpha: 迎角(deg)        beta: 侧滑角(deg)
    vn: 北向速度(m/s)       ve: 东向速度(m/s)       Hdot: 升降速度(m/s,向上)
    Vd: 地速(m/s)           lon: 经度(deg)          lat: 纬度(deg)          track: 航迹角(deg)
    """

    UDP_STRUCT_FORMAT = "<28d"
    UDP_STRUCT_SIZE = struct.calcsize("<28d")

    FIELD_NAMES = [
        "P", "R", "Q",
        "Ax", "Ay", "Az",
        "Vx", "Vy", "Vz",
        "Nx", "Ny", "Nz",
        "theta", "phi", "psi",
        "alt", "height", "Vi", "Vt",
        "alpha", "beta",
        "vn", "ve", "Hdot", "Vd",
        "lon", "lat", "track",
    ]

    def __init__(self, ip=UDP_DEFAULT_IP, port=UDP_DEFAULT_PORT, multicast_addr=None):
        """
        初始化UDP管理器

        参数：
            ip: 本机网络接口IP地址（用于指定接收组播的网卡，如192.168.1.5）
            port: 监听端口号（默认15610）
            multicast_addr: 组播组地址（如224.0.0.25），None表示单播模式
        """
        self.ip = ip
        self.port = port
        self.socket = None
        self.running = False
        # 组播地址：优先使用传入的multicast_addr，否则检测ip是否为组播地址
        self._multicast_addr = None
        if multicast_addr:
            self._multicast_addr = multicast_addr
        elif ip:
            try:
                ip_bytes = socket.inet_aton(ip)
                first_octet = ip_bytes[0]
                if 224 <= first_octet <= 239:
                    self._multicast_addr = ip
            except Exception:
                pass
        self.latest_command = None
        self.lock = threading.Lock()

    def start(self):
        """
        启动UDP监听
        创建UDP套接字，设置地址复用选项，绑定到指定IP和端口
        如果是组播地址（224.0.0.0-239.255.255.255），则加入组播组
        设置接收超时为UDP_RECV_TIMEOUT，实现非阻塞接收
        """
        try:
            self.socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            self.socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            self.socket.bind(("0.0.0.0", self.port))
            self.socket.settimeout(UDP_RECV_TIMEOUT)
            # 检查是否为组播地址，如果是则加入组播组
            multicast_addr = getattr(self, '_multicast_addr', None)
            if multicast_addr:
                if sys.platform == 'win32':
                    # Windows: 加入组播组，使用INADDR_ANY让系统自动选择网卡
                    mreq = struct.pack("4sl", socket.inet_aton(multicast_addr), socket.INADDR_ANY)
                    self.socket.setsockopt(socket.IPPROTO_IP, socket.IP_ADD_MEMBERSHIP, mreq)
                    self.socket.setsockopt(socket.IPPROTO_IP, socket.IP_MULTICAST_LOOP, 1)
                else:
                    mreq = struct.pack("4sl", socket.inet_aton(multicast_addr), socket.INADDR_ANY)
                    self.socket.setsockopt(socket.IPPROTO_IP, socket.IP_ADD_MEMBERSHIP, mreq)
            self.running = True
        except Exception as e:
            self.running = False
            raise e

    def _parse_struct(self, data):
        """
        解析ModelOutputStruct结构体数据

        参数：
            data: 原始字节流

        返回：
            解析后的指令字典（字段名→值），或None（解析失败）
        """
        if len(data) < self.UDP_STRUCT_SIZE:
            return None
        try:
            values = struct.unpack(self.UDP_STRUCT_FORMAT, data[:self.UDP_STRUCT_SIZE])
            return dict(zip(self.FIELD_NAMES, values))
        except struct.error:
            return None

    def receive_command(self):
        """
        接收UDP控制指令（非阻塞方式）
        超时返回None，收到指令时更新最新指令缓存

        返回：
            解析后的控制指令字典，或None（超时/解析失败/未启动）
        """
        if not self.running or self.socket is None:
            return None
        try:
            data, addr = self.socket.recvfrom(UDP_BUFFER_SIZE)
            command = self._parse_struct(data)
            if command is not None:
                with self.lock:
                    self.latest_command = command
            return command
        except socket.timeout:
            return None
        except Exception:
            return None

    def wait_for_first_packet(self, timeout=3.0):
        """
        预连接模式：等待飞控首包数据（阻塞调用，3秒超时）
        用于UDP模式下在连接仿真器之前获取飞控经纬度，作为场景home_geo_point

        参数：
            timeout: 等待超时时间（秒），默认3秒

        返回：
            解析后的控制指令字典，或None（超时/未启动/解析失败）
        """
        if not self.running or self.socket is None:
            return None
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            remaining = deadline - time.monotonic()
            if remaining <= 0:
                break
            try:
                self.socket.settimeout(min(remaining, 0.5))
                data, addr = self.socket.recvfrom(UDP_BUFFER_SIZE)
                command = self._parse_struct(data)
                if command is not None:
                    with self.lock:
                        self.latest_command = command
                    return command
            except socket.timeout:
                continue
            except Exception:
                continue
        return None

    def stop(self):
        """停止UDP监听，关闭套接字并释放资源"""
        self.running = False
        if self.socket:
            self.socket.close()
            self.socket = None
