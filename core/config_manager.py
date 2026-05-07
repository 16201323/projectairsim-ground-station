"""
核心模块 - 配置管理器

本模块实现仿真场景配置的自动生成：
ConfigManager：根据无人机型号动态生成场景配置文件
"""

import json
import os


class ConfigManager:
    """
    配置管理器：根据无人机型号动态生成场景配置文件

    工作原理：
    1. 读取模板场景配置文件（scene_adv_drone.jsonc）
    2. 移除JSONC中的注释行（//开头的注释），转换为标准JSON
    3. 将actors中type为"robot"的robot-config字段替换为用户选择的无人机配置文件
    4. 生成临时场景配置文件，供World加载使用

    注意：临时文件在控制线程退出时自动删除
    """

    def __init__(self, sim_config_path):
        """
        初始化配置管理器

        参数：
            sim_config_path: 仿真配置文件目录路径（包含所有jsonc配置文件）
        """
        self.sim_config_path = sim_config_path
        # 模板场景配置文件路径，包含场景基础设置和无人机占位符
        self.scene_template_path = os.path.join(sim_config_path, "scene_adv_drone.jsonc")

    def generate_scene_config(self, robot_config_file, home_geo_point=None):
        """
        根据无人机型号生成场景配置文件

        处理流程：
        1. 读取模板场景配置（JSONC格式，含注释）
        2. 逐行移除//注释（保留字符串内的//）
        3. 解析为JSON对象
        4. 替换机器人配置引用为用户选择的型号
        5. 写入临时文件

        参数：
            robot_config_file: 机器人配置文件名（如"robot_quadrotor_adv.jsonc"）
            home_geo_point: 自定义home地理坐标点，格式：{"latitude":xx, "longitude":xx, "altitude":xx}

        返回：
            生成的临时场景配置文件路径
        """
        # 读取模板场景配置（JSONC格式，包含//注释）
        with open(self.scene_template_path, "r", encoding="utf-8") as f:
            content = f.read()
        # 移除JSONC注释：逐行检查，移除//开头的注释
        # 但保留字符串内部的"//"（如URL中的斜杠）
        lines = content.split("\n")
        clean_lines = []
        for line in lines:
            # 查找//注释的起始位置
            comment_idx = line.find("//")
            if comment_idx >= 0:
                # 检查//是否在引号字符串内部
                # 通过统计//之前的双引号数量判断：奇数个引号说明在字符串内
                in_string = False
                for ch in line[:comment_idx]:
                    if ch == '"':
                        in_string = not in_string
                # 如果//不在字符串内，则移除注释部分
                if not in_string:
                    line = line[:comment_idx]
            clean_lines.append(line)

        # 将清理后的文本解析为JSON对象
        scene_data = json.loads("\n".join(clean_lines))

        # 遍历场景中的actor列表，找到类型为"robot"的actor
        # 将其robot-config字段替换为用户选择的无人机配置文件
        for actor in scene_data.get("actors", []):
            if actor.get("type") == "robot":
                actor["robot-config"] = robot_config_file

        # 如果指定了自定义home地理坐标，更新场景配置
        if home_geo_point:
            scene_data["home-geo-point"] = home_geo_point

        # 生成临时场景配置文件
        # 使用ensure_ascii=True避免中文在GBK编码系统上写入失败
        temp_scene_path = os.path.join(self.sim_config_path, "_scene_adv_drone_temp.jsonc")
        with open(temp_scene_path, "w", encoding="utf-8") as f:
            json.dump(scene_data, f, indent=2, ensure_ascii=True)

        return temp_scene_path
