import json
import socket
import numpy as np
from datetime import datetime


# ===================== 1. 模拟传感器输入（替换为真实摄像头/雷达数据） =====================
def get_sensor_data():
    """
    模拟小车传感器数据（实际场景替换为：摄像头识别+雷达测距的真实数据）
    返回：
        camera_data：摄像头识别结果（障碍位置：none/left/middle/right）
        radar_data：雷达测距（前/左/右 三个方向的距离，单位m）
    """
    # ------------ 模拟场景1：前方有障碍，左侧无障（可右转避让） ------------
    # camera_data = {"obstacle_pos": "middle"}  # 摄像头识别前方（画面中间）有障碍
    # radar_data = {"front": 0.3, "left": 1.2, "right": 0.4}  # 前方0.3m有障，左侧1.2m无障

    # ------------ 模拟场景2：无任何障碍（正常前进） ------------
    # camera_data = {"obstacle_pos": "none"}
    # radar_data = {"front": 1.5, "left": 1.8, "right": 1.6}

    # ------------ 模拟场景3：左右都有障（原地停车） ------------
    camera_data = {"obstacle_pos": "middle"}
    radar_data = {"front": 0.2, "left": 0.3, "right": 0.25}

    return camera_data, radar_data


# ===================== 2. 避障决策：根据传感器生成合理的移动指令 =====================
def obstacle_avoidance_decision(camera_data, radar_data):
    """
    避障核心逻辑：结合摄像头+雷达数据，生成小车移动指令
    返回：适配ROS Twist的移动参数（linear_x=前进速度，angular_z=转向角速度）
    """
    # 1. 定义安全阈值（可根据小车尺寸调整）
    SAFE_DISTANCE = 0.5  # 小于0.5m判定为有障碍
    NORMAL_SPEED = 0.3  # 正常前进速度（m/s）
    STOP_SPEED = 0.0  # 停车速度
    TURN_SPEED = 0.8  # 避让转向角速度（rad/s，正数左转，负数右转）

    # 2. 提取传感器关键数据
    front_dist = radar_data["front"]
    left_dist = radar_data["left"]
    right_dist = radar_data["right"]
    obstacle_pos = camera_data["obstacle_pos"]

    # 3. 避障决策逻辑
    move_cmd = {"linear_x": NORMAL_SPEED, "angular_z": 0.0}  # 默认正常直行

    # 情况1：前方有障碍（距离<安全阈值）
    if front_dist < SAFE_DISTANCE:
        move_cmd["linear_x"] = STOP_SPEED  # 先停车/减速

        # 子情况1：左侧无障 → 右转避让
        if left_dist > SAFE_DISTANCE:
            move_cmd["angular_z"] = -TURN_SPEED  # 右转（负数）
            print(f"避障：前方有障（{front_dist}m），左侧无障 → 右转避让")

        # 子情况2：右侧无障 → 左转避让
        elif right_dist > SAFE_DISTANCE:
            move_cmd["angular_z"] = TURN_SPEED  # 左转（正数）
            print(f"避障：前方有障（{front_dist}m），右侧无障 → 左转避让")

        # 子情况3：左右都有障 → 原地停车
        else:
            move_cmd["angular_z"] = 0.0
            print(f"避障：前方/左/右都有障 → 原地停车")

    # 情况2：前方无障，摄像头检测到左右侧有障（提前微调方向）
    elif obstacle_pos == "left" and left_dist < SAFE_DISTANCE + 0.2:
        move_cmd["angular_z"] = -0.4  # 轻微右转，远离左侧障碍
        print(f"提前避障：左侧有障 → 轻微右转")

    elif obstacle_pos == "right" and right_dist < SAFE_DISTANCE + 0.2:
        move_cmd["angular_z"] = 0.4  # 轻微左转，远离右侧障碍
        print(f"提前避障：右侧有障 → 轻微左转")

    # 情况3：无任何障碍 → 正常直行
    else:
        print(f"无障：正常前进（速度{NORMAL_SPEED}m/s）")

    return move_cmd


# ===================== 3. 转换为ROS兼容的Twist格式 =====================
def convert_to_ros_twist(move_cmd):
    """将避障后的移动指令转换为ROS Twist兼容的JSON格式"""
    ros_twist_msg = {
        "header": {
            "seq": 0,
            "stamp": datetime.now().strftime("%Y-%m-%d %H:%M:%S.%f"),
            "frame_id": "base_link"
        },
        "linear": {
            "x": move_cmd["linear_x"],  # 前进/后退速度
            "y": 0.0,  # 小车横向速度（默认0）
            "z": 0.0
        },
        "angular": {
            "x": 0.0,
            "y": 0.0,
            "z": move_cmd["angular_z"]  # 转向角速度
        },
        # 新增：携带传感器原始数据，方便小车端调试
        "sensor_data": {
            "camera": camera_data,
            "radar": radar_data
        }
    }
    return json.dumps(ros_twist_msg, ensure_ascii=False)


# ===================== 4. 发送到小车（UDP） =====================
def send_to_car(ros_data, car_ip="127.0.0.1", car_port=8888):
    """发送ROS格式的避障指令到小车"""
    try:
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        sock.sendto(ros_data.encode("utf-8"), (car_ip, car_port))
        print(f"\n指令发送成功 → {car_ip}:{car_port}")
        print(f"发送数据：{ros_data[:100]}...\n")
    except Exception as e:
        print(f"\n发送失败：{e}\n")


# ===================== 5. 主流程（循环发送，模拟小车持续运行） =====================
if __name__ == "__main__":
    import time

    # 循环发送避障指令（10Hz，模拟小车实时决策）
    while True:
        # 步骤1：获取传感器数据（摄像头+雷达）
        camera_data, radar_data = get_sensor_data()
        print(
            f"传感器数据 → 雷达（前：{radar_data['front']}m，左：{radar_data['left']}m，右：{radar_data['right']}m） | 摄像头：{camera_data['obstacle_pos']}")

        # 步骤2：避障决策，生成移动指令
        move_cmd = obstacle_avoidance_decision(camera_data, radar_data)

        # 步骤3：转换为ROS兼容格式
        ros_data = convert_to_ros_twist(move_cmd)

        # 步骤4：发送到小车（本地测试用127.0.0.1，实际小车替换为小车IP）
        send_to_car(ros_data, car_ip="127.0.0.1", car_port=8888)

        # 控制发送频率（10Hz，每0.1秒发送一次）
        time.sleep(0.1)