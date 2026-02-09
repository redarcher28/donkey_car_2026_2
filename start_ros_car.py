#!/usr/bin/env python3
"""
启动ROS集成的Donkeycar
"""

import rospy
from ros_bridge import create_ros_enabled_vehicle
import myconfig_ros as cfg


def main():
    print("启动ROS Donkeycar...")

    # 创建车辆
    V = create_ros_enabled_vehicle({
        'ros_topic': cfg.ROS_TOPIC_CMD_VEL,
        'model_path': cfg.MODEL_PATH
    })

    # 添加相机（如果需要）
    if cfg.CAMERA_TYPE == 'CV':
        from donkeycar.parts.cv import CvCam
        cam = CvCam(image_w=cfg.IMAGE_W, image_h=cfg.IMAGE_H,
                    image_d=cfg.IMAGE_DEPTH)
        V.add(cam, outputs=['image'], threaded=True)

    # 添加模型（如果需要本地处理）
    if cfg.MODEL_TYPE == 'linear':
        from donkeycar.parts.keras import KerasLinear
        kl = KerasLinear()
        kl.load(cfg.MODEL_PATH)

        # 添加自动驾驶部件
        V.add(kl, inputs=['image'], outputs=['pilot/angle', 'pilot/throttle'])

    print("车辆配置完成，启动中...")

    try:
        V.start(rate_hz=20)  # 20Hz循环
    except KeyboardInterrupt:
        print("正在关闭车辆...")
    finally:
        V.shutdown()


if __name__ == "__main__":
    main()