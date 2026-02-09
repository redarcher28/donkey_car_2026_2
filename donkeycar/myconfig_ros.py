"""
Donkeycar配置文件 - ROS版本
"""

# ROS配置
ROS_ENABLED = True
ROS_TOPIC_CMD_VEL = '/donkeycar/cmd_vel'
ROS_TOPIC_IMAGE = '/camera/image_raw'
ROS_NODE_NAME = 'donkeycar_node'

# 模型配置
MODEL_PATH = 'models/mypilot.h5'
MODEL_TYPE = 'linear'  # 可以是 'linear', 'categorical', 'imu', 'behavior', 'rnn'

# 相机配置
CAMERA_TYPE = 'CSIC'  # 如果是Jetson使用CSIC相机
IMAGE_W = 160
IMAGE_H = 120
IMAGE_DEPTH = 3

# 车辆控制配置
STEERING_CHANNEL = 1
THROTTLE_CHANNEL = 0
STEERING_LEFT_PWM = 400
STEERING_RIGHT_PWM = 300
THROTTLE_FORWARD_PWM = 400
THROTTLE_STOPPED_PWM = 360
THROTTLE_REVERSE_PWM = 320

# 自动驾驶配置
USE_JOYSTICK_AS_DEFAULT = False
AUTO_RECORD_ON_THROTTLE = True
CONTROLLER_TYPE = 'ros'  # 使用ROS控制器