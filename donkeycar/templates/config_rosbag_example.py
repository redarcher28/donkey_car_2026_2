# ROS Bag Data Configuration Example
# 用于ros_hardware_controlled.py模板的配置示例
#
# 添加这些配置到您的myconfig.py文件中以启用ROS bag数据输入

# =====================================
# ROS BAG数据输入配置
# ROS BAG Data Input Configuration  
# =====================================

# 设置摄像头类型为ROS_BAG以启用bag数据读取
CAMERA_TYPE = "ROS_BAG"

# 必需：ROS bag文件路径
ROS_BAG_FILE_PATH = "/path/to/your/data.bag"

# 可选：ROS话题名称配置（默认值如下）
ROS_CAMERA_TOPIC = '/camera/image_raw'
ROS_IMU_TOPIC = '/imu/data'
ROS_LIDAR_TOPIC = '/scan'
ROS_ODOM_TOPIC = '/odom'
ROS_GPS_TOPIC = '/gps/fix'

# 可选：播放控制配置
ROS_BAG_LOOP_PLAYBACK = True        # 是否循环播放bag文件
ROS_BAG_PLAYBACK_RATE = 1.0         # 播放速率倍数 (0.1 - 10.0)

# 可选：时间范围配置（秒，相对于bag开始时间）
# ROS_BAG_START_TIME = 10.0         # 从第10秒开始播放
# ROS_BAG_END_TIME = 300.0          # 播放到第300秒结束

# =====================================
# 其他支持的CAMERA_TYPE选项
# Other Supported CAMERA_TYPE Options
# =====================================

# 实时ROS摄像头（推荐用于纯ROS系统）
# CAMERA_TYPE = "ROS_CAMERA"

# PiCamera + ROS其他传感器
# CAMERA_TYPE = "PICAM"

# 模拟数据（用于测试）
# CAMERA_TYPE = "MOCK"

# =====================================
# ROS硬件控制配置
# ROS Hardware Control Configuration
# =====================================

# ROS命名空间
ROS_NAMESPACE = 'donkeycar'

# ROS硬件控制限制
ROS_MAX_SPEED_MS = 2.0              # 最大线速度 (m/s)
ROS_MAX_ANGULAR_RADS = 1.0          # 最大角速度 (rad/s)

# =====================================
# 数据录制配置
# Data Recording Configuration
# =====================================

# 是否录制ROS传感器数据
RECORD_ROS_SENSORS = True

# 自动创建新的tub目录
AUTO_CREATE_NEW_TUB = True

# AI训练期间也录制数据
RECORD_DURING_AI = False

# =====================================
# 使用示例
# Usage Examples
# =====================================

"""
使用ROS bag数据训练模型:

1. 设置配置:
   - CAMERA_TYPE = "ROS_BAG"
   - ROS_BAG_FILE_PATH = "/path/to/training_data.bag"
   - RECORD_ROS_SENSORS = True

2. 启动数据录制（收集训练数据）:
   python manage.py drive

3. 训练模型:
   python manage.py train --model models/bag_model.h5

4. 测试模型（使用bag数据）:
   python manage.py drive --model models/bag_model.h5

5. 部署到实际ROS系统:
   - 修改配置: CAMERA_TYPE = "ROS_CAMERA"
   - python manage.py drive --model models/bag_model.h5
"""

# =====================================
# 故障排除
# Troubleshooting
# =====================================

"""
常见问题和解决方案:

1. ImportError: ROS bag reading not available
   解决方案: 安装ROS和相关包
   sudo apt install ros-noetic-rosbag ros-noetic-cv-bridge

2. 文件不存在错误
   解决方案: 检查ROS_BAG_FILE_PATH路径是否正确

3. 话题不存在警告
   解决方案: 使用 rosbag info your_bag.bag 查看可用话题
   然后更新相应的ROS_*_TOPIC配置

4. 图像转换错误
   解决方案: 确保bag文件中的图像格式支持（rgb8, bgr8等）

5. 播放速度问题
   解决方案: 调整ROS_BAG_PLAYBACK_RATE，较低的值用于慢速播放
"""