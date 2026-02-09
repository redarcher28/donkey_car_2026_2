# DonkeyCar-ROS 集成系统使用手册


## 🎯 系统架构概览

```
┌─────────────┐    ┌──────────────┐    ┌─────────────┐    ┌──────────────┐
│ ROS 传感器  │───▶│ DataConverter│───▶│ DonkeyCar   │───▶│ DataConverter│
│ (相机/IMU/  │    │              │    │ 决策处理    │    │              │
│  激光雷达)  │    │              │    │ (AI模型)    │    │              │
└─────────────┘    └──────────────┘    └─────────────┘    └──────────────┘
                                                                    │
┌─────────────┐    ┌──────────────┐                                │
│ ROS 硬件控制│◀───│ 话题发布器   │◀───────────────────────────────┘
│ (电机/舵机) │    │ (/cmd_vel)   │
└─────────────┘    └──────────────┘
```

## � 数据融合与Tub格式处理架构

### 🔍 实现方案对比分析

我们的DonkeyCar-ROS集成采用了**DonkeyCar管道融合模式**，区别于简单的直接转换方案：

#### **方案A: 直接Tub写入模式 (传统方式)**
```python
ROS Topics → 数据转换器 → 直接写入Tub文件
优点: 简单直接，适合离线数据收集
缺点: 无法实时运行，不支持多源融合，训练数据不完整
```

#### **方案B: DonkeyCar管道集成模式 (我们的实现) ✅**
```python
ROS Topics → ROSMultiSensorBridge → DonkeyCar Vehicle Pipeline → TubWriter
优点: 实时运行+训练, 多传感器融合, 完整DonkeyCar生态集成
应用: 生产级ROS-DonkeyCar融合系统
```

### 🏗️ 数据流水线架构详解

#### **1. 数据输入层 - ROS传感器接入**
```python
# 支持的ROS消息类型
sensor_msgs/Image          → 相机图像
sensor_msgs/LaserScan      → 激光雷达距离数组  
sensor_msgs/Imu            → IMU 6轴数据
geometry_msgs/Twist        → 控制命令 (线速度/角速度)
nav_msgs/Odometry          → 里程计位置信息
```

#### **2. 数据转换层 - 格式标准化**
```python
# DataConverter 核心转换逻辑
class DataConverter:
    def ros_image_to_donkey(self, ros_image):
        """ROS Image → DonkeyCar RGB Array (160x120)"""
        cv_image = self.cv_bridge.imgmsg_to_cv2(ros_image, "rgb8")
        return np.array(cv_image)
    
    def twist_to_donkey_control(self, twist):
        """ROS Twist → DonkeyCar angle/throttle [-1.0, 1.0]"""
        throttle = twist.linear.x / max_speed
        angle = twist.angular.z / max_angular
        return np.clip(angle, -1, 1), np.clip(throttle, -1, 1)
```

#### **3. 数据融合层 - 多传感器同步**
```python
# ROSMultiSensorBridge 实现
class ROSMultiSensorBridge:
    def run(self):
        """同步输出所有传感器数据到DonkeyCar管道"""
        return {
            'cam/image_array': self.latest_image,      # 标准图像
            'imu/data': self.latest_imu,               # IMU字典
            'lidar/scan': self.latest_lidar,           # 雷达字典  
            'ros/angle': self.latest_angle,            # ROS角度
            'ros/throttle': self.latest_throttle       # ROS油门
        }
```

#### **4. Tub存储层 - 训练数据格式**
```python
# 智能Tub数据结构
# 基础DonkeyCar字段 (保持兼容)
inputs = ['cam/image_array', 'user/angle', 'user/throttle', 'user/mode']
types = ['image_array', 'float', 'float', 'str']

# ROS扩展字段 (可选开启)
if RECORD_ROS_SENSORS:
    inputs.extend([
        'imu/gyro_x', 'imu/gyro_y', 'imu/gyro_z',      # IMU陀螺仪
        'imu/accel_x', 'imu/accel_y', 'imu/accel_z',   # IMU加速度
        'lidar/range_count', 'lidar/range_min', 'lidar/range_max',  # 雷达摘要
        'ros/angle', 'ros/throttle'                     # ROS控制记录
    ])
```

### 🔄 数据同步机制

#### **线程安全的数据缓存**
```python
class BufferedDataSync:
    """实时数据同步器 - 解决ROS异步回调问题"""
    def put_data(self, data):
        with self.lock:
            timestamp = time.time()
            self.data_buffer.append((timestamp, data))
            self.latest_data = data
    
    def get_latest_data(self, timeout=0.1):
        """获取最新数据，超时返回None"""
        if time.time() - self.last_update_time > timeout:
            return None  # 数据过期
        return self.latest_data
```

#### **智能数据适配器**
```python  
class ROSSensorDataAdapter:
    """将ROS复杂数据转换为Tub兼容的标量值"""
    def run(self, imu_data, lidar_data):
        # 解构IMU字典为单独字段
        gyro_x = imu_data.get('gyro_x', 0.0) if isinstance(imu_data, dict) else 0.0
        # 解构雷达数据为统计值
        ranges = lidar_data.get('ranges', []) if isinstance(lidar_data, dict) else []
        range_count = len(ranges)
        range_min = min(ranges) if ranges else 0.0
        range_max = max(ranges) if ranges else 0.0
        
        return gyro_x, gyro_y, gyro_z, accel_x, accel_y, accel_z, \
               range_count, range_min, range_max
```

### 🎯 集成优势分析

#### **✅ 我们实现的核心优势**

| 特性 | 直接转换方案 | 我们的DonkeyCar集成 | 优势说明 |
|------|--------------|-------------------|----------|
| **实时运行** | ❌ 仅数据收集 | ✅ 实时+训练并行 | 同一代码支持运行和训练 |
| **数据丰富度** | 📊 ROS数据 | 📊 ROS+DonkeyCar融合 | 更完整的训练特征 |  
| **训练兼容性** | ⚠️ 需额外转换 | ✅ 原生兼容 | 直接使用DK训练流程 |
| **传感器融合** | ❌ 单一数据源 | ✅ 多传感器同步 | 支持视觉+IMU+雷达 |
| **扩展性** | ❌ 单一用途脚本 | ✅ 模块化组件 | 灵活配置不同传感器 |

#### **📈 训练数据质量对比**

**传统方案Tub结构:**
```python
# 仅ROS数据 - 训练特征单一
{
    'cam/image_array': ros_image,
    'lidar/dist_array': lidar_ranges, 
    'user/angle': converted_angle,
    'user/throttle': converted_throttle
}
```

**我们的融合Tub结构:**
```python
# DonkeyCar + ROS 融合数据 - 训练特征丰富
{
    # 标准DonkeyCar字段
    'cam/image_array': image,
    'user/angle': user_angle, 'user/throttle': user_throttle,
    
    # ROS传感器扩展  
    'imu/gyro_x': gyro_x, 'imu/accel_x': accel_x,
    'lidar/range_count': 360, 'lidar/range_min': 0.1,
    
    # 双源控制记录 (用于对比学习)
    'ros/angle': ros_angle, 'ros/throttle': ros_throttle,
    'final/angle': final_angle, 'final/throttle': final_throttle
}
```

### 🚀 实际应用场景

#### **场景1: 纯数据收集**
```python
# 配置: 仅记录，不运行AI
RECORD_DURING_AI = False
USE_ROS_AS_DRIVER = True

# 数据流: ROS传感器 → 转换 → Tub存储
# 适用: 收集大量标注数据用于训练
```

#### **场景2: 实时运行+数据收集**  
```python
# 配置: AI运行同时记录训练数据
RECORD_DURING_AI = True
USE_AI_PILOT = True

# 数据流: ROS传感器 → AI决策 → ROS控制 + Tub记录
# 适用: 持续学习和数据增强
```

#### **场景3: 多模态传感器融合**
```python
# 配置: 视觉+IMU+雷达融合决策  
USE_CAMERA = True
USE_ROS_SENSORS = True
RECORD_ROS_SENSORS = True

# 数据流: 多传感器 → 特征融合 → 增强决策
# 适用: 复杂环境的鲁棒自动驾驶
```

## �🚀 快速启动命令

### 1. 运行完整性测试
```bash
# 方法 1: 使用测试脚本
cd /path/to/donkeycar
python3 test_ros_pipeline.py --verbose

# 方法 2: 使用启动脚本  
./scripts/launch_donkey_ros.sh test
```

### 2. 启动完整系统

#### A. 使用 ROS Launch (推荐)
```bash
# 启动 roscore (在新终端)
roscore

# ⚠️ 新版本启动 (替换原有的 donkeycar_ros.launch)
# 旧命令: roslaunch donkeycar donkeycar_ros.launch
# 新命令: 
roslaunch donkeycar donkey_ros_integrated.launch \
    car_name:=my_robot \
    data_path:=/home/user/robot_data \
    camera_topic:=/camera/image_raw \
    cmd_vel_topic:=/cmd_vel

# 带高级功能启动
roslaunch donkeycar donkey_ros_integrated.launch \
    use_rviz:=true \
    record_bag:=true \
    web_interface:=true \
    use_compressed_image:=true
```

#### B. 使用快速启动脚本
```bash
# 数据收集模式 (手动控制)
./scripts/launch_donkey_ros.sh collect

# 自主驾驶模式 (AI模型控制)
./scripts/launch_donkey_ros.sh drive

# 仿真模式
./scripts/launch_donkey_ros.sh sim
```

#### C. 手动启动 (高级用户)
```bash
# 1. 启动 ROS 核心
roscore &

# 2. 启动 DonkeyCar 主程序
cd /path/to/donkeycar
python3 donkeycar/management/manage.py drive \
    --config=donkeycar/templates/cfg_ros_integrated.py \
    --tub=/home/user/robot_data

# 3. 启动传感器驱动 (另一个终端)
# 相机
rosrun usb_cam usb_cam_node _video_device:=/dev/video0 \
    _image_width:=160 _image_height:=120

# IMU (如果有)
rosrun imu_driver imu_node _device:=/dev/ttyUSB0

# 激光雷达 (如果有)  
rosrun rplidar_ros rplidarNode _serial_port:=/dev/ttyUSB1
```

## 📊 数据流测试命令

### 1. 检查 ROS 话题
```bash
# 查看所有活跃话题
rostopic list

# 监控图像数据
rostopic echo /camera/image_raw --noarr

# 监控控制命令
rostopic echo /cmd_vel

# 监控 IMU 数据
rostopic echo /imu/data

# 查看话题发布频率
rostopic hz /camera/image_raw
rostopic hz /cmd_vel
```

### 2. 手动发送测试数据
```bash
# 发送控制命令
rostopic pub /cmd_vel geometry_msgs/Twist \
    "linear: {x: 1.0, y: 0.0, z: 0.0}
     angular: {x: 0.0, y: 0.0, z: 0.5}" -1

# 发送停止命令
rostopic pub /cmd_vel geometry_msgs/Twist \
    "linear: {x: 0.0, y: 0.0, z: 0.0}
     angular: {x: 0.0, y: 0.0, z: 0.0}" -1
```

### 3. 数据转换验证
```python
# Python 测试脚本
import rospy
from donkeycar.parts.ros import DataConverter

# 初始化转换器
converter = DataConverter()

# 测试控制数据转换
angle, throttle = 0.5, 0.3
twist = converter.donkey_control_to_twist(angle, throttle)
print(f"DonkeyCar控制 ({angle}, {throttle}) -> ROS Twist: {twist}")

angle_back, throttle_back = converter.twist_to_donkey_control(twist)
print(f"ROS Twist -> DonkeyCar控制 ({angle_back}, {throttle_back})")
```

## 🔧 配置和调试

### 1. 检查系统状态
```bash
# 查看 ROS 节点
rosnode list
rosnode info /donkeycar_main

# 查看参数服务器
rosparam list
rosparam get /donkeycar_main

# 检查 TF 变换
rosrun tf tf_echo base_link camera_link
```

### 2. 性能监控
```bash
# CPU 和内存使用
top -p $(pgrep -f "python.*manage.py")

# ROS 系统监控
rosrun rqt_top rqt_top

# 话题吞吐量
rosrun rqt_plot rqt_plot /cmd_vel/linear/x:angular/z
```

### 3. 日志和调试
```bash
# 查看 ROS 日志
roscd && cd ../log
tail -f latest/rosout.log

# 查看 DonkeyCar 日志 
tail -f /tmp/donkeycar.log

# 详细调试模式启动
roslaunch donkeycar donkey_ros_integrated.launch --screen
```

## 📁 数据管理命令

### 1. 训练数据收集
```bash
# 启动数据收集模式
roslaunch donkeycar donkey_ros_integrated.launch \
    data_path:=/home/user/training_data \
    recording/enabled:=true \
    model/enabled:=false

# 检查收集的数据
ls -la /home/user/training_data/
python3 -c "
from donkeycar.parts.tub_v2 import Tub
tub = Tub('/home/user/training_data')
print(f'Records: {len(tub.manifest.catalog)}')
"
```

### 2. 模型训练
```bash
# 训练新模型
cd /home/user/training_data
python3 /path/to/donkeycar/donkeycar/management/manage.py train \
    --tub=/home/user/training_data \
    --model=models/mypilot.h5

# 使用快速脚本训练
DATA_PATH=/home/user/training_data ./scripts/launch_donkey_ros.sh train
```

### 3. 模型部署
```bash
# 自主驾驶模式
roslaunch donkeycar donkey_ros_integrated.launch \
    data_path:=/home/user/training_data \
    recording/enabled:=true \
    model/enabled:=true \
    model/path:=/home/user/training_data/models/mypilot.h5
```

## 🔍 故障排查

### 1. 常见问题诊断
```bash
# 检查 ROS 环境
echo $ROS_DISTRO
echo $ROS_MASTER_URI
which roscore

# 检查 Python 环境  
python3 -c "import donkeycar; print(donkeycar.__version__)"
python3 -c "import cv_bridge"
python3 -c "import sensor_msgs.msg"

# 检查设备连接
ls -la /dev/video*
ls -la /dev/ttyUSB*
```

### 2. 网络和通信测试
```bash
# 测试 ROS 通信
roscore &
sleep 2
rostopic pub /test std_msgs/String "data: 'hello'" &
rostopic echo /test

# 测试 DonkeyCar-ROS 桥接
python3 -c "
from donkeycar.parts.ros import ROSBridgeTest
test = ROSBridgeTest()
test.run_all_tests()
"
```

### 3. 重置和清理
```bash
# 清理 ROS 日志
rosclean purge -y

# 重启完整系统
pkill -f roscore
pkill -f "python.*manage.py"
sleep 2
./scripts/launch_donkey_ros.sh drive
```

## 🎛️ 高级配置选项

### Launch 文件参数
```xml
<!-- 在 donkey_ros_integrated.launch 中 -->
<param name="control/max_throttle" value="0.8"/>      <!-- 最大油门 -->
<param name="control/max_steering" value="1.0"/>      <!-- 最大转向 -->
<param name="control/deadzone" value="0.1"/>          <!-- 死区 -->
<param name="recording/inputs" value="custom_list"/>   <!-- 记录字段 -->
```

### 环境变量配置
```bash
export DONKEY_PATH=/path/to/donkeycar
export ROS_WS=/path/to/catkin_ws  
export DATA_PATH=/path/to/data
export ROS_DISTRO=noetic
export CUDA_VISIBLE_DEVICES=0  # GPU 配置
```

## 🔄 从旧版本迁移指南

### 如果您需要原版launch文件中的高级功能

1. **创建所需的ROS包**：
```bash
# 创建URDF模型包
catkin_create_pkg donkeycar_description urdf xacro

# 创建导航包  
catkin_create_pkg donkeycar_navigation navigation
```

2. **使用现有的通用包**：
```bash
# 安装通用机器人包
sudo apt install ros-$ROS_DISTRO-robot-state-publisher
sudo apt install ros-$ROS_DISTRO-navigation
sudo apt install ros-$ROS_DISTRO-web-video-server
```

3. **参考新版本进行定制**：
   - 复制 `donkey_ros_integrated.launch`
   - 添加您需要的特定功能
   - 修改话题映射和参数

### 命令替换对照表

| 旧命令 | 新命令 |
|--------|--------|
| `roslaunch donkeycar donkeycar_ros.launch` | `roslaunch donkeycar donkey_ros_integrated.launch` |
| `roslaunch donkeycar donkeycar_ros.launch use_rviz:=true` | `roslaunch donkeycar donkey_ros_integrated.launch use_rviz:=true` |
| `roslaunch donkeycar donkeycar_ros.launch record_bag:=true` | `roslaunch donkeycar donkey_ros_integrated.launch record_bag:=true` |

---

这个集成系统现在可以完整支持 **"ROS采集数据 → bridge → donkeycar处理数据做出决策 → bridge → ROS驱动小车运动"** 的架构流程！

### 🎉 总结

原有的 `donkeycar_ros.launch` 已被弃用，请使用新的 `donkey_ros_integrated.launch` 获得更好的集成体验！