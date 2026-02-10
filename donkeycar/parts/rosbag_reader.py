#!/usr/bin/env python3
"""
ROS Bag数据读取器 - ROS Bag Data Reader
用于从ROS bag文件中读取传感器数据并提供给DonkeyCar系统

This module provides functionality to read sensor data from ROS bag files
and integrate it with the DonkeyCar data processing pipeline.

Author: DonkeyCar ROS Integration Team  
"""

import logging
import time
import threading
from typing import Dict, Any, Optional, List
import numpy as np

logger = logging.getLogger(__name__)

try:
    import rospy
    import rosbag
    from sensor_msgs.msg import Image, Imu, LaserScan
    from nav_msgs.msg import Odometry
    from geometry_msgs.msg import PoseStamped
    from sensor_msgs.msg import NavSatFix
    from cv_bridge import CvBridge
    ROS_AVAILABLE = True
except ImportError as e:
    logger.warning(f"ROS not available for bag reading: {e}")
    ROS_AVAILABLE = False


class ROSBagSensorReader:
    """
    ROS Bag传感器数据读取器
    从ROS bag文件中按时间序列读取传感器数据，提供与ROSToDonkeySensorBridge兼容的接口
    """
    
    def __init__(self, 
                 bag_file_path: str,
                 camera_topic: str = '/camera/image_raw',
                 imu_topic: str = '/imu/data', 
                 lidar_topic: str = '/scan',
                 odom_topic: str = '/odom',
                 gps_topic: str = '/gps/fix',
                 loop_playback: bool = True,
                 playback_rate: float = 1.0,
                 start_time: Optional[float] = None,
                 end_time: Optional[float] = None):
        """
        初始化ROS Bag读取器
        
        Args:
            bag_file_path: bag文件路径
            camera_topic: 摄像头话题名
            imu_topic: IMU话题名  
            lidar_topic: 激光雷达话题名
            odom_topic: 里程计话题名
            gps_topic: GPS话题名
            loop_playback: 是否循环播放
            playback_rate: 播放速率倍数
            start_time: 开始时间（秒，相对于bag开始）
            end_time: 结束时间（秒，相对于bag开始）
        """
        if not ROS_AVAILABLE:
            raise ImportError("ROS libraries are required for bag reading")
            
        self.bag_file_path = bag_file_path
        self.camera_topic = camera_topic
        self.imu_topic = imu_topic
        self.lidar_topic = lidar_topic
        self.odom_topic = odom_topic  
        self.gps_topic = gps_topic
        self.loop_playback = loop_playback
        self.playback_rate = playback_rate
        self.start_time = start_time
        self.end_time = end_time
        
        # 数据缓存
        self.current_data = {
            'cam/image_array': np.zeros((120, 160, 3), dtype=np.uint8),
            'imu/accel_x': 0.0, 'imu/accel_y': 0.0, 'imu/accel_z': 0.0,
            'imu/gyro_x': 0.0, 'imu/gyro_y': 0.0, 'imu/gyro_z': 0.0, 
            'lidar/ranges': [], 
            'pos/x': 0.0, 'pos/y': 0.0, 'pos/z': 0.0,
            'pos/roll': 0.0, 'pos/pitch': 0.0, 'pos/yaw': 0.0,
            'gps/latitude': 0.0, 'gps/longitude': 0.0, 'gps/altitude': 0.0
        }
        
        # OpenCV bridge for image conversion
        self.cv_bridge = CvBridge()
        
        # 播放控制 
        self.is_playing = False
        self.playback_thread = None
        self.data_lock = threading.RLock()
        
        # Bag信息
        self.bag_info = None
        self._load_bag_info()
        
        logger.info(f"创建ROS Bag读取器: {bag_file_path}")
        logger.info(f"支持话题: {camera_topic}, {imu_topic}, {lidar_topic}, {odom_topic}, {gps_topic}")
        
    def _load_bag_info(self):
        """加载bag文件信息"""
        try:
            with rosbag.Bag(self.bag_file_path, 'r') as bag:
                self.bag_info = bag.get_type_and_topic_info()
                bag_start_time = bag.get_start_time()
                bag_end_time = bag.get_end_time()
                bag_duration = bag_end_time - bag_start_time
                
                logger.info(f"Bag文件信息:")
                logger.info(f"  持续时间: {bag_duration:.2f}秒")
                logger.info(f"  开始时间: {bag_start_time}")
                logger.info(f"  结束时间: {bag_end_time}")
                logger.info(f"  话题数量: {len(self.bag_info.topics)}")
                
                # 显示所有可用话题
                available_topics = list(self.bag_info.topics.keys())
                logger.info(f"  可用话题:")
                for topic in available_topics:
                    topic_info = self.bag_info.topics[topic]
                    logger.info(f"    {topic}: {topic_info.message_count} 消息, 类型: {topic_info.msg_type}")
                
                # 检查需要的话题是否存在，并尝试自动匹配
                required_topics = [self.camera_topic, self.imu_topic, self.lidar_topic, 
                                 self.odom_topic, self.gps_topic]
                
                logger.info(f"  话题匹配检查:")
                for topic in required_topics:
                    if topic in available_topics:
                        topic_info = self.bag_info.topics[topic]
                        logger.info(f"    ✓ {topic}: {topic_info.message_count} 消息, 类型: {topic_info.msg_type}")
                    else:
                        # 尝试找到相似的话题
                        suggested = self._find_similar_topic(topic, available_topics)
                        if suggested:
                            logger.warning(f"    ✗ {topic}: 话题不存在，建议使用: {suggested}")
                        else:
                            logger.warning(f"    ✗ {topic}: 话题不存在") 
                        
        except Exception as e:
            logger.error(f"加载bag文件信息失败: {e}")
            self.bag_info = None
    
    def _find_similar_topic(self, target_topic: str, available_topics: List[str]) -> Optional[str]:
        """
        查找相似的话题名称
        """
        # 提取目标话题的关键词
        target_parts = target_topic.lower().split('/')
        target_keywords = [part for part in target_parts if part]
        
        best_match = None
        best_score = 0
        
        for topic in available_topics:
            topic_parts = topic.lower().split('/')
            topic_keywords = [part for part in topic_parts if part]
            
            # 计算匹配分数
            score = 0
            for keyword in target_keywords:
                if any(keyword in topic_keyword for topic_keyword in topic_keywords):
                    score += 1
                    
            # 检查消息类型匹配
            if target_topic == self.camera_topic and 'image' in topic.lower():
                score += 2
            elif target_topic == self.imu_topic and 'imu' in topic.lower():
                score += 2
            elif target_topic == self.lidar_topic and ('scan' in topic.lower() or 'lidar' in topic.lower()):
                score += 2
            elif target_topic == self.odom_topic and 'odom' in topic.lower():
                score += 2
            elif target_topic == self.gps_topic and ('gps' in topic.lower() or 'fix' in topic.lower()):
                score += 2
                
            if score > best_score:
                best_score = score
                best_match = topic
                
        return best_match if best_score > 0 else None
    
    def start_playback(self):
        """开始播放bag数据"""
        if self.is_playing:
            logger.warning("Bag播放已在进行中")
            return
            
        if not self.bag_info:
            logger.error("无法开始播放：bag信息加载失败")
            return
            
        self.is_playing = True
        self.playback_thread = threading.Thread(target=self._playback_loop, daemon=True)
        self.playback_thread.start()
        logger.info("开始ROS Bag数据播放")
    
    def stop_playback(self):
        """停止播放"""
        self.is_playing = False
        if self.playback_thread and self.playback_thread.is_alive():
            self.playback_thread.join(timeout=1.0)
        logger.info("停止ROS Bag数据播放")
    
    def _playback_loop(self):
        """播放循环"""
        while self.is_playing:
            try:
                self._play_bag_once()
                if not self.loop_playback:
                    break
                logger.info("重新开始播放bag文件...")
                    
            except Exception as e:
                logger.error(f"播放bag文件时出错: {e}")
                break
                
        self.is_playing = False
    
    def _play_bag_once(self):
        """播放一次完整的bag文件"""
        try:
            with rosbag.Bag(self.bag_file_path, 'r') as bag:
                bag_start_time = bag.get_start_time()
                last_time = None
                
                # 确定播放时间范围
                start_time = bag_start_time + (self.start_time or 0)
                end_time = bag_start_time + (self.end_time or float('inf'))
                
                logger.info(f"播放时间范围: {self.start_time or 0:.2f}s - {self.end_time or 'end'}s")
                
                for topic, msg, t in bag.read_messages(
                    topics=[self.camera_topic, self.imu_topic, self.lidar_topic, 
                           self.odom_topic, self.gps_topic]):
                    
                    if not self.is_playing:
                        break
                        
                    # 检查时间范围
                    ros_time = t.to_sec()
                    if ros_time < start_time or ros_time > end_time:
                        continue
                    
                    # 处理消息
                    self._process_message(topic, msg)
                    
                    # 控制播放速率
                    if last_time is not None and self.playback_rate > 0:
                        time_diff = (ros_time - last_time) / self.playback_rate
                        if time_diff > 0:
                            time.sleep(time_diff)
                    
                    last_time = ros_time
                    
        except Exception as e:
            logger.error(f"读取bag文件时出错: {e}")
    
    def _process_message(self, topic: str, msg):
        """处理单个ROS消息"""
        with self.data_lock:
            try:
                if topic == self.camera_topic and hasattr(msg, 'encoding'):
                    # 处理图像消息
                    cv_image = self.cv_bridge.imgmsg_to_cv2(msg, "rgb8")
                    self.current_data['cam/image_array'] = cv_image
                    
                elif topic == self.imu_topic:
                    # 处理IMU消息
                    if hasattr(msg, 'linear_acceleration'):
                        self.current_data['imu/accel_x'] = msg.linear_acceleration.x
                        self.current_data['imu/accel_y'] = msg.linear_acceleration.y  
                        self.current_data['imu/accel_z'] = msg.linear_acceleration.z
                    if hasattr(msg, 'angular_velocity'):
                        self.current_data['imu/gyro_x'] = msg.angular_velocity.x
                        self.current_data['imu/gyro_y'] = msg.angular_velocity.y
                        self.current_data['imu/gyro_z'] = msg.angular_velocity.z
                        
                elif topic == self.lidar_topic:
                    # 处理激光雷达消息
                    if hasattr(msg, 'ranges'):
                        # 过滤无效距离值
                        ranges = []
                        for r in msg.ranges:
                            if np.isfinite(r) and r > msg.range_min and r < msg.range_max:
                                ranges.append(float(r))
                            else:
                                ranges.append(float('inf'))  # 用无穷大表示无效值
                        self.current_data['lidar/ranges'] = ranges
                        
                elif topic == self.odom_topic:
                    # 处理里程计消息
                    if hasattr(msg, 'pose') and hasattr(msg.pose, 'pose'):
                        pose = msg.pose.pose
                        self.current_data['pos/x'] = pose.position.x
                        self.current_data['pos/y'] = pose.position.y  
                        self.current_data['pos/z'] = pose.position.z
                        
                        # 转换四元数到欧拉角
                        from tf.transformations import euler_from_quaternion
                        quaternion = [pose.orientation.x, pose.orientation.y,
                                    pose.orientation.z, pose.orientation.w]
                        roll, pitch, yaw = euler_from_quaternion(quaternion)
                        self.current_data['pos/roll'] = roll
                        self.current_data['pos/pitch'] = pitch
                        self.current_data['pos/yaw'] = yaw
                        
                elif topic == self.gps_topic:
                    # 处理GPS消息
                    if hasattr(msg, 'latitude'):
                        self.current_data['gps/latitude'] = msg.latitude
                        self.current_data['gps/longitude'] = msg.longitude  
                        self.current_data['gps/altitude'] = msg.altitude
                        
            except Exception as e:
                logger.error(f"处理消息时出错 {topic}: {e}")
    
    def run(self) -> Dict[str, Any]:
        """
        获取当前传感器数据 - 与ROSToDonkeySensorBridge兼容的接口
        Returns:
            包含所有传感器数据的字典
        """
        with self.data_lock:
            return self.current_data.copy()
    
    def run_threaded(self) -> Dict[str, Any]:
        """线程安全的数据获取"""
        return self.run()
    
    def get_bag_status(self) -> Dict[str, Any]:
        """获取bag播放状态"""
        return {
            'is_playing': self.is_playing,
            'bag_file': self.bag_file_path,
            'loop_playback': self.loop_playback,
            'playback_rate': self.playback_rate,
            'has_bag_info': self.bag_info is not None
        }
    
    def set_playback_rate(self, rate: float):
        """设置播放速率"""
        self.playback_rate = max(0.1, rate)  # 最小0.1倍速
        logger.info(f"设置播放速率: {self.playback_rate}x")
    
    def shutdown(self):
        """清理资源"""
        self.stop_playback()
        logger.info("ROS Bag读取器已关闭")


class ROSBagSensorDataDistributor:
    """
    ROS Bag传感器数据分发器
    将ROSBagSensorReader的字典输出分发为独立的组件输出，与ROSSensorDataDistributor兼容
    """
    
    def __init__(self, bag_sensor_reader: ROSBagSensorReader):
        """
        初始化分发器
        Args:
            bag_sensor_reader: ROS bag传感器读取器实例
        """
        self.bag_reader = bag_sensor_reader
        
    def run(self):
        """
        从ROS bag读取器获取数据并分发为多个输出
        返回与ROSSensorDataDistributor相同格式的元组
        """
        bag_data = self.bag_reader.run()
        
        # 分发各种传感器数据 - 与ROSSensorDataDistributor完全相同的输出格式
        camera_image = bag_data.get('cam/image_array')
        imu_accel_x = bag_data.get('imu/accel_x', 0.0)
        imu_accel_y = bag_data.get('imu/accel_y', 0.0) 
        imu_accel_z = bag_data.get('imu/accel_z', 0.0)
        imu_gyro_x = bag_data.get('imu/gyro_x', 0.0)
        imu_gyro_y = bag_data.get('imu/gyro_y', 0.0)
        imu_gyro_z = bag_data.get('imu/gyro_z', 0.0)
        lidar_ranges = bag_data.get('lidar/ranges', [])
        pos_x = bag_data.get('pos/x', 0.0)
        pos_y = bag_data.get('pos/y', 0.0)
        pos_z = bag_data.get('pos/z', 0.0)
        pos_roll = bag_data.get('pos/roll', 0.0)
        pos_pitch = bag_data.get('pos/pitch', 0.0)
        pos_yaw = bag_data.get('pos/yaw', 0.0)
        gps_lat = bag_data.get('gps/latitude', 0.0)
        gps_lon = bag_data.get('gps/longitude', 0.0)
        gps_alt = bag_data.get('gps/altitude', 0.0)
        
        return (camera_image, 
                imu_accel_x, imu_accel_y, imu_accel_z,
                imu_gyro_x, imu_gyro_y, imu_gyro_z,
                lidar_ranges, 
                pos_x, pos_y, pos_z, pos_roll, pos_pitch, pos_yaw,
                gps_lat, gps_lon, gps_alt)
                
    def run_threaded(self):
        """线程安全版本"""
        return self.run()
    
    def start_playback(self):
        """开始播放bag数据"""
        self.bag_reader.start_playback()
        
    def stop_playback(self):
        """停止播放bag数据"""
        self.bag_reader.stop_playback()
        
    def get_status(self):
        """获取播放状态"""
        return self.bag_reader.get_bag_status()


# 向后兼容的别名
ROSBagReader = ROSBagSensorReader  # 简化的别名
ROSBagDistributor = ROSBagSensorDataDistributor  # 简化的别名


if __name__ == '__main__':
    """测试ROS Bag读取器"""
    import sys
    
    if len(sys.argv) < 2:
        print("用法: python rosbag_reader.py <bag_file_path> [camera_topic] [imu_topic] [lidar_topic] [odom_topic] [gps_topic]")
        print("示例: python rosbag_reader.py data.bag /usb_cam/image_raw")
        sys.exit(1)
    
    bag_file = sys.argv[1]
    
    # 允许通过命令行参数指定话题名称
    camera_topic = sys.argv[2] if len(sys.argv) > 2 else '/camera/image_raw'
    imu_topic = sys.argv[3] if len(sys.argv) > 3 else '/imu/data'
    lidar_topic = sys.argv[4] if len(sys.argv) > 4 else '/scan'
    odom_topic = sys.argv[5] if len(sys.argv) > 5 else '/odom'
    gps_topic = sys.argv[6] if len(sys.argv) > 6 else '/gps/fix'
    
    print(f"使用话题配置:")
    print(f"  摄像头: {camera_topic}")
    print(f"  IMU: {imu_topic}")
    print(f"  激光雷达: {lidar_topic}")
    print(f"  里程计: {odom_topic}")
    print(f"  GPS: {gps_topic}")
    print()
    
    # 创建读取器
    bag_reader = ROSBagSensorReader(
        bag_file,
        camera_topic=camera_topic,
        imu_topic=imu_topic,
        lidar_topic=lidar_topic,
        odom_topic=odom_topic,
        gps_topic=gps_topic
    )
    bag_distributor = ROSBagSensorDataDistributor(bag_reader)
    
    # 开始播放
    bag_distributor.start_playback()
    
    try:
        # 测试数据读取
        for i in range(100):
            data = bag_distributor.run()
            print(f"Frame {i}: Image shape: {data[0].shape if data[0] is not None else None}")
            print(f"  IMU: ax={data[1]:.3f}, ay={data[2]:.3f}, gz={data[6]:.3f}")
            print(f"  Lidar: {len(data[7])} ranges")
            print(f"  Position: x={data[8]:.3f}, y={data[9]:.3f}, yaw={data[13]:.3f}")
            time.sleep(0.1)
    
    except KeyboardInterrupt:
        print("停止测试")
    
    finally:
        bag_distributor.stop_playback()
        bag_reader.shutdown()