#!/usr/bin/env python3
"""
DonkeyCar ROS Hardware Bridge Node
将DonkeyCar ros_hardware 模板作为ROS节点运行

Usage:
    rosrun donkeycar_ros ros_hardware_bridge.py
    或通过launch文件启动:
    roslaunch donkeycar_ros donkey_ros_integrated.launch
"""

import rospy
import sys
import os
import threading
import signal
from std_msgs.msg import String, Bool

class DonkeyCarROSBridge:
    """DonkeyCar与ROS系统的桥接节点"""
    
    def __init__(self):
        # 初始化ROS节点
        rospy.init_node('donkeycar_bridge', anonymous=True)
        rospy.loginfo("🚀 DonkeyCar ROS Hardware Bridge starting...")
        
        # 获取ROS参数
        self.car_path = rospy.get_param('~car_path', os.path.expanduser('~/my_ros_car'))
        self.model_path = rospy.get_param('~model_path', '')
        self.model_type = rospy.get_param('~model_type', 'linear')
        
        # 数据源配置
        self.use_bag_file = rospy.get_param('~use_bag_file', False)
        self.bag_file_path = rospy.get_param('~bag_file_path', '')
        
        # ROS Topic配置
        self.camera_topic = rospy.get_param('~camera_topic', '/camera/image_raw')
        self.imu_topic = rospy.get_param('~imu_topic', '/imu/data')
        self.lidar_topic = rospy.get_param('~lidar_topic', '/scan')
        self.cmd_vel_topic = rospy.get_param('~cmd_vel_topic', '/cmd_vel')
        
        # 性能参数
        self.drive_loop_hz = rospy.get_param('~drive_loop_hz', 20)
        self.max_loops = rospy.get_param('~max_loops', 100000)
        
        # 验证车辆路径
        if not os.path.exists(self.car_path):
            rospy.logerr(f"❌ Car path not found: {self.car_path}")
            rospy.logerr("💡 Create car with: donkey createcar --template ros_hardware --path ~/my_ros_car")
            rospy.signal_shutdown("Car path not found")
            return
            
        # 切换到车辆目录
        os.chdir(self.car_path)
        
        # 动态导入DonkeyCar模块
        sys.path.insert(0, self.car_path)
        
        try:
            import donkeycar as dk
            from manage import drive, calibrate
            self.drive_func = drive
            self.calibrate_func = calibrate
        except ImportError as e:
            rospy.logerr(f"❌ Failed to import DonkeyCar modules: {e}")
            rospy.logerr(f"💡 Ensure {self.car_path} contains valid DonkeyCar ros_hardware setup")
            rospy.signal_shutdown("Import failed")
            return
            
        # 加载配置
        try:
            self.cfg = dk.load_config()
            rospy.loginfo("✅ DonkeyCar configuration loaded")
        except Exception as e:
            rospy.logerr(f"❌ Failed to load config: {e}")
            rospy.signal_shutdown("Config load failed")
            return
        
        # 根据ROS参数调整配置
        self._configure_from_ros_params()
        
        # 状态发布器
        self.status_pub = rospy.Publisher('/donkeycar/status', String, queue_size=1)
        self.info_pub = rospy.Publisher('/donkeycar/info', String, queue_size=1)
        
        # 系统信息
        self._log_system_info()
        
        # 启动DonkeyCar系统
        self.donkeycar_thread = None
        self.running = True
        
    def _configure_from_ros_params(self):
        """根据ROS参数动态配置DonkeyCar"""
        
        # 数据源配置
        if self.use_bag_file and self.bag_file_path:
            rospy.loginfo(f"📁 Using ROS bag: {self.bag_file_path}")
            self.cfg.CAMERA_TYPE = "ROS_BAG"
            self.cfg.ROS_BAG_FILE_PATH = self.bag_file_path
        else:
            rospy.loginfo("📡 Using live ROS topics")
            self.cfg.CAMERA_TYPE = "ROS_CAMERA"
        
        # Topic配置
        self.cfg.ROS_CAMERA_TOPIC = self.camera_topic
        self.cfg.ROS_IMU_TOPIC = self.imu_topic
        self.cfg.ROS_LIDAR_TOPIC = self.lidar_topic
        
        # 性能配置
        self.cfg.DRIVE_LOOP_HZ = self.drive_loop_hz
        self.cfg.MAX_LOOPS = self.max_loops
        
        # 强制启用ROS相关功能
        self.cfg.RECORD_ROS_SENSORS = True
        self.cfg.ROS_ENABLE_DIAGNOSTICS = True
        
    def _log_system_info(self):
        """记录系统配置信息"""
        rospy.loginfo("=" * 60)
        rospy.loginfo("🤖 DonkeyCar ROS Hardware Bridge Configuration")
        rospy.loginfo("=" * 60)
        rospy.loginfo(f"📂 Car Path: {self.car_path}")
        rospy.loginfo(f"🧠 Model: {self.model_path if self.model_path else 'None (Manual mode)'}")
        rospy.loginfo(f"📊 Data Source: {self.cfg.CAMERA_TYPE}")
        
        if self.use_bag_file:
            rospy.loginfo(f"📁 Bag File: {self.bag_file_path}")
        else:
            rospy.loginfo(f"📡 Camera Topic: {self.camera_topic}")
            rospy.loginfo(f"📡 IMU Topic: {self.imu_topic}")
            rospy.loginfo(f"📡 Lidar Topic: {self.lidar_topic}")
        
        rospy.loginfo(f"🎮 Control Output: {self.cmd_vel_topic}")
        rospy.loginfo(f"⚡ Loop Frequency: {self.drive_loop_hz} Hz")
        rospy.loginfo("=" * 60)
        
    def start_donkeycar(self):
        """在单独线程中启动DonkeyCar系统"""
        
        def run_donkeycar():
            try:
                rospy.loginfo("🚗 Starting DonkeyCar drive system...")
                
                # 发布启动状态
                self.status_pub.publish(String(data="STARTING"))
                
                # 启动DonkeyCar驱动系统
                self.drive_func(
                    cfg=self.cfg,
                    model_path=self.model_path if self.model_path else None,
                    model_type=self.model_type,
                    bag_path=self.bag_file_path if self.use_bag_file else None
                )
                
            except Exception as e:
                rospy.logerr(f"❌ DonkeyCar system error: {e}")
                self.status_pub.publish(String(data=f"ERROR: {e}"))
                self.running = False
            
        # 在单独线程中运行DonkeyCar
        self.donkeycar_thread = threading.Thread(target=run_donkeycar, daemon=True)
        self.donkeycar_thread.start()
        
        # 发布系统信息
        self.info_pub.publish(String(data="DonkeyCar ROS Bridge Active"))
        self.status_pub.publish(String(data="RUNNING"))
        
    def run(self):
        """主运行循环"""
        
        # 启动DonkeyCar
        self.start_donkeycar()
        
        # 状态监控循环
        rate = rospy.Rate(1)  # 1 Hz状态更新
        
        while not rospy.is_shutdown() and self.running:
            try:
                # 检查DonkeyCar线程状态
                if self.donkeycar_thread and not self.donkeycar_thread.is_alive():
                    rospy.logwarn("⚠️  DonkeyCar thread terminated")
                    self.status_pub.publish(String(data="STOPPED"))
                    break
                    
                # 发布心跳
                self.status_pub.publish(String(data="RUNNING"))
                
                rate.sleep()
                
            except KeyboardInterrupt:
                rospy.loginfo("🛑 Keyboard interrupt received")
                break
            except Exception as e:
                rospy.logerr(f"❌ Bridge error: {e}")
                break
        
        self.shutdown()
        
    def shutdown(self):
        """优雅关闭系统"""
        rospy.loginfo("🛑 Shutting down DonkeyCar ROS Bridge...")
        
        self.running = False
        self.status_pub.publish(String(data="SHUTTING_DOWN"))
        
        # 等待DonkeyCar线程结束
        if self.donkeycar_thread and self.donkeycar_thread.is_alive():
            rospy.loginfo("⏳ Waiting for DonkeyCar to shutdown...")
            self.donkeycar_thread.join(timeout=5.0)
            
        self.status_pub.publish(String(data="STOPPED"))
        rospy.loginfo("✅ DonkeyCar ROS Bridge shutdown complete")


def signal_handler(sig, frame):
    """处理系统信号"""
    rospy.loginfo("🛑 Received shutdown signal")
    rospy.signal_shutdown("Signal received")


if __name__ == '__main__':
    # 设置信号处理
    signal.signal(signal.SIGINT, signal_handler)
    signal.signal(signal.SIGTERM, signal_handler)
    
    try:
        # 创建并运行桥接节点
        bridge = DonkeyCarROSBridge()
        bridge.run()
        
    except rospy.ROSInterruptException:
        rospy.loginfo("🛑 ROS interrupt received")
    except Exception as e:
        rospy.logerr(f"❌ Bridge startup failed: {e}")
        rospy.logfatal("💡 Check car path and DonkeyCar installation")
    finally:
        rospy.loginfo("👋 DonkeyCar ROS Bridge terminated")