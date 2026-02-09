import rospy
import numpy as np
import time
import threading
from collections import deque
from std_msgs.msg import String, Int32, Float32, Bool, Header
from sensor_msgs.msg import Image, CompressedImage, Imu, NavSatFix, LaserScan
from geometry_msgs.msg import Twist, TwistWithCovariance, PoseWithCovariance, Pose
from nav_msgs.msg import Odometry
from cv_bridge import CvBridge
import logging

# Import ROS bag reading capabilities
try:
    from .rosbag_reader import ROSBagSensorReader, ROSBagSensorDataDistributor
    ROSBAG_AVAILABLE = True
except ImportError as e:
    logger.warning(f"ROS bag reading not available: {e}")
    ROSBAG_AVAILABLE = False


class ROSSensorDataDistributor:
    """
    通用传感器数据分发器 - 将传感器桥接器的字典输出分发为独立的组件输出
    Universal Sensor Data Distributor - Distributes sensor bridge dict output to individual component outputs
    
    支持多种传感器桥接器:
    - ROSToDonkeySensorBridge (实时ROS数据)
    - ROSBagSensorReader (bag文件数据)
    """
    
    def __init__(self, sensor_bridge):
        """
        初始化分发器
        Args:
            sensor_bridge: 传感器桥接器实例 (ROSToDonkeySensorBridge 或 ROSBagSensorReader)
        """
        self.sensor_bridge = sensor_bridge
        
    def run(self):
        """
        从传感器桥接器获取数据并分发为多个输出
        Get data from sensor bridge and distribute as multiple outputs
        
        Returns:
            tuple: (camera_image, 
                   imu_accel_x, imu_accel_y, imu_accel_z,
                   imu_gyro_x, imu_gyro_y, imu_gyro_z,
                   lidar_ranges, 
                   pos_x, pos_y, pos_z, pos_roll, pos_pitch, pos_yaw,
                   gps_lat, gps_lon, gps_alt)
        """
        sensor_data = self.sensor_bridge.run()
        
        # 分发各种传感器数据
        camera_image = sensor_data.get('cam/image_array')
        imu_accel_x = sensor_data.get('imu/accel_x', 0.0)
        imu_accel_y = sensor_data.get('imu/accel_y', 0.0) 
        imu_accel_z = sensor_data.get('imu/accel_z', 0.0)
        imu_gyro_x = sensor_data.get('imu/gyro_x', 0.0)
        imu_gyro_y = sensor_data.get('imu/gyro_y', 0.0)
        imu_gyro_z = sensor_data.get('imu/gyro_z', 0.0)
        lidar_ranges = sensor_data.get('lidar/ranges', [])
        pos_x = sensor_data.get('pos/x', 0.0)
        pos_y = sensor_data.get('pos/y', 0.0)
        pos_z = sensor_data.get('pos/z', 0.0)
        pos_roll = sensor_data.get('pos/roll', 0.0)
        pos_pitch = sensor_data.get('pos/pitch', 0.0)
        pos_yaw = sensor_data.get('pos/yaw', 0.0)
        gps_lat = sensor_data.get('gps/latitude', 0.0)
        gps_lon = sensor_data.get('gps/longitude', 0.0)
        gps_alt = sensor_data.get('gps/altitude', 0.0)
        
        return (camera_image, 
                imu_accel_x, imu_accel_y, imu_accel_z,
                imu_gyro_x, imu_gyro_y, imu_gyro_z,
                lidar_ranges, 
                pos_x, pos_y, pos_z, pos_roll, pos_pitch, pos_yaw,
                gps_lat, gps_lon, gps_alt)
                
    def run_threaded(self):
        """线程安全版本"""
        return self.run()

'''
sudo apt-get install python3-catkin-pkg
sudo apt-get install ros-noetic-cv-bridge
sudo apt-get install ros-noetic-geometry-msgs

ROS issues w python3:
https://discourse.ros.org/t/should-we-warn-new-users-about-difficulties-with-python-3-and-alternative-python-interpreters/3874/3

Extended DonkeyCar ROS interface with:
- Image data conversion
- Control command mapping
- Bidirectional data bridge
- Custom message types
- Synchronization support
'''

logger = logging.getLogger(__name__)

# DonkeyCar specific ROS message definitions
# These would typically be in separate .msg files in a ROS package
class DonkeyCarStatus:
    """Custom message type for DonkeyCar status information"""
    def __init__(self):
        self.header = Header()
        self.mode = ""           # user, local_angle, local
        self.recording = False
        self.pilot_active = False
        self.battery_voltage = 0.0
        self.loop_hz = 0.0
        
class DonkeyCarControl:
    """Custom message type for DonkeyCar control commands"""
    def __init__(self):
        self.header = Header()
        self.angle = 0.0        # -1.0 to 1.0
        self.throttle = 0.0     # -1.0 to 1.0
        self.mode = "user"      # user, local_angle, local
        self.enable_recording = False

class DataConverter:
    """Utility class for converting between DonkeyCar and ROS data formats"""
    
    def __init__(self):
        self.cv_bridge = CvBridge()

    # Part 1：Sensor Data Conversions
    # 1. Image Conversions    
    def donkey_image_to_ros(self, np_array, encoding='rgb8', frame_id="camera_link"):
        """Convert DonkeyCar numpy image array to ROS Image message"""
        try:
            # Ensure the image is in the correct format
            if len(np_array.shape) == 3:
                height, width, channels = np_array.shape
            else:
                height, width = np_array.shape
                channels = 1
                encoding = 'mono8'
                
            ros_image = Image()
            ros_image.header.stamp = rospy.Time.now()
            ros_image.header.frame_id = frame_id
            ros_image.height = height
            ros_image.width = width
            ros_image.encoding = encoding
            ros_image.is_bigendian = False
            ros_image.step = width * channels
            ros_image.data = np_array.tobytes()
            
            return ros_image
        except Exception as e:
            logger.error(f"Error converting donkey image to ROS: {e}")
            return None
    
    def ros_image_to_donkey(self, ros_image):
        """Convert ROS Image message to DonkeyCar numpy array"""
        try:
            cv_image = self.cv_bridge.imgmsg_to_cv2(ros_image, "rgb8")
            return np.array(cv_image)
        except Exception as e:
            logger.error(f"Error converting ROS image to donkey: {e}")
            return None
    
    
    # 2. IMU Conversions
    def donkey_imu_to_ros(self, gyro_x, gyro_y, gyro_z, accel_x, accel_y, accel_z, 
                         orientation_x=0, orientation_y=0, orientation_z=0, orientation_w=1,
                         frame_id="imu_link"):
        """Convert DonkeyCar IMU data to ROS Imu message"""
        imu_msg = Imu()
        imu_msg.header.stamp = rospy.Time.now()
        imu_msg.header.frame_id = frame_id
        
        # Orientation (quaternion)
        imu_msg.orientation.x = orientation_x
        imu_msg.orientation.y = orientation_y  
        imu_msg.orientation.z = orientation_z
        imu_msg.orientation.w = orientation_w
        
        # Angular velocity (rad/s)
        imu_msg.angular_velocity.x = gyro_x
        imu_msg.angular_velocity.y = gyro_y
        imu_msg.angular_velocity.z = gyro_z
        
        # Linear acceleration (m/s^2)
        imu_msg.linear_acceleration.x = accel_x
        imu_msg.linear_acceleration.y = accel_y
        imu_msg.linear_acceleration.z = accel_z
        
        return imu_msg
    
    def ros_imu_to_donkey(self, imu_msg):
        """Convert ROS Imu message to DonkeyCar IMU data"""
        return {
            'gyro_x': imu_msg.angular_velocity.x,
            'gyro_y': imu_msg.angular_velocity.y,
            'gyro_z': imu_msg.angular_velocity.z,
            'accel_x': imu_msg.linear_acceleration.x,
            'accel_y': imu_msg.linear_acceleration.y,
            'accel_z': imu_msg.linear_acceleration.z,
            'orientation_x': imu_msg.orientation.x,
            'orientation_y': imu_msg.orientation.y,
            'orientation_z': imu_msg.orientation.z,
            'orientation_w': imu_msg.orientation.w
        }
    

    # 3. Lidar Conversions
    def donkey_lidar_to_ros(self, ranges, angle_min=-3.14159, angle_max=3.14159, 
                           angle_increment=0.01745, range_min=0.1, range_max=30.0,
                           frame_id="laser_link"):
        """Convert DonkeyCar lidar data to ROS LaserScan message"""
        scan_msg = LaserScan()
        scan_msg.header.stamp = rospy.Time.now()
        scan_msg.header.frame_id = frame_id
        
        scan_msg.angle_min = angle_min          # Start angle (-pi)
        scan_msg.angle_max = angle_max          # End angle (pi)
        scan_msg.angle_increment = angle_increment  # Angular resolution (1 degree)
        scan_msg.time_increment = 0.0           # Time between measurements
        scan_msg.scan_time = 0.1                # Time for complete scan
        scan_msg.range_min = range_min          # Minimum range (0.1m)
        scan_msg.range_max = range_max          # Maximum range (30m)
        
        scan_msg.ranges = list(ranges)          # Range measurements
        scan_msg.intensities = []               # Optional intensity data
        
        return scan_msg
    
    def ros_lidar_to_donkey(self, scan_msg):
        """Convert ROS LaserScan message to DonkeyCar lidar data"""
        return {
            'ranges': np.array(scan_msg.ranges),
            'angle_min': scan_msg.angle_min,
            'angle_max': scan_msg.angle_max,
            'angle_increment': scan_msg.angle_increment,
            'range_min': scan_msg.range_min,
            'range_max': scan_msg.range_max
        }
    

    # 4. Odometry Conversions
    def donkey_odom_to_ros(self, x, y, z, roll, pitch, yaw, 
                          vel_x=0.0, vel_y=0.0, vel_angular=0.0,
                          frame_id="odom", child_frame_id="base_link"):
        """Convert DonkeyCar odometry data to ROS Odometry message"""
        odom_msg = Odometry()
        odom_msg.header.stamp = rospy.Time.now()
        odom_msg.header.frame_id = frame_id
        odom_msg.child_frame_id = child_frame_id
        
        # Position
        odom_msg.pose.pose.position.x = x
        odom_msg.pose.pose.position.y = y
        odom_msg.pose.pose.position.z = z
        
        # Orientation (convert from Euler to quaternion)
        try:
            from tf.transformations import quaternion_from_euler
            q = quaternion_from_euler(roll, pitch, yaw)
            odom_msg.pose.pose.orientation.x = q[0]
            odom_msg.pose.pose.orientation.y = q[1]
            odom_msg.pose.pose.orientation.z = q[2]
            odom_msg.pose.pose.orientation.w = q[3]
        except ImportError:
            logger.warning("tf.transformations not available, using identity quaternion")
            odom_msg.pose.pose.orientation.x = 0.0
            odom_msg.pose.pose.orientation.y = 0.0
            odom_msg.pose.pose.orientation.z = 0.0
            odom_msg.pose.pose.orientation.w = 1.0
        
        # Velocity
        odom_msg.twist.twist.linear.x = vel_x
        odom_msg.twist.twist.linear.y = vel_y
        odom_msg.twist.twist.angular.z = vel_angular
        
        return odom_msg
    
    def ros_odom_to_donkey(self, odom_msg):
        """Convert ROS Odometry message to DonkeyCar odometry data"""
        # Extract position
        x = odom_msg.pose.pose.position.x
        y = odom_msg.pose.pose.position.y
        z = odom_msg.pose.pose.position.z
        
        # Convert quaternion to Euler angles
        try:
            from tf.transformations import euler_from_quaternion
            q = [odom_msg.pose.pose.orientation.x,
                 odom_msg.pose.pose.orientation.y,
                 odom_msg.pose.pose.orientation.z,
                 odom_msg.pose.pose.orientation.w]
            roll, pitch, yaw = euler_from_quaternion(q)
        except ImportError:
            logger.warning("tf.transformations not available, using zero orientation")
            roll, pitch, yaw = 0.0, 0.0, 0.0
        
        return {
            'x': x, 'y': y, 'z': z,
            'roll': roll, 'pitch': pitch, 'yaw': yaw,
            'vel_x': odom_msg.twist.twist.linear.x,
            'vel_y': odom_msg.twist.twist.linear.y,
            'vel_angular': odom_msg.twist.twist.angular.z
        }
    

    # 5. GPS Conversions
    def donkey_gps_to_ros(self, latitude, longitude, altitude, frame_id="gps_link"):
        """Convert DonkeyCar GPS data to ROS NavSatFix message"""
        gps_msg = NavSatFix()
        gps_msg.header.stamp = rospy.Time.now()
        gps_msg.header.frame_id = frame_id
        
        gps_msg.latitude = latitude
        gps_msg.longitude = longitude
        gps_msg.altitude = altitude
        
        # Status (assume GPS fix is available)
        gps_msg.status.status = NavSatFix.STATUS_FIX
        gps_msg.status.service = NavSatFix.SERVICE_GPS
        
        return gps_msg
    
    def ros_gps_to_donkey(self, gps_msg):
        """Convert ROS NavSatFix message to DonkeyCar GPS data"""
        return {
            'latitude': gps_msg.latitude,
            'longitude': gps_msg.longitude,
            'altitude': gps_msg.altitude,
            'status': gps_msg.status.status
        }
    
    # Part 2: Control Command Conversions
    #  Control Command Conversions
    def donkey_control_to_twist(self, angle, throttle, max_speed=2.0, max_angular=1.0):
        """Convert DonkeyCar angle/throttle to ROS Twist message"""
        twist = Twist()
        # Map throttle (-1.0 to 1.0) to linear velocity
        twist.linear.x = throttle * max_speed
        twist.linear.y = 0.0
        twist.linear.z = 0.0
        
        # Map angle (-1.0 to 1.0) to angular velocity (left positive)
        twist.angular.x = 0.0
        twist.angular.y = 0.0
        twist.angular.z = angle * max_angular
        
        return twist
    
    def twist_to_donkey_control(self, twist, max_speed=2.0, max_angular=1.0):
        """Convert ROS Twist message to DonkeyCar angle/throttle"""
        # Clamp values to prevent division by zero
        throttle = twist.linear.x / max_speed if max_speed != 0 else 0.0
        angle = twist.angular.z / max_angular if max_angular != 0 else 0.0
        
        # Clamp to DonkeyCar range [-1.0, 1.0]
        throttle = max(-1.0, min(1.0, throttle))
        angle = max(-1.0, min(1.0, angle))
        
        return angle, throttle
        


class DonkeyToROSHardwareController(object):
    '''
    ROS-based hardware controller that replaces DonkeyCar's direct hardware control
    Sends control commands to ROS topics instead of directly controlling PWM
    '''
    
    def __init__(self, namespace='donkeycar', max_speed=2.0, max_angular=1.0):
        self.namespace = namespace
        self.converter = DataConverter()
        
        # Publishers for hardware control
        self.cmd_vel_pub = rospy.Publisher(f'/{namespace}/cmd_vel', Twist, queue_size=1)
        self.steering_pub = rospy.Publisher(f'/{namespace}/steering_angle', Float32, queue_size=1)
        self.throttle_pub = rospy.Publisher(f'/{namespace}/throttle', Float32, queue_size=1)
        
        # Configuration
        self.max_speed = max_speed
        self.max_angular = max_angular
        
        logger.info(f"ROS Hardware Controller initialized - namespace: {namespace}")
    
    def run(self, angle, throttle):
        """
        Send control commands to ROS instead of direct hardware control
        This replaces PWMSteering and PWMThrottle functionality
        """
        try:
            # Method 1: Send as cmd_vel (standard ROS robot control)
            twist = self.converter.donkey_control_to_twist(angle, throttle, 
                                                          self.max_speed, self.max_angular)
            self.cmd_vel_pub.publish(twist)
            
            # Method 2: Send individual steering/throttle commands (for compatibility)
            self.steering_pub.publish(Float32(data=angle))
            self.throttle_pub.publish(Float32(data=throttle))
            
        except Exception as e:
            logger.error(f"Error sending ROS control commands: {e}")


class ROSToDonkeySensorBridge(object):
    '''
    专用于ROS系统的传感器桥接器 - Dedicated sensor bridge for ROS systems
    
    适用场景 / Use Case: 纯ROS机器人运行DonkeyCar AI
    Your ROS robot → This bridge → DonkeyCar AI (training/inference)
    
    功能说明 / Functionality:
    - ROS传感器数据 → 转换为DonkeyCar格式 → 供AI训练/推理使用
    - ROS sensor data → Convert to DonkeyCar format → For AI training/inference
       
    数据流向 / Data Flow:
    ROS摄像头(/camera/image_raw) → DonkeyCar图像数组
    ROS IMU(/imu/data) → DonkeyCar IMU字典  
    ROS激光雷达(/scan) → DonkeyCar雷达数组
    ROS里程计(/odom) → DonkeyCar位置信息
    '''
    
    def __init__(self, node_name='donkeycar_ros_bridge', namespace='', 
                 camera_topic='/camera/image_raw',
                 imu_topic='/imu/data', 
                 lidar_topic='/scan',
                 odom_topic='/odom',
                 gps_topic='/gps/fix'):
        """
        初始化ROS传感器桥接器
        
        Args:
            namespace: ROS命名空间 (通常为空，使用标准ROS话题)
            camera_topic: ROS摄像头话题名 (默认: /camera/image_raw)
            imu_topic: ROS IMU话题名 (默认: /imu/data)
            lidar_topic: ROS激光雷达话题名 (默认: /scan)  
            odom_topic: ROS里程计话题名 (默认: /odom)
            gps_topic: ROS GPS话题名 (默认: /gps/fix)
        """
        self.node_name = node_name
        self.namespace = namespace
        self.converter = DataConverter()
        
        # Initialize ROS node
        try:
            rospy.init_node(node_name, anonymous=True)
            logger.info(f"Initialized ROS-to-DonkeyCar sensor bridge: {node_name}")
        except rospy.exceptions.ROSException:
            logger.debug("ROS node already initialized")
        
        # 订阅ROS传感器话题 - Subscribe to ROS sensor topics
        self.camera_sub = rospy.Subscriber(camera_topic, Image, self._camera_callback)
        self.imu_sub = rospy.Subscriber(imu_topic, Imu, self._imu_callback)
        self.lidar_sub = rospy.Subscriber(lidar_topic, LaserScan, self._lidar_callback)
        self.odom_sub = rospy.Subscriber(odom_topic, Odometry, self._odom_callback)
        self.gps_sub = rospy.Subscriber(gps_topic, NavSatFix, self._gps_callback)
        
        # DonkeyCar格式的传感器数据 - Sensor data in DonkeyCar format
        self.camera_image = None      # numpy array (120,160,3)
        self.imu_data = None          # dict with gyro/accel data
        self.lidar_data = None        # dict with ranges array
        self.odom_data = None         # dict with x,y,z,roll,pitch,yaw
        self.gps_data = None          # dict with lat,lon,alt
        
        # 数据同步和状态 - Data synchronization and status
        self.sensor_sync = BufferedDataSync()
        self.last_update_times = {
            'camera': 0, 'imu': 0, 'lidar': 0, 'odom': 0, 'gps': 0
        }
        
        logger.info("ROS-to-DonkeyCar Sensor Bridge initialized successfully")
        logger.info(f"Subscribed to: {camera_topic}, {imu_topic}, {lidar_topic}, {odom_topic}, {gps_topic}")
    
    def _camera_callback(self, image_msg):
        """处理ROS摄像头数据 → DonkeyCar图像数组 - Convert ROS camera to DonkeyCar image array"""
        try:
            self.camera_image = self.converter.ros_image_to_donkey(image_msg)
            self.last_update_times['camera'] = time.time()
        except Exception as e:
            logger.error(f"Camera conversion error: {e}")
    
    def _imu_callback(self, imu_msg):
        """处理ROS IMU数据 → DonkeyCar IMU字典 - Convert ROS IMU to DonkeyCar IMU dict"""
        try:
            self.imu_data = self.converter.ros_imu_to_donkey(imu_msg)
            self.last_update_times['imu'] = time.time()
        except Exception as e:
            logger.error(f"IMU conversion error: {e}")
    
    def _lidar_callback(self, scan_msg):
        """处理ROS激光雷达数据 → DonkeyCar雷达数组 - Convert ROS lidar to DonkeyCar lidar array"""
        try:
            self.lidar_data = self.converter.ros_lidar_to_donkey(scan_msg)
            self.last_update_times['lidar'] = time.time()
        except Exception as e:
            logger.error(f"Lidar conversion error: {e}")
    
    def _odom_callback(self, odom_msg):
        """处理ROS里程计数据 → DonkeyCar位置信息 - Convert ROS odometry to DonkeyCar position"""
        try:
            self.odom_data = self.converter.ros_odom_to_donkey(odom_msg)
            self.last_update_times['odom'] = time.time()
        except Exception as e:
            logger.error(f"Odometry conversion error: {e}")
            
    def _gps_callback(self, gps_msg):
        """处理ROS GPS数据 → DonkeyCar GPS信息 - Convert ROS GPS to DonkeyCar GPS info"""
        try:
            self.gps_data = self.converter.ros_gps_to_donkey(gps_msg)
            self.last_update_times['gps'] = time.time()
        except Exception as e:
            logger.error(f"GPS conversion error: {e}")
    
    def run(self):
        """
        获取转换后的ROS传感器数据 - Get converted ROS sensor data for DonkeyCar
        
        这是正确的数据流向 / This is the correct data flow:
        ROS传感器系统 → 自动转换 → DonkeyCar格式 → AI训练/推理
        ROS sensor system → Auto convert → DonkeyCar format → AI training/inference
            
        返回值 (Returns - DonkeyCar格式的传感器数据):
            dict: All sensor data in DonkeyCar format, ready for AI use
            {
                'cam/image_array': numpy.ndarray(120,160,3),  # 摄像头图像
                'imu/accel_x': float, 'imu/gyro_z': float,    # IMU数据  
                'lidar/ranges': numpy.ndarray,                # 激光雷达距离
                'pos/x': float, 'pos/y': float,               # 位置信息
                'gps/lat': float, 'gps/lon': float            # GPS坐标
            }
        """
        if rospy.is_shutdown():
            logger.warning("ROS is shutdown, returning empty data")
            return {}
            
        try:
            # 构建DonkeyCar格式的传感器数据字典
            # Build DonkeyCar format sensor data dictionary
            sensor_data = {}
            
            # 摄像头数据 - Camera data  
            if self.camera_image is not None:
                sensor_data['cam/image_array'] = self.camera_image
                
            # IMU数据 - IMU data
            if self.imu_data is not None:
                sensor_data.update({
                    'imu/accel_x': self.imu_data.get('accel_x', 0.0),
                    'imu/accel_y': self.imu_data.get('accel_y', 0.0), 
                    'imu/accel_z': self.imu_data.get('accel_z', 0.0),
                    'imu/gyro_x': self.imu_data.get('gyro_x', 0.0),
                    'imu/gyro_y': self.imu_data.get('gyro_y', 0.0),
                    'imu/gyro_z': self.imu_data.get('gyro_z', 0.0)
                })
                
            # 激光雷达数据 - Lidar data
            if self.lidar_data is not None:
                sensor_data['lidar/ranges'] = self.lidar_data.get('ranges')
                
            # 位置数据 - Position data
            if self.odom_data is not None:
                sensor_data.update({
                    'pos/x': self.odom_data.get('x', 0.0),
                    'pos/y': self.odom_data.get('y', 0.0),
                    'pos/z': self.odom_data.get('z', 0.0),
                    'pos/roll': self.odom_data.get('roll', 0.0),
                    'pos/pitch': self.odom_data.get('pitch', 0.0), 
                    'pos/yaw': self.odom_data.get('yaw', 0.0)
                })
                
            # GPS数据 - GPS data
            if self.gps_data is not None:
                sensor_data.update({
                    'gps/latitude': self.gps_data.get('latitude', 0.0),
                    'gps/longitude': self.gps_data.get('longitude', 0.0),
                    'gps/altitude': self.gps_data.get('altitude', 0.0)
                })
                
            return sensor_data
            
        except Exception as e:
            logger.error(f"Error in ROS-to-DonkeyCar sensor bridge: {e}")
            return {}
    
    def run_threaded(self):
        """线程安全的数据访问 - Thread-safe data access"""
        return self.run()
        
    def get_data_freshness(self):
        """检查数据新鲜度 - Check data freshness"""
        current_time = time.time()
        freshness = {}
        
        for sensor, last_time in self.last_update_times.items():
            age = current_time - last_time if last_time > 0 else float('inf')
            freshness[sensor] = {
                'age_seconds': age,
                'is_fresh': age < 1.0,  # 数据1秒内为新鲜
                'last_update': last_time
            }
            
        return freshness
        
    def get_status(self):
        """获取桥接器状态信息 - Get bridge status info"""
        freshness = self.get_data_freshness()
        
        status = {
            'active_sensors': [sensor for sensor, info in freshness.items() if info['is_fresh']],
            'total_sensors': len(self.last_update_times),
            'data_freshness': freshness,
            'ros_connected': not rospy.is_shutdown()
        }
        
        return status


# 为了向后兼容，保留原来的双向桥接类
# Backward compatibility: keep the original bidirectional bridge class
# ⚠️  DonkeyCarROSBridge 已废弃 - DonkeyCarROSBridge is DEPRECATED
# 推荐使用 ROSToDonkeySensorBridge + DonkeyToROSHardwareController
# Use ROSToDonkeySensorBridge + DonkeyToROSHardwareController instead

# REMOVED: DonkeyCarROSBridge class (720+ lines)
# 原因 / Reason: 双向桥接已被专业的单向组件替代
#                Bidirectional bridge replaced by specialized unidirectional components
# 迁移 / Migration:
#   OLD: DonkeyCarROSBridge(namespace='robot')
#   NEW: ROSToDonkeySensorBridge() + DonkeyToROSHardwareController(namespace='robot')


class BufferedDataSync:
    """Thread-safe data synchronization with buffering"""
    
    def __init__(self, max_buffer_size=10):
        self.max_buffer_size = max_buffer_size
        self.data_buffer = deque(maxlen=max_buffer_size)
        self.lock = threading.Lock()
        self.latest_data = None
        self.last_update_time = 0.0
        
    def put_data(self, data):
        """Store data in buffer with timestamp"""
        with self.lock:
            timestamp = time.time()
            self.data_buffer.append((timestamp, data))
            self.latest_data = data
            self.last_update_time = timestamp
    
    def get_latest_data(self, timeout=0.1):
        """Get the most recent data, None if too old"""
        with self.lock:
            if self.latest_data is None:
                return None
            
            if time.time() - self.last_update_time > timeout:
                return None
                
            return self.latest_data
    
    def get_data_at_time(self, target_time, tolerance=0.05):
        """Get data closest to target timestamp"""
        with self.lock:
            if not self.data_buffer:
                return None
                
            best_match = None
            best_diff = float('inf')
            
            for timestamp, data in self.data_buffer:
                diff = abs(timestamp - target_time)
                if diff < best_diff and diff <= tolerance:
                    best_diff = diff
                    best_match = data
                    
            return best_match


class RosPublisher(object):
    '''
    Enhanced ROS node to publish various data types with automatic conversion
    '''
    def __init__(self, node_name, channel_name, stream_type=String, 
                 anonymous=True, queue_size=10, data_converter=None):
        self.data = None
        self.last_published_data = None
        self.pub = rospy.Publisher(channel_name, stream_type, queue_size=queue_size)
        
        # Initialize node only if not already initialized
        try:
            rospy.init_node(node_name, anonymous=anonymous)
        except rospy.exceptions.ROSException:
            logger.debug(f"ROS node already initialized, using existing node")
            
        self.stream_type = stream_type
        self.converter = data_converter or DataConverter()
        self.publish_count = 0
        
        logger.info(f"ROS Publisher initialized: {channel_name} ({stream_type})")

    def run(self, data):
        '''
        Enhanced run method with data type detection and conversion
        '''
        if data is None or rospy.is_shutdown():
            return
            
        # Only publish when data changes (for efficiency)
        if np.array_equal(data, self.last_published_data) if isinstance(data, np.ndarray) else data == self.last_published_data:
            return
            
        try:
            # Convert data based on target ROS message type
            ros_message = self._convert_to_ros_message(data)
            
            if ros_message is not None:
                self.pub.publish(ros_message)
                self.last_published_data = data
                self.publish_count += 1
                
                if self.publish_count % 100 == 0:
                    logger.debug(f"Published {self.publish_count} messages")
                    
        except Exception as e:
            logger.error(f"Error publishing ROS message: {e}")
    
    def _convert_to_ros_message(self, data):
        """Convert DonkeyCar data to appropriate ROS message"""
        if self.stream_type == Image and isinstance(data, np.ndarray):
            return self.converter.donkey_image_to_ros(data)
        elif self.stream_type == Float32:
            return Float32(data=float(data))
        elif self.stream_type == String:
            return String(data=str(data))
        elif self.stream_type == Bool:
            return Bool(data=bool(data))
        elif self.stream_type == Int32:
            return Int32(data=int(data))
        else:
            logger.warning(f"Unsupported stream type: {self.stream_type}")
            return data
    

class RosSubscriber(object):
    '''
    Enhanced ROS node to subscribe to various data types with automatic conversion
    '''

    def __init__(self, node_name, channel_name, stream_type=String, 
                 anonymous=True, data_converter=None, buffer_size=10):
        self.data = None
        self.raw_data = None
        self.message_count = 0
        self.converter = data_converter or DataConverter()
        self.data_sync = BufferedDataSync(buffer_size)
        
        # Initialize node only if not already initialized
        try:
            rospy.init_node(node_name, anonymous=anonymous)
        except rospy.exceptions.ROSException:
            logger.debug(f"ROS node already initialized, using existing node")
            
        self.sub = rospy.Subscriber(channel_name, stream_type, self.on_data_recv)
        self.stream_type = stream_type
        
        logger.info(f"ROS Subscriber initialized: {channel_name} ({stream_type})")

    def on_data_recv(self, ros_message):
        """Enhanced callback with data conversion and buffering"""
        try:
            # Convert ROS message to DonkeyCar format
            converted_data = self._convert_from_ros_message(ros_message)
            
            if converted_data is not None:
                self.raw_data = ros_message
                self.data = converted_data
                self.data_sync.put_data(converted_data)
                self.message_count += 1
                
                if self.message_count % 100 == 0:
                    logger.debug(f"Received {self.message_count} messages")
                    
        except Exception as e:
            logger.error(f"Error processing ROS message: {e}")

    def _convert_from_ros_message(self, ros_message):
        """Convert ROS message to DonkeyCar format"""
        if self.stream_type == Image:
            return self.converter.ros_image_to_donkey(ros_message)
        elif hasattr(ros_message, 'data'):
            return ros_message.data
        else:
            return ros_message

    def run(self):
        """Get latest data with timeout protection"""
        return self.data_sync.get_latest_data()
    
    def run_threaded(self):
        """Thread-safe data access"""
        return self.run()


# class DonkeyCarROSBridge(object):
#     '''
#     ⚠️  已废弃 - DEPRECATED ⚠️ 
    
#     原双向桥接器已被专业组件替代:
#     Original bidirectional bridge replaced by specialized components:
    
#     迁移指南 / Migration Guide:
#     OLD: DonkeyCarROSBridge(namespace='robot')
#     NEW: ROSToDonkeySensorBridge(camera_topic='/camera/image_raw') + 
#          DonkeyToROSHardwareController(namespace='robot')
         
#     原因 / Reason: 双向设计复杂且容易混淆，专业组件更清晰
#                   Bidirectional design was complex and confusing, specialized components are clearer
#     '''
    
#     def __init__(self, node_name='donkeycar_bridge', namespace='donkeycar'):
#         logger.warning("⚠️  DonkeyCarROSBridge is DEPRECATED! Use ROSToDonkeySensorBridge + DonkeyToROSHardwareController instead")
#         logger.warning("Migration: https://docs.donkeycar.com/guide/ros_migration/")
        
#         # 为向后兼容保留基本初始化
#         self.node_name = node_name
#         self.namespace = namespace
#         self.converter = DataConverter()
#         self.running = False
    
#     def _cmd_vel_callback(self, twist_msg):
#         """向后兼容存根 - Backward compatibility stub"""
#         logger.warning("DonkeyCarROSBridge is deprecated, use ROSDriverInterface for ROS input")
    
#     def _mode_callback(self, mode_msg):
#         """向后兼容存根 - Backward compatibility stub"""
#         pass
        
#     def run(self, *args, **kwargs):
#         """向后兼容存根 - Backward compatibility stub"""
#         logger.error("DonkeyCarROSBridge is deprecated! Use:")
#         logger.error("  - ROSToDonkeySensorBridge for sensor input")
#         logger.error("  - DonkeyToROSHardwareController for control output")
#         return 0.0, 0.0, "user"
    
#     def _publish_donkey_data(self, *args, **kwargs):
#         """向后兼容存根 - Backward compatibility stub"""
#         pass
    
#     def shutdown(self):
#         """Clean shutdown of the bridge"""
#         logger.info("Shutting down deprecated DonkeyCar ROS Bridge")
#         self.running = False


class ROSDriverInterface(object):
    '''
    High-level interface for using ROS as a driver input in DonkeyCar
    This can be used as a drop-in replacement for joystick or web controller
    '''
    
    def __init__(self, namespace='donkeycar', max_age=0.5):
        self.namespace = namespace
        self.max_age = max_age  # Maximum age of ROS commands in seconds
        self.converter = DataConverter()
        
        # Initialize subscribers for control input
        self.cmd_vel_sub = rospy.Subscriber(f'{namespace}/cmd_vel', Twist, self._cmd_vel_callback)
        self.mode_sub = rospy.Subscriber(f'{namespace}/mode', String, self._mode_callback)
        
        # State
        self.angle = 0.0
        self.throttle = 0.0
        self.mode = "user"
        self.last_update = time.time()
        
        logger.info(f"ROS Driver Interface initialized on namespace: {namespace}")
    
    def _cmd_vel_callback(self, twist_msg):
        """Handle ROS velocity commands"""
        self.angle, self.throttle = self.converter.twist_to_donkey_control(twist_msg)
        self.last_update = time.time()
    
    def _mode_callback(self, mode_msg):
        """Handle mode changes"""
        self.mode = mode_msg.data
        
    def run_threaded(self):
        """Return current control state"""
        # Check if data is too old
        if time.time() - self.last_update > self.max_age:
            return 0.0, 0.0, "user"
            
        return self.angle, self.throttle, self.mode
    
    def run(self):
        """Non-threaded version"""
        return self.run_threaded()


# Convenience functions for easy integration
def create_ros_sensor_bridge(camera_topic='/camera/image_raw', 
                           imu_topic='/imu/data',
                           lidar_topic='/scan', 
                           odom_topic='/odom'):
    """Factory function to create ROS sensor bridge for ROS systems"""
    return ROSToDonkeySensorBridge(camera_topic=camera_topic,
                                  imu_topic=imu_topic,
                                  lidar_topic=lidar_topic, 
                                  odom_topic=odom_topic)

def create_simple_ros_bridge(namespace='donkeycar'):
    """⚠️ DEPRECATED - Factory function to create a basic ROS bridge (legacy)"""
    logger.warning("create_simple_ros_bridge() is deprecated!")
    logger.warning("Use: ROSToDonkeySensorBridge() + DonkeyToROSHardwareController() instead")
    return DonkeyCarROSBridge(namespace=namespace)

def create_ros_camera_publisher(topic_name='/donkeycar/camera/image_raw'):
    """Factory function to create a camera publisher"""
    return RosPublisher('camera_pub', topic_name, Image)

def create_ros_control_subscriber(topic_name='/donkeycar/cmd_vel'):
    """Factory function to create a control subscriber"""
    return RosSubscriber('control_sub', topic_name, Twist)


# Testing and validation functions
class ROSBridgeTest:
    """Test suite for ROS bridge functionality"""
    
    def __init__(self):
        self.converter = DataConverter()
        self.test_results = {}
    
    def test_image_conversion(self):
        """Test image data conversion roundtrip"""
        try:
            # Create test image
            test_image = np.random.randint(0, 255, (120, 160, 3), dtype=np.uint8)
            
            # Convert to ROS and back
            ros_image = self.converter.donkey_image_to_ros(test_image)
            converted_back = self.converter.ros_image_to_donkey(ros_image)
            
            # Check if conversion preserved data
            success = np.array_equal(test_image, converted_back)
            self.test_results['image_conversion'] = success
            logger.info(f"Image conversion test: {'PASS' if success else 'FAIL'}")
            
        except Exception as e:
            logger.error(f"Image conversion test failed: {e}")
            self.test_results['image_conversion'] = False
    
    def test_control_conversion(self):
        """Test control data conversion roundtrip"""
        try:
            # Test various control values
            test_cases = [
                (-1.0, -1.0),  # Full left, full reverse
                (0.0, 0.0),    # Center, stopped
                (1.0, 1.0),    # Full right, full forward
                (0.5, -0.3),   # Slight right, slight reverse
            ]
            
            success = True
            for angle, throttle in test_cases:
                twist = self.converter.donkey_control_to_twist(angle, throttle)
                angle_back, throttle_back = self.converter.twist_to_donkey_control(twist)
                
                if abs(angle - angle_back) > 0.01 or abs(throttle - throttle_back) > 0.01:
                    success = False
                    break
            
            self.test_results['control_conversion'] = success
            logger.info(f"Control conversion test: {'PASS' if success else 'FAIL'}")
            
        except Exception as e:
            logger.error(f"Control conversion test failed: {e}")
            self.test_results['control_conversion'] = False
    
    def run_all_tests(self):
        """Run complete test suite"""
        logger.info("Running ROS Bridge test suite...")
        self.test_image_conversion()
        self.test_control_conversion()
        
        passed = sum(1 for result in self.test_results.values() if result)
        total = len(self.test_results)
        
        logger.info(f"Test results: {passed}/{total} tests passed")
        return passed == total


# Backward compatibility aliases
ROSHardwareController = DonkeyToROSHardwareController  # Backward compatibility
ROSMultiSensorBridge = ROSToDonkeySensorBridge        # Backward compatibility  
RosPubisher = RosPublisher  # Fix original typo while maintaining compatibility


# Usage examples and integration guide
"""
=== DonkeyCar ROS Integration Usage Examples ===

1. ROS as Final Hardware Controller (RECOMMENDED for ROS robots):
```python
# In your DonkeyCar template - replaces PWM hardware control
from donkeycar.parts.ros import DonkeyToROSHardwareController

# Instead of PCA9685, PWMSteering, PWMThrottle
ros_hardware = DonkeyToROSHardwareController(namespace='mycar')
car.add(ros_hardware, inputs=['final/angle', 'final/throttle'])
# Commands sent to: /mycar/cmd_vel, /mycar/steering_angle, /mycar/throttle
```

2. ROS系统集成 - ROS System Integration (推荐 / RECOMMENDED):
```python
# 专用于ROS系统的传感器桥接器 - Dedicated ROS system sensor bridge
from donkeycar.parts.ros import ROSToDonkeySensorBridge, DonkeyToROSHardwareController

# 传感器层(从ros获取数据) - Sensor layer (get data from ROS)
sensor_bridge = ROSToDonkeySensorBridge(
    camera_topic='/camera/image_raw',
    imu_topic='/imu/data', 
    lidar_topic='/scan',
    odom_topic='/odom'
)
car.add(sensor_bridge, 
        outputs=['cam/image_array', 'imu/gyro_z', 'lidar/ranges', 'pos/x', 'pos/y'])

# AI层(DonkeyCar逻辑) - AI layer (DonkeyCar logic)
car.add(KerasLinear(), 
        inputs=['cam/image_array'],
        outputs=['pilot/angle', 'pilot/throttle'])

# 决策层(DonkeyCar逻辑) - Decision layer (DonkeyCar logic)
car.add(DriveMode(cfg), 
        inputs=['user/mode', 'user/angle', 'user/throttle', 'pilot/angle'],  
        outputs=['final/angle', 'final/throttle'])

# 控制层(发送到ROS) - Control layer (send to ROS)
ros_hardware = DonkeyToROSHardwareController(namespace='robot')
car.add(ros_hardware, inputs=['final/angle', 'final/throttle'])

# 数据流向 / Data Flow:
# ROS传感器 → DonkeyCar AI处理 → ROS电机控制
# ROS sensors → DonkeyCar AI processing → ROS motor control
```

3. 迁移指南 - Migration from deprecated DonkeyCarROSBridge:
```python
# ❌ 旧的废弃方式 - OLD (deprecated)
from donkeycar.parts.ros import DonkeyCarROSBridge
ros_bridge = DonkeyCarROSBridge(namespace='robot')
car.add(ros_bridge, inputs=[...], outputs=[...])

# ✅ 新的推荐方式 - NEW (recommended)  
from donkeycar.parts.ros import ROSToDonkeySensorBridge, DonkeyToROSHardwareController

# 传感器输入
sensor_bridge = ROSToDonkeySensorBridge()
car.add(sensor_bridge, outputs=['cam/image_array', 'imu/gyro_z'])

# 硬件控制输出
ros_hardware = DonkeyToROSHardwareController(namespace='robot')
car.add(ros_hardware, inputs=['final/angle', 'final/throttle'])
```
```python
# Comprehensive sensor bridge
from donkeycar.parts.ros import ROSMultiSensorBridge

sensor_bridge = ROSMultiSensorBridge(namespace='mycar')
car.add(sensor_bridge,
        inputs=['cam/image_array', 'imu/data', 'lidar/scan', 'gps/data'],
        outputs=['ros_sensors/status'])

# Published topics:
# /mycar/camera/image_raw (sensor_msgs/Image)
# /mycar/imu/data (sensor_msgs/Imu)  
# /mycar/scan (sensor_msgs/LaserScan)
# /mycar/gps/fix (sensor_msgs/NavSatFix)
# /mycar/odom (nav_msgs/Odometry)
```

4. 向后兼容的遗留集成 - Legacy Integration (不推荐 / DEPRECATED):
```python
# ⚠️ 以下方式已废弃，仅为向后兼容保留
# The following is deprecated, kept for backward compatibility only
from donkeycar.parts.ros import DonkeyCarROSBridge  # DEPRECATED!

# 请迁移到新的专业组件 - Please migrate to new specialized components
sensor_bridge = ROSToDonkeySensorBridge()
ros_hardware = DonkeyToROSHardwareController()
```

5. ROS Command Line Control:
```bash
# Control the robot via ROS
rostopic pub /donkeycar/cmd_vel geometry_msgs/Twist \\
  "linear: {x: 1.0} angular: {z: 0.5}"

# Monitor sensor data
rostopic echo /donkeycar/camera/image_raw
rostopic echo /donkeycar/imu/data
rostopic echo /donkeycar/scan

# External camera input
rosrun usb_cam usb_cam_node _video_device:=/dev/video0 \\
  _image_width:=160 _image_height:=120 \\
  _camera_frame_id:=camera_link _io_method:=mmap

# View camera with RViz  
rosrun rviz rviz -d donkeycar_view.rviz
```

6. ROS Launch Example for Complete System:
```xml
<launch>
    <arg name="namespace" default="donkeycar"/>
    
    <!-- DonkeyCar ROS Bridge -->
    <node name="donkeycar_node" pkg="donkeycar" type="ros_hardware_controlled.py" 
          output="screen" cwd="~/mycar"/>
    
    <!-- Hardware Controllers (replace DonkeyCar PWM control) -->
    <node name="motor_controller" pkg="my_robot_pkg" type="motor_driver_node">
        <remap from="cmd_vel" to="/$(arg namespace)/cmd_vel"/>
    </node>
    
    <!-- Sensors -->
    <node name="usb_cam" pkg="usb_cam" type="usb_cam_node">
        <remap from="image_raw" to="/$(arg namespace)/usb_cam/image_raw"/>
    </node>
    
    <node name="imu_driver" pkg="imu_pkg" type="imu_node">
        <remap from="imu/data" to="/$(arg namespace)/external_imu/data"/>
    </node>
    
    <node name="lidar_driver" pkg="rplidar_ros" type="rplidarNode">
        <remap from="scan" to="/$(arg namespace)/external_scan"/>
    </node>
</launch>
```

=== Architecture Comparison ===

**Traditional DonkeyCar (Direct Hardware Control):**
```
DonkeyCar → PCA9685/PWM → Servos/Motors → Robot Movement
```

**ROS-Controlled DonkeyCar (Recommended for ROS Robots):**  
```
DonkeyCar Decision → ROS Topics → ROS Hardware Nodes → Robot Movement
```

=== Supported ROS Interfaces ===

**Input Topics (ROS → DonkeyCar):**
- `/namespace/usb_cam/image_raw` (sensor_msgs/Image)
- `/namespace/external_imu/data` (sensor_msgs/Imu)  
- `/namespace/external_scan` (sensor_msgs/LaserScan)
- `/namespace/external_odom` (nav_msgs/Odometry)
- `/namespace/cmd_vel_input` (geometry_msgs/Twist)

**Output Topics (DonkeyCar → ROS):**
- `/namespace/camera/image_raw` (sensor_msgs/Image)
- `/namespace/cmd_vel` (geometry_msgs/Twist) 
- `/namespace/steering_angle` (std_msgs/Float32)
- `/namespace/throttle` (std_msgs/Float32)
- `/namespace/imu/data` (sensor_msgs/Imu)
- `/namespace/scan` (sensor_msgs/LaserScan)
- `/namespace/gps/fix` (sensor_msgs/NavSatFix)
- `/namespace/odom` (nav_msgs/Odometry)
- `/namespace/mode` (std_msgs/String)
- `/namespace/recording` (std_msgs/Bool)

=== Data Type Conversion Examples ===

**Camera Image:**
```
numpy(120,160,3) ↔ sensor_msgs/Image(height=120, width=160, encoding='rgb8')
```

**IMU Data:**
```  
{'gyro_x': 0.1, 'accel_x': 9.8} ↔ sensor_msgs/Imu(angular_velocity.x=0.1, linear_acceleration.x=9.8)
```

**Lidar Data:**
```
{'ranges': [0.5, 1.2, ...]} ↔ sensor_msgs/LaserScan(ranges=[0.5, 1.2, ...])
```

**Control Commands:**
```
angle=0.3, throttle=0.5 ↔ geometry_msgs/Twist(linear.x=1.0, angular.z=0.3)
```

=== Configuration Requirements ===

**ROS Dependencies:**
```bash
sudo apt install ros-noetic-cv-bridge ros-noetic-tf ros-noetic-geometry-msgs
sudo apt install ros-noetic-sensor-msgs ros-noetic-nav-msgs
```

**DonkeyCar Config:**
```python
# Enable ROS hardware control mode
USE_ROS_HARDWARE_CONTROL = True
ROS_NAMESPACE = "myrobot"  
ROS_MAX_SPEED_MS = 2.0
ROS_MAX_ANGULAR_RADS = 1.0

# Sensor integration
USE_ROS_IMU = True
USE_ROS_LIDAR = True
CAMERA_TYPE = "ROS_USB_CAM"  # Use ROS camera input
```
"""

if __name__ == "__main__":
    # Quick test when run directly
    logger.info("Testing DonkeyCar ROS Bridge components...")
    
    # Initialize ROS 
    try:
        import rospy
        rospy.init_node('donkeycar_test', anonymous=True)
        
        # Run test suite
        test_suite = ROSBridgeTest()
        if test_suite.run_all_tests():
            logger.info("All tests passed! ROS bridge is ready for use.")
        else:
            logger.warning("Some tests failed. Check ROS setup and dependencies.")
            
    except ImportError:
        logger.error("ROS not available. Install ROS and source setup.bash")
    except Exception as e:
        logger.error(f"Test failed: {e}")

