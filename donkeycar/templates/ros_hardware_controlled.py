#!/usr/bin/env python3
"""
DonkeyCar template with ROS as final hardware controller
This template shows how to use ROS as the final hardware control layer
while keeping DonkeyCar as the decision-making core.

Architecture: DonkeyCar Decision → ROS Bridge → ROS Hardware Control → Physical Robot

Usage:
    manage.py drive [--model=<model>] [--type=(linear|categorical|...)]
    manage.py calibrate
"""

from docopt import docopt
import logging
import os

import donkeycar as dk
from donkeycar.parts.tub_v2 import TubWriter
from donkeycar.parts.datastore import TubHandler
from donkeycar.parts.controller import LocalWebController
# Note: We DON'T import PCA9685, PWMSteering, PWMThrottle since ROS controls hardware
from donkeycar.pipeline.augmentations import ImageAugmentation

# Import enhanced ROS components
try:
    from donkeycar.parts.ros import (
        DonkeyToROSHardwareController,  # Donkey-to-ROS-based hardware control  
        ROSToDonkeySensorBridge,        # ROS-to-DonkeyCar sensor bridge
        ROSDriverInterface              # ROS as input driver
    )
    from sensor_msgs.msg import Image, Imu, LaserScan
    from geometry_msgs.msg import Twist
    from nav_msgs.msg import Odometry
    from std_msgs.msg import String, Bool, Float32
    ROS_AVAILABLE = True
except ImportError as e:
    print(f"ROS not available: {e}")
    ROS_AVAILABLE = False

logger = logging.getLogger(__name__)
logging.basicConfig(level=logging.INFO)


class ROSSensorDataDistributor:
    """
    ROS传感器数据分发器 - 将ROSToDonkeySensorBridge的字典输出分发为独立的组件输出
    ROS Sensor Data Distributor - Distributes ROSToDonkeySensorBridge dict output to individual component outputs
    """
    
    def __init__(self, sensor_bridge):
        self.sensor_bridge = sensor_bridge
        
    def run(self):
        """
        从ROS传感器桥接器获取数据并分发为多个输出
        Get data from ROS sensor bridge and distribute as multiple outputs
        """
        ros_data = self.sensor_bridge.run()
        
        # 分发各种传感器数据
        camera_image = ros_data.get('cam/image_array')
        imu_accel_x = ros_data.get('imu/accel_x', 0.0)
        imu_accel_y = ros_data.get('imu/accel_y', 0.0) 
        imu_accel_z = ros_data.get('imu/accel_z', 0.0)
        imu_gyro_x = ros_data.get('imu/gyro_x', 0.0)
        imu_gyro_y = ros_data.get('imu/gyro_y', 0.0)
        imu_gyro_z = ros_data.get('imu/gyro_z', 0.0)
        lidar_ranges = ros_data.get('lidar/ranges', [])
        pos_x = ros_data.get('pos/x', 0.0)
        pos_y = ros_data.get('pos/y', 0.0)
        pos_z = ros_data.get('pos/z', 0.0)
        pos_roll = ros_data.get('pos/roll', 0.0)
        pos_pitch = ros_data.get('pos/pitch', 0.0)
        pos_yaw = ros_data.get('pos/yaw', 0.0)
        gps_lat = ros_data.get('gps/latitude', 0.0)
        gps_lon = ros_data.get('gps/longitude', 0.0)
        gps_alt = ros_data.get('gps/altitude', 0.0)
        
        return (camera_image, 
                imu_accel_x, imu_accel_y, imu_accel_z,
                imu_gyro_x, imu_gyro_y, imu_gyro_z,
                lidar_ranges, 
                pos_x, pos_y, pos_z, pos_roll, pos_pitch, pos_yaw,
                gps_lat, gps_lon, gps_alt)
                
    def run_threaded(self):
        return self.run()


class ROSControlledDriveMode:
    """Enhanced drive mode that sends final commands to ROS instead of direct hardware"""
    
    def __init__(self, cfg):
        self.cfg = cfg

    def run(self, mode, user_angle, user_throttle, pilot_angle, pilot_throttle):
        """
        Standard DonkeyCar decision logic, but output goes to ROS
        """
        if mode == 'user':
            return user_angle, user_throttle
        elif mode == 'local_angle':
            return pilot_angle if pilot_angle else 0.0, user_throttle
        else:
            return pilot_angle if pilot_angle else 0.0, \
                   pilot_throttle * self.cfg.AI_THROTTLE_MULT if pilot_throttle else 0.0


class PilotCondition:
    """Helper class to determine who is in charge of driving"""
    def run(self, mode):
        return mode != 'user'


def drive(cfg, model_path=None, model_type=None):
    """
    Construct vehicle with ROS as final hardware controller
    """
    if not ROS_AVAILABLE:
        logger.error("ROS is required for this template but not available!")
        return
        
    logger.info(f'PID: {os.getpid()}')
    logger.info("Starting DonkeyCar with ROS Hardware Control")
    
    car = dk.vehicle.Vehicle()
    
    # === SENSOR INPUT SECTION ===
    
    # ROS传感器桥接器 - 专用于纯ROS系统 (推荐)
    # ROS sensor bridge - dedicated for pure ROS systems (recommended)
    sensor_bridge = ROSToDonkeySensorBridge(
        camera_topic=getattr(cfg, 'ROS_CAMERA_TOPIC', '/camera/image_raw'),
        imu_topic=getattr(cfg, 'ROS_IMU_TOPIC', '/imu/data'),
        lidar_topic=getattr(cfg, 'ROS_LIDAR_TOPIC', '/scan'),
        odom_topic=getattr(cfg, 'ROS_ODOM_TOPIC', '/odom'),
        gps_topic=getattr(cfg, 'ROS_GPS_TOPIC', '/gps/fix')
    )
    
    # Add sensors - ROS系统直接提供传感器数据
    if cfg.CAMERA_TYPE == "ROS_CAMERA":
        # 直接使用ROS摄像头数据 - Use ROS camera data directly
        ros_sensor_distributor = ROSSensorDataDistributor(sensor_bridge)
        car.add(ros_sensor_distributor, 
                outputs=['cam/image_array', 
                        'imu/accel_x', 'imu/accel_y', 'imu/accel_z',
                        'imu/gyro_x', 'imu/gyro_y', 'imu/gyro_z',
                        'lidar/ranges',
                        'pos/x', 'pos/y', 'pos/z', 'pos/roll', 'pos/pitch', 'pos/yaw',
                        'gps/latitude', 'gps/longitude', 'gps/altitude'],
                threaded=True)
                
    elif cfg.CAMERA_TYPE == "ROS_USB_CAM":
        # 兼容旧配置 - Backward compatibility
        logger.warning("ROS_USB_CAM is deprecated, use ROS_CAMERA instead")
        ros_sensor_distributor = ROSSensorDataDistributor(sensor_bridge)
        car.add(ros_sensor_distributor, 
                outputs=['cam/image_array', 
                        'imu/accel_x', 'imu/accel_y', 'imu/accel_z',
                        'imu/gyro_x', 'imu/gyro_y', 'imu/gyro_z',
                        'lidar/ranges',
                        'pos/x', 'pos/y', 'pos/z', 'pos/roll', 'pos/pitch', 'pos/yaw',
                        'gps/latitude', 'gps/longitude', 'gps/altitude'],
                threaded=True)
        
    elif cfg.CAMERA_TYPE == "PICAM":
        # 使用DonkeyCar PiCamera + 单独的ROS传感器桥接
        from donkeycar.parts.camera import PiCamera
        camera = PiCamera(image_w=cfg.IMAGE_W, image_h=cfg.IMAGE_H,
                         image_d=cfg.IMAGE_DEPTH,
                         vflip=cfg.CAMERA_VFLIP, hflip=cfg.CAMERA_HFLIP)
        car.add(camera, outputs=['cam/image_array'], threaded=True)
        
        # 添加ROS传感器数据（IMU、雷达等）
        class ROSAdditionalSensors:
            def __init__(self, sensor_bridge):
                self.sensor_bridge = sensor_bridge
                
            def run(self):
                # 从ROS获取除摄像头外的其他传感器数据
                ros_data = self.sensor_bridge.run()
                return (ros_data.get('imu/accel_x', 0.0),
                       ros_data.get('imu/accel_y', 0.0),
                       ros_data.get('imu/accel_z', 0.0),
                       ros_data.get('imu/gyro_x', 0.0),
                       ros_data.get('imu/gyro_y', 0.0),
                       ros_data.get('imu/gyro_z', 0.0),
                       ros_data.get('lidar/ranges', []),
                       ros_data.get('pos/x', 0.0),
                       ros_data.get('pos/y', 0.0),
                       ros_data.get('pos/z', 0.0),
                       ros_data.get('pos/roll', 0.0),
                       ros_data.get('pos/pitch', 0.0),
                       ros_data.get('pos/yaw', 0.0),
                       ros_data.get('gps/latitude', 0.0),
                       ros_data.get('gps/longitude', 0.0),
                       ros_data.get('gps/altitude', 0.0))
                       
        additional_sensors = ROSAdditionalSensors(sensor_bridge)
        car.add(additional_sensors, 
                outputs=['imu/accel_x', 'imu/accel_y', 'imu/accel_z',
                        'imu/gyro_x', 'imu/gyro_y', 'imu/gyro_z',
                        'lidar/ranges',
                        'pos/x', 'pos/y', 'pos/z', 'pos/roll', 'pos/pitch', 'pos/yaw',
                        'gps/latitude', 'gps/longitude', 'gps/altitude'],
                threaded=True)
        
    elif cfg.CAMERA_TYPE == "MOCK":
        # Mock模式 + 模拟ROS传感器数据
        from donkeycar.parts.camera import MockCamera
        camera = MockCamera(image_w=cfg.IMAGE_W, image_h=cfg.IMAGE_H,
                           image_d=cfg.IMAGE_DEPTH)
        car.add(camera, outputs=['cam/image_array'], threaded=True)
        
        # 添加模拟的传感器数据用于测试
        class MockROSSensors:
            def run(self):
                # Mock所有传感器数据
                return (0.0, 0.0, 0.0,        # IMU accel
                       0.0, 0.0, 0.0,        # IMU gyro
                       [1.0] * 360,          # Lidar ranges  
                       0.0, 0.0, 0.0,       # Position x,y,z
                       0.0, 0.0, 0.0,       # Orientation roll,pitch,yaw
                       0.0, 0.0, 0.0)       # GPS lat,lon,alt
                
        mock_sensors = MockROSSensors()
        car.add(mock_sensors, 
                outputs=['imu/accel_x', 'imu/accel_y', 'imu/accel_z',
                        'imu/gyro_x', 'imu/gyro_y', 'imu/gyro_z',
                        'lidar/ranges',
                        'pos/x', 'pos/y', 'pos/z', 'pos/roll', 'pos/pitch', 'pos/yaw',
                        'gps/latitude', 'gps/longitude', 'gps/altitude'])
    else:
        raise Exception(f"Unknown camera type: {cfg.CAMERA_TYPE}. Supported: ROS_CAMERA, PICAM, MOCK")

    # IMU和其他传感器数据处理说明:
    # - ROS_CAMERA模式: 所有传感器数据来自ROS (推荐用于纯ROS系统)
    #   包括: 摄像头图像、IMU 6轴数据、激光雷达扫描、里程计位置、GPS坐标
    # - PICAM模式: 摄像头来自Pi，其他传感器来自ROS
    # - MOCK模式: 所有数据都是模拟的，用于测试
    
    # 传感器数据现在可以在车辆的其他组件中使用:
    # - cam/image_array: 用于AI模型输入
    # - imu/accel_x, imu/gyro_z等: 用于姿态控制或增强学习
    # - lidar/ranges: 用于避障或路径规划
    # - pos/x, pos/y等: 用于定位和导航
    # - gps/latitude等: 用于全局定位
    
    # === LEGACY SENSOR COMPATIBILITY SECTION ===
    # 为了兼容性，保留旧的IMU/Lidar单独处理逻辑，但推荐直接使用ROS_CAMERA模式
    
    if cfg.CAMERA_TYPE in ['PICAM', 'MOCK']:  # 只有非ROS摄像头时才需要单独处理
        # Add IMU sensor (can be ROS or DonkeyCar)
        if getattr(cfg, 'USE_ROS_IMU', True):  # 默认启用ROS IMU
            logger.info("Using ROS IMU data (already added above)")
        else:
            # Add placeholder IMU data for consistency
            class PlaceholderIMU:
                def run(self):
                    return {'gyro_x': 0.0, 'gyro_y': 0.0, 'gyro_z': 0.0,
                           'accel_x': 0.0, 'accel_y': 0.0, 'accel_z': 0.0}
            
            placeholder_imu = PlaceholderIMU()
            car.add(placeholder_imu, outputs=['imu/data'])
        
        # Add Lidar sensor (can be ROS or DonkeyCar)  
        if getattr(cfg, 'USE_ROS_LIDAR', True):  # 默认启用ROS Lidar
            logger.info("Using ROS Lidar data (already added above)")
        else:
            # Add placeholder Lidar data for consistency
            class PlaceholderLidar:
                def run(self):
                    return {'ranges': [1.0] * 360, 'angle_min': -3.14, 'angle_max': 3.14}
            
            placeholder_lidar = PlaceholderLidar()
            car.add(placeholder_lidar, outputs=['lidar/scan'])
    
    # === CONTROL INPUT SECTION ===
    
    # Web controller for user input
    ctr = LocalWebController(port=cfg.WEB_CONTROL_PORT, mode=cfg.WEB_INIT_MODE)
    car.add(ctr,
            inputs=['cam/image_array'],
            outputs=['user/angle', 'user/throttle', 'user/mode', 'recording'],
            threaded=True)
    
    # Optional: ROS as alternative control input
    if getattr(cfg, 'ENABLE_ROS_CONTROL_INPUT', False):
        ros_driver = ROSDriverInterface(namespace=getattr(cfg, 'ROS_NAMESPACE', 'donkeycar'))
        car.add(ros_driver,
                outputs=['ros_user/angle', 'ros_user/throttle', 'ros_user/mode'],
                threaded=True)

    # === AI DECISION SECTION ===
    
    # Pilot condition
    car.add(PilotCondition(), inputs=['user/mode'], outputs=['run_pilot'])

    # Add autopilot
    if model_type is None:
        model_type = cfg.DEFAULT_MODEL_TYPE
    if model_path:
        kl = dk.utils.get_model_by_type(model_type, cfg)
        kl.load(model_path=model_path)
        inputs = ['cam/image_array']
        
        # Add image transformations
        if hasattr(cfg, 'TRANSFORMATIONS') and cfg.TRANSFORMATIONS:
            outputs = ['cam/image_array_trans']
            car.add(ImageAugmentation(cfg, 'TRANSFORMATIONS'),
                    inputs=inputs, outputs=outputs)
            inputs = outputs

        outputs = ['pilot/angle', 'pilot/throttle']
        car.add(kl, inputs=inputs, outputs=outputs, run_condition='run_pilot')

    # Decision making (DonkeyCar core logic)
    car.add(ROSControlledDriveMode(cfg=cfg),
            inputs=['user/mode', 'user/angle', 'user/throttle',
                    'pilot/angle', 'pilot/throttle'],
            outputs=['final/angle', 'final/throttle'])

    # === ROS HARDWARE CONTROL SECTION ===
    
    # This replaces PCA9685/PWMSteering/PWMThrottle
    # Final control commands are sent to ROS topics instead of direct hardware
    ros_hardware = DonkeyToROSHardwareController(
        namespace=getattr(cfg, 'ROS_NAMESPACE', 'donkeycar'),
        max_speed=getattr(cfg, 'ROS_MAX_SPEED_MS', 2.0),
        max_angular=getattr(cfg, 'ROS_MAX_ANGULAR_RADS', 1.0)
    )
    
    car.add(ros_hardware, inputs=['final/angle', 'final/throttle'])
    
    # === DATA PUBLISHING SECTION ===
    
    # 注意: ROSToDonkeySensorBridge已经自动处理ROS传感器数据订阅
    # 这里不再需要单独的publish调用，数据流向是: ROS → DonkeyCar
    # Note: ROSToDonkeySensorBridge automatically handles ROS sensor data subscription
    # No separate publish call needed here, data flows: ROS → DonkeyCar

    # === DATA RECORDING SECTION ===
    
    # Standard DonkeyCar data recording
    inputs = ['cam/image_array', 'user/angle', 'user/throttle', 'user/mode', 
              'final/angle', 'final/throttle']
    types = ['image_array', 'float', 'float', 'str', 'float', 'float']

    # Add sensor data to recording if available  
    if getattr(cfg, 'RECORD_ROS_SENSORS', False):
        # Convert ROS sensor dict data to individual scalar fields for tub compatibility
        inputs.extend([
            'imu/gyro_x', 'imu/gyro_y', 'imu/gyro_z',
            'imu/accel_x', 'imu/accel_y', 'imu/accel_z',
            'lidar/range_count', 'lidar/range_min', 'lidar/range_max'
        ])
        types.extend([
            'float', 'float', 'float',      # IMU gyro data
            'float', 'float', 'float',      # IMU accel data  
            'int', 'float', 'float'         # Lidar summary data
        ])
        
        # Add a data adapter to convert ROS sensor dict data to scalar values
        class ROSSensorDataAdapter:
            """Converts ROS sensor dict data to individual scalar values for tub recording"""
            
            def run(self, imu_accel_x, imu_accel_y, imu_accel_z, 
                   imu_gyro_x, imu_gyro_y, imu_gyro_z, lidar_ranges):
                # IMU数据已经是标量，直接使用
                # Lidar数据转换为汇总值
                ranges = lidar_ranges if isinstance(lidar_ranges, list) else []
                range_count = len(ranges) if ranges else 0
                range_min = min(ranges) if ranges else 0.0
                range_max = max(ranges) if ranges else 0.0
                
                return (imu_gyro_x, imu_gyro_y, imu_gyro_z, 
                       imu_accel_x, imu_accel_y, imu_accel_z, 
                       range_count, range_min, range_max)
        
        # Add the adapter to the vehicle
        ros_adapter = ROSSensorDataAdapter()
        car.add(ros_adapter,
                inputs=['imu/accel_x', 'imu/accel_y', 'imu/accel_z',
                       'imu/gyro_x', 'imu/gyro_y', 'imu/gyro_z', 'lidar/ranges'],
                outputs=['imu/gyro_x', 'imu/gyro_y', 'imu/gyro_z',
                        'imu/accel_x', 'imu/accel_y', 'imu/accel_z',
                        'lidar/range_count', 'lidar/range_min', 'lidar/range_max'])
    else:
        # When ROS sensors are not recorded, add placeholder data  
        inputs.extend(['imu/accel_x', 'imu/gyro_z', 'lidar/ranges'])  # 使用实际的传感器数据键
        types.extend(['float', 'float', 'str'])  # 使用对应的数据类型

    if model_path is None or cfg.RECORD_DURING_AI:
        tub_path = TubHandler(path=cfg.DATA_PATH).create_tub_path() if \
            cfg.AUTO_CREATE_NEW_TUB else cfg.DATA_PATH
        tub_writer = TubWriter(base_path=tub_path, inputs=inputs, types=types)
        car.add(tub_writer, inputs=inputs, outputs=["tub/num_records"],
                run_condition='recording')

    # === SYSTEM STARTUP ===
    
    logger.info("=== DonkeyCar-ROS System Configuration ===")
    logger.info(f"Camera Type: {cfg.CAMERA_TYPE}")
    logger.info(f"ROS Camera Topic: {getattr(cfg, 'ROS_CAMERA_TOPIC', '/camera/image_raw')}")
    logger.info(f"ROS IMU Topic: {getattr(cfg, 'ROS_IMU_TOPIC', '/imu/data')}")
    logger.info(f"ROS Lidar Topic: {getattr(cfg, 'ROS_LIDAR_TOPIC', '/scan')}")
    logger.info(f"ROS Namespace: {getattr(cfg, 'ROS_NAMESPACE', 'donkeycar')}")
    logger.info(f"Hardware Control: ROS Topics (not direct PWM)")
    logger.info(f"Data Flow: ROS Sensors → DonkeyCar AI → ROS Hardware Control")
    logger.info("==========================================")
    
    # Start the integrated system
    car.start(rate_hz=cfg.DRIVE_LOOP_HZ, max_loop_count=cfg.MAX_LOOPS)


def calibrate(cfg):
    """
    Calibration mode using ROS hardware control
    """
    if not ROS_AVAILABLE:
        logger.error("ROS is required for this template but not available!")
        return
        
    logger.info("ROS Hardware Calibration Mode")
    logger.info("Control commands will be sent to ROS topics for calibration")
    
    car = dk.vehicle.Vehicle()
    
    # Create ROS hardware controller for calibration
    ros_hardware = DonkeyToROSHardwareController(namespace=getattr(cfg, 'ROS_NAMESPACE', 'donkeycar'))
    
    # Web controller for manual control during calibration
    ctr = LocalWebController(port=cfg.WEB_CONTROL_PORT, mode="user")
    
    # Mock camera for web interface
    from donkeycar.parts.camera import MockCamera
    mock_cam = MockCamera(image_w=cfg.IMAGE_W, image_h=cfg.IMAGE_H)
    
    car.add(mock_cam, outputs=['cam/image_array'], threaded=True)
    car.add(ctr,
            inputs=['cam/image_array'],  
            outputs=['user/angle', 'user/throttle', 'user/mode', 'recording'],
            threaded=True)
    
    # Send calibration commands to ROS
    car.add(ros_hardware, inputs=['user/angle', 'user/throttle'])
    
    # Calibration monitor
    class CalibrationMonitor:
        def __init__(self):
            self.count = 0
            
        def run(self, angle, throttle):
            self.count += 1
            if self.count % 10 == 0:
                logger.info(f'ROS Calibration - Angle: {angle:+5.4f}, Throttle: {throttle:+5.4f}')
                logger.info(f'→ Sent to ROS topics: /{getattr(cfg, "ROS_NAMESPACE", "donkeycar")}/cmd_vel')
                logger.info(f'Monitor with: rostopic echo /{getattr(cfg, "ROS_NAMESPACE", "donkeycar")}/cmd_vel')
            
    cal_monitor = CalibrationMonitor()
    car.add(cal_monitor, inputs=['user/angle', 'user/throttle'])
    
    logger.info("Connect to web interface and use joystick/keyboard to test ROS hardware control")
    logger.info(f"Monitor ROS topics: rostopic echo /{getattr(cfg, 'ROS_NAMESPACE', 'donkeycar')}/cmd_vel")
    
    car.start(rate_hz=10, max_loop_count=cfg.MAX_LOOPS)


if __name__ == '__main__':
    args = docopt(__doc__)
    cfg = dk.load_config()
    
    if args['drive']:
        drive(cfg, model_path=args['--model'], model_type=args['--type'])
    elif args['calibrate']:
        calibrate(cfg)