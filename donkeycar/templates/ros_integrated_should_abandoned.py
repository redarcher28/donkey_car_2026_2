#!/usr/bin/env python3
"""
DonkeyCar template with integrated ROS support
This template demonstrates how to use the enhanced ROS bridge functionality
to create a fully integrated ROS-DonkeyCar system.

Usage:
    manage.py drive [--model=<model>] [--type=(linear|categorical|...)] [--ros-enabled]
    manage.py calibrate [--ros-enabled]

Options:
    -h --help          Show this screen.
    --ros-enabled      Enable ROS bridge functionality
"""

from docopt import docopt
import logging
import os
import sys

import donkeycar as dk
from donkeycar.parts.tub_v2 import TubWriter
from donkeycar.parts.datastore import TubHandler
from donkeycar.parts.controller import LocalWebController
from donkeycar.parts.actuator import PCA9685, PWMSteering, PWMThrottle
from donkeycar.pipeline.augmentations import ImageAugmentation

# Import ROS components
try:
    from donkeycar.parts.ros import (
        DonkeyCarROSBridge, 
        ROSDriverInterface,
        RosPublisher,
        RosSubscriber,
        ROSBridgeTest
    )
    from sensor_msgs.msg import Image
    from geometry_msgs.msg import Twist
    from std_msgs.msg import String, Bool, Float32
    ROS_AVAILABLE = True
except ImportError as e:
    print(f"ROS not available: {e}")
    ROS_AVAILABLE = False

logger = logging.getLogger(__name__)
logging.basicConfig(level=logging.INFO)


class ROSModeSelector:
    """Enhanced mode selector that handles both local and ROS control inputs"""
    
    def __init__(self, cfg):
        self.cfg = cfg
        
    def run(self, mode, user_angle, user_throttle, pilot_angle, pilot_throttle,
            ros_angle=None, ros_throttle=None, ros_mode=None):
        """
        Select control source based on mode and ROS availability
        Priority: ROS > User > Pilot
        """
        
        # If ROS is commanding, use ROS inputs
        if ros_mode and ros_mode != "user" and ros_angle is not None:
            return ros_angle, ros_throttle
        elif ros_angle is not None and ros_throttle is not None:
            # ROS override mode
            return ros_angle, ros_throttle
            
        # Standard DonkeyCar logic
        if mode == 'user':
            return user_angle, user_throttle
        elif mode == 'local_angle':
            return pilot_angle if pilot_angle else 0.0, user_throttle
        else:
            return pilot_angle if pilot_angle else 0.0, \
                   pilot_throttle * self.cfg.AI_THROTTLE_MULT if pilot_throttle else 0.0


class DriveMode:
    """Helper class to dispatch between ai and user driving"""
    
    def __init__(self, cfg):
        self.cfg = cfg

    def run(self, mode, user_angle, user_throttle, pilot_angle, pilot_throttle):
        if mode == 'user':
            return user_angle, user_throttle
        elif mode == 'local_angle':
            return pilot_angle if pilot_angle else 0.0, user_throttle
        else:
            return pilot_angle if pilot_angle else 0.0, \
                   pilot_throttle * self.cfg.AI_THROTTLE_MULT if \
                       pilot_throttle else 0.0


class PilotCondition:
    """Helper class to determine who is in charge of driving"""
    def run(self, mode):
        return mode != 'user'


def drive(cfg, model_path=None, model_type=None, ros_enabled=False):
    """
    Construct a robotic vehicle with optional ROS integration
    """
    logger.info(f'PID: {os.getpid()}')
    logger.info(f'ROS Integration: {"Enabled" if ros_enabled and ROS_AVAILABLE else "Disabled"}')
    
    car = dk.vehicle.Vehicle()
    
    # Add camera (same as basic template)
    inputs = []
    if cfg.DONKEY_GYM:
        from donkeycar.parts.dgym import DonkeyGymEnv 
        cam = DonkeyGymEnv(cfg.DONKEY_SIM_PATH, host=cfg.SIM_HOST,
                           env_name=cfg.DONKEY_GYM_ENV_NAME, conf=cfg.GYM_CONF,
                           delay=cfg.SIM_ARTIFICIAL_LATENCY)
        inputs = ['angle', 'throttle', 'brake']
    elif cfg.CAMERA_TYPE == "PICAM":
        from donkeycar.parts.camera import PiCamera
        cam = PiCamera(image_w=cfg.IMAGE_W, image_h=cfg.IMAGE_H,
                       image_d=cfg.IMAGE_DEPTH,
                       vflip=cfg.CAMERA_VFLIP, hflip=cfg.CAMERA_HFLIP)
    elif cfg.CAMERA_TYPE == "WEBCAM":
        from donkeycar.parts.camera import Webcam
        cam = Webcam(image_w=cfg.IMAGE_W, image_h=cfg.IMAGE_H,
                     image_d=cfg.IMAGE_DEPTH)
    elif cfg.CAMERA_TYPE == "MOCK":
        from donkeycar.parts.camera import MockCamera
        cam = MockCamera(image_w=cfg.IMAGE_W, image_h=cfg.IMAGE_H,
                         image_d=cfg.IMAGE_DEPTH)
    else:
        raise Exception(f"Unknown camera type: {cfg.CAMERA_TYPE}")

    car.add(cam, inputs=inputs, outputs=['cam/image_array'], threaded=True)

    # Add controller (web interface)
    ctr = LocalWebController(port=cfg.WEB_CONTROL_PORT, mode=cfg.WEB_INIT_MODE)
    car.add(ctr,
            inputs=['cam/image_array'],
            outputs=['user/angle', 'user/throttle', 'user/mode', 'recording'],
            threaded=True)

    # ROS Integration
    ros_outputs = []
    if ros_enabled and ROS_AVAILABLE:
        
        # Test ROS bridge functionality first
        logger.info("Running ROS bridge tests...")
        test_suite = ROSBridgeTest()
        if not test_suite.run_all_tests():
            logger.warning("ROS bridge tests failed, but continuing...")
        
        # Main ROS bridge - handles bidirectional communication
        ros_namespace = getattr(cfg, 'ROS_NAMESPACE', 'donkeycar')
        ros_bridge = DonkeyCarROSBridge(namespace=ros_namespace)
        
        car.add(ros_bridge,
                inputs=['cam/image_array', 'user/angle', 'user/throttle',
                        'pilot/angle', 'pilot/throttle', 'user/mode', 'recording'],
                outputs=['ros/angle', 'ros/throttle', 'ros/mode'],
                threaded=False)  # Keep in main loop for real-time performance
        
        ros_outputs = ['ros/angle', 'ros/throttle', 'ros/mode']
        
        # Optional: ROS as direct driver interface
        if getattr(cfg, 'ROS_AS_DRIVER', False):
            ros_driver = ROSDriverInterface(namespace=ros_namespace)
            car.add(ros_driver,
                    outputs=['ros_driver/angle', 'ros_driver/throttle', 'ros_driver/mode'],
                    threaded=True)
            
        logger.info(f"ROS bridge initialized with namespace: {ros_namespace}")
    else:
        # Provide default values when ROS is not available
        class MockROS:
            def run(self, *args):
                return 0.0, 0.0, "user"
        
        mock_ros = MockROS()
        car.add(mock_ros, outputs=['ros/angle', 'ros/throttle', 'ros/mode'])
        ros_outputs = ['ros/angle', 'ros/throttle', 'ros/mode']

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

    # Enhanced mode selector that handles ROS inputs
    if ros_enabled and ROS_AVAILABLE:
        mode_selector = ROSModeSelector(cfg=cfg)
        car.add(mode_selector,
                inputs=['user/mode', 'user/angle', 'user/throttle',
                        'pilot/angle', 'pilot/throttle'] + ros_outputs,
                outputs=['angle', 'throttle'])
    else:
        # Standard mode selector
        car.add(DriveMode(cfg=cfg),
                inputs=['user/mode', 'user/angle', 'user/throttle',
                        'pilot/angle', 'pilot/throttle'],
                outputs=['angle', 'throttle'])

    # Drive train setup (same as basic template)
    if cfg.DONKEY_GYM or cfg.DRIVE_TRAIN_TYPE == "MOCK":
        pass
    else:
        steering_controller = PCA9685(cfg.STEERING_CHANNEL,
                                      cfg.PCA9685_I2C_ADDR,
                                      busnum=cfg.PCA9685_I2C_BUSNUM)
        steering = PWMSteering(controller=steering_controller,
                               left_pulse=cfg.STEERING_LEFT_PWM,
                               right_pulse=cfg.STEERING_RIGHT_PWM)

        throttle_controller = PCA9685(cfg.THROTTLE_CHANNEL,
                                      cfg.PCA9685_I2C_ADDR,
                                      busnum=cfg.PCA9685_I2C_BUSNUM)
        throttle = PWMThrottle(controller=throttle_controller,
                               max_pulse=cfg.THROTTLE_FORWARD_PWM,
                               zero_pulse=cfg.THROTTLE_STOPPED_PWM,
                               min_pulse=cfg.THROTTLE_REVERSE_PWM)

        car.add(steering, inputs=['angle'])
        car.add(throttle, inputs=['throttle'])

    # Add tub to save data
    inputs = ['cam/image_array', 'user/angle', 'user/throttle', 'user/mode']
    types = ['image_array', 'float', 'float', 'str']

    # Add ROS data to tub if enabled
    if ros_enabled and ROS_AVAILABLE:
        inputs.extend(['ros/angle', 'ros/throttle', 'ros/mode'])
        types.extend(['float', 'float', 'str'])

    # Data recording
    if model_path is None or cfg.RECORD_DURING_AI:
        tub_path = TubHandler(path=cfg.DATA_PATH).create_tub_path() if \
            cfg.AUTO_CREATE_NEW_TUB else cfg.DATA_PATH
        tub_writer = TubWriter(base_path=tub_path, inputs=inputs, types=types)
        car.add(tub_writer, inputs=inputs, outputs=["tub/num_records"],
                run_condition='recording')

    # Start the car/ROS system
    logger.info("Starting DonkeyCar with ROS integration...")
    car.start(rate_hz=cfg.DRIVE_LOOP_HZ, max_loop_count=cfg.MAX_LOOPS)


def calibrate(cfg, ros_enabled=False):
    """
    Calibration mode with optional ROS monitoring
    """
    donkey_car = dk.vehicle.Vehicle()
    
    # Standard calibration (simplified version)
    if ros_enabled and ROS_AVAILABLE:
        logger.info("ROS-enabled calibration mode")
        
        # Publish calibration data to ROS for monitoring
        ros_bridge = DonkeyCarROSBridge(namespace='donkeycar_cal')
        
        class CalibrationMonitor:
            def __init__(self):
                self.count = 0
                
            def run(self, angle, throttle):
                self.count += 1
                if self.count % 10 == 0:  # Print every 10th reading
                    print(f'Calibration - Angle: {angle:+5.4f}, Throttle: {throttle:+5.4f}')
                return angle, throttle
        
        cal_monitor = CalibrationMonitor()
        donkey_car.add(cal_monitor, 
                      inputs=['user/angle', 'user/throttle'],
                      outputs=['cal/angle', 'cal/throttle'])
        
        # ROS bridge for calibration monitoring
        donkey_car.add(ros_bridge,
                      inputs=['cal/angle', 'cal/throttle'],
                      outputs=['ros/cal/angle', 'ros/cal/throttle'])
    else:
        logger.info("Standard calibration mode")
        
    donkey_car.start(rate_hz=10, max_loop_count=cfg.MAX_LOOPS)


if __name__ == '__main__':
    args = docopt(__doc__)
    cfg = dk.load_config()
    
    # Check for ROS enable flag
    ros_enabled = args.get('--ros-enabled', False)
    
    if not ROS_AVAILABLE and ros_enabled:
        logger.error("ROS support requested but not available. Please install ROS and required packages.")
        sys.exit(1)
    
    if args['drive']:
        drive(cfg, model_path=args['--model'], model_type=args['--type'], 
              ros_enabled=ros_enabled)
    elif args['calibrate']:
        calibrate(cfg, ros_enabled=ros_enabled)