# DonkeyCar Configuration with ROS Integration
# This configuration extends the standard DonkeyCar config with ROS-specific settings

import os

# Standard DonkeyCar Configuration
# ================================

# CAMERA
CAMERA_TYPE = "PICAM" # (PICAM|WEBCAM|CVCAM|CSIC|V4L|MOCK|IMAGE_LIST)
IMAGE_W = 160
IMAGE_H = 120
IMAGE_DEPTH = 3         # default RGB=3, make 1 for mono
CAMERA_FRAMERATE = 20
CAMERA_VFLIP = False
CAMERA_HFLIP = False

# DRIVETRAIN
DRIVE_LOOP_HZ = 20
MAX_LOOPS = 100000

# STEERING
STEERING_CHANNEL = 1
STEERING_LEFT_PWM = 460
STEERING_RIGHT_PWM = 290

# THROTTLE
THROTTLE_CHANNEL = 0
THROTTLE_FORWARD_PWM = 500
THROTTLE_STOPPED_PWM = 370
THROTTLE_REVERSE_PWM = 220

# training
BATCH_SIZE = 128
TRAIN_TEST_SPLIT = 0.8
MAX_EPOCHS = 100
PRINT_MODEL_SUMMARY = True
OPTIMIZER = None
LEARNING_RATE = 0.001
LEARNING_RATE_DECAY = 0.0
SEND_BEST_MODEL_TO_PI = False
CACHE_IMAGES = True

# model transfer settings
SEND_BEST_MODEL_TO_PI = False
SCP_USER = 'pi'
SCP_HOST = "donkeypi.local"

# ROS Integration Configuration
# =============================

# Enable/Disable ROS features
USE_ROS = True                    # Master switch for all ROS functionality
ROS_BRIDGE_ENABLED = True         # Enable the main ROS bridge
ROS_AS_DRIVER = True             # Allow ROS to directly control the car
ROS_RECORD_IN_TUB = True         # Include ROS data in recorded datasets

# ROS Namespace Configuration
ROS_NAMESPACE = "donkeycar"       # Base namespace for all ROS topics
ROS_FRAME_ID = "base_link"        # TF frame for the robot base
CAMERA_FRAME_ID = "camera_link"   # TF frame for the camera

# ROS Topic Names
ROS_TOPIC_CAMERA_RAW = "/camera/image_raw"
ROS_TOPIC_CAMERA_COMPRESSED = "/camera/image_raw/compressed"
ROS_TOPIC_CMD_VEL = "/cmd_vel"
ROS_TOPIC_CMD_VEL_INPUT = "/cmd_vel_input"  # External ROS control input
ROS_TOPIC_MODE = "/mode"
ROS_TOPIC_MODE_INPUT = "/mode_input"        # External mode control
ROS_TOPIC_RECORDING = "/recording"
ROS_TOPIC_STATUS = "/status"
ROS_TOPIC_DIAGNOSTICS = "/diagnostics"

# ROS Message Queue Sizes
ROS_QUEUE_SIZE_IMAGE = 1          # Keep only latest image
ROS_QUEUE_SIZE_CONTROL = 10       # Buffer control commands
ROS_QUEUE_SIZE_STATUS = 5         # Status messages

# Data Conversion Settings
ROS_IMAGE_ENCODING = "rgb8"       # ROS image encoding (rgb8|bgr8|mono8)
ROS_PUBLISH_COMPRESSED = False    # Use compressed image transport
ROS_MAX_SPEED_MS = 2.0           # Maximum speed in m/s for Twist conversion  
ROS_MAX_ANGULAR_RADS = 1.0       # Maximum angular velocity in rad/s

# ROS Control Timeouts
ROS_CMD_TIMEOUT = 0.5            # Timeout for ROS control commands (seconds)
ROS_HEARTBEAT_HZ = 1.0           # Frequency for status updates
ROS_DATA_MAX_AGE = 0.2           # Maximum age for incoming ROS data

# ROS Safety Features  
ROS_ENABLE_SAFETY_STOP = True    # Emergency stop via ROS
ROS_SAFETY_TOPIC = "/emergency_stop"
ROS_FAILSAFE_MODE = "user"       # Fallback mode when ROS control fails
ROS_ENABLE_WATCHDOG = True       # Monitor ROS node health

# Advanced ROS Features
ROS_ENABLE_TF_BROADCAST = True   # Publish TF transforms
ROS_ENABLE_DIAGNOSTICS = True    # Publish diagnostic information
ROS_ENABLE_PARAMETER_SERVER = True  # Use ROS parameter server for config

# ROS Navigation Integration
ROS_NAV_ENABLED = False          # Enable navigation stack integration  
ROS_TOPIC_ODOM = "/odom"         # Odometry topic
ROS_TOPIC_GOAL = "/move_base_simple/goal"  # Navigation goal topic
ROS_TOPIC_LASER = "/scan"        # Laser scan topic (if lidar present)

# Map and Localization
ROS_TOPIC_MAP = "/map"           # Map topic
ROS_TOPIC_AMCL_POSE = "/amcl_pose"  # AMCL localization pose
ROS_USE_FAKE_LOCALIZATION = False   # Use fake localization for testing

# ROS Logging and Debugging
ROS_LOG_LEVEL = "INFO"           # ROS log level (DEBUG|INFO|WARN|ERROR|FATAL)
ROS_ENABLE_ROSBAG_RECORD = False # Automatically record rosbag during sessions
ROS_ROSBAG_TOPICS = [            # Topics to record in rosbag
    "/camera/image_raw",
    "/cmd_vel", 
    "/status",
    "/tf"
]

# Performance and Optimization
ROS_USE_NODELETS = False         # Use nodelets for better performance
ROS_THREADING_MODEL = "threaded" # (threaded|callback_groups)
ROS_BUFFER_SIZE = 10             # Internal buffer sizes
ROS_SPIN_RATE = 50               # ROS spin rate (Hz)

# Platform-specific ROS Settings
ROS_DISTRO = "noetic"            # ROS distribution (melodic|noetic|foxy|galactic)
ROS_PYTHON_VERSION = 3           # Python version for ROS
ROS_WORKSPACE = "~/catkin_ws"    # ROS workspace path

# Hardware Integration via ROS
ROS_ENABLE_IMU = False           # Publish IMU data to ROS
ROS_TOPIC_IMU = "/imu/data"      # IMU topic name
ROS_ENABLE_GPS = False           # Publish GPS data to ROS  
ROS_TOPIC_GPS = "/gps/fix"       # GPS topic name
ROS_ENABLE_ENCODERS = False      # Publish wheel encoder data
ROS_TOPIC_ENCODERS = "/encoders" # Encoder topic name

# Custom ROS Services
ROS_ENABLE_SERVICES = True       # Enable ROS service interface
ROS_SERVICE_START_RECORDING = "/start_recording"
ROS_SERVICE_STOP_RECORDING = "/stop_recording"
ROS_SERVICE_CALIBRATE = "/calibrate"
ROS_SERVICE_SET_MODE = "/set_mode"

# Multi-Robot Configuration
ROS_ROBOT_ID = "donkey_01"       # Unique robot identifier
ROS_MULTI_ROBOT_MODE = False     # Enable multi-robot support
ROS_ROBOT_NAMESPACE_PREFIX = "/robot"  # Namespace prefix for multi-robot

# Development and Testing
ROS_SIMULATION_MODE = False      # Enable simulation-specific features
ROS_MOCK_HARDWARE = False        # Use mock hardware for testing
ROS_ENABLE_RVIZ = True          # Launch RViz visualization
ROS_RVIZ_CONFIG = "donkeycar.rviz"  # RViz config file

# Integration with Existing DonkeyCar Features
WEB_CONTROL_PORT = 8887
WEB_INIT_MODE = "user"

# Data paths
DATA_PATH = './data'
MODELS_PATH = './models'

# TUB
TUB_PATH = './data'
TUB_RGB_ON = False
TUB_GRAYSCALE_ON = False

# Model paths for different configurations
MODEL_PATH = './models/pilot'

# Training Configuration with ROS data
if ROS_RECORD_IN_TUB:
    # Include ROS channels in training data
    TRAIN_INPUTS = ['cam/image_array', 'user/angle', 'user/throttle', 'ros/angle', 'ros/throttle']
    TRAIN_OUTPUTS = ['angle', 'throttle']
else:
    # Standard training inputs
    TRAIN_INPUTS = ['cam/image_array', 'user/angle', 'user/throttle']
    TRAIN_OUTPUTS = ['angle', 'throttle']

# ROS Launch Configuration
ROS_LAUNCH_FILE = "donkeycar.launch"  # Default launch file
ROS_LAUNCH_ARGS = []                  # Additional launch arguments

print("DonkeyCar ROS Configuration Loaded")
print(f"ROS Integration: {'Enabled' if USE_ROS else 'Disabled'}")
print(f"ROS Namespace: {ROS_NAMESPACE}")
print(f"ROS Bridge: {'Enabled' if ROS_BRIDGE_ENABLED else 'Disabled'}")
if ROS_AS_DRIVER:
    print("ROS can directly control the vehicle")
if ROS_NAV_ENABLED:
    print("ROS Navigation stack integration enabled")