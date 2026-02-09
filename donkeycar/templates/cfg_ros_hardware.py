# DonkeyCar Configuration for ROS Hardware Template
# =================================================
# This configuration is designed for the ros_hardware.py template where
# ROS acts as the final hardware controller layer while DonkeyCar provides
# AI decision-making and data processing capabilities.
#
# Architecture: DonkeyCar Decision-Making → ROS Bridge → ROS Hardware Control
#
# Usage Examples:
#   python manage.py drive --bag=/path/to/data.bag
#   python manage.py drive --model=models/pilot.h5 --bag=/path/to/data.bag
#   python manage.py calibrate

import os

# ====================================
# Standard DonkeyCar Core Configuration
# ====================================

# Image processing settings
IMAGE_W = 160
IMAGE_H = 120
IMAGE_DEPTH = 3         # RGB=3, Grayscale=1
CAMERA_FRAMERATE = 20
CAMERA_VFLIP = False
CAMERA_HFLIP = False

# Vehicle control loop
DRIVE_LOOP_HZ = 20      # Main control loop frequency
MAX_LOOPS = 100000      # Maximum iterations before auto-stop

# AI Model settings
AI_THROTTLE_MULT = 1.0  # Global AI throttle multiplier
DEFAULT_MODEL_TYPE = "linear"  # Default model architecture

# Web interface settings  
WEB_CONTROL_PORT = 8887
WEB_INIT_MODE = "user"  # Start in manual control mode

# Data storage
DATA_PATH = './data'
MODELS_PATH = './models'
AUTO_CREATE_NEW_TUB = True

# Training settings
BATCH_SIZE = 128
TRAIN_TEST_SPLIT = 0.8
MAX_EPOCHS = 100
LEARNING_RATE = 0.001
OPTIMIZER = None
PRINT_MODEL_SUMMARY = True

# Data recording
RECORD_DURING_AI = True
TUB_WRITE_THREADED = True

# ====================================
# Camera/Sensor Input Configuration
# ====================================

# Primary data source selection - Choose ONE:
#   "ROS_BAG"     - Use ROS bag file data (recommended for research)
#   "ROS_CAMERA"  - Use live ROS sensor topics  
#   "PICAM"       - DonkeyCar PiCamera + ROS sensors for hardware control
#   "MOCK"        - Mock sensors for testing
CAMERA_TYPE = "ROS_BAG"

# ====================================
# ROS System Configuration
# ====================================

# ROS Core Settings
ROS_NAMESPACE = "donkeycar"       # Base namespace for all ROS topics
ROS_FRAME_ID = "base_link"        # Robot base TF frame
CAMERA_FRAME_ID = "camera_link"   # Camera TF frame

# Hardware Control Topics (where DonkeyCar commands are sent)
ROS_TOPIC_CMD_VEL = "/cmd_vel"               # Velocity commands to robot
ROS_TOPIC_STATUS = "/donkeycar/status"       # DonkeyCar status publication
ROS_TOPIC_MODE = "/donkeycar/mode"           # Current driving mode

# Control Limits (for converting DonkeyCar outputs to ROS Twist messages)
ROS_MAX_SPEED_MS = 2.0           # Maximum linear velocity (m/s)
ROS_MAX_ANGULAR_RADS = 1.0       # Maximum angular velocity (rad/s)

# Safety and Monitoring
ROS_CMD_TIMEOUT = 0.5            # Command timeout (seconds)
ROS_HEARTBEAT_HZ = 2.0           # Status update frequency
ROS_ENABLE_SAFETY_STOP = True    # Emergency stop capability
ROS_SAFETY_TOPIC = "/emergency_stop"

# ====================================
# ROS Sensor Input Topics  
# ====================================
# These topics provide sensor data TO DonkeyCar

# Camera input from ROS
ROS_CAMERA_TOPIC = "/camera/image_raw"       # Live camera stream
#ROS_CAMERA_TOPIC = "/usb_cam/image_raw"     # Alternative USB camera

# Additional sensor topics (optional)
ROS_IMU_TOPIC = "/imu/data"                  # IMU sensor data
ROS_LIDAR_TOPIC = "/scan"                    # Lidar scan data
ROS_ODOM_TOPIC = "/odom"                     # Odometry/pose data
ROS_GPS_TOPIC = "/gps/fix"                   # GPS position data

# Sensor data conversion settings
ROS_IMAGE_ENCODING = "rgb8"      # Expected ROS image encoding
ROS_DATA_MAX_AGE = 0.2          # Maximum age for sensor data (seconds)

# ====================================
# ROS Bag File Configuration
# ====================================
# Settings for playing back recorded ROS bag files

# Bag file path (can be overridden with --bag command line argument)
ROS_BAG_FILE_PATH = None         # Set to bag file path or use --bag parameter
#ROS_BAG_FILE_PATH = "/mnt/d/code/robot_data_full.bag"  # Example path

# Playback control settings
ROS_BAG_LOOP_PLAYBACK = True     # Loop the bag file indefinitely
ROS_BAG_PLAYBACK_RATE = 1.0      # Playback speed multiplier (1.0 = normal speed)
ROS_BAG_START_TIME = None        # Start playback at specific time (seconds)
ROS_BAG_END_TIME = None          # Stop playback at specific time (seconds)

# Topic selection for bag playback (auto-detected if None)
#ROS_CAMERA_TOPIC = "/usb_cam/image_raw"     # Override for specific bag topics
#ROS_IMU_TOPIC = "/imu/data_raw"             # Override if bag uses different names

# ====================================
# Alternative Control Input (Optional)
# ====================================
# Enable ROS as an additional control input source alongside web interface

ENABLE_ROS_CONTROL_INPUT = False  # Allow ROS to send driving commands to DonkeyCar
ROS_CONTROL_INPUT_TOPIC = "/donkeycar/cmd_vel_input"  # Topic for ROS control commands

# ====================================
# Data Recording Configuration  
# ====================================
# Control what data gets recorded in DonkeyCar tub files

RECORD_ROS_SENSORS = True        # Include ROS sensor data in recordings
RECORD_DURING_AI = True          # Continue recording during AI driving

# Recorded data fields (when RECORD_ROS_SENSORS=True):
#   - Standard: cam/image_array, user/*, final/*
#   - ROS Sensors: imu/gyro_*, imu/accel_*, lidar/*, pos/*, gps/*

# ====================================
# Performance and Debug Settings
# ====================================

# Threading and performance
ROS_THREADING_MODEL = "threaded"  # Use threaded execution
ROS_SPIN_RATE = 50               # ROS node spin rate (Hz)
ROS_QUEUE_SIZE = 10              # Message queue size

# Debug and logging
ROS_LOG_LEVEL = "INFO"           # Log level (DEBUG|INFO|WARN|ERROR)
VERBOSE_ROS = True               # Enable detailed ROS status logging

# Enable diagnostic publishing
ROS_ENABLE_DIAGNOSTICS = True
ROS_TOPIC_DIAGNOSTICS = "/donkeycar/diagnostics"

# ====================================
# Template-Specific Settings
# ====================================

# Image transformations (augmentations)
TRANSFORMATIONS = []             # Image augmentations during AI inference
#TRANSFORMATIONS = ['CROP', 'TRAPEZE']  # Example transformations

# Model loading settings
DEFAULT_MODEL_PATH = None        # Default model (override with --model argument)
DEFAULT_MODEL_TYPE = "linear"    # Model architecture type

# Behavior settings
AI_LAUNCH_ENABLE_BUTTON = False  # Require button press to start AI
AI_LAUNCH_ENABLE_SPEED = 0.0     # Minimum speed to enable AI

# ====================================
# Hardware Integration Notes
# ====================================
# This template assumes ROS handles ALL hardware control:
# - No direct PWM/GPIO control from DonkeyCar
# - No PCA9685 servo driver configuration  
# - All steering/throttle commands sent via ROS topics
# - Physical hardware controlled by ROS nodes (not DonkeyCar)
#
# For traditional DonkeyCar hardware setup, use the standard complete.py template.
# This template is designed for integration with existing ROS robot platforms.

# ====================================
# Example Configurations
# ====================================

# Example 1: ROS Bag Research Setup
# CAMERA_TYPE = "ROS_BAG"
# ROS_BAG_FILE_PATH = "/path/to/research_data.bag"
# ROS_BAG_LOOP_PLAYBACK = True
# RECORD_ROS_SENSORS = True

# Example 2: Live ROS Robot Integration
# CAMERA_TYPE = "ROS_CAMERA" 
# ROS_CAMERA_TOPIC = "/camera/image_raw"
# ENABLE_ROS_CONTROL_INPUT = True
# ROS_ENABLE_SAFETY_STOP = True

# Example 3: Hybrid Setup (DonkeyCar Camera + ROS Hardware)
# CAMERA_TYPE = "PICAM"
# ROS_IMU_TOPIC = "/imu/data"
# ROS_LIDAR_TOPIC = "/scan"
# RECORD_ROS_SENSORS = True

print("=" * 50)
print("DonkeyCar ROS Hardware Template Configuration")
print("=" * 50)
print(f"Primary Data Source: {CAMERA_TYPE}")
if CAMERA_TYPE == "ROS_BAG":
    if ROS_BAG_FILE_PATH:
        print(f"ROS Bag File: {ROS_BAG_FILE_PATH}")
    else:
        print("ROS Bag File: Use --bag parameter or set ROS_BAG_FILE_PATH")
    print(f"Loop Playback: {ROS_BAG_LOOP_PLAYBACK}")
    print(f"Playback Rate: {ROS_BAG_PLAYBACK_RATE}x")
elif CAMERA_TYPE == "ROS_CAMERA":
    print(f"Live ROS Camera: {ROS_CAMERA_TOPIC}")
    print(f"ROS Control Input: {'Enabled' if ENABLE_ROS_CONTROL_INPUT else 'Disabled'}")
elif CAMERA_TYPE == "PICAM":
    print("DonkeyCar PiCamera + ROS Sensors")
elif CAMERA_TYPE == "MOCK":
    print("Mock sensors (testing mode)")

print(f"ROS Namespace: {ROS_NAMESPACE}")
print(f"ROS Hardware Control: {ROS_TOPIC_CMD_VEL}")
print(f"Record ROS Sensors: {'Yes' if RECORD_ROS_SENSORS else 'No'}")
print(f"Data Path: {DATA_PATH}")
print("=" * 50)
print("Data Flow: Sensors → DonkeyCar AI → ROS Hardware")
print("=" * 50)