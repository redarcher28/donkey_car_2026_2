#!/bin/bash
# DonkeyCar-ROS Integration Quick Start Script
# 快速启动 DonkeyCar-ROS 集成系统

set -e

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

# Default paths
DONKEY_PATH=${DONKEY_PATH:-"/home/$USER/donkeycar"}
ROS_WS=${ROS_WS:-"/home/$USER/catkin_ws"}
DATA_PATH=${DATA_PATH:-"$HOME/donkey_data"}

print_header() {
    echo -e "${BLUE}================================================================${NC}"
    echo -e "${BLUE}            DonkeyCar-ROS Integration Launcher${NC}"
    echo -e "${BLUE}================================================================${NC}"
}

print_status() {
    echo -e "${GREEN}[INFO]${NC} $1"
}

print_warning() {
    echo -e "${YELLOW}[WARNING]${NC} $1"
}

print_error() {
    echo -e "${RED}[ERROR]${NC} $1"
}

check_dependencies() {
    print_status "Checking dependencies..."
    
    # Check ROS installation
    if ! command -v roscore &> /dev/null; then
        print_error "ROS not installed or not in PATH"
        exit 1
    fi
    
    # Check if roscore is running
    if ! pgrep -f roscore > /dev/null; then
        print_warning "roscore is not running. Starting roscore..."
        roscore &
        sleep 3
    else
        print_status "roscore is already running"
    fi
    
    # Check Python environment
    if ! python3 -c "import donkeycar" 2>/dev/null; then
        print_error "DonkeyCar not installed in Python environment"
        print_error "Please run: pip install -e $DONKEY_PATH"
        exit 1
    fi
    
    # Check required ROS packages
    missing_packages=()
    
    for pkg in "cv_bridge" "image_transport" "tf2_ros"; do
        if ! python3 -c "import $pkg" 2>/dev/null; then
            missing_packages+=("ros-$ROS_DISTRO-${pkg//_/-}")
        fi
    done
    
    if [ ${#missing_packages[@]} -gt 0 ]; then
        print_error "Missing ROS packages: ${missing_packages[*]}"
        print_error "Install with: sudo apt install ${missing_packages[*]}"
        exit 1
    fi
    
    print_status "All dependencies satisfied"
}

setup_environment() {
    print_status "Setting up environment..."
    
    # Source ROS environment
    if [ -f "/opt/ros/$ROS_DISTRO/setup.bash" ]; then
        source /opt/ros/$ROS_DISTRO/setup.bash
        print_status "Sourced ROS $ROS_DISTRO"
    else
        print_error "ROS environment not found. Please check ROS_DISTRO environment variable"
        exit 1
    fi
    
    # Source workspace if exists
    if [ -f "$ROS_WS/devel/setup.bash" ]; then
        source $ROS_WS/devel/setup.bash
        print_status "Sourced catkin workspace: $ROS_WS"
    fi
    
    # Create data directory if not exists
    mkdir -p $DATA_PATH
    print_status "Data directory: $DATA_PATH"
    
    # Set Python path for DonkeyCar
    export PYTHONPATH="$DONKEY_PATH:$PYTHONPATH"
}

run_pipeline_test() {
    print_status "Running pipeline integrity test..."
    
    cd $DONKEY_PATH
    if python3 test_ros_pipeline.py --verbose; then
        print_status "Pipeline test PASSED"
        return 0
    else
        print_error "Pipeline test FAILED"
        return 1
    fi
}

launch_system() {
    local mode=$1
    print_status "Starting DonkeyCar-ROS system in mode: $mode"
    
    case $mode in
        "test")
            print_status "Test mode: Running data pipeline validation"
            run_pipeline_test
            ;;
        "collect")
            print_status "Data collection mode: Manual control with data recording"
            roslaunch donkeycar donkey_ros_integrated.launch \
                data_path:=$DATA_PATH \
                recording/enabled:=true \
                model/enabled:=false \
                record_bag:=true \
                web_interface:=true
            ;;
        "train")
            print_status "Training mode: Preparing training data"
            cd $DATA_PATH
            python3 $DONKEY_PATH/donkeycar/management/manage.py train \
                --tub=$DATA_PATH \
                --model=$DATA_PATH/models/mypilot.h5
            ;;
        "drive")
            print_status "Autonomous driving mode: AI model control"
            roslaunch donkeycar donkey_ros_integrated.launch \
                data_path:=$DATA_PATH \
                recording/enabled:=true \
                model/enabled:=true
            ;;
        "sim")
            print_status "Simulation mode: Using Gazebo/simulator"
            roslaunch donkeycar donkey_ros_integrated.launch \
                data_path:=$DATA_PATH \
                camera_topic:=/gazebo/camera/image_raw \
                cmd_vel_topic:=/gazebo/cmd_vel
            ;;
        *)
            print_error "Unknown mode: $mode"
            show_help
            exit 1
            ;;
    esac
}

show_help() {
    cat << EOF
${BLUE}DonkeyCar-ROS Integration Launcher${NC}

${GREEN}Usage:${NC}
    $0 <mode> [options]

${GREEN}Modes:${NC}
    ${YELLOW}test${NC}      - Run data pipeline integrity tests
    ${YELLOW}collect${NC}   - Start manual data collection mode
    ${YELLOW}train${NC}     - Train AI model from collected data  
    ${YELLOW}drive${NC}     - Start autonomous driving mode
    ${YELLOW}sim${NC}       - Start simulation mode (Gazebo)

${GREEN}Environment Variables:${NC}
    ${YELLOW}DONKEY_PATH${NC}  - Path to DonkeyCar installation (default: /home/$USER/donkeycar)
    ${YELLOW}ROS_WS${NC}       - Path to ROS workspace (default: /home/$USER/catkin_ws)
    ${YELLOW}DATA_PATH${NC}    - Path to data directory (default: $HOME/donkey_data)
    ${YELLOW}ROS_DISTRO${NC}   - ROS distribution (melodic/noetic/etc.)

${GREEN}Examples:${NC}
    # Test the complete pipeline
    $0 test
    
    # Collect training data manually
    $0 collect
    
    # Train AI model
    $0 train
    
    # Run autonomous driving
    $0 drive
    
    # Run in simulation
    ROS_DISTRO=noetic $0 sim

${GREEN}System Architecture:${NC}
    ROS Sensors → DataConverter → DonkeyCar Memory → AI Model → DataConverter → ROS Hardware

EOF
}

main() {
    print_header
    
    if [ $# -eq 0 ]; then
        show_help
        exit 1
    fi
    
    local mode=$1
    
    if [ "$mode" = "help" ] || [ "$mode" = "-h" ] || [ "$mode" = "--help" ]; then
        show_help
        exit 0
    fi
    
    check_dependencies
    setup_environment
    launch_system $mode
}

# Handle Ctrl+C gracefully
trap 'print_warning "Shutting down..."; exit 0' INT

main "$@"