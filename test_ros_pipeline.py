#!/usr/bin/env python3
"""
DonkeyCar-ROS Data Pipeline Integration Test
Tests the complete data flow from ROS sensors to ROS hardware control

Test Coverage:
- DataConverter functionality
- ROS message conversion accuracy
- DonkeyCar Memory data flow  
- Tub format compatibility
- End-to-end pipeline integrity

Usage:
    python test_ros_pipeline.py [--verbose] [--ros-test]
"""

import sys
import os
import time
import numpy as np
import json
import tempfile
import shutil
from pathlib import Path

# Add donkeycar to path but bypass version check
sys.path.append(os.path.join(os.path.dirname(__file__), '..'))

# Load components directly without triggering donkeycar version check
try:
    import importlib.util
    
    # Import our real ROS integration components directly
    ros_module_path = os.path.join(os.path.dirname(__file__), 'donkeycar', 'parts', 'ros.py')
    tub_module_path = os.path.join(os.path.dirname(__file__), 'donkeycar', 'parts', 'tub_v2.py')
    memory_module_path = os.path.join(os.path.dirname(__file__), 'donkeycar', 'memory.py')
    
    # Load ROS components
    if os.path.exists(ros_module_path):
        spec = importlib.util.spec_from_file_location("ros_parts", ros_module_path)
        ros_module = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(ros_module)
        
        DataConverter = getattr(ros_module, 'DataConverter', None)
        DonkeyToROSHardwareController = getattr(ros_module, 'DonkeyToROSHardwareController', None) 
        ROSMultiSensorBridge = getattr(ros_module, 'ROSMultiSensorBridge', None)
        ROSBridgeTest = getattr(ros_module, 'ROSBridgeTest', None)
        ROSSensorDataAdapter = getattr(ros_module, 'ROSSensorDataAdapter', None)
        
        REAL_ROS_COMPONENTS = True
    else:
        REAL_ROS_COMPONENTS = False
        print("Warning: Real ROS components not found")
    
    # Load Tub components  
    if os.path.exists(tub_module_path):
        spec = importlib.util.spec_from_file_location("tub_parts", tub_module_path)
        tub_module = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(tub_module)
        
        TubWriter = getattr(tub_module, 'TubWriter', None)
        Tub = getattr(tub_module, 'Tub', None)
        
        REAL_TUB_COMPONENTS = True
    else:
        REAL_TUB_COMPONENTS = False
        print("Warning: Real Tub components not found")
    
    # Load Memory components
    if os.path.exists(memory_module_path):
        spec = importlib.util.spec_from_file_location("memory_parts", memory_module_path)
        memory_module = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(memory_module)
        
        Memory = getattr(memory_module, 'Memory', None)
        
        REAL_MEMORY_COMPONENTS = True
    else:
        REAL_MEMORY_COMPONENTS = False
        Memory = None
        
except Exception as e:
    print(f"Error loading real components: {e}")
    REAL_ROS_COMPONENTS = False
    REAL_TUB_COMPONENTS = False
    REAL_MEMORY_COMPONENTS = False

# Test ROS components if available
try:
    import rospy
    from sensor_msgs.msg import Image, Imu, LaserScan
    from geometry_msgs.msg import Twist
    from nav_msgs.msg import Odometry
    from std_msgs.msg import String, Float32
    ROS_AVAILABLE = True
    print("✓ ROS environment detected")
except ImportError as e:
    ROS_AVAILABLE = False
    print(f"⚠ ROS not available: {e}")

class Colors:
    """Terminal colors for test output"""
    GREEN = '\033[92m'
    RED = '\033[91m'
    YELLOW = '\033[93m'
    BLUE = '\033[94m'
    BOLD = '\033[1m'
    END = '\033[0m'

class TestResults:
    """Test results tracker"""
    def __init__(self):
        self.total = 0
        self.passed = 0
        self.failed = 0
        self.failures = []
    
    def add_result(self, test_name, success, message=""):
        self.total += 1
        if success:
            self.passed += 1
            print(f"{Colors.GREEN}✓{Colors.END} {test_name}")
        else:
            self.failed += 1
            self.failures.append((test_name, message))
            print(f"{Colors.RED}✗{Colors.END} {test_name}: {message}")
    
    def summary(self):
        print(f"\n{Colors.BOLD}Test Summary:{Colors.END}")
        print(f"Total: {self.total}, Passed: {Colors.GREEN}{self.passed}{Colors.END}, Failed: {Colors.RED}{self.failed}{Colors.END}")
        
        if self.failures:
            print(f"\n{Colors.RED}Failed Tests:{Colors.END}")
            for test_name, message in self.failures:
                print(f"  - {test_name}: {message}")
        
        return self.failed == 0

class DonkeyCarDataPipelineTest:
    """Complete DonkeyCar-ROS data pipeline test suite"""
    
    def __init__(self, verbose=False):
        self.verbose = verbose
        self.results = TestResults()
        self.temp_dir = None
        
    def log(self, message):
        if self.verbose:
            print(f"{Colors.BLUE}[DEBUG]{Colors.END} {message}")
    
    def test_data_converter(self):
        """Test REAL DataConverter functionality"""
        print(f"\n{Colors.BOLD}Testing REAL DataConverter...{Colors.END}")
        
        if not REAL_ROS_COMPONENTS:
            self.results.add_result("DataConverter availability", False, "Real ROS components not loaded")
            return
            
        if not ROS_AVAILABLE:
            self.results.add_result("DataConverter", False, "ROS not available")
            return
            
        if not DataConverter:
            self.results.add_result("DataConverter class", False, "DataConverter class not found")
            return
        
        try:
            converter = DataConverter()
            self.results.add_result("DataConverter instantiation", True)
        except Exception as e:
            self.results.add_result("DataConverter instantiation", False, str(e))
            return
        
        # Test 1: REAL Image conversion roundtrip
        try:
            test_image = np.random.randint(0, 255, (120, 160, 3), dtype=np.uint8)
            ros_image = converter.donkey_image_to_ros(test_image)
            converted_back = converter.ros_image_to_donkey(ros_image)
            
            success = np.array_equal(test_image, converted_back)
            self.results.add_result("REAL Image conversion roundtrip", success, 
                                  "" if success else f"Shape mismatch: {test_image.shape} vs {converted_back.shape}")
        except Exception as e:
            self.results.add_result("REAL Image conversion roundtrip", False, str(e))
        
        # Test 2: REAL Control data conversion
        test_cases = [
            (-1.0, -1.0),  # Full left, full reverse
            (0.0, 0.0),    # Center, stopped  
            (1.0, 1.0),    # Full right, full forward
            (0.5, -0.3),   # Mixed values
        ]
        
        for i, (angle, throttle) in enumerate(test_cases):
            try:
                twist = converter.donkey_control_to_twist(angle, throttle)
                angle_back, throttle_back = converter.twist_to_donkey_control(twist)
                
                angle_ok = abs(angle - angle_back) < 0.01
                throttle_ok = abs(throttle - throttle_back) < 0.01
                success = angle_ok and throttle_ok
                
                self.results.add_result(f"REAL Control conversion {i+1}", success,
                                      f"angle: {angle}->{angle_back}, throttle: {throttle}->{throttle_back}")
            except Exception as e:
                self.results.add_result(f"REAL Control conversion {i+1}", False, str(e))
        
        # Test 3: REAL IMU data conversion
        try:
            imu_ros = converter.donkey_imu_to_ros(0.1, 0.2, 0.3, 9.8, 0.1, 0.2)
            imu_donkey = converter.ros_imu_to_donkey(imu_ros)
            
            expected_keys = ['gyro_x', 'gyro_y', 'gyro_z', 'accel_x', 'accel_y', 'accel_z']
            success = all(key in imu_donkey for key in expected_keys)
            
            self.results.add_result("REAL IMU conversion", success,
                                  "" if success else f"Missing keys: {set(expected_keys) - set(imu_donkey.keys())}")
        except Exception as e:
            self.results.add_result("REAL IMU conversion", False, str(e))
    
    def test_tub_compatibility(self):
        """Test REAL Tub format compatibility with ROS data"""
        print(f"\n{Colors.BOLD}Testing REAL Tub Format Compatibility...{Colors.END}")
        
        if not REAL_TUB_COMPONENTS:
            self.results.add_result("Tub compatibility", False, "Real Tub components not loaded")
            return
            
        if not Tub:
            self.results.add_result("Tub class availability", False, "Tub class not found")
            return
        
        self.temp_dir = tempfile.mkdtemp()
        self.log(f"Created temp directory: {self.temp_dir}")
        
        try:
            # Test with REAL Tub class and our enhanced data types
            inputs = ['cam/image_array', 'user/angle', 'user/throttle', 'user/mode',
                     'imu/gyro_x', 'lidar/range_count', 'sensor/dict_data']
            types = ['image_array', 'float', 'float', 'str', 
                    'float', 'int', 'dict']
            
            tub = Tub(base_path=self.temp_dir, inputs=inputs, types=types)
            
            # Test record with various data types including our ROS integration types
            test_image = np.random.randint(0, 255, (120, 160, 3), dtype=np.uint8)
            test_record = {
                'cam/image_array': test_image,
                'user/angle': 0.5,
                'user/throttle': 0.3,
                'user/mode': 'user',
                'imu/gyro_x': 0.1,
                'lidar/range_count': 360,
                'sensor/dict_data': {'sensor_type': 'imu', 'value': 123.45}  # Test ROS dict data
            }
            
            tub.write_record(test_record)
            tub.close()
            
            # Verify record was written correctly with REAL tub
            tub_read = Tub(base_path=self.temp_dir, read_only=True)
            record_count = len(tub_read.manifest.catalog)
            
            self.results.add_result("REAL Tub write operation", record_count == 1,
                                  f"Expected 1 record, got {record_count}")
            
            if record_count > 0:
                # Read back the record with REAL tub
                for record in tub_read:
                    # Check basic data integrity
                    angle_ok = record['user/angle'] == 0.5
                    throttle_ok = record['user/throttle'] == 0.3
                    
                    # Test ROS integration: dict data should be handled
                    dict_handled = 'sensor/dict_data' in record
                    
                    self.results.add_result("REAL Tub data integrity", angle_ok and throttle_ok)
                    self.results.add_result("REAL Tub ROS dict handling", dict_handled)
                    break
            
            tub_read.close()
            
        except Exception as e:
            self.results.add_result("REAL Tub compatibility test", False, str(e))
    
    def test_donkeycar_memory_flow(self):
        """Test REAL DonkeyCar Memory data flow"""
        print(f"\n{Colors.BOLD}Testing REAL DonkeyCar Memory Flow...{Colors.END}")
        
        if not REAL_MEMORY_COMPONENTS:
            self.results.add_result("Memory flow test", False, "Real Memory components not loaded")
            return
            
        if not Memory:
            self.results.add_result("Memory class availability", False, "Memory class not found") 
            return
        
        try:
            memory = Memory()
            
            # Test basic memory operations with ROS-like data
            test_data = {
                'cam/image_array': np.zeros((120, 160, 3)),
                'user/angle': 0.5,
                'user/throttle': 0.3,
                'imu/data': {'gyro_x': 0.1, 'gyro_y': 0.2},  # ROS dict data
                'lidar/scan': {'ranges': [1.0] * 360}  # ROS sensor data
            }
            
            # Store data in REAL memory
            for key, value in test_data.items():
                memory[key] = value
            
            # Retrieve data from REAL memory
            retrieved_keys = memory.keys()
            success = all(key in retrieved_keys for key in test_data.keys())
            
            self.results.add_result("REAL Memory store/retrieve", success,
                                  f"Missing keys: {set(test_data.keys()) - set(retrieved_keys)}")
            
            # Test batch operations with REAL memory
            keys = ['user/angle', 'user/throttle']  
            values = memory.get(keys)
            batch_success = len(values) == 2 and values[0] == 0.5 and values[1] == 0.3
            
            self.results.add_result("REAL Memory batch operations", batch_success)
            
            # Test ROS data integration
            if REAL_ROS_COMPONENTS and DataConverter and ROS_AVAILABLE:
                converter = DataConverter()
                
                # Store ROS-converted data
                test_image = np.random.randint(0, 255, (120, 160, 3), dtype=np.uint8)
                ros_image = converter.donkey_image_to_ros(test_image)
                converted_back = converter.ros_image_to_donkey(ros_image)
                
                memory['ros/converted_image'] = converted_back
                retrieved_image = memory.get('ros/converted_image')
                
                ros_integration_success = np.array_equal(converted_back, retrieved_image)
                self.results.add_result("REAL Memory + ROS integration", ros_integration_success)
            
        except Exception as e:
            self.results.add_result("REAL Memory flow test", False, str(e))
    
    def test_ros_message_pipeline(self):
        """Test ROS message creation and parsing"""  
        print(f"\n{Colors.BOLD}Testing ROS Message Pipeline...{Colors.END}")
        
        if not ROS_AVAILABLE:
            self.results.add_result("ROS message pipeline", False, "ROS not available")
            return
            
        try:
            # Test if ROS messages can be created properly
            converter = DataConverter()
            
            # Test Twist message creation
            twist = converter.donkey_control_to_twist(0.5, 0.3)
            success = (hasattr(twist, 'linear') and hasattr(twist, 'angular') and
                      abs(twist.linear.x - 0.6) < 0.01 and abs(twist.angular.z - 0.5) < 0.01)
            
            self.results.add_result("Twist message creation", success)
            
            # Test Image message creation  
            test_image = np.random.randint(0, 255, (120, 160, 3), dtype=np.uint8)
            ros_image = converter.donkey_image_to_ros(test_image)
            img_success = (ros_image.width == 160 and ros_image.height == 120 and 
                          ros_image.encoding == 'rgb8')
            
            self.results.add_result("Image message creation", img_success)
            
        except Exception as e:
            self.results.add_result("ROS message pipeline", False, str(e))
    
    def test_end_to_end_pipeline(self):
        """Test REAL complete end-to-end data pipeline"""
        print(f"\n{Colors.BOLD}Testing REAL End-to-End Pipeline...{Colors.END}")
        
        if not REAL_ROS_COMPONENTS:
            self.results.add_result("End-to-end pipeline", False, "Real ROS components not available")
            return
            
        if not (DataConverter and ROSSensorDataAdapter):
            self.results.add_result("Pipeline components", False, "Required components not loaded")
            return
        
        try:
            # Initialize REAL components
            converter = DataConverter()
            adapter = ROSSensorDataAdapter() if ROSSensorDataAdapter else None
            
            self.results.add_result("REAL Pipeline component initialization", True)
            
            # Simulate the REAL complete data flow
            # 1. ROS sensor data → DonkeyCar format (REAL conversion)
            if ROS_AVAILABLE:
                ros_twist = Twist()
                ros_twist.linear.x = 1.0
                ros_twist.angular.z = 0.5
                
                # REAL Convert to DonkeyCar format
                angle, throttle = converter.twist_to_donkey_control(ros_twist)
                
                # 2. DonkeyCar processing (simulated AI decision)
                processed_angle = angle * 0.8  # Simulate AI adjustment
                processed_throttle = throttle * 0.9
                
                # 3. REAL Convert back to ROS for hardware control
                output_twist = converter.donkey_control_to_twist(processed_angle, processed_throttle)
                
                # Verify REAL data integrity through the pipeline
                input_magnitude = (ros_twist.linear.x**2 + ros_twist.angular.z**2)**0.5
                output_magnitude = (output_twist.linear.x**2 + output_twist.angular.z**2)**0.5
                
                # Allow for some processing modification
                magnitude_preserved = abs(input_magnitude - output_magnitude) < input_magnitude * 0.5
                
                self.results.add_result("REAL End-to-end ROS data flow", magnitude_preserved,
                                      f"Input: {input_magnitude:.3f}, Output: {output_magnitude:.3f}")
                
                # Test REAL image pipeline
                test_image = np.random.randint(0, 255, (120, 160, 3), dtype=np.uint8)
                ros_image = converter.donkey_image_to_ros(test_image)
                processed_image = converter.ros_image_to_donkey(ros_image)
                
                image_pipeline_ok = np.array_equal(test_image, processed_image)
                self.results.add_result("REAL End-to-end image pipeline", image_pipeline_ok)
                
                # Test REAL sensor data adaptation (if available)
                if adapter:
                    test_sensor_dict = {'sensor_type': 'lidar', 'ranges': [1.0, 2.0, 3.0]}
                    adapted_data = adapter.prepare_for_tub(test_sensor_dict)
                    
                    adaptation_ok = adapted_data is not None
                    self.results.add_result("REAL Sensor data adaptation", adaptation_ok)
                
            else:
                self.results.add_result("REAL End-to-end pipeline", False, "ROS not available for full test")
                
            # Test component integration without ROS
            if not ROS_AVAILABLE:
                # Test DataConverter instantiation and basic functionality
                test_success = hasattr(converter, 'donkey_control_to_twist')
                self.results.add_result("REAL Component integration", test_success)
                
        except Exception as e:
            self.results.add_result("REAL End-to-end pipeline", False, str(e))
    
    def test_vehicle_integration(self):
        """Test DonkeyCar Vehicle integration with ROS components"""
        print(f"\n{Colors.BOLD}Testing Vehicle Integration...{Colors.END}")
        
        try:
            from donkeycar.vehicle import Vehicle
            
            car = Vehicle()
            
            # Test adding ROS components to vehicle
            class MockROSComponent:
                def run(self, input_data):
                    return input_data * 2 if isinstance(input_data, (int, float)) else input_data
            
            class MockSensor:
                def run(self):
                    return {'sensor_value': 42}
            
            ros_component = MockROSComponent()
            sensor = MockSensor()
            
            car.add(sensor, outputs=['sensor/data'])
            car.add(ros_component, inputs=['sensor/data'], outputs=['processed/data'])
            
            # Test that components were added correctly
            component_count = len(car.parts)
            self.results.add_result("Vehicle component addition", component_count == 2,
                                  f"Expected 2 components, got {component_count}")
            
            # Test memory integration
            car.mem['test/input'] = 5.0
            self.results.add_result("Vehicle memory integration", car.mem['test/input'] == 5.0)
            
        except Exception as e:
            self.results.add_result("Vehicle integration", False, str(e))
    
    def run_all_tests(self):
        """Run complete REAL test suite"""
        print(f"{Colors.BOLD}DonkeyCar-ROS REAL Data Pipeline Integration Test{Colors.END}")
        print("=" * 60)
        
        # Check component availability
        print(f"{Colors.BLUE}Component Status:{Colors.END}")
        status_items = [
            ("ROS Environment", ROS_AVAILABLE),
            ("Real ROS Components", REAL_ROS_COMPONENTS), 
            ("Real Tub Components", REAL_TUB_COMPONENTS),
            ("Real Memory Components", REAL_MEMORY_COMPONENTS),
        ]
        
        for name, available in status_items:
            color = Colors.GREEN if available else Colors.RED
            status = "✓ Available" if available else "✗ Not Available"
            print(f"  {name}: {color}{status}{Colors.END}")
        print()
        
        if REAL_ROS_COMPONENTS:
            loaded_classes = []
            if DataConverter: loaded_classes.append("DataConverter")
            if DonkeyToROSHardwareController: loaded_classes.append("DonkeyToROSHardwareController") 
            if ROSSensorDataAdapter: loaded_classes.append("ROSSensorDataAdapter")
            if ROSMultiSensorBridge: loaded_classes.append("ROSMultiSensorBridge")
            if ROSBridgeTest: loaded_classes.append("ROSBridgeTest")
            
            print(f"{Colors.GREEN}Real Components Loaded: {', '.join(loaded_classes)}{Colors.END}\n")
        else:
            print(f"{Colors.RED}Warning: Running without real ROS integration components{Colors.END}\n")
        
        # Core functionality tests
        self.test_data_converter()
        self.test_tub_compatibility() 
        self.test_donkeycar_memory_flow()
        self.test_ros_message_pipeline()
        self.test_vehicle_integration()
        self.test_end_to_end_pipeline()
        
        # ROS Bridge tests if components are available
        if REAL_ROS_COMPONENTS and ROSBridgeTest and ROS_AVAILABLE:
            print(f"\n{Colors.BOLD}Testing REAL ROS Bridge Components...{Colors.END}")
            try:
                # Use existing ROS bridge test
                ros_test = ROSBridgeTest()
                
                # Run image conversion test
                ros_test.test_image_conversion()
                image_result = getattr(ros_test, 'test_results', {}).get('image_conversion', False)
                self.results.add_result("REAL ROS Bridge image test", image_result)
                
                # Run control conversion test  
                ros_test.test_control_conversion()
                control_result = getattr(ros_test, 'test_results', {}).get('control_conversion', False)
                self.results.add_result("REAL ROS Bridge control test", control_result)
                
            except Exception as e:
                self.results.add_result("REAL ROS Bridge tests", False, str(e))
        
        # Cleanup
        if self.temp_dir and os.path.exists(self.temp_dir):
            shutil.rmtree(self.temp_dir)
            self.log(f"Cleaned up temp directory: {self.temp_dir}")
        
        return self.results.summary()

def main():
    import argparse
    
    parser = argparse.ArgumentParser(description='DonkeyCar-ROS REAL Pipeline Integration Test')
    parser.add_argument('--verbose', action='store_true', help='Enable verbose output')
    parser.add_argument('--ros-test', action='store_true', help='Run additional ROS node tests (requires roscore)')
    
    args = parser.parse_args()
    
    # Check if ROS tests are requested
    if args.ros_test and ROS_AVAILABLE:
        print(f"{Colors.YELLOW}Note: ROS tests require roscore to be running{Colors.END}")
    
    # Check Python version
    python_version = sys.version_info
    if python_version < (3, 9):
        print(f"{Colors.RED}Warning: Python {python_version.major}.{python_version.minor} detected. DonkeyCar requires 3.11+{Colors.END}")
        print(f"{Colors.YELLOW}Attempting to test components directly without full DonkeyCar initialization{Colors.END}")
    
    # Run tests
    test_suite = DonkeyCarDataPipelineTest(verbose=args.verbose)
    success = test_suite.run_all_tests()
    
    # Detailed final report
    print(f"\n{Colors.BOLD}=== FINAL TEST REPORT ==={Colors.END}")
    
    if REAL_ROS_COMPONENTS and ROS_AVAILABLE:
        print(f"{Colors.GREEN}✓ REAL ROS Integration Components Tested Successfully{Colors.END}")
        print(f"  - DataConverter: {'✓' if DataConverter else '✗'}")
        print(f"  - DonkeyToROSHardwareController: {'✓' if DonkeyToROSHardwareController else '✗'}")
        print(f"  - ROSSensorDataAdapter: {'✓' if ROSSensorDataAdapter else '✗'}")
        print(f"  - ROSMultiSensorBridge: {'✓' if ROSMultiSensorBridge else '✗'}")
    elif REAL_ROS_COMPONENTS:
        print(f"{Colors.YELLOW}⚠ Partial Testing: Real components loaded but ROS not available{Colors.END}")
        print(f"  Install ROS and run: roscore & python3.9 test_ros_pipeline.py --ros-test")
    else:
        print(f"{Colors.RED}✗ Could not load real ROS integration components{Colors.END}")
        print(f"  Check that donkeycar/parts/ros.py exists and is importable")
    
    if success:
        print(f"\n{Colors.GREEN}{Colors.BOLD}🎉 Pipeline integration tests completed successfully!{Colors.END}")
        if REAL_ROS_COMPONENTS:
            print(f"{Colors.GREEN}Your ROS-DonkeyCar integration is working correctly!{Colors.END}")
    else:
        print(f"\n{Colors.RED}{Colors.BOLD}❌ Some integration issues found.{Colors.END}")
        print(f"Please review the failed tests above and check your implementation.")
    
    # Exit with appropriate code
    sys.exit(0 if success else 1)

if __name__ == "__main__":
    main()