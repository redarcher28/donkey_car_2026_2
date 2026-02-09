#!/usr/bin/env python3.9
"""
Direct ROS Components Test
Tests our implemented ROS integration components directly without DonkeyCar version checks

Usage: python3.9 test_direct_ros.py [--verbose]
"""

import sys
import os
import numpy as np
import json
import tempfile
import shutil
from pathlib import Path

# Colors for output
class Colors:
    GREEN = '\033[92m'
    RED = '\033[91m'
    YELLOW = '\033[93m' 
    BLUE = '\033[94m'
    BOLD = '\033[1m'
    END = '\033[0m'

class TestResults:
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
        return self.failed == 0

def load_ros_components():
    """Load our ROS components directly"""
    try:
        # Load the ros.py file directly
        import importlib.util
        
        ros_file = os.path.join(os.path.dirname(__file__), 'donkeycar', 'parts', 'ros.py')
        if not os.path.exists(ros_file):
            print(f"{Colors.RED}Error: {ros_file} not found{Colors.END}")
            return None, None, None, None
            
        # Read the file content and modify it to skip problematic imports
        with open(ros_file, 'r') as f:
            content = f.read()
        
        # Create a temporary modified version  
        temp_file = "/tmp/ros_test.py"
        
        # Replace problematic imports with try-except
        modified_content = content.replace(
            "import rospy",
            "try:\n    import rospy\nexcept ImportError:\n    rospy = None"
        ).replace(
            "from cv_bridge import CvBridge", 
            "try:\n    from cv_bridge import CvBridge\nexcept ImportError:\n    CvBridge = None"
        ).replace(
            "from sensor_msgs.msg import",
            "try:\n    from sensor_msgs.msg import"
        ).replace(
            "from geometry_msgs.msg import",
            "\nexcept ImportError:\n    pass\ntry:\n    from geometry_msgs.msg import"  
        ).replace(
            "from nav_msgs.msg import",
            "\nexcept ImportError:\n    pass\ntry:\n    from nav_msgs.msg import"
        ).replace(
            "from tf.transformations import euler_from_quaternion",
            "\nexcept ImportError:\n    pass\ntry:\n    from tf.transformations import euler_from_quaternion\nexcept ImportError:\n    euler_from_quaternion = None"
        )
        
        with open(temp_file, 'w') as f:
            f.write(modified_content)
            
        # Load the modified module
        spec = importlib.util.spec_from_file_location("ros_components", temp_file)
        ros_module = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(ros_module)
        
        # Extract components
        DataConverter = getattr(ros_module, 'DataConverter', None)
        DonkeyToROSHardwareController = getattr(ros_module, 'DonkeyToROSHardwareController', None)
        ROSMultiSensorBridge = getattr(ros_module, 'ROSMultiSensorBridge', None)
        ROSSensorDataAdapter = getattr(ros_module, 'ROSSensorDataAdapter', None)
        
        # Clean up
        if os.path.exists(temp_file):
            os.remove(temp_file)
            
        return DataConverter, DonkeyToROSHardwareController, ROSMultiSensorBridge, ROSSensorDataAdapter
        
    except Exception as e:
        print(f"{Colors.RED}Error loading ROS components: {e}{Colors.END}")
        return None, None, None, None

def test_component_loading():
    """Test if our components can be loaded"""
    print(f"{Colors.BOLD}Testing Component Loading...{Colors.END}")
    results = TestResults()
    
    DataConverter, DonkeyToROSHardwareController, ROSMultiSensorBridge, ROSSensorDataAdapter = load_ros_components()
    
    results.add_result("DataConverter loading", DataConverter is not None)
    results.add_result("DonkeyToROSHardwareController loading", DonkeyToROSHardwareController is not None)
    results.add_result("ROSMultiSensorBridge loading", ROSMultiSensorBridge is not None)
    results.add_result("ROSSensorDataAdapter loading", ROSSensorDataAdapter is not None)
    
    return results, (DataConverter, DonkeyToROSHardwareController, ROSMultiSensorBridge, ROSSensorDataAdapter)

def test_data_converter_functionality(DataConverter):
    """Test DataConverter basic functionality"""
    print(f"\n{Colors.BOLD}Testing DataConverter Functionality...{Colors.END}")
    results = TestResults()
    
    if not DataConverter:
        results.add_result("DataConverter functionality", False, "DataConverter not loaded")
        return results
    
    try:
        # Test instantiation
        converter = DataConverter()
        results.add_result("DataConverter instantiation", True)
        
        # Test method existence
        methods = ['donkey_control_to_twist', 'twist_to_donkey_control', 
                  'donkey_image_to_ros', 'ros_image_to_donkey',
                  'donkey_imu_to_ros', 'ros_imu_to_donkey']
        
        for method in methods:
            has_method = hasattr(converter, method)
            results.add_result(f"Method {method}", has_method)
        
        # Test control conversion logic (without ROS messages)
        if hasattr(converter, 'donkey_control_to_twist'):
            try:
                # This might work even without ROS if implementation is robust
                result = converter.donkey_control_to_twist(0.5, 0.3)
                results.add_result("Control conversion execution", True)
            except Exception as e:
                results.add_result("Control conversion execution", False, f"Expected without ROS: {e}")
        
    except Exception as e:
        results.add_result("DataConverter functionality", False, str(e))
    
    return results

def test_sensor_adapter_functionality(ROSSensorDataAdapter):
    """Test ROSSensorDataAdapter functionality"""
    print(f"\n{Colors.BOLD}Testing ROSSensorDataAdapter Functionality...{Colors.END}")
    results = TestResults()
    
    if not ROSSensorDataAdapter:
        results.add_result("ROSSensorDataAdapter functionality", False, "ROSSensorDataAdapter not loaded")
        return results
    
    try:
        # Test instantiation
        adapter = ROSSensorDataAdapter()
        results.add_result("ROSSensorDataAdapter instantiation", True)
        
        # Test method existence
        methods = ['convert_dict_to_scalars', 'prepare_for_tub']
        
        for method in methods:
            has_method = hasattr(adapter, method)
            results.add_result(f"Method {method}", has_method)
        
        # Test dict conversion
        if hasattr(adapter, 'convert_dict_to_scalars'):
            test_dict = {'sensor_type': 'imu', 'gyro_x': 0.1, 'gyro_y': 0.2}
            try:
                result = adapter.convert_dict_to_scalars(test_dict)
                results.add_result("Dict conversion execution", True)
            except Exception as e:
                results.add_result("Dict conversion execution", False, str(e))
        
        # Test tub preparation
        if hasattr(adapter, 'prepare_for_tub'):
            test_data = {'sensor_type': 'lidar', 'ranges': [1.0, 2.0, 3.0]}
            try:
                result = adapter.prepare_for_tub(test_data)
                results.add_result("Tub preparation execution", True)
            except Exception as e:
                results.add_result("Tub preparation execution", False, str(e))
        
    except Exception as e:
        results.add_result("ROSSensorDataAdapter functionality", False, str(e))
    
    return results

def test_hardware_controller_functionality(DonkeyToROSHardwareController):
    """Test DonkeyToROSHardwareController functionality"""
    print(f"\n{Colors.BOLD}Testing DonkeyToROSHardwareController Functionality...{Colors.END}")
    results = TestResults()
    
    if not DonkeyToROSHardwareController:
        results.add_result("DonkeyToROSHardwareController functionality", False, "DonkeyToROSHardwareController not loaded")
        return results
    
    try:
        # Test instantiation without ROS init
        controller = DonkeyToROSHardwareController(cmd_topic='/test_topic', init_ros=False)
        results.add_result("DonkeyToROSHardwareController instantiation", True)
        
        # Test method existence
        methods = ['run', 'shutdown']
        
        for method in methods:
            has_method = hasattr(controller, method)
            results.add_result(f"Method {method}", has_method)
        
        # Test run method (should handle gracefully without ROS)
        if hasattr(controller, 'run'):
            try:
                controller.run(0.5, 0.3)  # angle, throttle
                results.add_result("Run method execution", True)
            except Exception as e:
                results.add_result("Run method execution", False, str(e))
        
    except Exception as e:
        results.add_result("DonkeyToROSHardwareController functionality", False, str(e))
    
    return results

def test_component_integration(components):
    """Test component integration"""
    print(f"\n{Colors.BOLD}Testing Component Integration...{Colors.END}")
    results = TestResults()
    
    DataConverter, DonkeyToROSHardwareController, ROSMultiSensorBridge, ROSSensorDataAdapter = components
    
    # Test that components can work together
    if DataConverter and ROSSensorDataAdapter:
        try:
            converter = DataConverter()
            adapter = ROSSensorDataAdapter()
            
            # Test data flow simulation
            test_data = {'sensor_type': 'imu', 'gyro_x': 0.1}
            adapted_data = adapter.prepare_for_tub(test_data)
            
            results.add_result("DataConverter + ROSSensorDataAdapter integration", True)
        except Exception as e:
            results.add_result("DataConverter + ROSSensorDataAdapter integration", False, str(e))
    else:
        results.add_result("Component integration", False, "Required components not loaded")
    
    return results

def main():
    import argparse
    
    parser = argparse.ArgumentParser(description='Direct ROS Components Test')
    parser.add_argument('--verbose', action='store_true', help='Enable verbose output')
    
    args = parser.parse_args()
    
    print(f"{Colors.BOLD}DonkeyCar ROS Components Direct Test{Colors.END}")
    print("=" * 50)
    print(f"Python Version: {sys.version}")
    print()
    
    # Load and test components
    all_results = TestResults()
    
    # Test 1: Component Loading
    loading_results, components = test_component_loading()
    all_results.total += loading_results.total
    all_results.passed += loading_results.passed
    all_results.failed += loading_results.failed
    all_results.failures.extend(loading_results.failures)
    
    DataConverter, DonkeyToROSHardwareController, ROSMultiSensorBridge, ROSSensorDataAdapter = components
    
    if DataConverter:
        # Test 2: DataConverter
        dc_results = test_data_converter_functionality(DataConverter)
        all_results.total += dc_results.total
        all_results.passed += dc_results.passed
        all_results.failed += dc_results.failed
        all_results.failures.extend(dc_results.failures)
    
    if ROSSensorDataAdapter:
        # Test 3: ROSSensorDataAdapter
        adapter_results = test_sensor_adapter_functionality(ROSSensorDataAdapter)
        all_results.total += adapter_results.total
        all_results.passed += adapter_results.passed
        all_results.failed += adapter_results.failed
        all_results.failures.extend(adapter_results.failures)
    
    if DonkeyToROSHardwareController:
        # Test 4: DonkeyToROSHardwareController
        hc_results = test_hardware_controller_functionality(DonkeyToROSHardwareController)
        all_results.total += hc_results.total
        all_results.passed += hc_results.passed
        all_results.failed += hc_results.failed
        all_results.failures.extend(hc_results.failures)
    
    # Test 5: Integration
    integration_results = test_component_integration(components)
    all_results.total += integration_results.total
    all_results.passed += integration_results.passed
    all_results.failed += integration_results.failed
    all_results.failures.extend(integration_results.failures)
    
    # Final report
    success = all_results.summary()
    
    print(f"\n{Colors.BOLD}=== Component Test Report ==={Colors.END}")
    
    if all([DataConverter, DonkeyToROSHardwareController, ROSMultiSensorBridge, ROSSensorDataAdapter]):
        print(f"{Colors.GREEN}✓ All ROS integration components loaded successfully{Colors.END}")
        
        if success:
            print(f"{Colors.GREEN}✓ All component functionality tests passed{Colors.END}")
            print(f"{Colors.GREEN}Your ROS integration components are working correctly!{Colors.END}")
        else:
            print(f"{Colors.YELLOW}⚠ Components loaded but some functionality issues detected{Colors.END}")
    else:
        missing = []
        if not DataConverter: missing.append("DataConverter")
        if not DonkeyToROSHardwareController: missing.append("DonkeyToROSHardwareController")
        if not ROSMultiSensorBridge: missing.append("ROSMultiSensorBridge")
        if not ROSSensorDataAdapter: missing.append("ROSSensorDataAdapter")
        
        print(f"{Colors.RED}✗ Missing components: {', '.join(missing)}{Colors.END}")
    
    print(f"\n{Colors.BLUE}Note: This test validates component structure and basic functionality{Colors.END}")
    print(f"{Colors.BLUE}Full ROS integration requires: roscore + ROS messages{Colors.END}")
    
    sys.exit(0 if success else 1)

if __name__ == "__main__":
    main()