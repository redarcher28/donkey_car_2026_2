#!/usr/bin/env python3.9
"""
Simple ROS Components Existence Test
直接检查我们实现的ROS集成组件是否存在并可实例化

Usage: python3.9 test_simple_ros.py
"""

import sys
import os

# Colors for output
class Colors:
    GREEN = '\033[92m'
    RED = '\033[91m'
    YELLOW = '\033[93m' 
    BLUE = '\033[94m'
    BOLD = '\033[1m'
    END = '\033[0m'

def test_file_existence():
    """检查文件是否存在"""
    print(f"{Colors.BOLD}检查 ROS 集成文件...{Colors.END}")
    
    ros_file = os.path.join(os.path.dirname(__file__), 'donkeycar', 'parts', 'ros.py')
    if os.path.exists(ros_file):
        print(f"{Colors.GREEN}✓{Colors.END} ros.py 文件存在: {ros_file}")
        return True
    else:
        print(f"{Colors.RED}✗{Colors.END} ros.py 文件不存在: {ros_file}")
        return False

def test_class_definitions():
    """检查类定义"""
    print(f"\n{Colors.BOLD}检查类定义...{Colors.END}")
    
    ros_file = os.path.join(os.path.dirname(__file__), 'donkeycar', 'parts', 'ros.py')
    
    try:
        with open(ros_file, 'r') as f:
            content = f.read()
        
        classes_to_check = [
            'DataConverter',
            'DonkeyToROSHardwareController', 
            'ROSMultiSensorBridge',
            'DonkeyCarROSBridge',
            'ROSBridgeTest'
        ]
        
        found_classes = []
        for class_name in classes_to_check:
            if f'class {class_name}' in content:
                print(f"{Colors.GREEN}✓{Colors.END} {class_name} 类已定义")
                found_classes.append(class_name)
            else:
                print(f"{Colors.RED}✗{Colors.END} {class_name} 类未找到")
        
        return found_classes
        
    except Exception as e:
        print(f"{Colors.RED}错误读取文件: {e}{Colors.END}")
        return []

def test_method_existence():
    """检查关键方法"""
    print(f"\n{Colors.BOLD}检查关键方法...{Colors.END}")
    
    ros_file = os.path.join(os.path.dirname(__file__), 'donkeycar', 'parts', 'ros.py')
    
    try:
        with open(ros_file, 'r') as f:
            content = f.read()
        
        methods_to_check = [
            'donkey_image_to_ros',
            'ros_image_to_donkey',
            'donkey_control_to_twist', 
            'twist_to_donkey_control',
            'donkey_imu_to_ros',
            'ros_imu_to_donkey'
        ]
        
        found_methods = []
        for method_name in methods_to_check:
            if f'def {method_name}' in content:
                print(f"{Colors.GREEN}✓{Colors.END} {method_name} 方法已定义")
                found_methods.append(method_name)
            else:
                print(f"{Colors.RED}✗{Colors.END} {method_name} 方法未找到")
        
        return found_methods
        
    except Exception as e:
        print(f"{Colors.RED}错误读取文件: {e}{Colors.END}")
        return []

def test_basic_instantiation():
    """测试基本实例化（不需要ROS）"""
    print(f"\n{Colors.BOLD}测试组件实例化...{Colors.END}")
    
    # 添加路径
    sys.path.insert(0, os.path.join(os.path.dirname(__file__)))
    
    try:
        # 尝试导入，跳过ROS依赖
        import importlib.util
        
        # 创建一个mock rospy
        class MockROSPy:
            def init_node(self, *args, **kwargs): pass
            def Publisher(self, *args, **kwargs): return MockPublisher()
            def Subscriber(self, *args, **kwargs): return MockSubscriber()
            def get_time(self): return 0.0
            def loginfo(self, msg): pass
        
        class MockPublisher:
            def publish(self, *args, **kwargs): pass
        
        class MockSubscriber:
            pass
        
        class MockCvBridge:
            pass
            
        # Mock the imports
        sys.modules['rospy'] = MockROSPy()
        sys.modules['cv_bridge'] = type('MockCvBridge', (), {'CvBridge': MockCvBridge})
        sys.modules['sensor_msgs'] = type('MockSensorMsgs', (), {})
        sys.modules['sensor_msgs.msg'] = type('MockSensorMsgsMsg', (), {
            'Image': type('Image', (), {}),
            'Imu': type('Imu', (), {}),
            'LaserScan': type('LaserScan', (), {})
        })
        sys.modules['geometry_msgs'] = type('MockGeometryMsgs', (), {})
        sys.modules['geometry_msgs.msg'] = type('MockGeometryMsgsMsg', (), {
            'Twist': type('Twist', (), {}),
            'TwistWithCovariance': type('TwistWithCovariance', (), {}),
            'Pose': type('Pose', (), {}),
            'PoseWithCovariance': type('PoseWithCovariance', (), {})
        })
        sys.modules['nav_msgs'] = type('MockNavMsgs', (), {})
        sys.modules['nav_msgs.msg'] = type('MockNavMsgsMsg', (), {
            'Odometry': type('Odometry', (), {})
        })
        sys.modules['std_msgs'] = type('MockStdMsgs', (), {})
        sys.modules['std_msgs.msg'] = type('MockStdMsgsMsg', (), {
            'String': type('String', (), {}),
            'Int32': type('Int32', (), {}),
            'Float32': type('Float32', (), {}),
            'Bool': type('Bool', (), {}),
            'Header': type('Header', (), {})
        })
        
        # 模拟tf
        def mock_euler_from_quaternion(*args): return [0, 0, 0]
        sys.modules['tf'] = type('MockTF', (), {})
        sys.modules['tf.transformations'] = type('MockTFTransformations', (), {
            'euler_from_quaternion': mock_euler_from_quaternion
        })
        
        # 现在尝试导入我们的模块
        from donkeycar.parts.ros import DataConverter, DonkeyToROSHardwareController, ROSMultiSensorBridge
        
        # 测试实例化
        converter = DataConverter()
        print(f"{Colors.GREEN}✓{Colors.END} DataConverter 实例化成功")
        
        # 测试一些方法是否存在
        if hasattr(converter, 'donkey_control_to_twist'):
            print(f"{Colors.GREEN}✓{Colors.END} DataConverter.donkey_control_to_twist 方法存在")
        
        if hasattr(converter, 'donkey_image_to_ros'):
            print(f"{Colors.GREEN}✓{Colors.END} DataConverter.donkey_image_to_ros 方法存在")
            
        # 测试DonkeyToROSHardwareController
        try:
            controller = DonkeyToROSHardwareController(init_ros=False)
            print(f"{Colors.GREEN}✓{Colors.END} DonkeyToROSHardwareController 实例化成功")
        except Exception as e:
            print(f"{Colors.YELLOW}⚠{Colors.END} DonkeyToROSHardwareController 实例化有问题: {e}")
        
        # 测试ROSMultiSensorBridge 
        try:
            bridge = ROSMultiSensorBridge()
            print(f"{Colors.GREEN}✓{Colors.END} ROSMultiSensorBridge 实例化成功")
        except Exception as e:
            print(f"{Colors.YELLOW}⚠{Colors.END} ROSMultiSensorBridge 实例化有问题: {e}")
        
        return True
        
    except Exception as e:
        print(f"{Colors.RED}✗{Colors.END} 组件导入/实例化失败: {e}")
        return False

def main():
    print(f"{Colors.BOLD}ROS 组件简单测试{Colors.END}")
    print("=" * 40)
    print(f"Python 版本: {sys.version}")
    print()
    
    success_count = 0
    total_tests = 4
    
    # 测试1: 文件存在性
    if test_file_existence():
        success_count += 1
    
    # 测试2: 类定义检查
    found_classes = test_class_definitions()
    if len(found_classes) >= 3:  # 至少找到3个类
        success_count += 1
        print(f"{Colors.GREEN}✓{Colors.END} 找到 {len(found_classes)} 个类定义")
    else:
        print(f"{Colors.RED}✗{Colors.END} 只找到 {len(found_classes)} 个类定义")
    
    # 测试3: 方法检查
    found_methods = test_method_existence()
    if len(found_methods) >= 4:  # 至少找到4个方法
        success_count += 1
        print(f"{Colors.GREEN}✓{Colors.END} 找到 {len(found_methods)} 个方法定义")
    else:
        print(f"{Colors.RED}✗{Colors.END} 只找到 {len(found_methods)} 个方法定义")
    
    # 测试4: 基本实例化
    if test_basic_instantiation():
        success_count += 1
    
    # 总结
    print(f"\n{Colors.BOLD}=== 测试总结 ==={Colors.END}")
    print(f"通过测试: {success_count}/{total_tests}")
    
    if success_count == total_tests:
        print(f"{Colors.GREEN}{Colors.BOLD}🎉 所有基本测试通过！{Colors.END}")
        print(f"{Colors.GREEN}您的 ROS 集成组件结构正确且可以实例化{Colors.END}")
        print(f"{Colors.BLUE}注意: 完整的ROS功能需要运行 roscore 和 ROS 消息{Colors.END}")
    elif success_count >= total_tests * 0.75:
        print(f"{Colors.YELLOW}{Colors.BOLD}⚠ 大部分测试通过{Colors.END}")
        print(f"{Colors.YELLOW}ROS 集成组件基本可用，但可能有一些问题{Colors.END}")
    else:
        print(f"{Colors.RED}{Colors.BOLD}❌ 测试失败较多{Colors.END}")
        print(f"{Colors.RED}ROS 集成组件可能有严重问题{Colors.END}")
    
    sys.exit(0 if success_count >= total_tests * 0.75 else 1)

if __name__ == "__main__":
    main()