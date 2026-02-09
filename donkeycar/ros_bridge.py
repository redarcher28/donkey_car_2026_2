#!/usr/bin/env python3
"""
ROS Bridge for Donkeycar
将Donkeycar的模型输出转换为ROS消息格式并发送到小车
"""

import rospy
import numpy as np
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image
from donkeycar.parts.keras import KerasPilot
from cv_bridge import CvBridge
import threading
import time


class DonkeycarToROSBridge:
    """
    将Donkeycar的输出转换为ROS消息
    """

    def __init__(self, model_path, steering_topic='/cmd_vel',
                 image_topic='/camera/image_raw', throttle_scale=1.0):
        """
        初始化ROS桥接器

        参数:
            model_path: Donkeycar模型文件路径
            steering_topic: ROS控制话题（默认：/cmd_vel）
            image_topic: ROS图像话题（默认：/camera/image_raw）
            throttle_scale: 油门缩放系数
        """
        # 初始化ROS节点
        rospy.init_node('donkeycar_ros_bridge', anonymous=True)

        # 创建发布器
        self.cmd_pub = rospy.Publisher(steering_topic, Twist, queue_size=10)

        # 创建CV桥接器
        self.bridge = CvBridge()

        # 加载Donkeycar模型
        print(f"加载模型: {model_path}")
        self.pilot = KerasPilot()
        self.pilot.load(model_path)

        # 参数设置
        self.throttle_scale = throttle_scale
        self.running = True

        # 创建子线程处理图像订阅
        self.image_subscriber = threading.Thread(target=self.setup_image_subscriber,
                                                 args=(image_topic,))
        self.image_subscriber.daemon = True
        self.image_subscriber.start()

        print("ROS桥接器初始化完成")

    def setup_image_subscriber(self, image_topic):
        """
        设置图像订阅器（在子线程中运行）
        """
        rospy.Subscriber(image_topic, Image, self.image_callback)
        rospy.spin()

    def image_callback(self, ros_image):
        """
        处理ROS图像消息
        """
        try:
            # 将ROS图像转换为OpenCV格式
            cv_image = self.bridge.imgmsg_to_cv2(ros_image, "bgr8")

            # 预处理图像（与Donkeycar训练时相同）
            processed_image = self.preprocess_image(cv_image)

            # 使用模型进行预测
            if processed_image is not None:
                self.predict_and_publish(processed_image)

        except Exception as e:
            rospy.logerr(f"图像处理错误: {e}")

    def preprocess_image(self, image):
        """
        预处理图像以匹配模型输入
        """
        try:
            # 调整图像尺寸（根据你的模型输入尺寸修改）
            from cv2 import resize
            img = resize(image, (120, 160))  # Donkeycar常用尺寸

            # 归一化
            img = img / 255.0

            # 添加批次维度
            img = np.expand_dims(img, axis=0)

            return img

        except Exception as e:
            rospy.logerr(f"图像预处理错误: {e}")
            return None

    def predict_and_publish(self, image):
        """
        使用模型预测并发布ROS控制消息
        """
        try:
            # 模型预测
            predictions = self.pilot.model.predict(image)

            # 提取转向和油门值
            if len(predictions[0]) == 2:
                steering = float(predictions[0][0])
                throttle = float(predictions[0][1])
            else:
                # 如果模型只输出转向，使用固定油门
                steering = float(predictions[0][0])
                throttle = 0.3  # 固定油门值

            # 发布ROS消息
            self.publish_control(steering, throttle * self.throttle_scale)

        except Exception as e:
            rospy.logerr(f"预测错误: {e}")

    def publish_control(self, steering, throttle):
        """
        发布ROS控制消息
        """
        twist_msg = Twist()

        # 设置线速度和角速度
        # 这里根据你的小车运动模型进行调整
        twist_msg.linear.x = throttle  # 油门转换为线速度
        twist_msg.angular.z = steering  # 转向转换为角速度

        # 发布消息
        self.cmd_pub.publish(twist_msg)

        rospy.loginfo(f"控制命令: 转向={steering:.3f}, 油门={throttle:.3f}")

    def run(self):
        """
        运行主循环
        """
        print("ROS桥接器运行中...")
        try:
            while not rospy.is_shutdown() and self.running:
                time.sleep(0.1)
        except KeyboardInterrupt:
            print("正在关闭...")
        finally:
            self.running = False

    def shutdown(self):
        """
        关闭桥接器
        """
        self.running = False
        rospy.signal_shutdown("正常关闭")


# 车辆模板扩展：添加ROS支持
class ROSVehicle:
    """
    Donkeycar车辆模板，支持ROS输出
    """

    def __init__(self, model_path, ros_topic='/cmd_vel'):
        from donkeycar import Vehicle
        import rospy

        # 创建Donkeycar车辆
        self.V = Vehicle()

        # 初始化ROS
        rospy.init_node('donkeycar_vehicle', anonymous=True)
        self.cmd_pub = rospy.Publisher(ros_topic, Twist, queue_size=10)

        # 加载模型
        self.pilot = KerasPilot()
        self.pilot.load(model_path)

        # ROS消息转换器
        self.bridge = CvBridge()

        print("ROS车辆初始化完成")

    def add_ros_publisher(self):
        """
        添加ROS发布器到车辆
        """
        from donkeycar.parts.keras import KerasCategorical

        # 创建自定义部件
        class ROSPublisher:
            def run(self, steering, throttle):
                # 创建并发布ROS消息
                twist = Twist()
                twist.linear.x = throttle
                twist.angular.z = steering

                # 这里需要访问车辆的publisher
                # 实际实现中需要调整这部分代码
                return steering, throttle

        # 添加部件到车辆
        ros_part = ROSPublisher()
        self.V.add(ros_part, inputs=['pilot/angle', 'pilot/throttle'],
                   outputs=['ros/angle', 'ros/throttle'])

    def start(self):
        """
        启动车辆
        """
        self.V.start()


def create_ros_enabled_vehicle(config):
    """
    创建支持ROS的车辆配置
    """
    import rospy
    from geometry_msgs.msg import Twist

    # 创建车辆
    from donkeycar import Vehicle
    V = Vehicle()

    # 添加ROS控制发布器
    class ROSControlPublisher:
        def __init__(self, topic='/cmd_vel'):
            rospy.init_node('donkeycar_control', anonymous=True)
            self.publisher = rospy.Publisher(topic, Twist, queue_size=10)

        def run(self, angle, throttle):
            # 创建ROS消息
            msg = Twist()
            msg.linear.x = throttle
            msg.angular.z = angle

            # 发布消息
            self.publisher.publish(msg)
            return angle, throttle

    # 添加部件
    ros_pub = ROSControlPublisher(config['ros_topic'])
    V.add(ros_pub, inputs=['pilot/angle', 'pilot/throttle'],
          outputs=['ros_angle', 'ros_throttle'])

    return V


# 使用示例
if __name__ == "__main__":
    # 示例1：使用ROS桥接器
    model_path = "path/to/your/model.h5"  # 替换为你的模型路径
    bridge = DonkeycarToROSBridge(
        model_path=model_path,
        steering_topic='/donkeycar/cmd_vel',
        image_topic='/donkeycar/camera',
        throttle_scale=0.8
    )

    # 运行桥接器
    try:
        bridge.run()
    except KeyboardInterrupt:
        bridge.shutdown()