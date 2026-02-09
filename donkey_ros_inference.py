import rospy
import cv2
import numpy as np
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge
from donkeycar.parts.keras import KerasLinear

class DonkeyRosInference:
    def __init__(self):
        rospy.init_node('donkey_inference_node')
        
        # 1. 加载训练好的大脑 (.h5模型)
        self.model = KerasLinear()
        # 换成你刚才训练好的模型路径
        self.model.load('/home/roxy/pilot_from_bag.h5') 
        
        self.bridge = CvBridge()
        
        # 2. 订阅小车摄像头（获取图片）
        rospy.Subscriber('/usb_cam/image_raw', Image, self.callback)
        
        # 3. 创建发布者（发送指令）
        self.cmd_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
        
        print("AI 推理节点已启动，正在等待摄像头图像...")

    def callback(self, data):
        # A. 把 ROS 图片转为 OpenCV 图片
        img = self.bridge.imgmsg_to_cv2(data, "bgr8")
        img = cv2.resize(img, (160, 120)) # Donkeycar 默认模型输入大小
        
        # B. 让 AI 大脑预测动作
        # steering 是转向, throttle 是油门
        steering, throttle = self.model.run(img)
        
        # C. 转换为 ROS 兼容的数据 (Twist 消息)
        move_cmd = Twist()
        move_cmd.linear.x = float(throttle)   # 油门 -> 线速度
        move_cmd.angular.z = float(steering) * -1.0  # 转向 -> 角速度 (根据车况决定是否乘-1)
        
        # D. 发给小车
        self.cmd_pub.publish(move_cmd)
        rospy.loginfo(f"AI 决策: 转向={steering:.2f}, 油门={throttle:.2f}")

if __name__ == '__main__':
    try:
        node = DonkeyRosInference()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass