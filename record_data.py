#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
import message_filters
import numpy as np
import os
import cv2
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge

# 数据保存配置
OUTPUT_DIR = "training_data"
if not os.path.exists(OUTPUT_DIR):
    os.makedirs(OUTPUT_DIR)

class DataRecorder:
    def __init__(self):
        rospy.init_node('data_recorder_sync', anonymous=True)
        self.bridge = CvBridge()
        self.image_buffer = []
        self.cmd_buffer = []
        self.count = 0
        self.lateset_twist = Twist()
        
        rospy.Subscriber('/cmd_vel', Twist, self.cmd_cb)
        rospy.Subscriber('/image', Image, self.img_cb)
        
        rospy.loginfo("简单录制模式已启动，正在监听数据...")
        
        rospy.loginfo("数据录制节点已启动，正在等待同步数据...")
        
# 只要手柄动了，就更新指令
    def cmd_cb(self, msg):
        self.latest_twist = msg

    # 只要摄像头画面来了，就立刻把画面和【当前最新】的指令打包存起来
    def img_cb(self, img_msg):
        print("Recive the photo!")
        try:
            cv_image = self.bridge.imgmsg_to_cv2(img_msg, "bgr8")
            
            # 提取当前最新的指令
            linear_x = self.latest_twist.linear.x
            angular_z = self.latest_twist.angular.z
            
            self.image_buffer.append(cv_image)
            self.cmd_buffer.append(np.array([linear_x, angular_z]))
            
            self.count += 1
            if self.count % 50 == 0:
                print(f">>> 已收集 {self.count} 组数据...")
        except Exception as e:
            rospy.logerr(f"处理出错: {e}")   



    def save_data(self):
        if len(self.image_buffer) > 0:
            print(f"\n正在保存 {len(self.image_buffer)} 组数据到 {OUTPUT_DIR}...")
            images = np.array(self.image_buffer)
            cmds = np.array(self.cmd_buffer)
            
            save_path = os.path.join(OUTPUT_DIR, "yeahbot_dataset.npz")
            np.savez_compressed(save_path, images=images, commands=cmds)
            print(f"✅ 保存成功！文件名: {save_path}")
        else:
            print("❌ 未接收到任何数据，取消保存。")

if __name__ == '__main__':
    recorder = DataRecorder()
    # 只要节点没关闭，就一直运行
    rospy.spin()
    # 当 Ctrl+C 关闭节点时，执行保存逻辑
    recorder.save_data()
