import os
import rosbag
import cv2
import numpy as np
import json

# 配置路径
BAG_FILE = 'robot_data_full.bag'
OUTPUT_DIR = 'training_data_from_bag'
IMAGE_TOPIC = '/usb_cam/image_raw' 
CMD_TOPIC = '/cmd_vel'

if not os.path.exists(OUTPUT_DIR):
    os.makedirs(OUTPUT_DIR)

bag = rosbag.Bag(BAG_FILE)
count = 0
last_steering = 0.0
last_throttle = 0.0

print("正在使用【手动解码模式】转换数据，避开 cv_bridge 冲突...")

try:
    for topic, msg, t in bag.read_messages(topics=[IMAGE_TOPIC, CMD_TOPIC]):
        if topic == CMD_TOPIC:
            last_steering = msg.angular.z
            last_throttle = msg.linear.x
            
        if topic == IMAGE_TOPIC:
            # --- 不使用 cv_bridge，直接手动解析字节流 ---
            # 假设图片是普通的 RGB8 或 BGR8 格式
            img_array = np.frombuffer(msg.data, dtype=np.uint8)
            cv_img = img_array.reshape(msg.height, msg.width, -1)
            
            # ROS 默认通常是 RGB，OpenCV 存图需要 BGR
            if msg.encoding == "rgb8":
                cv_img = cv2.cvtColor(cv_img, cv2.COLOR_RGB2BGR)
            
            img_name = f"{count}_cam-image_array_.jpg"
            img_path = os.path.join(OUTPUT_DIR, img_name)
            cv2.imwrite(img_path, cv_img)
            
            # 保存 Donkeycar JSON
            json_data = {
                "user/angle": last_steering,
                "user/throttle": last_throttle,
                "user/mode": "user",
                "cam/image_array": img_name
            }
            
            json_name = f"record_{count}.json"
            with open(os.path.join(OUTPUT_DIR, json_name), 'w') as f:
                json.dump(json_data, f)
                
            count += 1
            if count % 100 == 0:
                print(f"已提取 {count} 张图片...")
except Exception as e:
    print(f"转换中断: {e}")
finally:
    bag.close()

print(f"\n恭喜！转换完成。")
print(f"提取数据总量: {count}")
print(f"数据存放位置: {OUTPUT_DIR}")
