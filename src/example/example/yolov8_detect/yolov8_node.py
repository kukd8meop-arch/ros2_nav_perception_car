#!/usr/bin/env python3
# encoding: utf-8
# @data:2022/11/07
# @author:aiden
# yolov8目标检测(yolov8 target detection)

import os
import cv2
import time
import queue
import rclpy
import signal
import threading
import numpy as np
import sdk.fps as fps
from sdk import common
from rclpy.node import Node
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from std_srvs.srv import Trigger
from interfaces.msg import ObjectInfo, ObjectsInfo
from ultralytics import YOLO
import logging
logging.getLogger('ultralytics').setLevel(logging.ERROR)

from example.yolov8_detect.utils import Colors,plot_one_box

MODE_PATH = os.path.split(os.path.realpath(__file__))[0] + '/models'

class Yolov8Node(Node):
    def __init__(self, name):
        rclpy.init()
        super().__init__(name, allow_undeclared_parameters=True, automatically_declare_parameters_from_overrides=True)
        
        self.bgr_image = None
        self.start = False
        self.running = True

        self.bridge = CvBridge()
        self.image_queue = queue.Queue(maxsize=2)
        signal.signal(signal.SIGINT, self.shutdown)
        
        self.fps = fps.FPS()  # fps计算器(FPS calculator)
        # engine = self.get_parameter('engine').value
        # lib = self.get_parameter('lib').value
        conf_thresh = self.get_parameter('conf').value
        self.get_logger().info('\033[1;32m%s\033[0m' % str(conf_thresh))
        
        self.classes = self.get_parameter('classes').value
        # 配置参数
        self.model_name = self.get_parameter('model_name').value # 模型名称
        
        self.conf = self.get_parameter('conf').value             # 置信度阈值

        self.yolo_detect = YOLO(( MODE_PATH + "/" + self.model_name), task='detect', verbose=False)

        self.colors = Colors()
        self.create_service(Trigger, '/yolo/start', self.start_srv_callback)  # 进入玩法(enter the game)
        self.create_service(Trigger, '/yolo/stop', self.stop_srv_callback)  # 退出玩法(exit the game)
        self.image_sub = self.create_subscription(Image, '/depth_cam/rgb0/image_raw', self.image_callback, 1)  # 摄像头订阅(subscribe to the camera)

        self.yolo_detet_flag = False
        self.object_pub = self.create_publisher(ObjectsInfo, '~/object_detect', 1)
        self.result_image_pub = self.create_publisher(Image, '~/object_image', 1)
        threading.Thread(target=self.image_proc, daemon=True).start()
        self.create_service(Trigger, '~/init_finish', self.get_node_state)
        self.get_logger().info('\033[1;32m%s\033[0m' % 'start')

    def get_node_state(self, request, response):
        response.success = True
        return response

    def yolo_detect_start(self, request, response):
        response.success = True
        return response
    
    def start_srv_callback(self, request, response):
        self.get_logger().info('\033[1;32m%s\033[0m' % "start yolo detect")

        self.start = True
        response.success = True
        response.message = "start"
        return response

    def stop_srv_callback(self, request, response):
        self.get_logger().info('\033[1;32m%s\033[0m' % "stop yolo detect")

        self.start = False
        response.success = True
        response.message = "start"
        return response

    def image_callback(self, ros_image):
        cv_image = self.bridge.imgmsg_to_cv2(ros_image, "bgr8")
        bgr_image = np.array(cv_image, dtype=np.uint8)
        if self.image_queue.full():
            # 如果队列已满，丢弃最旧的图像(if the queue is full, remove the oldest image)
            self.image_queue.get()
            # 将图像放入队列(put the image into the queue)
        self.image_queue.put(bgr_image)

    def shutdown(self, signum, frame):
        self.running = False
        self.get_logger().info('\033[1;32m%s\033[0m' % "shutdown")

    def image_proc(self):
        while self.running:
            try:
                image = self.image_queue.get(block=True, timeout=1)
            except queue.Empty:
                if not self.running:
                    break
                else:
                    continue
            try:
                if self.start:
                    objects_info = []
                    h, w = image.shape[:2]
                    results = self.yolo_detect(image, conf=self.conf)
                    if not self.yolo_detet_flag:
                        self.create_service(Trigger, '~/yolo_start_detect', self.yolo_detect_start)
                        self.yolo_detet_flag = True
                    for result in results:
                        boxes = result.boxes
                        if boxes is not None and len(boxes) > 0:
                            # 遍历每个检测到的物体
                            for i in range(len(boxes)):
                                box_coords = boxes.xyxy[i].cpu().numpy().astype(int).tolist() # 获取 [x1, y1, x2, y2] 坐标，并转为int列表
                                confidence = boxes.conf[i].item() # 获取置信度
                                class_id = int(boxes.cls[i].item()) # 获取类别 ID
                                
                                # 确保 class_id 在 self.classes 的有效范围内
                                if class_id < len(self.classes):
                                    class_name = self.classes[class_id]
                                else:
                                    class_name = f"Unknown Class {class_id}"
                                    self.get_logger().warn(f"检测到未知类别ID: {class_id}")

                                # 构建 ObjectInfo 消息
                                object_info = ObjectInfo()
                                object_info.class_name = class_name
                                object_info.box = box_coords 
                                object_info.score = float(confidence)
                                object_info.width = w
                                object_info.height = h
                                objects_info.append(object_info)

                                # 绘制单个检测框
                                color = self.colors(class_id, True) # 获取 BGR 颜色
                                plot_one_box(
                                    box_coords, # 传入 [x1, y1, x2, y2]
                                    image,
                                    color=color,
                                    label=f"{class_name} {confidence:.2f}"
                                )
                    object_msg = ObjectsInfo()
                    object_msg.objects = objects_info
                    self.object_pub.publish(object_msg)
                else:
                    time.sleep(0.01)
            except BaseException as e:
                print('error', e)

            self.fps.update()
            result_image = self.fps.show_fps(image)
            self.result_image_pub.publish(self.bridge.cv2_to_imgmsg(result_image, "bgr8"))
        else:
            time.sleep(0.01)
        self.yolo.destroy() 
        rclpy.shutdown()

def main():
    node = Yolov8Node('yolo')
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
