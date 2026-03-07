#!/usr/bin/env python3
# encoding: utf-8
# @data:2022/11/07
# @author:aiden
# yolov5目标检测(yolov5 target detection)
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
from example.yolov11_detect.yolo11_det_trt import YoLo11TRT,Colors,plot_one_box

MODE_PATH = os.path.split(os.path.realpath(__file__))[0]

class Yolov11Node(Node):
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
        engine = self.get_parameter('engine').value
        lib = self.get_parameter('lib').value
        conf_thresh = self.get_parameter('conf').value
        self.get_logger().info('\033[1;32m%s\033[0m' % str(conf_thresh))
        
        self.classes = self.get_parameter('classes').value
        
        self.yolo = YoLo11TRT(os.path.join(MODE_PATH, engine), os.path.join(MODE_PATH, lib), self.classes)
        self.colors = Colors()
        self.create_service(Trigger, '/yolo/start', self.start_srv_callback)  # 进入玩法(enter the game)
        self.create_service(Trigger, '/yolo/stop', self.stop_srv_callback)  # 退出玩法(exit the game)
        self.image_sub = self.create_subscription(Image, '/depth_cam/rgb0/image_raw', self.image_callback, 1)  # 摄像头订阅(subscribe to the camera)


        self.object_pub = self.create_publisher(ObjectsInfo, '~/object_detect', 1)
        self.result_image_pub = self.create_publisher(Image, '~/object_image', 1)
        threading.Thread(target=self.image_proc, daemon=True).start()
        self.create_service(Trigger, '~/init_finish', self.get_node_state)
        self.get_logger().info('\033[1;32m%s\033[0m' % 'start')

    def get_node_state(self, request, response):
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
                    boxes, scores, classid = self.yolo.infer_video(image)
                    for box, cls_conf, cls_id in zip(boxes, scores, classid):
                        color = self.colors(cls_id, True)
                        # color = common.colors(cls_id, True)
                        object_info = ObjectInfo()
                        object_info.class_name = self.classes[cls_id]
                        object_info.box = box.astype(int).tolist()
                        object_info.score = float(cls_conf)
                        object_info.width = w
                        object_info.height = h
                        objects_info.append(object_info)

                        plot_one_box(
                        box,
                        image,
                        color=color,
                        label="{} {:.2f}".format(
                            self.classes[cls_id], cls_conf
                        ),
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
    node = Yolov11Node('yolo')
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
