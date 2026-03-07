#!/usr/bin/env python3
# encoding: utf-8
# 城市交通

import os
import cv2
import math
import time
import queue
import rclpy
import threading
import numpy as np
import sdk.pid as pid
from rclpy.node import Node
import sdk.common as common
from app.common import Heart
from cv_bridge import CvBridge
from sensor_msgs.msg import Image, LaserScan
from geometry_msgs.msg import Twist
from interfaces.msg import ObjectsInfo
from std_srvs.srv import SetBool, Trigger
from sdk.common import colors, plot_one_box
from example.self_driving import lane_detect
from rclpy.executors import MultiThreadedExecutor
from servo_controller_msgs.msg import ServosPosition
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.qos import QoSProfile, QoSReliabilityPolicy
from speech import speech

wav_path = '/home/ubuntu/ros2_ws/src/example/example/self_driving/include'

MAX_SCAN_ANGLE = 110

class UrbanTraffic(Node):
    def __init__(self, name):
        super().__init__(name)

        self.image_queue = queue.Queue(maxsize=1)
        self.machine_type = os.environ.get('MACHINE_TYPE')

        self.declare_parameter('debug_mode', False)
        self.debug_mode = self.get_parameter('debug_mode').value
        self.declare_parameter('start', False)
        self.start = self.get_parameter('start').value
        self.declare_parameter('only_line_follow', False)
        self.only_line_follow = self.get_parameter('only_line_follow').value
        self.declare_parameter('depth_camera_name', 'depth_cam')  # 默认相机名称
        self.depth_camera_name = self.get_parameter('depth_camera_name').value
        self.declare_parameter('use_depth_cam', True)
        self.use_depth_cam = self.get_parameter('use_depth_cam').value


        self.lane_detect = lane_detect.LaneDetector("yellow")
        self.classes = ['left', 'right', 'red', 'green', 'yellow', 'one', 'two', 'three', 'four', 'duck', 'crosswalk']
        self.language = os.environ.get('ASR_LANGUAGE')
        self.is_running = True
        self.bridge = CvBridge()
        self.lock = threading.Lock()

        # Publishers, Subscribers, Services, Clients
        self.result_publisher = self.create_publisher(Image, '~/result_image', 1)
        self.cmd_vel_topic = '/controller/cmd_vel'
        self.debug_mode = self.get_parameter('debug_mode').value
        self.camera_adjust_finish = True
        if self.debug_mode:
            self.cmd_vel_topic = '/controller/cmd_vel1'
            # new param 
            self.camera_adjust = -1  # 摄像头角度调节
            self.camera_adjust_finish = False

        self.mecanum_pub = self.create_publisher(Twist, self.cmd_vel_topic, 1)
        self.joints_pub = self.create_publisher(ServosPosition, '/servo_controllers/goal_position', 1)  # 假设如果需要移植

        self.param_init()
        self.create_service(Trigger, '~/enter', self.enter_srv_callback)
        self.create_service(Trigger, '~/exit', self.exit_srv_callback)
        self.create_service(SetBool, '~/set_running', self.set_running_srv_callback)
        Heart(self, self.get_fully_qualified_name() + '/heartbeat', 5, self.exit_srv_callback)

        timer_cb_group = ReentrantCallbackGroup()
        self.get_logger().info('\033[1;32m%s\033[0m' % '[等待底盘服务启动]')
        self.client = self.create_client(Trigger, '/controller_manager/init_finish')
        self.client.wait_for_service()
        self.get_logger().info('\033[1;32m%s\033[0m' % '[底盘服务启动成功]')

        self.get_logger().info('\033[1;32m%s\033[0m' % '[等待Yolo服务启动]')
        self.yolo_client = self.create_client(Trigger, '/yolo/init_finish')
        self.yolo_client.wait_for_service()
        self.start_yolo_client = self.create_client(Trigger, '/yolo/start', callback_group=timer_cb_group)
        self.start_yolo_client.wait_for_service()
        self.stop_yolo_client = self.create_client(Trigger, '/yolo/stop', callback_group=timer_cb_group)
        self.stop_yolo_client.wait_for_service()
        self.get_logger().info('\033[1;32m%s\033[0m' % '[Yolo服务启动成功]')

        # 环形麦克风播报
        #threading.Thread(target=self.Mic_Play).start()
        # 初始化 init 函数
        self.timer = self.create_timer(0.0, self.init_process, callback_group=timer_cb_group)

    def param_init(self):

        self.sign_tag = ''
        self.sign_tag_record = ''
        if not self.camera_adjust_finish:
            self.current_mode = -1
        else:
            self.current_mode = 0


        # new param
        self.normal_speed = 0.15
        self.slow_speed = 0.1
        
        # mode0
        self.turn_sign_x = -1
        self.turn_sign_y = -1
        self.turn_sign_area = 0
        self.mode1_start_turn = False
        self.mode1_start_turn_direct = 1
        self.mode1_start_turn_count = 1


        # mode 1 巡线
        self.line_turn_time_stamp = 0
        self.start_turn_time_stamp = 0
        self.start_turn = False
        self.count_turn = 0
        self.delay_count = 0
        self.forward_count = 0
        self.right_turn_delay_count = 0
        self.right_turn_delay_thresh = 10
        self.next_mode_flag = False
        
        # mode 2 雷达避障
        self.laser_left_min = 0.0
        self.laser_right_min = 0.0
        self.start_obs_avoid = False
        self.start_obs_avoid_current = 0

        # duck
        self.duck_detect = False
        self.duck_detect_distance = -1
        self.duck_turn_count = 0
        
        # mode 3 红绿灯和转向指示灯
        self.traffic_area = -1
        self.traffic_color = ''

        # mode 4 转向指示牌
        self.guid_road_signs = ''
        self.guid_park_number = 2
        self.guid_park_detect_count = 0
        self.guid_road_detect = False
        self.guid_road_signs_x = 0
        self.guid_road_signs_y = 0
        self.guid_road_signs_area = -1
        self.guid_turn_count = 0


        # 入库
        self.park_1 = ''
        self.park_2 = ''
        self.park_3 = ''
        self.park_4 = ''
        self.park_left_right = ''
        self.park_direction = 1 # 默认向左
        self.park_direction_flag = False
        self.parking_detect_area = 0
        self.next_count = 0
        self.next_count_flag = False
        self.start_garge = False
        
        # 停车
        self.parking = False
        ## 停车三个动作
        self.parking_moving_1 = 0
        self.parking_moving_2 = 0
        self.parking_moving_3 = 0
        self.parking_moving_4 = 0
        
        # 出库
        self.out_ship_message = False
        self.outship_flag = False
        self.outship_detect = None
        self.outship_area = 0
        self.outship_count = 0
        self.outship_center_x = 0
        self.outship_center_y = 0
        self.go_out_mic_flag = False

        # 回到起始点（冲线停车）
        self.stop_count = 0
        self.stop_detect = False
        
        # 音频播放
        self.mic_play_flag = False
        self.mode_0_play = True
        self.mode_1_play = True
        self.mode_2_play = True
        self.mode_3_play = True
        self.mode_4_play = True
        self.mode_5_play = True
        self.mode_6_play = True
        self.mode_7_play = True
        self.mode_8_play = True
        self.mode_9_play = True
        self.mode_10_play = True
        self.traffic_voice_play = True
        self.mic_wav = ''
        self.language = os.environ.get('ASR_LANGUAGE')


        self.pid = pid.PID(0.005, 0.0, 0.0)
        self.pid_road_choose_z = pid.PID(0.01, 0.0, 0.0)
        self.pid_road_choose_x = pid.PID(0.001, 0.0, 0.0)
        self.park_number_choose = pid.PID(0.005, 0.0, 0.0)
        
        self.car_speed_slow = 0.05
        self.scan_angle = math.radians(360)

        self.objects_info = []

        self.road_choose = True
        self.road_choose_l_or_r = None
        self.road_choose_area = 0.0
        self.road_choose_mic = False
        self.road_choose_center = 0
        self.can_goto_lidar = False
        self.ending_orin_mic_flag = False

        self.start_turn_time_stamp = 0
        self.count_turn = 0
        self.line_message = []
        self.line_entry_finish = False
        self.left_turn_count = 0

        self.laser_turn_threshold = 0.3
        self.laser_left_min = 0.0
        self.laser_right_min = 0.0



        self.duck_detect = False
        self.duck_detect_distance = 0


        self.parking_detect_area = 0

        self.stop_2_center_x = 0
        self.stop_2_area = 0
        
        self.stop_3_center_x = 0
        self.stop_3_area = 0


        self.road_signs_center = pid.PID(0.1, 0.0, 0.0)
        self.road_signs_area = pid.PID(0.2, 0.0, 0.0)
        self.start_park_stop_pid_center = pid.PID(0.01, 0.0, 0.0)


    def init_process(self):
        self.timer.cancel()
        self.mecanum_pub.publish(Twist())
        if not self.get_parameter('only_line_follow').value:
            self.send_request(self.start_yolo_client, Trigger.Request())

        time.sleep(1)
        self.display = False
        if self.get_parameter('start').value:
            self.display = True
            self.enter_srv_callback(Trigger.Request(), Trigger.Response())
            request = SetBool.Request()
            request.data = True
            self.set_running_srv_callback(request, SetBool.Response())

        self.get_logger().info('\033[1;32m [Please wait YOLO START]\033[0m')
        self.client = self.create_client(Trigger, '/yolo/yolo_start_detect')
        self.client.wait_for_service()

        if self.debug_mode:
            threading.Thread(target=self.debug_function, daemon=True).start()
        else:
            # threading.Thread(target=self.audio_play).start() 
            threading.Thread(target=self.Entry, daemon=True).start()
        self.create_service(Trigger, '~/init_finish', self.get_node_state)
        self.get_logger().info('\033[1;32m%s\033[0m' % 'start')

    def get_node_state(self, request, response):
        response.success = True
        return response

    def send_request(self, client, msg):
        future = client.call_async(msg)
        while rclpy.ok():
            if future.done() and future.result():
                return future.result()

    def enter_srv_callback(self, request, response):
        self.get_logger().info('\033[1;32m%s\033[0m' % "urban traffic enter")
        with self.lock:
            self.start = False
            self.image_sub = self.create_subscription(Image, '/depth_cam/rgb0/image_raw', self.image_callback, 1)

            self.object_sub = self.create_subscription(ObjectsInfo, '/yolo/object_detect', self.get_object_callback, 1)
            self.mecanum_pub.publish(Twist())
            self.enter = True
            qos = QoSProfile(depth=1, reliability=QoSReliabilityPolicy.BEST_EFFORT)
            self.lidar_sub = self.create_subscription(LaserScan, '/scan_raw', self.lidar_callback,qos)
            
        response.success = True
        response.message = "enter"
        return response

    def exit_srv_callback(self, request, response):
        self.get_logger().info('\033[1;32m%s\033[0m' % "self driving exit")
        with self.lock:
            try:
                if hasattr(self, 'image_sub') and self.image_sub is not None:
                    self.destroy_subscription(self.image_sub)
                if hasattr(self, 'object_sub') and self.object_sub is not None:
                    self.destroy_subscription(self.object_sub)
                if hasattr(self, 'lidar_sub') and self.lidar_sub is not None:
                    self.destroy_subscription(self.lidar_sub)
            except Exception as e:
                self.get_logger().info('\033[1;32m%s\033[0m' % str(e))
            self.mecanum_pub.publish(Twist())
        self.param_init()

        response.success = True
        response.message = "exit"
        return response

    def set_running_srv_callback(self, request, response):
        self.get_logger().info('\033[1;32m%s\033[0m' % "set_running")
        with self.lock:
            self.start = request.data
            if not self.start:
                self.mecanum_pub.publish(Twist())
        response.success = True
        response.message = "set_running"
        return response

    def shutdown(self, signum, frame):
        self.is_running = False

    # 摄像头角度调节(Camera angle adjustment)
    def debug_function(self):
        debug_y_thre = 112
        debug_count = 0
        while self.is_running:
            if debug_count >= 50:
                # 标志摄像头调整完成（Logo camera adjustment completed）
                if not self.camera_adjust_finish:
                    self.camera_adjust_finish = True
                self.get_logger().info('\033[1;32m Adjust Finish\033[0m')
                break
            try:
                image = self.image_queue.get(block=True, timeout=1)
            except queue.Empty:
                if not self.is_running:
                    break
                else:
                    continue
            result_image = image.copy()
            result_image = cv2.cvtColor(result_image, cv2.COLOR_RGB2BGR)
            if self.start:
                h, w = image.shape[:2]
                if self.camera_adjust > 0:
                    if  abs(self.camera_adjust - debug_y_thre) < 20: # 在范围内
                        debug_count += 1
                        cv2.putText(result_image,
                                    'Hold the Camera Angle'
                                    ,org=(10, 30),               
                                    fontFace=cv2.FONT_HERSHEY_SIMPLEX,
                                    fontScale=0.8,
                                    color=(255, 0, 0),          
                                    thickness=2,
                                    lineType=cv2.LINE_AA
                        )
                        cv2.line(result_image,(1,self.camera_adjust),[w-1,self.camera_adjust],(255, 0, 0),2)
                    else:
                        debug_count = 0
                        cv2.putText(result_image,
                                    text=f'Y: {self.camera_adjust}',
                                    org=(10, 30),               
                                    fontFace=cv2.FONT_HERSHEY_SIMPLEX,
                                    fontScale=0.8,
                                    color=(0, 255, 0),          
                                    thickness=2,
                                    lineType=cv2.LINE_AA
                        )
                        cv2.line(result_image,(1,self.camera_adjust),[w-1,self.camera_adjust],(0, 255, 0),2)
                    self.get_logger().info('center_y: ' + str(self.camera_adjust))
                else:
                    self.get_logger().info('Please put the robot to the Start Place')
            else:
                time.sleep(0.01)
            
            cv2.imshow('result_image', result_image)
            cv2.waitKey(1)
        self.get_logger().info('You have completed the calibration!!!')
        self.mecanum_pub.publish(Twist())
        # rclpy.shutdown()



    def Entry(self):
        while self.is_running:
            try:
                image = self.image_queue.get(block=True, timeout=1)
                img_h, img_w = image.shape[:2]
            except queue.Empty:
                if not self.is_running:
                    break
                else:
                    continue
            result_image = image.copy()
            if self.start:
                twist = Twist()
                # twist.linear.x = 0.15
                # twist.angular.z = -0.55
                # 道路选择
                self.get_logger().info('\033[1;32m [self.current_mode:%d] \033[0m'% self.current_mode)
                if self.current_mode == 0:
                    if self.turn_sign_x > 0 and self.turn_sign_y > 0 and self.turn_sign_area > 0:
                        twist.linear.x = self.slow_speed
                        # 如果还没开启转弯
                        if not self.mode1_start_turn:
                            self.mode1_start_turn_count = 0

                            # 当大于一定值的时候开始进入转弯
                            if self.turn_sign_area > 1500:
                                self.mode1_start_turn = True
                            else:
                                self.get_logger().info('\033[1;32m Go\033[0m')
                                target_center_x = int(img_w*0.5)
                                self.pid_road_choose_z.SetPoint = target_center_x  # 保持正视路牌
                                if abs(self.turn_sign_x - target_center_x) < 20:
                                    self.turn_sign_x =target_center_x
                                self.pid_road_choose_z.update(self.turn_sign_x)
                                twist.angular.z = common.set_range(self.pid_road_choose_z.output, -0.1,0.1)

                    # 开始转弯（固定状态）
                    if self.mode1_start_turn:
                        self.mode1_start_turn_count += 1
                        if self.sign_tag == 'right':
                            if self.mode1_start_turn_count < 20:
                                twist.angular.z = -0.55 
                            if self.mode1_start_turn_count > 20 and self.mode1_start_turn_count < 50:
                                twist.angular.z = 0.0
                            if self.mode1_start_turn_count > 50:
                                twist.angular.z = 0.55
                        if self.sign_tag == 'left':
                            if self.mode1_start_turn_count < 20:
                                twist.angular.z = 0.55 
                            if self.mode1_start_turn_count > 20 and self.mode1_start_turn_count < 50:
                                twist.angular.z = 0.0
                            if self.mode1_start_turn_count > 50:
                                twist.angular.z = -0.45
                        # 超过一定值的时候，退出转弯的状态
                        if self.mode1_start_turn_count > 60:
                            self.mode1_start_turn_count = 0
                            self.mode1_start_turn = False
                            self.mecanum_pub.publish(Twist())
                            self.current_mode = 1 # 进入到巡线状态
                    self.mecanum_pub.publish(twist)

                # 巡线
                elif self.current_mode == 1:
                    twist = Twist()
                    twist.linear.x = self.normal_speed
                    binary_image = self.lane_detect.get_binary(image)
                    # cv2.imshow('bin_img',binary_image)
                    # cv2.waitKey(1)
                    result_image, lane_angle, lane_x, max_area = self.lane_detect(binary_image, image.copy())  # 在处理后的图上提取车道线中心(Obtain the center of the lane on the processed image)
                    self.get_logger().info('\033[1;32m [lane_x:%d] \033[0m'% lane_x)

                    if lane_x > 115:
                        self.count_turn += 1
                        if self.count_turn > 2 and not self.start_turn:  # 稳定转弯(stable turning)
                            self.start_turn = True
                            self.count_turn = 0
                            self.line_turn_time_stamp = time.time()
                        twist.angular.z = -0.50  # 转弯速度(turning speed)
                        self.delay_count = 0
                        if self.forward_count > 50:
                            self.right_turn_delay_count += 1
                        self.get_logger().info('\033[1;32m [self.right_turn_delay_count:%d] \033[0m'% self.right_turn_delay_count)

                    else:  # 直道由pid计算转弯修正(use PID algorithm to correct turns on a straight road)
                        self.count_turn = 0
                        self.delay_count += 1
                        self.forward_count += 1
                        self.right_turn_delay_count = 0
                        self.get_logger().info('\033[1;32m [self.line_turn_time_stamp:%s] \033[0m'% str(time.time() - self.line_turn_time_stamp))
                        if time.time() - self.line_turn_time_stamp >= 1.5 and self.start_turn:
                            self.start_turn = False
                        if time.time() - self.line_turn_time_stamp >= 3.5 and self.start_turn:
                            self.start_turn = False
                        if not self.start_turn:
                            self.pid.SetPoint = 85  # 在车道中间时线的坐标(the coordinate of the line while the robot is in the middle of the lane)
                            if abs(lane_x - 85) < 20:
                                lane_x = 85
                            if self.sign_tag == 'right':
                                if lane_x < 40: 
                                    lane_x = 85
                            self.pid.update(lane_x)
                            twist.angular.z = common.set_range(self.pid.output, -0.8, 0.8)
                    self.mecanum_pub.publish(twist)
                    self.get_logger().info('\033[1;32m [self.delay_count:%d] \033[0m'% self.delay_count)
                        # 超过多少的次数才能允许进入到下一个模式
                    
                    if self.right_turn_delay_count > self.right_turn_delay_thresh:
                        self.next_mode_flag = True
                    # 判断是否能进入到下一个模式
                    if self.next_mode_flag:
                        if self.sign_tag_record == 'right':
                            if self.delay_count > 2:
                                self.current_mode = 2
                                self.delay_count = 0
                                self.right_turn_delay_count = 0
                                self.mecanum_pub.publish(Twist())
                        elif self.sign_tag_record == 'left':
                            if self.delay_count > 35:
                                self.current_mode = 2
                                self.delay_count = 0
                                self.mecanum_pub.publish(Twist())

                # 雷达避障
                elif self.current_mode == 2:
                    if not self.start_obs_avoid:
                        twist = Twist()
                        sub_distance = self.laser_left_min - self.laser_right_min
                        self.get_logger().info('\033[1;32msub_distance: %.2f cm\033[0m' % (sub_distance))
                        if self.laser_left_min != 0.0 or self.laser_right_min != 0.0:
                            if (self.laser_left_min <= 20 or self.laser_right_min <=20) and abs(sub_distance) < 30:
                                self.get_logger().info('\033[1;32m [Start obs avoid]\033[0m' % (sub_distance))
                                self.start_obs_avoid = True
                        twist.linear.x = self.slow_speed
                        twist.angular.z = 0.0
                        self.mecanum_pub.publish(twist)
                    else:
                        self.start_obs_avoid_current += 1
                        if self.start_obs_avoid_current < 40:
                            self.car_move(0.0,-self.normal_speed,0.0)
                        elif self.start_obs_avoid_current >= 40 and self.start_obs_avoid_current <= 80:
                            self.car_move(self.normal_speed,0.0,0.0)
                        elif self.start_obs_avoid_current >= 80 and self.start_obs_avoid_current <= 85:
                            self.car_move(self.normal_speed,0.0,-0.05)
                        elif self.start_obs_avoid_current >= 85 and self.start_obs_avoid_current <= 125:
                            self.car_move(0.0,self.normal_speed,0.0)
                        elif self.start_obs_avoid_current >= 125:
                            if self.duck_detect_distance > 60 and self.duck_detect:
                                self.duck_turn_count += 1
                                self.car_move(0.0,0.0,-self.normal_speed)
                                if self.duck_turn_count >= 20:
                                    self.duck_turn_count = 0
                                    self.start_obs_avoid_current = 0
                                    self.current_mode = 3
                                twist_return = self.line_follow_function(image)

                                self.mecanum_pub.publish(twist_return)
                # 红绿灯
                elif self.current_mode == 3:
                    twist_return = self.line_follow_function(image)
                    self.mecanum_pub.publish(twist_return)
                    if self.guid_road_detect:
                        self.current_mode = 4

                # 道路选择(准备入库)
                elif self.current_mode == 4:
                    twist = Twist()
                    self.get_logger().info('\033[1;32m [self.guid_road_signs_area:%d]\033[0m' % self.guid_road_signs_area)
                    if self.guid_road_signs_area < 2000 and not self.start_garge:
                        self.park_number_choose.SetPoint = 300  # 在车道中间时线的坐标
                        if abs(self.guid_road_signs_x - 300) < 20:
                            self.guid_road_signs_x = 300
                        self.park_number_choose.update(self.guid_road_signs_x)
                        twist.linear.y = 0.0
                        twist.linear.x = self.normal_speed
                        twist.angular.z = common.set_range(self.park_number_choose.output, -0.5, 0.5)
                        self.get_logger().info('\033[1;32m [twist.angular.z: %f]\033[0m' % twist.angular.z)
                        self.mecanum_pub.publish(twist)
                    else:
                        self.start_garge = True
                        self.guid_turn_count += 1
                        self.car_move(self.slow_speed * 1.5,0.0,self.normal_speed*5)
                        if self.guid_turn_count > 25:
                            self.guid_turn_count = 0
                            self.current_mode = 5

                # 入库
                elif self.current_mode == 5:
                    twist = Twist()
                    twist.linear.x = self.slow_speed
                    area_2_3 = int((self.stop_2_area + self.stop_3_area)/2)
                    center_2_3 = int((self.stop_2_center_x + self.stop_3_center_x)/2)
                    self.get_logger().info('\033[1;32m [mode_5 area_2_3: %f]\033[0m' % area_2_3)
                    self.get_logger().info('\033[1;32m [mode_5 center_2_3: %f]\033[0m' % center_2_3)
                    # 校准车体
                    twist.linear.x = self.car_speed_slow
                    twist = self.road_signs_align(2100,area_2_3,320,center_2_3)

                    # 判断进入下一个模式的条件
                    if not self.park_direction_flag and not self.next_count_flag:
                        if self.stop_3_center_x != 0 and self.stop_2_center_x != 0:
                            if self.stop_3_center_x > self.stop_2_center_x:
                                # 1 2 3 4
                                self.park_left_right = 'left'
                            else:
                                # 4 3 2 1
                                self.park_left_right = 'right'
                            if self.park_left_right == 'left':
                                if self.guid_park_number <= 2:
                                    self.park_direction = 1
                                else:
                                    self.park_direction = -1
                            if self.park_left_right == 'right':
                                if self.guid_park_number <= 2:
                                    self.park_direction = -1
                                else:
                                    self.park_direction = 1
                            if area_2_3 > 1000:
                                self.park_direction_flag = True
                        self.mecanum_pub.publish(twist)
                    else:
                        if area_2_3 > 2000:
                            self.next_count_flag = True
                        else:
                            self.mecanum_pub.publish(twist)
                    if self.next_count_flag and self.park_direction_flag:
                        self.mecanum_pub.publish(Twist())
                        self.current_mode = 6

                # 停车
                elif self.current_mode == 6:
                    twist = Twist()
                    twist.linear.x = 0.0
                    twist.linear.z = 0.0
    
                    if self.guid_road_signs == self.park_signs and not self.parking:
                        twist.linear.y = twist.linear.y
                        self.start_park_stop_pid_center.SetPoint = 335
                        if abs(self.park_x - 335) < 10:
                            self.park_x = 335
                        self.start_park_stop_pid_center.update(self.park_x)
                        twist.linear.y = common.set_range(self.start_park_stop_pid_center.output, -0.1,0.1)
                        if abs(twist.linear.y) < 0.05:
                            twist.linear.y = 0.0
                            if not self.parking:
                                self.parking = True
                                if (self.guid_road_signs == 'four' and self.park_left_right == 'left') or (self.guid_road_signs == 'one' and self.park_left_right == 'right'):
                                    self.out_ship_message = True
                                else:
                                    self.out_ship_message = False
                                self.mecanum_pub.publish(Twist())
                            else:
                                self.get_logger().info('\033[1;32m [Parking Now:]\033[0m')
                    else:
                        twist.linear.y = self.slow_speed * self.park_direction
                    
                    if not self.parking:
                        self.mecanum_pub.publish(twist)
                    else:
                        if self.car_parking_function(40,45,70,45):
                            self.current_mode = 7
                            
                            
                # 出库
                elif self.current_mode == 7:
                    twist = Twist()
                    twist.linear.x = self.normal_speed
                    area = self.outship_area
                    center_x = self.outship_center_x

                    if area > 500 and not self.outship_flag:
                        twist.linear.x = self.slow_speed
                        self.get_logger().info('\033[1;32m [mode_7 area: %f]\033[0m' % area)
                        self.get_logger().info('\033[1;32m [mode_7 center_x: %f]\033[0m' % center_x)
                        # 校准车体
                        # 出库左转路标的面积
                        twist = self.road_signs_align(2100,area,320,center_x)

                        
                        if (twist.linear.x == 0.0 and twist.angular.z) == 0.0 and  area >= 2100:
                            self.outship_flag = True
                            if not self.mic_play_flag and self.mode_7_play:
                                self.mic_wav = 'go_out'
                                self.mode_7_play = False

                    if self.outship_flag:
                        self.get_logger().info('\033[1;32m [outship now]\033[0m')
                        self.get_logger().info('\033[1;32m [self.outship_count: %d]\033[0m' %self.outship_count)
                        self.outship_count += 1
                        
                        if self.outship_count < 25:
                            self.car_move(self.normal_speed * 1.2,0.0,self.normal_speed * 3)
                            
                        elif self.outship_count >= 25 and self.outship_count < 40:
                            self.car_move(self.normal_speed,0.0,0.0)
                        
                        elif self.outship_count >= 40 and self.outship_count < 65:
                            self.car_move(self.normal_speed * 1.2,0.0,-self.normal_speed * 3)
                            

                        elif self.outship_count >= 65:
                            self.current_mode = 8
                            self.outship_flag = False
                    else:
                        twist.linear.x = self.normal_speed
                        self.mecanum_pub.publish(twist)
                
                # 巡线出库 
                elif self.current_mode == 8:
                    twist_return = self.line_follow_function(image)
                    self.mecanum_pub.publish(twist_return)
                    if self.stop_detect:
                        self.current_mode = 9
                
                # 对齐冲线停止
                elif self.current_mode == 9:
                    stop_target_area = 1100
                    twist = Twist()
                    twist.linear.x = self.slow_speed
                    area = self.stop_area
                    center_x = self.stop_center_x
                    self.get_logger().info('\033[1;32m [mode_9 area: %f]\033[0m' % area)
                    self.get_logger().info('\033[1;32m [mode_9 center_x: %f]\033[0m' % center_x)
                    # 校准车体

                    if area <= stop_target_area:
                        twist = self.road_signs_align(stop_target_area,area,320,center_x)
                        self.mecanum_pub.publish(twist)
                    else:
                        self.mecanum_pub.publish(Twist())
                        self.current_mode = 10
                elif self.current_mode == 10:
                    if not self.mic_play_flag and self.mode_10_play:
                        self.mic_wav = 'end_orin'
                        self.mode_10_play = False

                if self.objects_info != []:
                    for i in self.objects_info:
                        box = i.box
                        class_name = i.class_name
                        cls_conf = i.score
                        cls_id = self.classes.index(class_name)
                        color = colors(cls_id, True)
                        plot_one_box(
                            box,
                            result_image,
                            color=color,
                            label="{}:{:.2f}".format(class_name, cls_conf),
                        )

            bgr_image = cv2.cvtColor(result_image, cv2.COLOR_RGB2BGR)
            cv2.imshow('result_image', bgr_image)
            self.result_publisher.publish(self.bridge.cv2_to_imgmsg(bgr_image, "bgr8"))
            cv2.waitKey(1)




    def image_callback(self, ros_image):  # 目标检查回调
        cv_image = self.bridge.imgmsg_to_cv2(ros_image, "rgb8")
        rgb_image = np.array(cv_image, dtype=np.uint8)
        if self.image_queue.full():
            # 如果队列已满，丢弃最旧的图像(if the queue is full, remove the oldest image)
            self.image_queue.get()
        # 将图像放入队列(put the image into the queue)
        self.image_queue.put(rgb_image)


    def get_object_callback(self, msg):
        self.objects_info = msg.objects
        if self.objects_info == []:
            return
        for obj in self.objects_info:
            obj_class = obj.class_name
            center = (int((obj.box[0] + obj.box[2])/2), int((obj.box[1] + obj.box[3])/2))
            area = abs(obj.box[0] - obj.box[2])*abs(obj.box[1] - obj.box[3])
            if self.debug_mode:
                area = abs(obj.box[0] - obj.box[2])*abs(obj.box[1] - obj.box[3])
                self.get_logger().info('\033[1;32m [obj_class%s: x%d, y%d,area%d]\033[0m' % (obj_class,center[0],center[1],area))

            if obj_class == 'left' or obj_class == 'right':
                self.turn_sign_x = center[0]
                self.turn_sign_y = center[1]
                self.turn_sign_area = abs(obj.box[0] - obj.box[2])*abs(obj.box[1] - obj.box[3])
                self.get_logger().info('\033[1;32m%d\033[0m' % self.turn_sign_area)
                self.sign_tag = obj_class

                # 如果是道路选择话，就需要记录当前的状态
                if self.current_mode == 0:
                    if self.turn_sign_area > 1000:
                        self.sign_tag_record = obj_class

                if not self.camera_adjust_finish:
                    self.camera_adjust = center[1]
                    
                if self.current_mode == 7:
                    self.outship_area = area
                    self.outship_center_x = center[0]
                    self.outship_center_y = center[1]

                if self.current_mode >= 8:
                    self.stop_count += 1
                    if self.stop_count >= 10:
                        self.stop_area = area
                        self.stop_center_x = center[0]
                        self.stop_center_y = center[1]
                        self.stop_detect = True
                    
            if obj_class == 'duck':
                self.duck_detect_distance = center[1]
                self.duck_detect = True
            if obj_class == 'red' or obj_class == 'green':
                self.traffic_color = obj_class
                # 计算面积
                self.traffic_area = area
                if not self.mic_play_flag and self.traffic_area > 800 and self.traffic_voice_play:
                    self.mic_wav = obj_class + '_light'
                    self.traffic_voice_play = False
                self.get_logger().info('\033[1;32m [self.traffic_area:] %f\033[0m' % self.traffic_area)

            if self.current_mode < 5:
                if obj_class == 'one' or obj_class == 'two' or obj_class == 'three' or obj_class == 'four':
                    self.guid_road_signs_area = area
                    self.get_logger().info('\033[1;32m [guid_road_signs_area:%d]\033[0m' % self.guid_road_signs_area)
                    self.guid_park_detect_count += 1
                    if self.guid_road_signs_area >= 800:
                        self.guid_road_signs = obj_class
                        if not self.mic_play_flag and self.mode_4_play:
                            self.mic_wav = 'stop_' + obj_class
                            self.mode_4_play = False

                        self.guid_road_signs_x = center[0]
                        self.guid_road_signs_y = center[1]
                        self.guid_road_detect = True
                        if obj_class == 'one':
                            self.guid_park_number = 1
                        elif obj_class == 'two':
                            self.guid_park_number = 2
                        elif obj_class == 'three':
                            self.guid_park_number = 3
                        elif obj_class == 'four':
                            self.guid_park_number = 4
                    else:
                        self.guid_road_detect = False

            if self.current_mode >= 5 and self.current_mode < 7:
                if obj_class == 'one' or obj_class == 'two' or obj_class == 'three' or obj_class == 'four':
                    if self.guid_road_signs == obj_class:
                        self.park_signs = obj_class
                        self.park_x = center[0]
                        self.parking_detect_area =  area
                        self.get_logger().info('\033[1;32m [self.parking_detect_area:%d]\033[0m' % self.parking_detect_area)

                if obj_class == 'two':
                    self.stop_2_center_x = center[0]
                    self.stop_2_area = area
                if obj_class == 'three':
                    self.stop_3_center_x = center[0]
                    self.stop_3_area = area
                self.get_logger().info('\033[1;32m [self.stop_3_center_x:%d]\033[0m' % self.stop_3_center_x)
                self.get_logger().info('\033[1;32m [self.stop_2_center_x:%d]\033[0m' % self.stop_2_center_x)
                
                self.get_logger().info('\033[1;32m [self.stop_3_area:%d]\033[0m' % self.stop_3_area)
                self.get_logger().info('\033[1;32m [self.stop_2_area:%d]\033[0m' % self.stop_2_area)        
            # self.get_logger().info('\033[1;32m%s\033[0m' % obj_class)


    def lidar_callback(self, lidar_data):
        
        max_index = int(math.radians(MAX_SCAN_ANGLE / 2.0) / lidar_data.angle_increment)
        left_ranges = lidar_data.ranges[:max_index]  # 左半边数据 (the left data)
        right_ranges = lidar_data.ranges[::-1][:max_index]  # 右半边数据 (the right data)
        angle = self.scan_angle / 2
        angle_index = int(angle / lidar_data.angle_increment + 0.50)
        left_range, right_range = np.array(left_ranges[:angle_index]), np.array(right_ranges[:angle_index])

        left_nonzero = left_range.nonzero()
        right_nonzero = right_range.nonzero()
        left_nonan = np.isfinite(left_range[left_nonzero])
        right_nonan = np.isfinite(right_range[right_nonzero])
        min_dist_left_ = left_range[left_nonzero][left_nonan]
        min_dist_right_ = right_range[right_nonzero][right_nonan]

        self.laser_left_min = min_dist_left_.min() * 100  # 左边最近的距离
        self.laser_right_min = min_dist_right_.min() * 100  # 右边最近的距离
        if self.current_mode == 2:
            self.get_logger().info(
                '\033[1;32mLeft: %.2f cm, Right: %.2f cm\033[0m' % 
                (self.laser_left_min, self.laser_right_min)
            )

    def car_move(self,x_speed,y_speed,z_speed):
        twist = Twist()
        twist.linear.x = x_speed
        twist.linear.y = y_speed
        twist.angular.z = z_speed
        self.mecanum_pub.publish(twist)

    def car_parking_function(self,pk1_count,pk2_count,pk3_count,pk4_count):
        # 每一个动作依次执行，用 count 进行计数
        # →
        if self.parking_moving_1 < pk1_count:
            self.parking_moving_1 += 1
            self.car_move(0.0,0.0,-0.5)
        else:
            if self.parking_moving_2 < pk2_count:
                # 平移 ↑
                self.parking_moving_2 += 1
                self.car_move(0.0,0.1,0.0)
            else:
                if self.parking_moving_3 < pk3_count:
                    self.parking_moving_3 += 1
                    self.mecanum_pub.publish(Twist())     # Stop
                else:
                    # 如果无法满足直接出库的条件
                    if not self.out_ship_message:
                        if self.parking_moving_4 < pk4_count:
                            self.parking_moving_4 += 1
                            self.car_move(0.0,-0.1,0.0)     # 出库
                        else:
                            return True
                    else:
                        self.parking_moving_1 = 0
                        self.parking_moving_2 = 0
                        self.parking_moving_3 = 0
                        self.parking_moving_4 = 0
                        return True


    def road_signs_align(self,area_target,area,center_x_target,center_x):
        twist = Twist()
        twist.linear.x = self.slow_speed
        self.road_signs_area.SetPoint = area_target
        if abs(area - area_target) < 150:
            area = area_target
        self.road_signs_area.update(area)
        twist.linear.x = common.set_range(self.road_signs_area.output, -self.slow_speed,self.slow_speed)
        if abs(twist.linear.x) < 0.05:
            twist.linear.x = 0.0
        
        # 停入到车库里面 2-3 的center的position
        self.road_signs_center.SetPoint = center_x_target  # 在车道中间时线的坐标
        if abs(center_x - center_x_target) < 20:
            center_x = center_x_target    
        self.road_signs_center.update(center_x)

        twist.angular.z = common.set_range(self.road_signs_center.output, -0.1, 0.1)
        twist.linear.y = 0.0
        return twist
        
        
    def line_follow_function(self,image):
        # 距离红灯太近就停下
        if self.traffic_color == 'red' and self.traffic_area > 900:
            return Twist()
        twist = Twist()
        twist.linear.x = self.normal_speed
        
        binary_image = self.lane_detect.get_binary(image)
        result_image, lane_angle, lane_x, max_area = self.lane_detect(binary_image, image.copy())  # 在处理后的图上提取车道线中心(Obtain the center of the lane on the processed image)
        if lane_x > 115:
            self.count_turn += 1
            if self.count_turn > 5 and not self.start_turn:  # 稳定转弯(stable turning)
                self.start_turn = True
                self.count_turn = 0
                self.start_turn_time_stamp = time.time()
            twist.angular.z = -0.50  # 转弯速度(turning speed)
        else:  # 直道由pid计算转弯修正(use PID algorithm to correct turns on a straight road)
            self.count_turn = 0
            if time.time() - self.start_turn_time_stamp > 2 and self.start_turn:
                self.start_turn = False
            if not self.start_turn:
                self.pid.SetPoint = 85  # 在车道中间时线的坐标(the coordinate of the line while the robot is in the middle of the lane)
                if abs(lane_x - 85) < 20:
                    lane_x = 85
                if lane_x < 5:
                    lane_x = 85
                self.pid.update(lane_x)
                twist.angular.z = common.set_range(self.pid.output, -0.8, 0.8)

        return twist
    


    def get_path(self,f):
        if self.language == 'Chinese':
            return os.path.join(wav_path, f + '.wav')
        else:    
            return os.path.join(wav_path, 'english', f + '_en'+'.wav')

    def play(self,voice, volume=80):
        try:
            speech.set_volume(volume)
            # os.system('amixer -q -D pulse set Master {}%'.format(volume))
            speech.play_audio(self.get_path(voice))
            # os.system('aplay -q -Dplughw:1,0 ' + get_path(voice, language))
        except BaseException as e:
            print('error', e)

    def audio_play(self):
        while self.is_running:
            try:
                if self.turn_sign_area > 100 and self.current_mode == 0:
                    if not self.mic_play_flag and self.mode_0_play:
                        self.mic_wav = 'turn_' + self.sign_tag + '_orin'
                        self.mode_0_play = False
                if self.mic_play_flag:
                    self.play(self.mic_wav)
                    self.mic_play_flag = False
                time.sleep(0.2)
            except queue.Empty:
                if not self.is_running:
                    break
                else:
                    continue

def main(args=None):
    rclpy.init(args=args)
    try:
        node = UrbanTraffic(name='urban_traffic')
        executor = MultiThreadedExecutor()
        executor.add_node(node)
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
