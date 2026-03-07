#!/usr/bin/env python3
# encoding: utf-8
import os
import math
import rclpy
import numpy as np
from enum import Enum
from rclpy.node import Node
from sdk.common import val_map
from std_srvs.srv import Trigger
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist
from visualization_msgs.msg import Marker, MarkerArray
from ros_robot_controller_msgs.msg import BuzzerState
from ros_robot_controller_msgs.msg import BuzzerState, SetPWMServoState, PWMServoState

# 添加手柄数据获取相关的导入
import time
import threading
from std_msgs.msg import UInt16, Bool
from ros_robot_controller.ros_robot_controller_sdk import Board
from ros_robot_controller_msgs.srv import GetBusServoState, GetPWMServoState
from ros_robot_controller_msgs.msg import ButtonState, LedState, MotorsState, BusServoState, SetBusServoState, ServosPosition, Sbus, OLEDState

AXES_MAP = 'lx', 'ly', 'rx', 'ry', 'r2', 'l2', 'hat_x', 'hat_y'
BUTTON_MAP = 'cross', 'circle', '', 'square', 'triangle', '', 'l1', 'r1', 'l2', 'r2', 'select', 'start', '', 'l3', 'r3', '', 'hat_xl', 'hat_xr', 'hat_yu', 'hat_yd', ''


class ButtonState(Enum):
    Normal = 0
    Pressed = 1
    Holding = 2
    Released = 3

class JoystickController(Node):
    def __init__(self, name):
        rclpy.init()
        super().__init__(name)

        # 初始化手柄板卡
        self.board = Board()
        self.board.enable_reception()
        
        # 创建手柄数据发布器
        self.joy_pub = self.create_publisher(Joy, 'ros_robot_controller/joy', 1)
        
        # 创建手柄数据发布线程
        self.joy_pub_thread = threading.Thread(target=self.publish_joy_data)
        self.joy_pub_thread.daemon = True
        self.joy_pub_thread.start()

        self.min_value = 0.1
        self.declare_parameter('max_linear', 0.1)
        self.declare_parameter('max_angular', 0.7)
        self.declare_parameter('disable_servo_control', True)

        self.max_linear = self.get_parameter('max_linear').value
        self.max_angular = self.get_parameter('max_angular').value
        self.disable_servo_control = self.get_parameter('disable_servo_control').value
        self.machine = os.environ['MACHINE_TYPE']
        self.get_logger().info('\033[1;32m%s\033[0m' % self.max_linear)
        self.get_logger().info('\033[1;32m%s\033[0m' % self.max_angular)
        self.servo_state_pub = self.create_publisher(SetPWMServoState, 'ros_robot_controller/pwm_servo/set_state', 1)
        self.joy_sub = self.create_subscription(Joy, 'ros_robot_controller/joy', self.joy_callback, 1)
        self.buzzer_pub = self.create_publisher(BuzzerState, 'ros_robot_controller/set_buzzer', 1)
        self.mecanum_pub = self.create_publisher(Twist, 'controller/cmd_vel', 1)

        self.mark_pub = self.create_publisher(MarkerArray, 'path_point', 1)

        self.last_axes = dict(zip(AXES_MAP, [0.0, ] * len(AXES_MAP)))
        self.last_buttons = dict(zip(BUTTON_MAP, [0.0, ] * len(BUTTON_MAP)))
        self.mode = 0
        
        # 添加状态跟踪变量
        self.last_twist = Twist()
        self.has_active_input = False
        self.zero_twist_published = False  # 标记是否已经发布过停止消息

        self.client = self.create_client(Trigger, 'llm_control_move/joy_up')

        self.garb_pick_client = self.create_client(Trigger, 'llm_control_move/garb_pick')
        self.garb_place_client = self.create_client(Trigger, 'llm_control_move/garb_place')

        self.create_service(Trigger, '~/init_finish', self.get_node_state)
        self.get_logger().info('\033[1;32m%s\033[0m' % 'start')

    def publish_joy_data(self):
        """发布手柄数据的线程函数"""
        while rclpy.ok():
            try:
                data = self.board.get_gamepad()
                if data is not None:
                    msg = Joy()
                    msg.axes = data[0]
                    msg.buttons = data[1]
                    msg.header.stamp = self.get_clock().now().to_msg()
                    self.joy_pub.publish(msg)
                time.sleep(0.01)  # 100Hz 发布频率
            except Exception as e:
                self.get_logger().error(f'发布手柄数据时出错: {str(e)}')
                time.sleep(0.1)

    def get_node_state(self, request, response):
        response.success = True
        return response

    def call_joy_up(self):
        request = Trigger.Request()
        future = self.client.call_async(request)
        future.add_done_callback(self.joy_up_callback)

    def call_garb_pick(self):
        request = Trigger.Request()
        future = self.garb_pick_client.call_async(request)
        future.add_done_callback(self.service_callback)

    def call_garb_place(self):
        request = Trigger.Request()
        future = self.garb_place_client.call_async(request)
        future.add_done_callback(self.service_callback)

    def joy_up_callback(self, future):
        try:
            response = future.result()
        except Exception as e:
            self.get_logger().error(f'服务调用异常: {str(e)}')
        finally:
            self.hat_x_cooldown = False  # 重置冷却标志

    def service_callback(self, future):
        """通用服务回调函数"""
        try:
            response = future.result()
        except Exception as e:
            self.get_logger().error(f'服务调用异常: {str(e)}')

    def is_twist_zero(self, twist):
        """检查Twist消息是否为零（无运动）"""
        return (abs(twist.linear.x) < 1e-6 and 
                abs(twist.linear.y) < 1e-6 and 
                abs(twist.linear.z) < 1e-6 and
                abs(twist.angular.x) < 1e-6 and
                abs(twist.angular.y) < 1e-6 and
                abs(twist.angular.z) < 1e-6)

    def axes_callback(self, axes):
        twist = Twist()
        has_valid_input = False
        
        # 检查是否有有效的摇杆输入
        if (abs(axes['lx']) >= self.min_value or 
            abs(axes['ly']) >= self.min_value or 
            abs(axes['rx']) >= self.min_value or 
            abs(axes['ry']) >= self.min_value):
            has_valid_input = True
        
        if has_valid_input:
            # 处理摇杆死区
            lx = axes['lx'] if abs(axes['lx']) >= self.min_value else 0
            ly = axes['ly'] if abs(axes['ly']) >= self.min_value else 0
            rx = axes['rx'] if abs(axes['rx']) >= self.min_value else 0
            ry = axes['ry'] if abs(axes['ry']) >= self.min_value else 0
            
            if 'Mecanum' in self.machine or 'Differential' in self.machine:
                twist.linear.y = val_map(lx, -1, 1, -self.max_linear, self.max_linear) 
                twist.linear.x = val_map(ly, -1, 1, -self.max_linear, self.max_linear)
                twist.angular.z = val_map(rx, -1, 1, -self.max_angular, self.max_angular)
            elif 'Acker' in self.machine:
                twist.linear.x = val_map(ly, -1, 1, -self.max_linear, self.max_linear)
                steering_angle = val_map(rx, -1, 1, -math.radians(350/2000*180), math.radians(350/2000*180))
                
                if steering_angle == 0:  
                    twist.angular.z = 0.0
                    if not self.disable_servo_control:
                        servo_state = PWMServoState()
                        servo_state.id = [1]
                        servo_state.position = [1500] 
                        data = SetPWMServoState()
                        data.state = [servo_state]
                        data.duration = 0.02
                        self.servo_state_pub.publish(data)
                else:
                    R = 0.17706/math.tan(steering_angle)
                    twist.angular.z = float(twist.linear.x / R)  

                    if not self.disable_servo_control:
                        servo_state = PWMServoState()
                        servo_state.id = [1]
                        servo_state.position = [1500 + int(math.degrees(-steering_angle) / 180 * 2000)]
                        data = SetPWMServoState()
                        data.state = [servo_state]
                        data.duration = 0.02
                        self.servo_state_pub.publish(data)
            
            # 发布控制消息
            self.mecanum_pub.publish(twist)
            self.last_twist = twist
            self.has_active_input = True
            self.zero_twist_published = False
            
        else:
            # 没有有效输入，检查是否需要发布停止消息
            if self.has_active_input and not self.zero_twist_published:
                # 从有输入变为无输入，发布一次停止消息
                stop_twist = Twist()
                self.mecanum_pub.publish(stop_twist)
                self.last_twist = stop_twist
                self.has_active_input = False
                self.zero_twist_published = True
                self.get_logger().debug('发布停止消息')

    def select_callback(self, new_state):
        pass

    def l1_callback(self, new_state):
        pass

    def l2_callback(self, new_state):
        pass

    def r1_callback(self, new_state):
        pass

    def r2_callback(self, new_state):
        pass

    def square_callback(self, new_state):
        pass

    def cross_callback(self, new_state):
        pass

    def circle_callback(self, new_state):
        pass

    def triangle_callback(self, new_state):
        pass

    def start_callback(self, new_state):
        if new_state == ButtonState.Pressed:
            msg = BuzzerState()
            msg.freq = 2500
            msg.on_time = 0.05
            msg.off_time = 0.01
            msg.repeat = 1
            self.buzzer_pub.publish(msg)

    def hat_xl_callback(self, new_state):
        pass

    def hat_xr_callback(self, new_state):
        pass

    def hat_yd_callback(self, new_state):
        pass

    def hat_yu_callback(self, new_state):
        pass

    def clear_all_markers(self):
        """清除所有命名空间下的标记点"""
        clear_marker = Marker()
        clear_marker.header.stamp = self.get_clock().now().to_msg()
        clear_marker.header.frame_id = "map"  # 根据你的坐标系修改
        clear_marker.ns = ""  # 空命名空间表示所有
        clear_marker.id = 0
        clear_marker.action = Marker.DELETEALL
        
        clear_array = MarkerArray()
        clear_array.markers.append(clear_marker)
        
        self.mark_pub.publish(clear_array)
        self.get_logger().info('所有标记点已全部清除')

    def joy_callback(self, joy_msg):
        axes = dict(zip(AXES_MAP, joy_msg.axes))
        axes_changed = False
        hat_x, hat_y = axes['hat_x'], axes['hat_y']
        hat_xl, hat_xr = 1 if hat_x > 0.5 else 0, 1 if hat_x < -0.5 else 0
        hat_yu, hat_yd = 1 if hat_y > 0.5 else 0, 1 if hat_y < -0.5 else 0
        buttons = list(joy_msg.buttons)
        buttons.extend([hat_xl, hat_xr, hat_yu, hat_yd, 0])
        buttons = dict(zip(BUTTON_MAP, buttons))
        
        for key, value in axes.items(): 
            if self.last_axes[key] != value:
                axes_changed = True
                
        if axes_changed:
            try:
                self.axes_callback(axes)
            except Exception as e:
                self.get_logger().error(str(e))
                
        for key, value in buttons.items():
            if value != self.last_buttons[key]:
                new_state = ButtonState.Pressed if value > 0 else ButtonState.Released
            else:
                new_state = ButtonState.Holding if value > 0 else ButtonState.Normal
            callback = "".join([key, '_callback'])
            if new_state != ButtonState.Normal:
                self.get_logger().info(str(new_state))
                if  hasattr(self, callback):
                    try:
                        getattr(self, callback)(new_state)
                    except Exception as e:
                        self.get_logger().error(str(e))
                        
        # 到达目的地
        if axes['hat_x'] == 1 and self.last_axes['hat_x'] != 1:
            self.call_joy_up()
        # 清空标记
        if axes['hat_x'] == -1 and self.last_axes['hat_x'] != -1:
            self.clear_all_markers()

        if axes['hat_y'] == 1 and self.last_axes['hat_y'] != 1:
            self.call_garb_pick()
        if axes['hat_y'] == -1 and self.last_axes['hat_y'] != -1:
            self.call_garb_place()

        self.last_buttons = buttons
        self.last_axes = axes

    def destroy_node(self):
        """重写销毁函数，确保线程正确退出"""
        # 发布最后的停止消息确保机器人停止
        stop_twist = Twist()
        self.mecanum_pub.publish(stop_twist)
        
        if hasattr(self, 'joy_pub_thread') and self.joy_pub_thread.is_alive():
            self.joy_pub_thread.join(timeout=1.0)
        super().destroy_node()

def main():
    node = JoystickController('joystick_control')
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()
