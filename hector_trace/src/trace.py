#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist  # 用于控制无人机运动
import math
from std_msgs.msg import Header
from sensor_msgs.msg import Image
from yolov5_ros_msgs.msg import BoundingBox, BoundingBoxes
# PID控制器类
class PIDController:
    def __init__(self, kp, ki, kd):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.prev_error = 0
        self.integral = 0

    def compute(self, error, dt):
        self.integral += error * dt
        derivative = (error - self.prev_error) / dt
        self.prev_error = error
        output = self.kp * error + self.ki * self.integral + self.kd * derivative
        return output

# 全局变量
target_x = 0.0
target_y = 0.0
target_area = 0.0

frame_width = 640  # 假设图像宽度
frame_height = 480  # 假设图像高度

desired_area = 3000  # 期望的目标区域面积，具体值需要根据实际情况调整

# 处理边界框消息的回调函数
def bounding_box_callback(msg):
    msg= BoundingBox()
    global target_x, target_y, target_area
    bbox_center_x = (msg.xmin + msg.xmax) / 2
    bbox_center_y = (msg.ymin + msg.ymax) / 2
    target_x = bbox_center_x - frame_width / 2
    target_y = bbox_center_y - frame_height / 2
    target_area = (msg.xmax - msg.xmin) * (msg.ymax - msg.ymin)

def limit_amplitude(value, lower_limit, upper_limit):  
    """  
    限幅函数，将输入值限制在给定的范围内。  
  
    参数:  
    value (float): 要限幅的值。  
    lower_limit (float): 范围的下限。  
    upper_limit (float): 范围的上限。  
  
    返回:  
    float: 被限制在给定范围内的值。  
    """  
    return max(min(value, upper_limit), lower_limit)  
  
def main():
    global target_x, target_y, target_area

    rospy.init_node('uav_target_tracking', anonymous=True)
    rospy.Subscriber('/yolov5/BoundingBoxes', BoundingBoxes, bounding_box_callback)  # 请替换为实际的话题名称
    cmd_vel_pub = rospy.Publisher('/uav1/cmd_vel', Twist, queue_size=10)

    rate = rospy.Rate(10)  # 10 Hz

    pid_x = PIDController(0.1, 0.01, 0.05)
    pid_y = PIDController(0.1, 0.01, 0.05)
    pid_z = PIDController(0.1, 0.01, 0.05)  # 高度控制PID

    while not rospy.is_shutdown():
        dt = 1.0 / 100  # 每次循环的时间间隔

        error_x = target_x   # 归一化误差/ (frame_width / 2)
        error_y = target_y  # 归一化误差 / (frame_height / 2)
        error_z = desired_area - target_area  # 面积误差

        control_x = pid_y.compute(error_x, dt)/100
        control_y = pid_x.compute(error_y, dt)/100
        control_z = pid_z.compute(error_z/500, dt)
        
        control_x=limit_amplitude(control_x,-2,2)
        control_y=limit_amplitude(control_y,-2,2)
        control_z=limit_amplitude(control_z,-2,2)

        twist = Twist()
        # if(abs(error_x)<20):
        #     twist.linear.z=0 
        # else: 
        #     twist.linear.z = -control_x  # 高低移动
        # if(abs(error_y)<20):
        #      twist.angular.z=0 
        # else: 
        #     twist.angular.z = -control_y 
        if(abs(error_z)<1500):
             twist.linear.x=0 
        else: 
            twist.linear.x = control_z  # 前后移动速度由面积误差控制


            # twist.linear.x = control_z  # 前后移动速度由面积误差控制
            # twist.angular.y = 0  # 左右移动
            # twist.linear.z = -control_x  # 高低移动
            
            # twist.angular.z = -control_y 

        cmd_vel_pub.publish(twist)

        rate.sleep()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass