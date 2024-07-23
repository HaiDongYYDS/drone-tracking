#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist

# 定义全局变量来存储Twist消息和控制状态
twist = Twist()
current_drone = 1

def joy_callback(data):
    global twist, current_drone

    # 切换无人机控制的按钮 按钮A为无人机1，按钮B为无人机2.
    if data.buttons[0] == 1:
        current_drone =1 
    if data.buttons[1] == 1:
        current_drone = 2 
    rospy.loginfo("Switched control to drone %d", current_drone)

    # 左摇杆控制前后左右
    twist.linear.x = data.axes[1] # 前后
    twist.linear.y = data.axes[0] # 左右

    # 右摇杆控制升降和旋转
    twist.linear.z = data.axes[4]*1.5 # 升降
    twist.angular.z = data.axes[3] # 旋转

def main():
    global current_drone

    rospy.init_node('drone_teleop')

    pub_drone1 = rospy.Publisher('/uav1/cmd_vel', Twist, queue_size=5)
    pub_drone2 = rospy.Publisher('/uav2/cmd_vel', Twist, queue_size=5)

    rospy.Subscriber('/joy', Joy, joy_callback)

    rate = rospy.Rate(10) # 10 Hz
    while not rospy.is_shutdown():
        if current_drone == 1:
            pub_drone1.publish(twist)
        else:
            pub_drone2.publish(twist)

        rate.sleep()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass