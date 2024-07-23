#!/usr/bin/env python3
#  -*- coding: utf-8 -*-
from tkinter import ttk
from std_msgs.msg import Empty
from geometry_msgs.msg import Twist, PoseStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Image
import tkinter as tk
import math
import rospy
from cv_bridge import CvBridge, CvBridgeError
import cv2
import numpy as np
from queue import Queue  
from yolov5_ros_msgs.msg import BoundingBox, BoundingBoxes

root = tk.Tk(className="Hector Quadrotor Leader")
root.title("Hector Quadrotor Leader")
image_width, image_height = 640,480
biases_queue = Queue()  
center_x, center_y = image_width // 2, image_height // 2 

# Imagen de icono
iconImg = """iVBORw0KGgoAAAANSUhEUgAAADAAAAAwCAYAAABXAvmHAAAABmJLR0QA/wD/AP+gvaeTAAAE4klEQVRoge2YW2wUVRjHf+fMXqHGECVB5Vq8EJW
        LAsG0knBpJZCQlmJIKKWSWEuMMSwPvvggMfpiTGTRNECiwbRbYxq6pfGhGisRuQQKNOXyYDS0QA0gGMQIvezuzOfDttDt7nZnZxtRsv992DPf9f+d
        b86cmQN55JFHHnnk8aCjtLKmo3TT68f/i/lcBw4elUxGdZ/vA1HYsS1fUazG0o93Pp3J4IFAaWWNlFbWZJyN+5FvzHYDBEPhDmBx/EpOBKrWv5QTu3HOZ
        +cWWnxvqJY4p2YbWeVL24EDB4+sAbUXmDpaV7ajGIDW9486IZiEDPF+E5HadStfbkulHKMDqcnfB0xVSu1Np3Q5ifjnE8+Czrh8/pV4ab1afjiyeqjyaU
        6JjRN6lVK1ZcuLvk2ltFV207Fjfne/PC5aHlOWnqaRhShKBOaPB0MFZxDaLdRp0VavstTVqF9d2VBU1G/D1zkOHDy8BPSXwByHIX4Ga0v5iqUnnHLI+UZ
        ubj9caGh9wYmvaVmz15cs7c4lf06vEh990fpQ7+9/VDr1v3ztxsa6pqaCXDg4LiDYuH+V12v+gsgHTmMo+DAacf0abNy/ymkMRwUEG8PbEd0GTAF+ROR6
        tjHE4roIh4ApiG7b1dCyzQmXrAsIhsJvIHwCmCi2BaoqlgtsAXqzCNOrDfXa9s0Vy5SoAGCKkuDOhuaabPlktYg/bQjPtxTHAY8oqrdvqmjMNmEqBEMtV
        SD1wCDaWhKofPWsXd+sOiCKzwAfqJ3jRR4gULUuhBAEfFhqt4jYnljbBexsbC4RWArcjFoTdjghOhbc3th7wE1QRcGvwivt+tl+F1Ki3gRQsPud6lV30t
        mVP//1K0rRAkwYpfpLi5Q0n994KpXfWxs23A42hvcgvDuUq90OL1sd+Lj+u4nAGsCyROrS2a1d+M0ErdiTgjzAw5ZSDVtm7vOl8ze0UQcIsHooZ0bY6oB
        X9xVb4FNwLrB5/dVhedWi1lKt2AvMikss8PnHCjXH8vn7qye3Dl/3gNTWnyxvB3h7Y9mVXaHweYG5bnWnCPg+EzdbHRDkRQBRcijBOYH8PcQsk77IILcH
        Brg9MEBfZJCYaaYKPUtIetf/CUAp9YIdbrY6IDAv/q9PjyYw2nYwGiVixhJkpiX0WxE84sLrcifoFBSOvLaUOqVEEGSBHW72FrFSTyGClrE3q5hlEjFjG
        IbBogXzKJwxHUHouXSZU13niMRiGErjMoy0MbTQO3Qc8eT4FSDyKIAg7cFQ+K64c1eiWSQan/lFC+ZRvOxpAG5dg+fmPIMIdHR2ETFjSQUEQ+G7RyjC8D
        CeMxPs7gOP2DGyJJ68cMZ0UDDyYGf2rBkAmJZlM6WyldPuPuAH8N9xT9y6dW3fsLB6cWvC4ZOMoHzrKgmQoeKUSt5kA1UVd4V1TU0F0YjrbyDt43Yk7Hb
        AAJg0qX9wTCMdD3fh4qUkXXdPXGakKGAkbkyePDA0tDW5OX2Rje5AzDTpj0YwDIOF8+dSOHM6AN0XL3P6zFlM08Lv8eDSiWug/mSZYx6OjlXSBjMMPOIi
        EovR0dlFR2dXgt5juJLI54pcT6d7Rgu8Ljd+tweX1qihn0tr/B4PXrc7KYDA/fsmBqlNRcBlGPg9Xgp8Pgp8Pvweb8qZF+jW6NrcOOSRRx7/a/wDNeu3t
        iRlt6UAAAAASUVORK5CYII="""
img = tk.PhotoImage(data=iconImg)
root.tk.call('wm','iconphoto',root._w,img)

mainframe = ttk.Frame(root, padding ="3 3 12 12")
mainframe.grid(column=0, row=0, sticky=(tk.N,tk.W,tk.E,tk.S))
mainframe.columnconfigure(0,weight=1)

x_p = tk.StringVar()
y_p = tk.StringVar()
z_p = tk.StringVar()
z_o = tk.StringVar()

cam0_path  = "../yolo_images/"    # 已经建立好的存储cam0 文件的目录

class PIDController:  
    def __init__(self, kp=1.0, ki=0.0, kd=0.0, dt=1.0, output_limits=(-1.0, 1.0)):  
        self.kp = kp  
        self.ki = ki  
        self.kd = kd  
        self.dt = dt  
        self.output_limits = output_limits  
          
        self.prev_error = 0.0  
        self.integral = 0.0  
  
    def update(self, setpoint, measured_value):  
        error = (setpoint - measured_value)/200  
  
        p = self.kp * error  
        i = self.ki * self.integral  
        derivative = (error - self.prev_error) / self.dt  
        d = self.kd * derivative  
  
        output = p + i + d  
  
        # 限制输出在[-1, 1]的范围内  
        output = max(min(output, self.output_limits[1]), self.output_limits[0])  
  
        # 更新误差以供下一次迭代使用  
        self.prev_error = error  
        self.integral += error * self.dt  
  
        return output  
  
  
# # 示例用法：  
# pid = PIDController(kp=1.0, ki=0.1, kd=0.01, dt=0.1, output_limits=(-1.0, 1.0))  
# setpoint = 10.0  # 设定点  
# measured_value = 0.0  # 初始测量值  
  
# for _ in range(100):  # 假设我们迭代100次来模拟系统响应  
#     output = pid.update(setpoint, measured_value)  
#     print(f"Output: {output}")  
#     # 在这里，您应该根据输出值更新measured_value以模拟系统的响应  
#     # 例如：measured_value += output * some_factor（这取决于您的系统模型）  
#     measured_value += 0.1 * output  # 示例：简单模拟系统响应
def quaterionToRads(data):
	x = data.pose.orientation.x
	y = data.pose.orientation.y
	z = data.pose.orientation.z
	w = data.pose.orientation.w

	t3 = 2.0 * (w * z + x * y)
	t4 = 1.0 - 2.0 * (y * y + z * z)
	yawZActual = math.atan2(t3, t4)
	if yawZActual < 0:
		yawZActual = 2*math.pi + yawZActual

	return yawZActual

#Callback de pose y orientacion simulador
def pose_callback(data):
    x_p.set("{0:.2f}".format(data.pose.pose.position.x))
    y_p.set("{0:.2f}".format(data.pose.pose.position.y))
    z_p.set("{0:.2f}".format(data.pose.pose.position.z))

def rot_callback(data):
    z_o.set("{0:.2f}".format( math.degrees(quaterionToRads(data)) ))

def detect_callback(boxs):
    biases = []  
    i = 0
    while i < len(boxs.bounding_boxes):  
        box = boxs.bounding_boxes[i] 
        if np.float64(box.probability) > 0.75:
            # 计算边界框的中心  
            bbox_center_x = (np.int64(box.xmax) + np.int64(box.xmin)) // 2  
            bbox_center_y = (np.int64(box.ymax) + np.int64(box.ymin)) // 2  
            # print(box.xmax,box.xmin)
            # 计算边界框中心相对于图像中心的偏差（可以是绝对值或相对值）  
            bias_x = bbox_center_x - center_x 
            bias_y = bbox_center_y - center_y
            distance = (np.int64(box.xmax) - np.int64(box.xmin)) * (np.int64(box.ymax) + np.int64(box.ymin))
            set_distance = 600*400
            # 将偏差添加到列表中  
            biases.append((bias_x, bias_y))
            vel_msg = Twist()
            
            output_x = pid_line.update(center_x, bbox_center_x) 
            output_y = pid_line_h.update(center_y, bbox_center_y) 
            # vel_msg.linear.y = float(output_x)
            vel_msg.angular.z = float(output_x)
            vel_msg.linear.z = float(output_y)
            vel_msg.linear.x = float(pid.update(set_distance/1000,distance/1000))
            # print (biases)
            print(vel_msg.linear.z,vel_msg.angular.z,vel_msg.linear.x)
            vel_pub.publish(vel_msg)
        i += 1

    biases_queue.put(biases)  


def image_callback(data):
    ## define picture to_down' coefficient of ratio
    #scaling_factor = 0.5
    global count,bridge
    count = count + 1
    if count == 60:
        count = 0
        cv_img = bridge.imgmsg_to_cv2(data, "bgr8")
        timestr = "%.2f" %  data.header.stamp.to_sec()
              #%.6f表示小数点后带有6位，可根据精确度需要修改；
        image_name = timestr + "(2).jpg" #图像命名：时间戳.jpg
        cv2.imwrite(cam0_path + image_name, cv_img)  #保存；
        #cv2.imshow("frame" , cv_img)
        cv2.waitKey(3)
    else:
        pass

rospy.init_node('HectorQ_GUI_Lider', anonymous=False)
#Subscribers
posicionLider_sub = rospy.Subscriber("/uav1/ground_truth/state", Odometry , pose_callback)
orientaLider_sub = rospy.Subscriber("/uav1/ground_truth_to_tf/pose", PoseStamped , rot_callback)
detection_sub = rospy.Subscriber("/yolov5/BoundingBoxes", BoundingBoxes , detect_callback)
#image_sub = rospy.Subscriber('/uav1/front_cam/camera/image', Image, image_callback)
#Publishers
takeoff_pub = rospy.Publisher('/ardrone/takeoff', Empty, queue_size=1)
land_pub = rospy.Publisher('/ardrone/land', Empty, queue_size=1)
vel_pub = rospy.Publisher('/uav1/cmd_vel', Twist, queue_size=1)
####################################################################
pid = PIDController(kp=1.2, ki=0.05, kd=0.01, dt=0.1, output_limits=(-1.0, 2.5)) # leader可加速拦截，目前2.5
pid_line = PIDController(kp=2, ki=0.05, kd=0.01, dt=0.05, output_limits=(-1.0, 1.0))
pid_line_h = PIDController(kp=1.5, ki=0.01, kd=0.01, dt=0.05, output_limits=(-1.0, 1.0))
def setText(text):
    ttk.Label(mainframe, text="              ").grid(column=3, row=1, sticky=tk.W)
    ttk.Label(mainframe, text=text).grid(column=3, row=1, sticky=tk.W)
## ffffffffind
def find_fun():
    if not biases_queue.empty():  
        biases = biases_queue.get() 
        setText("RUSH!")
        # vel_msg.linear.y = -(float(biases[0]) / 100)
        # # vel_msg.linear.z = float(biases[1]) / 500
        # vel_msg.angular.z = -(float(biases[0]) / 100)
    else:
        setText("Find Mode")
        vel_msg = Twist()
        vel_msg.linear.z = float(1.0)
        vel_msg.angular.z = float(1.0)
        vel_pub.publish(vel_msg)
    while (not biases_queue.empty()):
        sss = biases_queue.get()

def land_fun():
    setText("photo")
    global count,bridge
    count = 0
    bridge = CvBridge()
    rospy.Subscriber('/uav1/front_cam/camera/image', Image, image_callback)
    #land_pub.publish(Empty())

def enviar_velocidad(vx,vy,vz,vaz):
    vel_msg = Twist()
    vel_msg.linear.x = float(vx)
    vel_msg.linear.y = float(vy)
    vel_msg.linear.z = float(vz)
    vel_msg.angular.z = float(vaz)
    vel_msg.angular.x = float(0.0)
    vel_msg.angular.y = float(0.0)
    vel_pub.publish(vel_msg)

def hover_pub():
    setText("Hover")
    enviar_velocidad(0.0,0.0,0.0,0.0)

def takeoff_fun():
    setText("TakeOff")
    takeoff_pub.publish(Empty())

def up_fun():
    setText("Up")
    vel_msg = Twist()
    vel_msg.linear.z = float(1.0)
    vel_pub.publish(vel_msg)

def down_fun():
    setText("Down")
    vel_msg = Twist()
    vel_msg.linear.z = float(-1.0)
    vel_pub.publish(vel_msg)

def forward_fun():
    setText("Fordward")
    vel_msg = Twist()
    vel_msg.linear.x = float(1.0)
    vel_pub.publish(vel_msg)

def backward_fun():
    setText("Backward")
    vel_msg = Twist()
    vel_msg.linear.x = float(-1.0)
    vel_pub.publish(vel_msg)

def right_fun():
    setText("Right")
    vel_msg = Twist()
    vel_msg.linear.y = float(-1.0)
    vel_pub.publish(vel_msg)

def left_fun():
    setText("Left")
    vel_msg = Twist()
    vel_msg.linear.y = float(1.0)
    vel_pub.publish(vel_msg)

def cw_fun():
    setText("Turn Right")
    vel_msg = Twist()
    vel_msg.angular.z = float(-1.0)
    vel_pub.publish(vel_msg)

def ccw_fun():
    setText("Turn Left")
    vel_msg = Twist()
    vel_msg.angular.z = float(1.0)
    vel_pub.publish(vel_msg)

#-------------- Despliegue datos de odometria y altura -------------------------
ttk.Label(mainframe, textvariable=x_p).grid(column=1, row=2, sticky=(tk.W,tk.E))
ttk.Label(mainframe, textvariable=y_p).grid(column=2, row=2, sticky=(tk.W,tk.E))
ttk.Label(mainframe, textvariable=z_p).grid(column=3, row=2, sticky=(tk.W,tk.E))
ttk.Label(mainframe, textvariable=z_o).grid(column=4, row=2, sticky=(tk.W,tk.E))

ttk.Label(mainframe, text="X (m)").grid(column=1, row=3, sticky=tk.W)
ttk.Label(mainframe, text="Y (m)").grid(column=2, row=3, sticky=tk.W)
ttk.Label(mainframe, text="Z (m)").grid(column=3, row=3, sticky=tk.W)
ttk.Label(mainframe, text="Yaw (°)").grid(column=4, row=3, sticky=tk.W)
#-------------- Despliegue datos de odometria y altura -------------------------

#---------------------------- Botones de Control -------------------------------
ttk.Button(mainframe, text="Turn Left", command=ccw_fun).grid(column=1, row=5, sticky=tk.W)
ttk.Button(mainframe, text="Fordward", command=forward_fun).grid(column=2, row=5, sticky=tk.W)
ttk.Button(mainframe, text="Turn Right", command=cw_fun).grid(column=3, row=5, sticky=tk.W)


ttk.Button(mainframe, text="Left", command=left_fun).grid(column=1, row=6, sticky=tk.W)
ttk.Button(mainframe, text="Hover", command=hover_pub).grid(column=2, row=6, sticky=tk.W)
ttk.Button(mainframe, text="Right", command=right_fun).grid(column=3, row=6, sticky=tk.W)

ttk.Button(mainframe, text="Up", command=up_fun).grid(column=1, row=7, sticky=tk.W)
ttk.Button(mainframe, text="Backward", command=backward_fun).grid(column=2, row=7, sticky=tk.W)
ttk.Button(mainframe, text="Down", command=down_fun).grid(column=3, row=7, sticky=tk.W)
#---------------------------- Botones de Control -------------------------------


ttk.Button(mainframe, text="Take Off", command=takeoff_fun).grid(column=3, row=4, sticky=tk.W)
ttk.Button(mainframe, text="photo", command=land_fun).grid(column=2, row=4, sticky=tk.W)
ttk.Button(mainframe, text="Find", command=find_fun).grid(column=1, row=4, sticky=tk.W)

ttk.Label(mainframe, text="1").grid(column=3, row=1, sticky=tk.W)


for child in mainframe.winfo_children(): child.grid_configure(padx=5, pady=5)

root.mainloop()
