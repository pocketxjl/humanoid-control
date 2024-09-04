#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist
import threading
from std_msgs.msg import Bool
from std_msgs.msg import String

vx = 0
vy = 0
w = 0
key_Y_pressed = False
pre_key_Y_press_status = False
key_X_pressed = False
pre_key_X_press_status = False

def callback(data):
    global vx, vy, w, key_Y_pressed, key_X_pressed
    vx = data.axes[1] * 0.5
    vy = data.axes[0] * 0.25
    w = data.axes[3] * 0.4
    if data.buttons[3] == 1:
        key_Y_pressed = True
    else:
        key_Y_pressed = False
    if data.buttons[2] == 1:
        key_X_pressed = True
    else:
        key_X_pressed = False

        
rospy.init_node('joy11')
cmd_pub = rospy.Publisher('/cmd_vel',Twist,queue_size=1)
rospy.Subscriber('joy',Joy,callback)

publisher = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
rate = rospy.Rate(150)

hw_switch_bool = False
hw_switch_publisher = rospy.Publisher('/hwswitch', Bool, queue_size=1)

gait_str = "trot"
gait_str_publisher = rospy.Publisher('/desired_gait_str', String, queue_size=1)

def ros_publish():
    while not rospy.is_shutdown():
        twist_msg = Twist()
        twist_msg.linear.x = vx
        twist_msg.linear.y = vy
        twist_msg.angular.z = w
        publisher.publish(twist_msg)

        global key_Y_pressed
        global pre_key_Y_press_status
        global hw_switch_bool
        global key_X_pressed
        global gait_str

        if key_Y_pressed and not pre_key_Y_press_status:
            hw_switch_bool = not hw_switch_bool
            print(f'\n Switch the output status to {hw_switch_bool}')
        pre_key_Y_press_status = key_Y_pressed

        hw_switch_msg = Bool()
        hw_switch_msg.data = hw_switch_bool
        hw_switch_publisher.publish(hw_switch_msg)

        if key_X_pressed and not pre_key_X_press_status:
            if gait_str == "trot":
                gait_str = "walk"
            elif gait_str == "walk":
                gait_str = "trot"
            print(f'\n Switch the desire gait to {gait_str}')
        pre_key_X_press_status = key_X_pressed

        gait_str_msg = String()
        gait_str_msg.data = gait_str
        gait_str_publisher.publish(gait_str_msg)

        rate.sleep()

thread = threading.Thread(target=ros_publish)
thread.start()

rospy.spin()
