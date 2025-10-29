#!/usr/bin/env python3

import rospy
import math
from std_msgs.msg import Float32
from chut.msg import pid_input
from geometry_msgs.msg import Twist
kp=0.8#0.35
kd=0.0
prev_error=0

pub = rospy.Publisher('/cmd_vel', Twist, queue_size=100)

def callback(data):
    global prev_error, kp, kd
    angle=0.0
    print("before kd=",kp*data.pid_error,kd*((prev_error - (kp*data.pid_error) )))
    angle = (kp*data.pid_error + kd*((prev_error - (kp*data.pid_error) )))*1
    message=Twist()
    message.linear.x=1.3
    print("angle=",angle)
    print("-----------------------")
    message.angular.z=angle
    pub.publish(message)
    prev_error=angle



if __name__ == "__main__":
    print("Controls")
    rospy.init_node('Control')
    rospy.Subscriber('/err', pid_input, callback)
    
    rospy.spin()