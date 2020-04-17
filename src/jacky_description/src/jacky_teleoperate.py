#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist
import keyboard
import random

def cmd_pub():
    rospy.init_node("jacky_teleoperate_node")
    pub = rospy.Publisher("/jacky/cmd_vel", Twist, queue_size = 10)
    rate = rospy.Rate(30) # 30 Hz
    cmd = Twist()
    while not rospy.is_shutdown():
        print("listening for keys")
        # need to listen to arrow keys
        if keyboard.is_pressed("up"):
            cmd.linear.x = 0.25
            cmd.angular.z = 0.0
        elif keyboard.is_pressed("left"):
            cmd.linear.x = 0.0
            cmd.angular.z = 0.5
        elif keyboard.is_pressed("right"):
            cmd.linear.x = 0.0
            cmd.angular.z = -0.5
        else:
            cmd.linear.x = 0.0
            cmd.angular.z = 0.0

        pub.publish(cmd)

if __name__ == "__main__":
    try:
        cmd_pub()
    except rospy.ROSInterruptException:
        pass
