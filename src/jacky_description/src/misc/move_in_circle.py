#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist

rospy.init_node("move_in_a_circle_node")

def cmdPublisher():
    cmd_pub = rospy.Publisher("/jacky/cmd_vel", Twist, queue_size = 10)
    rate = rospy.Rate(10)
    cmd = Twist()
    cmd.linear.x = 0.25
    cmd.angular.z = 0.75

    while not rospy.is_shutdown():
        cmd_pub.publish(cmd)
        rate.sleep()

if __name__ == "__main__":
    try:
        cmdPublisher()
    except rospy.ROSInterruptException:
        pass
