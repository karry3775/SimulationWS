#!/usr/bin/env python
import rospy
from geometry_msgs.msg import PoseStamped
from tf.transformations import euler_from_quaternion

rospy.init_node("get_vo_pose_node")
file = open("vo_pose.txt",'w')
def pose_cb(msg):
    global file
    x = msg.pose.position.x
    y = msg.pose.position.y
    q = msg.pose.orientation
    (roll, pitch, yaw) = euler_from_quaternion([q.x, q.y, q.z, q.w])
    file.write(str(x) + "," + str(y) + "," + str(yaw) + "\n")

if __name__ == "__main__":
    rospy.Subscriber("/stereo_odometer/pose", PoseStamped, pose_cb)
    rospy.spin()
    file.close()
