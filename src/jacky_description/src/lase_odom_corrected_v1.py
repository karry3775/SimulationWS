#!/usr/bin/env python
import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PointStamped
from tf.transformations import euler_from_quaternion, quaternion_from_euler
import message_filters
import math as m

rospy.init_node("laser_odom_corrected_node")
corrected_odom_pub = rospy.Publisher("/odom_rf2o_corrected", Odometry, queue_size = 10)
raw_odom_pub = rospy.Publisher("/odom_rf2o_raw", Odometry, queue_size = 10)
# Global variables
x = 0
y = 0
yaw = 0
prev_yaw = 0

def wrapToPi(theta):
    return m.atan2(m.sin(theta), m.cos(theta))

def combined_cb(msg_delta, msg_gt):
    print("RECEIVED BRO !")
    global x, y, yaw, prev_yaw
    # gt pose
    pose_gt = msg_gt.pose.pose
    q_gt = pose_gt.orientation
    (_, _, yaw_gt) = euler_from_quaternion([q_gt.x, q_gt.y, q_gt.z, q_gt.w])

    # delta calculations
    delta_x_loc = msg_delta.point.x
    delta_y_loc = msg_delta.point.y
    delta_x = delta_x_loc * m.cos(prev_yaw) - delta_y_loc * m.sin(prev_yaw)
    delta_y = delta_x_loc * m.sin(prev_yaw) + delta_y_loc * m.cos(prev_yaw)

    delta_yaw = msg_delta.point.z
    delta_yaw_gt = wrapToPi(yaw_gt - prev_yaw)
    prev_yaw = yaw_gt

    x = x + delta_x
    y = y + delta_y
    yaw = wrapToPi(yaw + delta_yaw_gt)
    yaw_raw = wrapToPi(yaw + delta_yaw)
    # convert yaw to quaternion for publishing
    q = quaternion_from_euler(0,0,yaw)
    q_raw = quaternion_from_euler(0,0,yaw_raw)

    odom_msg = Odometry()
    odom_msg.pose.pose.position.x = x
    odom_msg.pose.pose.position.y = y
    odom_msg.pose.pose.orientation.x = q[0]
    odom_msg.pose.pose.orientation.y = q[1]
    odom_msg.pose.pose.orientation.z = q[2]
    odom_msg.pose.pose.orientation.w = q[3]

    odom_msg_raw = Odometry()
    odom_msg_raw.pose.pose.position.x = x
    odom_msg_raw.pose.pose.position.y = y
    odom_msg_raw.pose.pose.orientation.x = q_raw[0]
    odom_msg_raw.pose.pose.orientation.y = q_raw[1]
    odom_msg_raw.pose.pose.orientation.z = q_raw[2]
    odom_msg_raw.pose.pose.orientation.w = q_raw[3]

    t = rospy.Time.now()

    odom_msg.header.stamp = t
    odom_msg_raw.header.stamp = t

    corrected_odom_pub.publish(odom_msg)
    raw_odom_pub.publish(odom_msg_raw)


if __name__ == "__main__":
    try:
        delta_sub = message_filters.Subscriber("/rf2o_laser_odometry/delta_topic", PointStamped)
        gt_sub = message_filters.Subscriber("/ground_truth/state", Odometry)
        combined_sub = message_filters.ApproximateTimeSynchronizer([delta_sub, gt_sub], queue_size = 5, slop = 0.05)
        combined_sub.registerCallback(combined_cb)
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
