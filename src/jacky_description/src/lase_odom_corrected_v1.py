#!/usr/bin/env python
import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PointStamped
from sensor_msgs.msg import Imu
from tf.transformations import euler_from_quaternion, quaternion_from_euler
import message_filters
import math as m

rospy.init_node("laser_odom_corrected_node")
corrected_odom_pub = rospy.Publisher("/odom_rf2o_corrected", Odometry, queue_size = 10)
raw_odom_pub = rospy.Publisher("/odom_rf2o_raw", Odometry, queue_size = 10)
# Global variables
x_cor = 0
y_cor = 0
yaw_cor = 0

x_raw = 0
y_raw = 0
yaw_raw = 0

prev_yaw_gt = 0
prev_yaw_raw = 0

imu_x = 0
imu_y = 0
imu_yaw = 0
prev_imu_yaw = 0

def wrapToPi(theta):
    return m.atan2(m.sin(theta), m.cos(theta))

def combined_cb(msg_delta, msg_gt):
    print("RECEIVED BRO !")
    global x_cor, y_cor, yaw_cor, x_raw, y_raw, yaw_raw, prev_yaw_gt, prev_yaw_raw
    # gt pose
    pose_gt = msg_gt.pose.pose
    q_gt = pose_gt.orientation
    (_, _, yaw_gt) = euler_from_quaternion([q_gt.x, q_gt.y, q_gt.z, q_gt.w])

    # delta values in local frame
    delta_x_loc = msg_delta.point.x
    delta_y_loc = msg_delta.point.y

    # global delta value from gt
    delta_x_cor = delta_x_loc * m.cos(prev_yaw_gt) - delta_y_loc * m.sin(prev_yaw_gt)
    delta_y_cor = delta_x_loc * m.sin(prev_yaw_gt) + delta_y_loc * m.cos(prev_yaw_gt)

    # global delta value wihout correction
    delta_x_raw = delta_x_loc * m.cos(prev_yaw_raw) - delta_y_loc * m.sin(prev_yaw_raw)
    delta_y_raw = delta_x_loc * m.sin(prev_yaw_raw) + delta_y_loc * m.cos(prev_yaw_raw)

    delta_yaw_raw = msg_delta.point.z
    prev_yaw_gt = yaw_gt
    prev_yaw_raw = yaw_raw

    ### test code
    x_cor = x_cor + delta_x_cor
    y_cor = y_cor + delta_y_cor
    yaw_cor = yaw_gt

    x_raw = x_raw + delta_x_raw
    y_raw = y_raw + delta_y_raw
    yaw_raw = yaw_raw + delta_yaw_raw

    ###
    q_raw = quaternion_from_euler(0,0,yaw_raw)

    odom_msg = Odometry()
    odom_msg.pose.pose.position.x = x_cor
    odom_msg.pose.pose.position.y = y_cor
    odom_msg.pose.pose.orientation.x = q_gt.x
    odom_msg.pose.pose.orientation.y = q_gt.y
    odom_msg.pose.pose.orientation.z = q_gt.z
    odom_msg.pose.pose.orientation.w = q_gt.w

    odom_msg_raw = Odometry()
    odom_msg_raw.pose.pose.position.x = x_raw
    odom_msg_raw.pose.pose.position.y = y_raw
    odom_msg_raw.pose.pose.orientation.x = q_raw[0]
    odom_msg_raw.pose.pose.orientation.y = q_raw[1]
    odom_msg_raw.pose.pose.orientation.z = q_raw[2]
    odom_msg_raw.pose.pose.orientation.w = q_raw[3]

    t = rospy.Time.now()

    odom_msg.header.stamp = t
    odom_msg_raw.header.stamp = t

    corrected_odom_pub.publish(odom_msg)
    raw_odom_pub.publish(odom_msg_raw)

def imu_gt_cb(msg_imu, msg_gt):
    global imu_x, imu_y, imu_yaw, prev_imu_yaw
    # get gt pose
    pose_gt = msg_gt.pose.pose
    q_gt = pose_gt.orientation
    (_, _, yaw_gt) = euler_from_quaternion([q_gt.x, q_gt.y, q_gt.z, q_gt.w])



if __name__ == "__main__":
    try:
        delta_sub = message_filters.Subscriber("/rf2o_laser_odometry/delta_topic", PointStamped)
        gt_sub = message_filters.Subscriber("/ground_truth/state", Odometry)
        imu_sub = message_filters.Subscriber("/imu", Imu)
        combined_sub = message_filters.ApproximateTimeSynchronizer([delta_sub, gt_sub], queue_size = 5, slop = 0.05)
        imu_gt_sub = message_filters.ApproximateTimeSynchronizer([imu_sub, gt_sub], queue_size = 5, slop = 0.05)
        combined_sub.registerCallback(combined_cb)
        imu_gt_sub.registerCallback(imu_gt_cb)
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
