#!/usr/bin/env python
import rospy
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion, quaternion_from_euler
import message_filters

rospy.init_node("laser_odom_corrected_node")
corrected_odom_pub = rospy.Publisher("/odom_rf2o_corrected", Odometry, queue_size = 10)

def combined_cb(msg_rf2o, msg_gt):
    pose_rf2o = msg_rf2o.pose.pose
    pose_gt = msg_gt.pose.pose
    q_gt = pose_gt.orientation
    (_, _, yaw_gt) = euler_from_quaternion([q_gt.x, q_gt.y, q_gt.z, q_gt.w])
    x_rf2o = pose_rf2o.position.x
    y_rf2o = pose_rf2o.position.y

    odom_msg = Odometry()
    odom_msg.pose.pose.position.x = x_rf2o
    odom_msg.pose.pose.position.y = y_rf2o
    odom_msg.pose.pose.orientation = q_gt
    odom_msg.header.stamp = rospy.Time.now()

    corrected_odom_pub.publish(odom_msg)


if __name__ == "__main__":
    try:
        odom_rf2o_sub = message_filters.Subscriber("/odom_rf2o", Odometry)
        gt_sub = message_filters.Subscriber("/ground_truth/state", Odometry)
        combined_sub = message_filters.ApproximateTimeSynchronizer([odom_rf2o_sub, gt_sub], queue_size = 5, slop = 0.05)
        combined_sub.registerCallback(combined_cb)
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
