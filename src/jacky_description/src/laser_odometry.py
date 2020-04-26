#!/usr/bin/env python
import rospy
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion, quaternion_from_euler
import matplotlib.pyplot as plt

rospy.init_node("laser_odometry_node")

# Global variables
laser_x = []
laser_y = []
laser_yaw = []
laser_time = []

gt_x = []
gt_y = []
gt_yaw = []
gt_time = []

laser_raw_x = []
laser_raw_y = []
laser_raw_yaw = []
laser_raw_time = []

laser_corrected_x = []
laser_corrected_y = []
laser_corrected_yaw = []
laser_corrected_time = []

def laser_cb(msg):
    global laser_x, laser_y, laser_time, laser_yaw
    pose = msg.pose.pose
    t = msg.header.stamp.secs + msg.header.stamp.nsecs * (1e-9)
    x = pose.position.x
    y = pose.position.y
    q = pose.orientation
    (_,_,yaw) = euler_from_quaternion([q.x, q.y, q.z, q.w])

    laser_x.append(x)
    laser_y.append(y)
    laser_yaw.append(yaw)
    laser_time.append(t)

def gt_cb(msg):
    global gt_x, gt_y, gt_time, gt_yaw
    pose = msg.pose.pose
    t = msg.header.stamp.secs + msg.header.stamp.nsecs * (1e-9)
    x = pose.position.x
    y = pose.position.y
    q = pose.orientation
    (_,_,yaw) = euler_from_quaternion([q.x, q.y, q.z, q.w])

    gt_x.append(x)
    gt_y.append(y)
    gt_yaw.append(yaw)
    gt_time.append(t)

def laser_corrected_cb(msg):
    global laser_corrected_x, laser_corrected_y, laser_corrected_time, laser_corrected_yaw
    pose = msg.pose.pose
    t = msg.header.stamp.secs + msg.header.stamp.nsecs * (1e-9)
    x = pose.position.x
    y = pose.position.y
    q = pose.orientation
    (_,_,yaw) = euler_from_quaternion([q.x, q.y, q.z, q.w])

    laser_corrected_x.append(x)
    laser_corrected_y.append(y)
    laser_corrected_yaw.append(yaw)
    laser_corrected_time.append(t)

def laser_raw_cb(msg):
    global laser_raw_x, laser_raw_y, laser_raw_time, laser_raw_yaw
    pose = msg.pose.pose
    t = msg.header.stamp.secs + msg.header.stamp.nsecs * (1e-9)
    x = pose.position.x
    y = pose.position.y
    q = pose.orientation
    (_,_,yaw) = euler_from_quaternion([q.x, q.y, q.z, q.w])

    laser_raw_x.append(x)
    laser_raw_y.append(y)
    laser_raw_yaw.append(yaw)
    laser_raw_time.append(t)

if __name__ == "__main__":
    try:
        laser_sub = rospy.Subscriber("/odom_rf2o", Odometry, laser_cb)
        gt_sub = rospy.Subscriber("/odom", Odometry, gt_cb)
        laser_corrected_sub = rospy.Subscriber("/odom_rf2o_corrected", Odometry, laser_corrected_cb)
        laser_raw_sub = rospy.Subscriber("/odom_rf2o_raw", Odometry, laser_raw_cb)
        rospy.spin()

        # find the associated errors

        # plot it
        fig, axs = plt.subplots(2)
        axs[0].set_aspect('equal')
        axs[0].set_title("Trajectory")
        axs[0].plot(gt_x, gt_y, 'r-', label = "gt")
        # axs[0].plot(laser_x, laser_y, 'b-', label = "laser")
        axs[0].plot(laser_corrected_x, laser_corrected_y, 'm-', label = "laser_corrected")
        axs[0].plot(laser_raw_x, laser_raw_y, 'g-', label = "laser_raw")

        axs[1].set_title("Yaw")
        axs[1].plot(gt_time, gt_yaw, 'r-', label = "gt")
        # axs[1].plot(laser_time, laser_yaw, 'bo-', label = "laser")
        axs[1].plot(laser_corrected_time, laser_corrected_yaw, 'm-', label = "laser_corrected")
        axs[1].plot(laser_raw_time, laser_raw_yaw, 'g-', label = "laser_raw")

        lines, labels = fig.axes[-1].get_legend_handles_labels()
        fig.legend(lines, labels, loc = 'upper center')
        plt.show()


    except rospy.ROSInterruptException:
        pass
