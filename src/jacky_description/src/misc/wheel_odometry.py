#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import time
import math as m
import message_filters
from tf.transformations import euler_from_quaternion, quaternion_from_euler
import matplotlib.pyplot as plt

rospy.init_node("wheel_odometry_node")

# Global variables
wheel_odom_x = []
wheel_odom_y = []
refined_odom_x = []
refined_odom_y = []
gt_x = []
gt_y = []

# wheel odom raw
wo_x = 0
wo_y = 0
wo_yaw = 0

# refined odom
r_x = 0
r_y = 0
r_yaw = 0


INI_SET_CMD = False
INI_SET_CMD_GT = False
prev_time_cmd = time.time()
prev_time_cmd_gt = time.time()

def cmd_vel_cb(msg):
    global prev_time_cmd, INI_SET_CMD, wheel_odom_x, wheel_odom_y, wo_x, wo_y, wo_yaw
    lin_vel = msg.linear.x
    ang_vel = msg.angular.z

    if not INI_SET_CMD:
        INI_SET_CMD = True
        wheel_odom_x.append(0)
        wheel_odom_y.append(0)

    cur_time = time.time()
    dt = cur_time - prev_time_cmd
    prev_time_cmd = cur_time

    wo_x = wo_x + lin_vel * m.cos(wo_yaw) * dt
    wo_y = wo_y + lin_vel * m.sin(wo_yaw) * dt
    wo_yaw = wo_yaw + ang_vel * dt

    wheel_odom_x.append(wo_x)
    wheel_odom_y.append(wo_y)

def gt_cb(msg):
    global gt_x, gt_y
    x = msg.pose.pose.position.x
    y = msg.pose.pose.position.y
    gt_x.append(x)
    gt_y.append(y)

def cmd_gt_cb(msg):
    global prev_time_cmd_gt, INI_SET_CMD_GT, refined_odom_x, refined_odom_y, r_x, r_y, r_yaw
    lin_vel = msg.linear.x
    ang_vel = msg.angular.z

    if not INI_SET_CMD_GT:
        INI_SET_CMD_GT = True
        refined_odom_x.append(0)
        refined_odom_y.append(0)

    cur_time = time.time()
    dt = cur_time - prev_time_cmd_gt
    prev_time_cmd_gt = cur_time

    gt_msg = rospy.wait_for_message("/ground_truth/state", Odometry)
    q = gt_msg.pose.pose.orientation
    (gtroll, gtpitch, gtyaw) = euler_from_quaternion([q.x, q.y, q.z, q.w])
    r_yaw = gtyaw

    r_x = r_x + lin_vel * m.cos(r_yaw) * dt
    r_y = r_y + lin_vel * m.sin(r_yaw) * dt
    r_yaw = r_yaw + ang_vel * dt

    refined_odom_x.append(r_x)
    refined_odom_y.append(r_y)

if __name__ == "__main__":
    try:
        cmd_vel_sub = rospy.Subscriber("/jacky/cmd_vel", Twist, cmd_vel_cb)
        cmd_gt_sub = rospy.Subscriber("/jacky/cmd_vel", Twist, cmd_gt_cb)
        gt_sub = rospy.Subscriber("/ground_truth/state", Odometry, gt_cb)
        rospy.spin()
        # plot it
        fig, axs = plt.subplots(1)
        print("gt : {}".format(gt_x))
        print("odom : {}".format(wheel_odom_x))
        print("refined: {}".format(refined_odom_x))
        axs.set_aspect('equal')
        axs.plot(gt_x, gt_y, 'r-', label = "gt")
        axs.plot(wheel_odom_x, wheel_odom_y, 'b-', label = "wheel odom")
        axs.plot(refined_odom_x, refined_odom_y, 'm-', label = "refined odom")
        plt.legend()
        plt.show()
    except rospy.ROSInterruptException:
        pass
