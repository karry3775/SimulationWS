#!/usr/bin/env python
import rospy
from nav_msgs.msg import Odometry
from jacky_description.msg import DeltaMsgStamped
from tf.transformations import euler_from_quaternion
import matplotlib.pyplot as plt
import time
import math as m

rospy.init_node("delta_laser_downsampled_verification_node")

#global variables
## odom normal
odom_n_x = []
odom_n_y = []
odom_n_yaw = []
odom_n_time = []

## odom fabricated
odom_f_x = []
odom_f_y = []
odom_f_yaw = []
odom_f_time = []
xg = 0
yg = 0
yawg = 0

#tg
tg = time.time()

def wrapToPi(theta):
    return m.atan2(m.sin(theta), m.cos(theta))

def odom_n_cb(msg):
    global odom_n_x, odom_n_y, odom_n_yaw, odom_n_time
    pose = msg.pose.pose
    x = pose.position.x
    y = pose.position.y
    q = pose.orientation
    (_, _, yaw) = euler_from_quaternion([q.x, q.y, q.z, q.w])
    t = time.time() - tg

    #append
    odom_n_x.append(x)
    odom_n_y.append(y)
    odom_n_yaw.append(yaw)
    odom_n_time.append(t)

def odom_f_cb(msg):
    global xg, yg, yawg, odom_f_x, odom_f_y, odom_f_yaw, odom_f_time
    dx, dy, dyaw = msg.data
    dx_cor = dx * m.cos(yawg) - dy * m.sin(yawg)
    dy_cor = dx * m.sin(yawg) + dy * m.cos(yawg)

    xg = xg + dx_cor
    yg = yg + dy_cor
    yawg =  wrapToPi(yawg + dyaw)
    t = time.time() - tg
    # append
    odom_f_x.append(xg)
    odom_f_y.append(yg)
    odom_f_yaw.append(yawg)
    odom_f_time.append(t)


if __name__ == "__main__":
    try:
        odom_n_sub = rospy.Subscriber("/odom_rf2o", Odometry, odom_n_cb)
        odom_f_sub = rospy.Subscriber("/laser_odometer_downsampled/delta_topic", DeltaMsgStamped, odom_f_cb)
        rospy.spin()

        # plot it
        fig1, axs1 = plt.subplots(1)
        fig2, axs2 = plt.subplots(1)

        axs1.set_aspect("equal")
        axs1.set_title("Trajectory")
        axs1.set_xlabel("x [m]")
        axs1.set_ylabel("y [m]")
        axs1.plot(odom_n_x, odom_n_y, 'r-', label = "normal")
        axs1.plot(odom_f_x, odom_f_y, 'b--', label = "fabricated")
        axs1.legend()

        axs2.set_xlabel("time [s]")
        axs2.set_ylabel("yaw [rad]")
        axs2.set_title("Yaw")
        axs2.plot(odom_n_time, odom_n_yaw, 'r-', label = "normal")
        axs2.plot(odom_f_time, odom_f_yaw, 'b--', label = "fabricated")
        axs2.legend()

        plt.show()

    except rospy.ROSInterruptException:
        pass
