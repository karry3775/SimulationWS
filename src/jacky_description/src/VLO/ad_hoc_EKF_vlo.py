#!/usr/bin/env python
import rospy
from jacky_description.msg import DeltaMsgStamped
from tf.transformations import euler_from_quaternion
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
import message_filters # for time synchronization
import time
import matplotlib.pyplot as plt
import math as m
import numpy as np

rospy.init_node("ad_hoc_EKF_vlo_node")

prev_time = time.time()

TIME_GAP = []
USAGE = []

## global
x_fused = 0
y_fused = 0
yaw_fused = 0

xf = []
yf = []
yawf = []

gtx = []
gty = []
gtyaw = []

sx = []
sy = []

lx = []
ly = []

## global velocity command
vel = 0
omega = 0

## Kalman Filter variables
# state covariance (initial is implicit)
P = np.array([[1.0, 0, 0],
              [0, 1.0, 0],
              [0, 0, 1.0]])
# process noise (assumed randomly , can be refined using statistical analysis of motion model) (PFI)
Q = np.array([[0.5, 0, 0],
              [0, 0.5, 0],
              [0, 0, 0.5]])

# measurement noise for stereo (PFI)
Rs = np.array([[0.2, 0, 0],
               [0, 0.2, 0],
               [0, 0, 0.3]])

# measurement noise for laser (PFI)
Rl = np.array([[0.01, 0, 0],
               [0, 0.01, 0],
               [0, 0, 0.02]])

def wrapToPi(theta):
    return m.atan2(m.sin(theta), m.cos(theta))

def integrateOnce(x, y, yaw, vel, omega):
    global P
    dt = 0.1
    x_new = x + vel * m.cos(yaw) * dt
    y_new = y + vel * m.sin(yaw) * dt
    yaw_new = wrapToPi(yaw + omega * dt)
    P += Q
    return [x_new, y_new, yaw_new]

def integrateTillT(x, y, yaw, vel, omega, T):
    dt = 0.1
    ini_time = dt

    if ini_time > T:
        x, y, yaw = integrateOnce(x, y, yaw, vel, omega)
    else:
        while ini_time < T:
            x, y, yaw = integrateOnce(x, y, yaw, vel, omega)
            ini_time += dt

    return [x, y, yaw]

def vlo_cb(msg_laser, msg_stereo):
    global prev_time, x_fused, y_fused, yaw_fused, vel, omega
    # global variables for kalman filtering
    global P
    global xf, yf, yawf

    # wait for gt yaw value
    odom_msg = rospy.wait_for_message("/ground_truth/state", Odometry)
    q = odom_msg.pose.pose.orientation
    (_, _, yaw_gt) = euler_from_quaternion([q.x, q.y, q.z, q.w])

    cur_time = time.time()

    time_gap = cur_time - prev_time
    prev_time = cur_time

    # find delta using cmd_vel and integration
    dx_cmd, dy_cmd, dyaw_cmd = integrateTillT(0, 0, 0, vel, 0, time_gap) # setting cmd_vel omega as zero, since we are getting the yaw from ground truth
    dyaw_cmd = wrapToPi(yaw_gt - yaw_fused) # externally set the dyaw as by the ground truth (POTENTIAL FOR IMPROVEMENT) --> PFI

    # extract delta information from laser and stereo
    dx_laser, dy_laser, dyaw_laser = msg_laser.data
    dx_stereo, dy_stereo, dyaw_stereo = msg_stereo.data

    # check if time_gap exceeds the threshold
    if time_gap >= 0.15 and time_gap <= 0.25:
        print("using EKF")
        # Find out which delta value is closer to cmd vel
        # lets take euclidean distance for now (PFI)
        laser_cost = dx_laser #m.sqrt((dx_laser - dx_cmd)**2 + (dy_laser - dy_cmd)**2 + (dyaw_laser - dyaw_cmd)**2)
        stereo_cost = dx_stereo #m.sqrt((dx_stereo - dx_cmd)**2 + (dy_stereo - dy_cmd)**2 + (dyaw_stereo - dyaw_cmd)**2)
        if laser_cost > stereo_cost:
            print("using laser as measurement model")
            meas = np.array([[dx_laser], [dy_laser], [dyaw_laser]])
            state = np.array([[dx_cmd], [dy_cmd], [dyaw_cmd]])
            # compute Kalman gain
            K = np.dot(P, np.linalg.inv(P + Rl))

            # correction
            state = state + np.dot(K, meas - state)
            state = [float(val) for val in state]
            dx_fused, dy_fused , dyaw_fused = state

            # update covariance
            P = np.dot(np.eye(3) - K, P)
        else:
            print("using stereo as measurement model")
            meas = np.array([[dx_stereo], [dy_stereo], [dyaw_stereo]])
            state = np.array([[dx_cmd], [dy_cmd], [dyaw_cmd]])
            # compute Kalman gain
            K = np.dot(P, np.linalg.inv(P + Rs))

            # correction
            state = state + np.dot(K, meas - state)
            state = [float(val) for val in state]
            dx_fused, dy_fused , dyaw_fused = state

            # update covariance
            P = np.dot(np.eye(3) - K, P)
    else:
        print("using cmd_vel")
        dx_fused = dx_cmd
        dy_fused = dy_cmd
        dyaw_fused = dyaw_cmd

    print("Hi VLO received your message! {}".format(time_gap))

    # rewrite the dyaw_fused before calculating
    dyaw_fused = dyaw_cmd

    # now we have received a fused estimate of delta values lets convert them into pose
    dx_global = dx_fused * m.cos(yaw_fused) - dy_fused * m.sin(yaw_fused)
    dy_global = dx_fused * m.sin(yaw_fused) + dy_fused * m.cos(yaw_fused)
    dyaw_global = dyaw_fused

    x_fused = x_fused + dx_global
    y_fused = y_fused + dy_global
    yaw_fused = wrapToPi(yaw_fused + dyaw_global)

    # append to global variables
    xf.append(x_fused)
    yf.append(y_fused)
    yawf.append(yaw_fused)

def cmd_vel_cb(msg):
    global vel, omega
    vel = msg.linear.x
    omega = msg.angular.z

def gt_cb(msg):
    global gtx, gty
    pose = msg.pose.pose
    x = pose.position.x
    y = pose.position.y
    gtx.append(x)
    gty.append(y)

def stereo_only_cb(msg):
    global sx, sy
    pose = msg.pose.pose
    x = pose.position.x
    y = pose.position.y
    sx.append(x)
    sy.append(y)

def laser_only_cb(msg):
    global lx, ly
    pose = msg.pose.pose
    x = pose.position.x
    y = pose.position.y
    lx.append(x)
    ly.append(y)

if __name__ == "__main__":
    try:
        laser_sub = message_filters.Subscriber("/laser_odometer_downsampled/delta_topic", DeltaMsgStamped)
        stereo_sub = message_filters.Subscriber("/stereo_odometer/delta_topic", DeltaMsgStamped)
        vlo_sub =  message_filters.ApproximateTimeSynchronizer([laser_sub, stereo_sub], queue_size = 5, slop = 0.2)
        gt_sub = rospy.Subscriber("/ground_truth/state", Odometry, gt_cb)

        stereo_only_sub = rospy.Subscriber("/stereo_odometer/odometry", Odometry, stereo_only_cb)
        laser_only_sub = rospy.Subscriber("/odom_rf2o_corrected", Odometry, laser_only_cb)

        cmd_vel_sub = rospy.Subscriber("/jacky/cmd_vel", Twist, cmd_vel_cb)

        vlo_sub.registerCallback(vlo_cb)
        rospy.spin()

        # plot it
        fig1, axs1 = plt.subplots(1)
        axs1.set_aspect('equal')
        axs1.set_title('Trajectory')
        axs1.set_xlabel('x [m]')
        axs1.set_ylabel('y [m]')
        axs1.plot(gtx, gty, 'r-', label = "gt")
        axs1.plot(xf, yf, 'b--', label = 'fused')
        axs1.plot(sx, sy, 'g--', label = 'vo')
        axs1.plot(lx, ly, 'm--', label = 'laser')
        axs1.legend()

        plt.show()
    except rospy.ROSInterruptException:
        pass
