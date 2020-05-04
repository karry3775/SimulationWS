#!/usr/bin/env python
import rospy
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion
import matplotlib.pyplot as plt
import time
import math as m
import message_filters # for time synchronization

rospy.init_node("stereo_laser_comparator_node")

# Global Variables
## laser
l_x = []
l_y = []
l_yaw = []
l_time = []
l_e_lon = []
l_e_lat = []
l_e_yaw = []

## laser corrected
lc_x = []
lc_y = []
lc_yaw = []
lc_time = []
lc_e_lon = []
lc_e_lat = []
lc_e_yaw = []

## gt
gt_x = []
gt_y = []
gt_yaw = []
gt_time = []

## stereo
s_x = []
s_y = []
s_yaw = []
s_time = []
s_e_lon = []
s_e_lat = []
s_e_yaw = []

# time
tg = time.time()

def laser_cb(msg):
    global l_x, l_y, l_time, l_yaw
    pose = msg.pose.pose
    x = pose.position.x
    y = pose.position.y
    q = pose.orientation
    (_, _, yaw) = euler_from_quaternion([q.x, q.y, q.z, q.w])
    t = time.time() - tg

    # append values
    l_x.append(x)
    l_y.append(y)
    l_yaw.append(yaw)
    l_time.append(t)

def laser_corrected_cb(msg):
    global lc_x, lc_y, lc_time, lc_yaw
    pose = msg.pose.pose
    x = pose.position.x
    y = pose.position.y
    q = pose.orientation
    (_, _, yaw) = euler_from_quaternion([q.x, q.y, q.z, q.w])
    t = time.time() - tg

    # append values
    lc_x.append(x)
    lc_y.append(y)
    lc_yaw.append(yaw)
    lc_time.append(t)

def stereo_cb(msg):
    global s_x, s_y, s_time, s_yaw
    pose = msg.pose.pose
    x = pose.position.x
    y = pose.position.y
    q = pose.orientation
    (_, _, yaw) = euler_from_quaternion([q.x, q.y, q.z, q.w])
    t = time.time() - tg

    # append values
    s_x.append(x)
    s_y.append(y)
    s_yaw.append(yaw)
    s_time.append(t)

def gt_cb(msg):
    global gt_x, gt_y, gt_time, gt_yaw
    pose = msg.pose.pose
    x = pose.position.x
    y = pose.position.y
    q = pose.orientation
    (_, _, yaw) = euler_from_quaternion([q.x, q.y, q.z, q.w])
    t = time.time() - tg

    # append values
    gt_x.append(x)
    gt_y.append(y)
    gt_yaw.append(yaw)
    gt_time.append(t)

def avg_cb(msg_l, msg_lc, msg_s, msg_gt):
    global l_e_lon, lc_e_lon, s_e_lon, l_e_lat, lc_e_lat, s_e_lat, l_e_yaw, lc_e_yaw, s_e_yaw
    # extract values
    pose_l = msg_l.pose.pose
    pose_lc = msg_lc.pose.pose
    pose_s = msg_s.pose.pose
    pose_gt = msg_gt.pose.pose

    # extract pose values
    ## laser
    lx = pose_l.position.x
    ly = pose_l.position.y
    (_, _, lyaw) = euler_from_quaternion([pose_l.orientation.x, pose_l.orientation.y, pose_l.orientation.z, pose_l.orientation.w])

    ## laser corrected
    lcx = pose_lc.position.x
    lcy = pose_lc.position.y
    (_, _, lcyaw) = euler_from_quaternion([pose_lc.orientation.x, pose_lc.orientation.y, pose_lc.orientation.z, pose_lc.orientation.w])

    ## stereo
    sx = pose_s.position.x
    sy = pose_s.position.y
    (_, _, syaw) = euler_from_quaternion([pose_s.orientation.x, pose_s.orientation.y, pose_s.orientation.z, pose_s.orientation.w])

    ## gt
    gx = pose_gt.position.x
    gy = pose_gt.position.y
    (_, _, gyaw) = euler_from_quaternion([pose_gt.orientation.x, pose_gt.orientation.y, pose_gt.orientation.z, pose_gt.orientation.w])

    # find the longitudinal and latitudinal errors
    l1, l2, l3 = calcTerminalErrorLonLat(lx, ly, lyaw, gx, gy, gyaw)
    lc1, lc2, lc3 = calcTerminalErrorLonLat(lcx, lcy, lcyaw, gx, gy, gyaw)
    s1, s2, s3 = calcTerminalErrorLonLat(sx, sy, syaw, gx, gy, gyaw)

    # append to values
    l_e_lon.append(l1);  l_e_lat.append(l2); l_e_yaw.append(l3)
    lc_e_lon.append(lc1);  lc_e_lat.append(lc2); lc_e_yaw.append(lc3)
    s_e_lon.append(s1);  s_e_lat.append(s2); s_e_yaw.append(s3)

def wrapToPi(theta):
    return m.atan2(m.sin(theta), m.cos(theta))

def calcTerminalError(sensorx, sensory, sensor_yaw, gtx, gty, gt_yaw):
    te_x = abs(sensorx - gtx)
    te_y = abs(sensory - gty)
    te_yaw = wrapToPi(abs(sensor_yaw - gt_yaw))

    return [te_x, te_y, te_yaw]

def calcTerminalErrorLonLat(sensorx, sensory, sensor_yaw, gtx, gty, gt_yaw):
    d = m.sqrt((sensorx - gtx)**2 + (sensory - gty)**2)
    alpha = m.atan2(sensory - gty, sensorx - gtx)
    beta = wrapToPi(alpha - gt_yaw)

    # first off lets calculate the yaw error
    te_yaw = wrapToPi(abs(sensor_yaw - gt_yaw))
    lon_e = abs(d * m.cos(beta))
    lat_e = abs(d * m.sin(beta))

    return [lon_e, lat_e, te_yaw]

if __name__ == "__main__":
    try:
        laser_sub = rospy.Subscriber("/odom_rf2o", Odometry, laser_cb)
        laser_corrected_sub = rospy.Subscriber("/odom_rf2o_corrected", Odometry, laser_corrected_cb)
        stereo_sub = rospy.Subscriber("/stereo_odometer/odometry", Odometry, stereo_cb)
        gt_sub = rospy.Subscriber("/ground_truth/state", Odometry, gt_cb)

        # extra Subscribers for average trajectory error calculations
        l_sub = message_filters.Subscriber("/odom_rf2o", Odometry)
        lc_sub = message_filters.Subscriber("/odom_rf2o_corrected", Odometry)
        s_sub = message_filters.Subscriber("/stereo_odometer/odometry", Odometry)
        g_sub = message_filters.Subscriber("/ground_truth/state", Odometry)
        avg_sub = message_filters.ApproximateTimeSynchronizer([l_sub, lc_sub, s_sub, g_sub], queue_size = 5, slop = 0.5)
        avg_sub.registerCallback(avg_cb)

        rospy.spin()

        # plot it
        fig1, axs1 = plt.subplots(1)
        fig2, axs2 = plt.subplots(1)

        axs1.set_aspect('equal')
        axs1.set_title('Trajectory')
        axs1.set_xlabel("x [m]")
        axs1.set_ylabel("y [m]")
        axs1.plot(gt_x, gt_y, 'r-', label = 'gt')
        axs1.plot(l_x, l_y, 'b-', label = 'laser')
        axs1.plot(lc_x, lc_y, 'm-', label = 'laser_corrected')
        axs1.plot(s_x, s_y, 'g-', label = 'vo')
        axs1.plot(gt_x[0], gt_y[0], 'ro', label = "gt_s")
        axs1.plot(gt_x[-1], gt_y[-1], 'r*', label = "gt_e")
        axs1.plot(l_x[0], l_y[0], 'bo', label = "l_s")
        axs1.plot(l_x[-1], l_y[-1], 'b*', label = "l_e")
        axs1.plot(lc_x[0], lc_y[0], 'mo', label = "lc_s")
        axs1.plot(lc_x[-1], lc_y[-1], 'm*', label = "lc_e")
        axs1.plot(s_x[0], s_y[0], 'go', label = "vo_s")
        axs1.plot(s_x[-1], s_y[-1], 'g*', label = "vo_e")
        axs1.legend()

        axs2.set_xlabel("time [s]")
        axs2.set_ylabel("yaw [rad]")
        axs2.set_title('Yaw')
        axs2.plot(gt_time, gt_yaw, 'r-', label = 'gt')
        axs2.plot(l_time, l_yaw, 'b-', label = 'laser')
        axs2.plot(lc_time, lc_yaw, 'm-', label = 'laser_corrected')
        axs2.plot(s_time, s_yaw, 'g-', label = 'vo')
        axs2.legend()

        plt.show()

        # lets also calculate the terminal errors
        te_l_lon, te_l_lat, te_l_yaw = calcTerminalErrorLonLat(l_x[-1], l_y[-1], l_yaw[-1], gt_x[-1], gt_y[-1], gt_yaw[-1])
        te_lc_lon, te_lc_lat, te_lc_yaw = calcTerminalErrorLonLat(lc_x[-1], lc_y[-1], lc_yaw[-1], gt_x[-1], gt_y[-1], gt_yaw[-1])
        te_s_lon, te_s_lat, te_s_yaw = calcTerminalErrorLonLat(s_x[-1], s_y[-1], s_yaw[-1], gt_x[-1], gt_y[-1], gt_yaw[-1])





        # lets also calculated the length of trajectory
        traj_len = 0
        for i in range(1, len(gt_x)):
            x_prev = gt_x[i-1]
            y_prev = gt_y[i-1]
            x = gt_x[i]
            y = gt_y[i]

            delta = m.sqrt((x_prev - x)**2 + (y_prev - y)**2)
            traj_len += delta

        # print the average errors
        l_avg_lon = sum(l_e_lon) / len(l_e_lon)
        l_avg_lat = sum(l_e_lat) / len(l_e_lat)
        l_avg_yaw = sum(l_e_yaw) / len(l_e_yaw)

        lc_avg_lon = sum(lc_e_lon) / len(lc_e_lon)
        lc_avg_lat = sum(lc_e_lat) / len(lc_e_lat)
        lc_avg_yaw = sum(lc_e_yaw) / len(lc_e_yaw)

        s_avg_lon = sum(s_e_lon) / len(s_e_lon)
        s_avg_lat = sum(s_e_lat) / len(s_e_lat)
        s_avg_yaw = sum(s_e_yaw) / len(s_e_yaw)

        print("The results are reported for a trajectory length of : {}".format(traj_len))
        print("****************************************************")
        print("Ther terminal errors are :")
        print("Terminal longitudinal for laser : {}, {}%".format(te_l_lon, te_l_lon / traj_len))
        print("Terminal latitudinal for laser : {}, {}%".format(te_l_lat, te_l_lat / traj_len))
        print("Terminal yaw for laser : {}, {}%".format(te_l_yaw, te_l_yaw / traj_len))
        print("####################################################")
        print("Terminal longitudinal for laser corrected : {}, {}%".format(te_lc_lon, te_lc_lon / traj_len))
        print("Terminal latitudinal for laser corrected: {}, {}%".format(te_lc_lat, te_lc_lat / traj_len))
        print("Terminal yaw for laser corrected: {}, {}%".format(te_lc_yaw, te_lc_yaw / traj_len))
        print("####################################################")
        print("Terminal longitudinal for VO : {}, {}%".format(te_s_lon, te_s_lon / traj_len))
        print("Terminal latitudinal for VO : {}, {}%".format(te_s_lat, te_s_lat / traj_len))
        print("Terminal yaw for VO : {}, {}%".format(te_s_yaw, te_s_yaw / traj_len))
        print("####################################################")

        print("****************************************************")
        print("The average errors are:")
        print("Avg longitudinal for laser : {}".format(l_avg_lon))
        print("Avg latitudinal for laser : {}".format(l_avg_lat))
        print("Avg yaw for laser : {}".format(l_avg_yaw))
        print("####################################################")
        print("Avg longitudinal for laser corrected : {}".format(lc_avg_lon))
        print("Avg latitudinal for laser corrected: {}".format(lc_avg_lat))
        print("Avg yaw for laser corrected: {}".format(lc_avg_yaw))
        print("####################################################")
        print("Avg longitudinal for VO : {}".format(s_avg_lon))
        print("Avg latitudinal for VO : {}".format(s_avg_lat))
        print("Avg yaw for VO : {}".format(s_avg_yaw))
        print("####################################################")
    except rospy.ROSInterruptException:
        pass
