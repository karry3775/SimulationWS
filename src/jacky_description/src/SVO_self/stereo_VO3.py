#!/usr/bin/env python
import rospy
from sensor_msgs.msg import Image
from sensor_msgs.msg import Imu
from sensor_msgs.msg import PointCloud
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Point32
from cv_bridge import CvBridge, CvBridgeError
import cv2
import numpy as np
from scipy.linalg import svd
import time
import matplotlib.pyplot as plt
import math as m
import message_filters # for time synchronization
import matplotlib.pyplot as plt
import random

# intialize the node
rospy.init_node("stereo_VO3_node")

pcl_pub = rospy.Publisher("/jacky_pcl", PointCloud, queue_size = 10)

def stereo_cb(msg_left, msg_right):
    global disp, Z_val
    try:
        cv_left_image_cur = CvBridge().imgmsg_to_cv2(msg_left, "bgr8")
        cv_right_image_cur = CvBridge().imgmsg_to_cv2(msg_right, "bgr8")
        cv_left_image_cur = cv2.cvtColor(cv_left_image_cur, cv2.COLOR_BGR2GRAY)
        cv_right_image_cur = cv2.cvtColor(cv_right_image_cur, cv2.COLOR_BGR2GRAY)
    except CvBridgeError as e:
        print(e)
    else:
        stereo = cv2.StereoBM_create(numDisparities = 16, blockSize = 15)
        disparity = stereo.compute(cv_left_image_cur, cv_right_image_cur).astype(np.float32) / 16.0

        # lets see how the disparity based point cloud looks like
        f = 238.3515418
        b = 0.064
        pcl_msg = PointCloud()
        for i in range(0, disparity.shape[0]):
            for j in range(0, disparity.shape[1]):
                # row i will be y value vL
                # col j will be x value xL
                uL = j
                vL = i
                xL = uL - 200.5
                yL = vL - 200.5

                d = disparity[i,j]

                if(d == 0):
                    d = 1.0 / 100000

                Z = (f * b) / d;
                X = (Z * xL) / f;
                Y = (Z * yL) / f;

                R = np.array([[0,0,1],
                              [-1,0,0],
                              [0,1,0]])
                pt = np.array([[X],[Y],[Z]])
                pt = R.dot(pt)
                point_msg = Point32()
                point_msg.x = pt[0]
                point_msg.y = pt[1]
                point_msg.z = pt[2]
                pcl_msg.points.append(point_msg)

        pcl_msg.header.stamp.secs = rospy.Time.now().secs
        pcl_msg.header.stamp.nsecs = rospy.Time.now().nsecs
        pcl_msg.header.frame_id = "base_link"
        pcl_pub.publish(pcl_msg)


if __name__ == "__main__":
    try:
        left_cam_sub = message_filters.Subscriber("/stereo/head_C_left/image_raw", Image)
        right_cam_sub = message_filters.Subscriber("/stereo/head_C_right/image_raw", Image)
        cam_sub = message_filters.ApproximateTimeSynchronizer([left_cam_sub, right_cam_sub], queue_size = 5, slop = 0.1)
        cam_sub.registerCallback(stereo_cb)
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
