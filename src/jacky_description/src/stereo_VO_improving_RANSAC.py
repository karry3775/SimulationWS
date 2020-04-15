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
import message_filters
import random
from tf.transformations import quaternion_from_euler, euler_from_quaternion

# initiate the node
rospy.init_node("stereo_VO_improved_RANSAC_node")

# create publishers
image_pub = rospy.Publisher("/features", Image, queue_size = 10)
pose_pub = rospy.Publisher("/jacky_vo_pose", PoseStamped, queue_size = 10)
pcl_pub_prev = rospy.Publisher("/jacky_pcl_prev", PointCloud, queue_size = 10)
pcl_pub_cur = rospy.Publisher("/jacky_pcl_cur", PointCloud, queue_size = 10)

# Global variables
cv_left_image_prev = None
cv_right_image_prev = None
ini_image_obtained = False
prev_time = None
H = np.eye(4) # initial pose
transfromNum = 0

# camera calibration matrix
k_calib = np.array([[238.3515418, 0.0, 200.5],
                    [0.0, 238.3515418, 200.5],
                    [0.0, 0.0, 1.0]])

def ORB_detect(img,c):
    orb = cv2.ORB_create() # nfeatures=1000000, scoreType=cv2.ORB_FAST_SCORE
    kp, des = orb.detectAndCompute(img, None)
    img2 = cv2.drawKeypoints(img, kp, None, color = c)
    return kp, des, img2

def ORB_draw_matches(img1, img2, kp1, kp2, des1, des2, publish = True):
    # Create BFMatcher object
    bf = cv2.BFMatcher(cv2.NORM_HAMMING, crossCheck = True)
    # Match descriptor
    matches = bf.match(des1, des2)
    # Sort them in order of their distances
    matches = sorted(matches, key = lambda x:x.distance)
    # Draw first 10 matches, or all
    img3 = cv2.drawMatches(img1, kp1, img2, kp2, matches[:10], None, flags = cv2.DrawMatchesFlags_NOT_DRAW_SINGLE_POINTS)
    if publish:
        rosimg = CvBridge().cv2_to_imgmsg(img3, "bgr8")
        image_pub.publish(rosimg)
    return matches

def transformPoints(X, Y, Z):
    # conver to correct cordinate frame
    R = np.array([[0, 0, 1],
                  [-1, 0, 0],
                  [0, -1, 0]])
    pt = np.array([[X],[Y],[Z]])
    pt = np.matmul(R, pt)
    return [float(pt[0]), float(pt[1]), float(pt[2])]

def stereo_cb(msg_left, msg_right):
    global cv_left_image_cur, cv_right_image_prev, ini_image_obtained, prev_time, H, transformNum
    try:
        cur_time = time.time()

        keyFrameThreshold = 0.1
        if prev_time is not None and cur_time - prev_time <= keyFrameThreshold:
            return

        cv_left_image_cur = CvBridge().imgmsg_to_cv2(msg_left, "bgr8")
        cv_right_image_cur = CvBridge().imgmsg_to_cv2(msg_right, "bgr8")
        # convert to gray
        cv_left_image_cur = cv2.cvtColor(cv_left_image_cur, cv2.COLOR_BGR2GRAY)
        cv_right_image_cur = cv2.cvtColor(cv_right_image_cur, cv2.COLOR_BGR2GRAY)
        # blur the image
        cv_left_image_cur = cv2.GaussianBlur(cv_left_image_cur, (5,5), 0)
        cv_right_image_cur = cv2.GaussianBlur(cv_right_image_cur, (5,5), 0)
    except CvBridgeError as e:
        print("[Error] : {}".format(e))
        return

    if not ini_image_obtained:
        ini_image_obtained = True
    else:
        stereo = cv2.StereoBM_create(numDisparities = 16, blockSize = 15)
        # compute disparities
        disp_prev = stereo.compute(cv_left_image_prev, cv_right_image_prev).astype(np.float32) / 16.0
        disp_cur = stereo.compute(cv_left_image_cur, cv_right_image_cur).astype(np.float32) / 16.0
        # match features in cv_left_image_prev, cv_left_image_cur
        kp_prev, des_prev, prevImgWKp = ORB_detect(cv_left_image_prev, (255,0,0))
        kp_cur, des_cur, curImgWKp = ORB_detect(cv_left_image_cur, (0,255,0))
        # generate a point cloud for left image
        # lets compute the 3D point for left image
        b = 0.064
        f = 238.3515418
        cx = 200.5
        cy = 200.5

        # find reprojection matrix
        Q = np.array([[1,0,0,-cx],
                      [0,1,0,-cy],
                      [0,0,0,-f],
                      [0,0,-1/b, 0]])

        # we need to find matches and use those matching point and this disparity maps
        matches = ORB_draw_matches(cv_left_image_prev, cv_left_image_cur, kp_prev, kp_cur, des_prev, des_cur)

        # defining a finIndex to only get the best matches
        finIndex = int(0.75 * len(matches))
        d = {} # : (prev) : (cur)
        for match in matches[:finIndex]:
            qidx = match.queryIdx
            tids = match.trainIdx
            # get the first img matched keypoints
            p1x, p1y = kp_prev[qidx].pt
            p2x, p2y = kp_cur[tidx].pt
            #populate d
            d[(p1x, p1y)] = (p2x, p2y)

        pairs = {}
        #find the triangulated point from dictionary "d" we previously created
        pcl_msg_prev = PointCloud()
        pcl_msg_cur = PointCloud()

        for pt_prev, pt_cur in d.items():
            x_prev, y_prev = pt_prev
            x_cur, y_cur = pt_cur

            x_prev = int(round(x_prev))
            y_prev = int(round(y_prev))
            x_cur = int(round(x_cur))
            y_cur = int(round(y_cur))

            d_prev = disp_prev[y_prev, x_prev]
            d_cur = disp_cur[y_cur, x_cur]

            if d_prev == 0 or d_cur == 0:
                return

            # lets put it in to the dictionary of pairs
            Zprev = abs((f * b) / d_prev)

            if Zprev > 4:
                continue

            Xprev = (Zprev * (x_prev - cx)) / f
            Yprev = (Zprev * (y_prev - cy)) / f

            Zcur = abs((f * b) / d_cur)
            if Zcur > 4:
                continue

            Xcur = (Zcur * (x_cur - cx)) / f
            Ycur = (Zcur * (y_cur - cy)) / f

            # We need to transform the points
            Xprev, Yprev, Zprev = transformPoints(Xprev, Yprev, Zprev)
            Xcur, Ycur, Zcur = transformPoints(Xcur, Ycur, Zcur)
            pairs[(Xprev, Yprev, Zprev)] = (Xcur, Ycur, Zcur)

        # end for
        # we will have an approximate mapping between previous and current points
        # we need to run this through a RANSAC and get answer for this
        # note that we are going to introduce delays this way
            



if __name__ == "__main__":
    try:
        left_cam_sub = message_filters.Subscriber("/stereo/left/image_rect", Image)
        right_cam_sub = message_filters.Subscriber("/stereo/right/image_rect", Image)
        cam_sub = message_filters.ApproximateTimeSynchronizer([left_cam_sub, right_cam_sub], queue_size = 5, slop = 0.01) # this is lowest order at which approximate time sync seems to work
        cam_sub.registerCallback(stereo_cb)

        rospy.spin()
    except rospy.ROSInterruptException:
        pass
