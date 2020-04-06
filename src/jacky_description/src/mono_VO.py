#!/usr/bin/env python
import rospy
from sensor_msgs.msg import Image
from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped
from cv_bridge import CvBridge, CvBridgeError
import cv2
import numpy as np
from scipy.linalg import svd
import time
import matplotlib.pyplot as plt
import math as m
import message_filters # for time synchronization
import matplotlib.pyplot as plt

# intialize the node
rospy.init_node("mono_VO_node")

# create a image_publisher
image_pub = rospy.Publisher("/features", Image, queue_size = 10)
pose_pub = rospy.Publisher("/jacky_vo_pose", PoseStamped, queue_size = 10)

# Global Variables
cv_image_prev = None
ini_image_obtained = False
prev_time = None
H = np.eye(4)
ShiTomasi =  False

# Camera calibration matrix
k_calib = np.array([[476.70308, 0.0, 400.5],
                    [0.0, 476.70308, 400.5],
                    [0.0, 0.0, 1.0]])
# Parameters for ShiTomasi corner detection
feature_params = dict(maxCorners = 100,#1200,
                      qualityLevel = 0.3,
                      minDistance = 7,
                      blockSize = 7)

# Parameters for lucas kanade optical flow
lk_params = dict(winSize = (15,15),
                maxLevel = 2,
                criteria = (cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT, 10, 0.03))

#create some random colors
color = np.random.randint(0, 255, (500, 3))

def plot_side_by_side(prev_frame, cv_image, good_old, good_new):
    #we know that image width is 640, so every pixel
    if(good_old.shape[0]>8):
        fin_image = cv2.hconcat([prev_frame, cv_image])

        # creating mask
        mask = np.zeros_like(fin_image)
        #draw the tracks
        for i, (new, old) in enumerate(zip(good_new, good_old)):
            a, b = new.ravel()
            c, d = old.ravel()
            mask = cv2.line(mask, (int(c),int(d)), (int(a+640),int(b)),color[i].tolist(), 2)
            fin_image = cv2.circle(fin_image, (int(c),int(d)), 5, color[i].tolist(), -1)
        fin_image = cv2.add(fin_image, mask)

        cv2.imshow("combined", fin_image)
        cv2.waitKey(3)

def FAST_detect(img,c):
    # Initiate fast object with default values
    fast = cv2.FastFeatureDetector_create()
    # find and draw the keypoints
    kp = fast.detect(img, None)
    img2 = cv2.drawKeypoints(img, kp, None, color = c)
    return img2

def ORB_detect(img,c):
    orb = cv2.ORB_create()
    kp, des = orb.detectAndCompute(img, None)
    img2 = cv2.drawKeypoints(img, kp, None, color = c)
    return kp, des, img2

def SURF_detect(img, c):
    surf = cv2.xfeatures2d.SURF_create()
    kp, des = surf.detectAndCompute(img, None)
    img2 = cv2.drawKeypoints(img, kp, None, color = c)
    return kp, des, img2

def SIFT_detect(img, c):
    sift = cv2.xfeatures2d.SIFT_create()
    kp, des = sift.detectAndCompute(img, None)
    img2 = cv2.drawKeypoints(img, kp, None, color = c)
    return kp, des, img2

def ORB_draw_matches(img1, img2, kp1, kp2, des1, des2):
    # Create BFMatcher object
    bf = cv2.BFMatcher(cv2.NORM_HAMMING, crossCheck = True)
    # Match descriptor
    matches = bf.match(des1, des2)
    # Sort them in order of their distances
    matches = sorted(matches, key = lambda x:x.distance)
    # Draw first 10 matches, or all
    img3 = cv2.drawMatches(img1, kp1, img2, kp2, matches[:10], None, flags = cv2.DrawMatchesFlags_NOT_DRAW_SINGLE_POINTS)
    rosimg = CvBridge().cv2_to_imgmsg(img3, "bgr8")
    image_pub.publish(rosimg)
    return matches

def SURF_draw_matches(img1, img2, kp1, kp2, des1, des2):
    # Create BFMatcher object
    bf = cv2.BFMatcher(cv2.NORM_L1, crossCheck = True)
    # Match descriptor
    matches = bf.match(des1, des2)
    # Sort them in order of their distances
    matches = sorted(matches, key = lambda x:x.distance)
    # Draw first 10 matches, or all
    img3 = cv2.drawMatches(img1, kp1, img2, kp2, matches[:10], None, flags = cv2.DrawMatchesFlags_NOT_DRAW_SINGLE_POINTS)
    rosimg = CvBridge().cv2_to_imgmsg(img3, "bgr8")
    image_pub.publish(rosimg)
    return matches

def SIFT_draw_matches(img1, img2, kp1, kp2, des1, des2):
    # Create BFMatcher object
    bf = cv2.BFMatcher(cv2.NORM_L1, crossCheck = True)
    # Match descriptor
    matches = bf.match(des1, des2)
    # Sort them in order of their distances
    matches = sorted(matches, key = lambda x:x.distance)
    # Draw first 10 matches, or all
    img3 = cv2.drawMatches(img1, kp1, img2, kp2, matches[:10], None, flags = cv2.DrawMatchesFlags_NOT_DRAW_SINGLE_POINTS)
    rosimg = CvBridge().cv2_to_imgmsg(img3, "bgr8")
    image_pub.publish(rosimg)
    return matches

oneTimePrint = False
def computeRt(matches, kp1, kp2):
    global PRINTIT
    image1_points = []
    image2_points = []

    for m in matches:
        query_idx = m.queryIdx
        train_idx = m.trainIdx

        # get first img matched keypoints
        p1_x, p1_y = kp1[query_idx].pt
        image1_points.append([p1_x, p1_y])

        # get second img matched keypoints
        p2_x, p2_y = kp2[train_idx].pt
        image2_points.append([p2_x, p2_y])

    if oneTimePrint:
        print("Size of image_points : {}".format(np.array(image1_points).shape))
        print(image1_points)
        oneTimePrint = False

    # essential matrix
    E, mask = cv2.findEssentialMat(np.array(image1_points), np.array(image2_points), k_calib)
    _, R, t, mask = cv2.recoverPose(E, np.array(image1_points), np.array(image2_points), k_calib)

    return R, t


def mono_cb(msg):
    global cv_image_prev, ini_image_obtained, prev_time, H
    try:
        cv_image_cur = CvBridge().imgmsg_to_cv2(msg, "bgr8")
        cv_left_image_cur = cv2.cvtColor(cv_image_cur, cv2.COLOR_BGR2GRAY)
        cur_time = time.time()
    except CvBridgeError as e:
        print(e)
        return
    if not ini_image_obtained:
        ini_image_obtained = True
    else:
        ## Use ORB feature detect and compute
        kp_prev, des_prev, prev_img_w_kp = ORB_detect(cv_image_prev, (255, 0, 0))
        kp_cur, des_cur, cur_img_w_kp = ORB_detect(cv_image_cur, (0, 255, 0))

        if not ShiTomasi:
            # draw matched featues (drawing with plt.pause causes frame skipping,use ros publisher instead)
            matches = ORB_draw_matches(prev_img_w_kp, cur_img_w_kp, kp_prev, kp_cur, des_prev, des_cur)
            # print("time difference: {}".format(cur_time - prev_time))

            # Estimate motion
            R, t = computeRt(matches, kp_prev, kp_cur)
            # print("R : {}, t : {}".format(R, t))
            # print(np.linalg.det(R))
        else:
            # use shitomasi features
            pt_prev = cv2.goodFeaturesToTrack(cv_left_image_prev, mask = None, **feature_params)
            #create a mask for drawing purpose
            mask = np.zeros_like(cv_left_image_prev)
            #lets calculate optical flow now
            pt_curr, st, err = cv2.calcOpticalFlowPyrLK(cv_left_image_prev, cv_left_image_cur, pt_prev, None, **lk_params)

            #selet good points
            good_new = pt_curr[st==1]
            good_old = pt_prev[st==1]
            """
            plot images side by side
            """
            plot_side_by_side(cv_left_image_prev, cv_left_image_cur, good_old, good_new)
            # need to calculate the essential matrix and recover the Pose
            E, mask = cv2.findEssentialMat(np.array(good_old), np.array(good_new), k_calib)
            _, R, t, mask = cv2.recoverPose(E, np.array(good_old), np.array(good_new), k_calib)


        # Compute the homogenous transformation matrix
        H_new = np.eye(4)
        H_new[0:3, 0:3] = R.T
        H_new[0:3, 3:] = -(R.T).dot(t)
        H = H.dot(H_new)

        # extract x y z and rotation from here
        # qw= sqrt(1 + m00 + m11 + m22) /2
        # qx = (m21 - m12)/( 4 *qw)
        # qy = (m02 - m20)/( 4 *qw)
        # qz = (m10 - m01)/( 4 *qw)

        x = H[0,3]
        y = H[1,3]
        z = H[2,3]
        m00 = H[0,0]
        m11 = H[1,1]
        m22 = H[2,2]
        m21 = H[2,1]
        m12 = H[1,2]
        m02 = H[0,2]
        m20 = H[2,0]
        m10 = H[1,0]
        m01 = H[0,1]

        qw = m.sqrt(1 + m00 + m11 + m22) / 2.0
        qx = (m21 - m12) / (4 * qw)
        qy = (m02 - m20) / (4 * qw)
        qz = (m10 - m01) / (4 * qw)

        # create a pose message
        pose = PoseStamped()
        pose.pose.position.x = 0
        pose.pose.position.y = 0
        pose.pose.position.z = 0
        pose.pose.orientation.x = qx
        pose.pose.orientation.y = qy
        pose.pose.orientation.z = qz
        pose.pose.orientation.w = qw
        pose.header.frame_id = "odom"
        pose.header.stamp.secs = rospy.Time().secs
        pose.header.stamp.nsecs = rospy.Time().nsecs
        pose_pub.publish(pose)


    cv_image_prev = cv_image_cur
    prev_time = cur_time

if __name__ == "__main__":
    try:
        cam_sub = rospy.Subscriber("/upward_camera/image_raw", Image, mono_cb)
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
