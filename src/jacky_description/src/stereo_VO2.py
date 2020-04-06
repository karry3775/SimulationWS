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
rospy.init_node("stereo_VO_node")

# create a image_publisher
image_pub = rospy.Publisher("/features", Image, queue_size = 10)
pose_pub = rospy.Publisher("/jacky_vo_pose", PoseStamped, queue_size = 10)
pcl_pub = rospy.Publisher("/jacky_pcl", PointCloud, queue_size = 10)

# Global Variables
cv_left_image_prev = None
cv_right_image_prev = None
cv_left_image_prev_color = None
cv_right_image_prev_color = None
ini_image_obtained = False
prev_time = None
H = np.eye(4)
ShiTomasi =  False

# Camera calibration matrix
k_calib = np.array([[238.3515418, 0.0, 200.5],
                    [0.0, 238.3515418, 200.5],
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
    orb = cv2.ORB_create(nfeatures=100000, scoreType=cv2.ORB_FAST_SCORE)
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

def ORB_draw_matches(img1, img2, kp1, kp2, des1, des2, publish = False):
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

def computeRt(matches, kp1, kp2):
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

    # essential matrix
    E, mask = cv2.findEssentialMat(np.array(image1_points), np.array(image2_points), k_calib)
    _, R, t, mask = cv2.recoverPose(E, np.array(image1_points), np.array(image2_points), k_calib)

    return R, t

def get_image_points(matches, kp):
    img_points = []

    for m in matches:
        query_idx = m.queryIdx
        px, py = kp[query_idx].pt
        img_points.append([px, py])

    return img_points

OTP = True
def find_common_matches(match1, match2, kp1, kp2, kp3):
    global OTP
    # Note that match1 is between kp1 and kp2
    # match2 is between is betwern kp2 and kp3
    common_matches = []
    # Go through the first match1 list and for every pair check if there is a pair exisiting in the match2
    # to accelerate the process we can make use of a dict which can ensure constant time lookup
    # lets first make a dict1
    dict1 = {}
    for m in match1:
        query_idx = m.queryIdx
        train_idx = m.trainIdx
        # get first img matched keypoints
        p1_x, p1_y = kp1[query_idx].pt
        # get second img matched keypoints
        p2_x, p2_y = kp2[train_idx].pt
        dict1[(p1_x, p1_y)] = (p2_x, p2_y)

    dict2 = {}
    for m in match2:
        query_idx = m.queryIdx
        train_idx = m.trainIdx
        # get first img matched keypoints
        p1_x, p1_y = kp2[query_idx].pt
        # get second img matched keypoints
        p2_x, p2_y = kp3[train_idx].pt
        dict2[(p1_x, p1_y)] = (p2_x, p2_y)


    for key, value in dict1.items():
        if value in dict2:
            common_matches.append([key, value, dict2[value]])

    return common_matches

def visualize_common_matches(common_matches, cv_left_image_prev, cv_left_image_cur, cv_right_image_prev):
    cv_left_image_prev = cv2.cvtColor(cv_left_image_prev,cv2.COLOR_GRAY2RGB)
    cv_left_image_cur = cv2.cvtColor(cv_left_image_cur,cv2.COLOR_GRAY2RGB)
    cv_right_imagcv_left_image_prev, cv_left_image_cur, kp_prev_left, kp_cur_left, des_prev_left, des_cur_lefte_prev = cv2.cvtColor(cv_right_image_prev,cv2.COLOR_GRAY2RGB)

    for matches in common_matches:
        f,s,t = matches
        r = random.randint(0, 255)
        g = random.randint(0, 255)
        b = random.randint(0, 255)
        cv_left_image_prev = cv2.circle(cv_left_image_prev, (int(f[0]), int(f[1])), 2, (r,g,b),-1 )
        cv_left_image_cur = cv2.circle(cv_left_image_cur, (int(s[0]), int(s[1])), 2, (r,g,b), -1)
        cv_right_image_prev = cv2.circle(cv_right_image_prev, (int(t[0]), int(t[1])), 2, (r,g,b), -1)

    combined_image = cv2.hconcat([cv_left_image_prev, cv_left_image_cur, cv_right_image_prev])
    cv2.imshow("prev_left, cur_left, prev_right", combined_image)
    cv2.waitKey(1)

def get_point_cloud(common_matches, stereo_base, focal_length):
    # It is important to note that common_matches holds information in the following format
    # previous left image, current left image and previous right image
    # we are calculating 3D locations from left and right images so we should only care about 0th and 2nd index
    #for simplicity lets redefine some variables
    b = stereo_base
    f = focal_length

    pcl_msg = PointCloud()
    print("the matches were: {}".format(common_matches))

    for matches in common_matches:
        pt_L, _, pt_R = matches
        uL, vL = pt_L
        uR, vR = pt_R
        u0 = 200.5
        v0 = 200.5
        #calculate xL, xR and yL and yR
        xL = uL - u0
        xR = uR - u0
        yL = vL - v0
        print("Computing between : xL: {}, xR : {}, yL : {}".format(xL, xR, yL))
        # calculate disparity
        d = xL - xR
        if(d == 0):
            continue
        print("disparity: {}".format(d))
        # calculate cordinates
        Z = (f * b) / d # need to take care of the case when disparity = 0
        X = (Z * xL) / f
        Y = (Z * yL) / f
        print("BEFORE TRANSFORMATION: Z : {}, X : {}, Y : {}".format(Z, X, Y))
        # convert to correct cordinate frame
        R = np.array([[0, 0 , 1],
                      [-1, 0, 0],
                      [0, 1, 0]])
        pt = np.array([[X],[Y],[Z]])
        pt = R.dot(pt)
        print("Computed points: {}".format(pt))
        print("################################")
        print("")
        point_msg = Point32()
        point_msg.x = pt[0]
        point_msg.y = pt[1]
        point_msg.z = pt[2]
        pcl_msg.points.append(point_msg)

    pcl_msg.header.stamp.secs = rospy.Time.now().secs
    pcl_msg.header.stamp.nsecs = rospy.Time.now().nsecs
    pcl_msg.header.frame_id = "base_link"
    return pcl_msg


def stereo_cb(msg_left, msg_right):
    global cv_left_image_prev, cv_right_image_prev, ini_image_obtained, prev_time, H, GET_ONCE
    VISUALIZE_COMMON = False
    try:
        cv_left_image_cur = CvBridge().imgmsg_to_cv2(msg_left, "bgr8")
        cv_right_image_cur = CvBridge().imgmsg_to_cv2(msg_right, "bgr8")
        cv_left_image_cur = cv2.cvtColor(cv_left_image_cur, cv2.COLOR_BGR2GRAY)
        cv_right_image_cur = cv2.cvtColor(cv_right_image_cur, cv2.COLOR_BGR2GRAY)
        cur_time = time.time()
    except CvBridgeError as e:
        print(e)
        return
    if not ini_image_obtained:
        ini_image_obtained = True
    else:
        stereo = cv2.StereoBM_create(numDisparities = 16, blockSize = 15)
        # Captured the images
        # Undistort the images and rectify the images -> NOT NEEDED BECAUSE SIMULATION
        # Compute the disparity maps
        disparity_prev =  stereo.compute(cv_left_image_prev, cv_right_image_prev)
        disparity_cur =  stereo.compute(cv_left_image_cur, cv_right_image_cur)
        ## Use ORB feature detect and compute
        kp_prev_left, des_prev_left, prev_img_w_kp = ORB_detect(cv_left_image_prev, (255, 0, 0))
        kp_cur_left, des_cur_left, cur_img_w_kp = ORB_detect(cv_left_image_cur, (0, 255, 0))
        kp_prev_right, des_prev_right, prev_img_w_kp_right = ORB_detect(cv_right_image_prev, (255, 255, 0))

        # From the above line of code we will have kp, des for each image
        match1 = ORB_draw_matches(cv_left_image_prev, cv_left_image_cur, kp_prev_left, kp_cur_left, des_prev_left, des_cur_left)
        match2 = ORB_draw_matches(cv_left_image_cur, cv_right_image_prev, kp_cur_left, kp_prev_right, des_cur_left, des_prev_right, True)

        # find common matches
        common_matches = find_common_matches(match1, match2, kp_prev_left, kp_cur_left, kp_prev_right)

        if VISUALIZE_COMMON:
            # lets visualize these common_matches
            visualize_common_matches(common_matches, cv_left_image_prev, cv_left_image_cur, cv_right_image_prev)

        # Now we have the common matches we need a way to triangulate, here the base length of the stereo_camera will come into picture
        stereo_base = 0.064 # in m\
        focal_length = 238.3515418 # in unit pixels
        # lets take cv_left_image_prev and cv_right_image_prev and now that we have the x and y locations we can calculate the the 3D X,Y,Z points of the features
        # most likely we can publish it to a point cloud data
        pcl = get_point_cloud(common_matches, stereo_base, focal_length)
        # print(pcl)
        pcl_pub.publish(pcl)

        # lets assume we have the point cloud and the common matches

    cv_left_image_prev = cv_left_image_cur
    cv_right_image_prev = cv_right_image_cur
    prev_time = cur_time

if __name__ == "__main__":
    try:
        left_cam_sub = message_filters.Subscriber("/stereo/head_C_left/image_raw", Image)
        right_cam_sub = message_filters.Subscriber("/stereo/head_C_right/image_raw", Image)
        cam_sub = message_filters.ApproximateTimeSynchronizer([left_cam_sub, right_cam_sub], queue_size = 5, slop = 0.1)
        cam_sub.registerCallback(stereo_cb)
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
