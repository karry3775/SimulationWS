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
import iterative_closest_point as icp

# Initiate the node
rospy.init_node("stereo_VO2_v3_node")

"""
CHECKING PERFORMANCE WITH KEY FRAME SELECTION
"""

# create publishers
image_pub = rospy.Publisher("/features", Image, queue_size = 10)
pose_pub = rospy.Publisher("jacky_vo_pose", PoseStamped, queue_size = 10)
pcl_pub_prev = rospy.Publisher("/jacky_pcl_prev", PointCloud, queue_size = 10)
pcl_pub_cur = rospy.Publisher("/jacky_pcl_cur", PointCloud, queue_size = 10)
# Global variables
cv_left_image_prev = None
cv_right_image_prev = None
ini_image_obtained = False
prev_time = None
H = np.eye(4) # initial pose
transformNum = 0

trajActX = []
trajActY = []
trajActTh = []
trajVoX = []
trajVoY = []
trajVoTh = []


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
"""
RANSAC STARTS HERE
"""
def getTransform(pairs):
    # print("inside getTransform")
    # the length of the dictionary "pairs" will define m in the following relation
    # (3M X 12) . (12 X 1) = (3M X 1)
    M = len(pairs)
    # we need to define a block matrix of the following form
    # [[x, y, z, 1, 0, 0, 0, 0, 0, 0, 0, 0],
    #  [0, 0, 0, 0, x, y, z, 1, 0, 0, 0, 0],
    #  [0, 0, 0, 0, 0, 0, 0, 0, x, y, z, 1]]
    # We will end up having a relation like A X = b
    A = None
    b = None
    for key, value in pairs.items():
        # lets form the block_matrix
        x_prev, y_prev, z_prev = key
        x_cur, y_cur, z_cur = value

        A_block = np.array([ [x_cur, y_cur, z_cur, 1, 0, 0, 0, 0, 0, 0, 0, 0],
                             [0, 0, 0, 0, x_cur, y_cur, z_cur, 1, 0, 0, 0, 0],
                             [0, 0, 0, 0, 0, 0, 0, 0, x_cur, y_cur, z_cur, 1] ])
        b_block = np.array([[x_prev],
                            [y_prev],
                            [z_prev]])
        if A is None:
            A = A_block
            b = b_block
        else:
            A = np.concatenate((A, A_block), axis = 0)
            b = np.concatenate((b, b_block), axis = 0)

    # print("A is: \n{}\n".format(A))
    # print("b is: \n{}\n".format(b))
    # print("/////////////////////////////////////////////////////////////////////")

    # lets get the least squares solution to this problem
    ATA_inv = np.linalg.pinv((A.T).dot(A))
    multiplier = ATA_inv.dot(A.T)
    X = multiplier.dot(b)

    # now lets form transformation matrix
    H = np.array([[float(X[0]), float(X[1]), float(X[2]), float(X[3])],
                  [float(X[4]), float(X[5]), float(X[6]), float(X[7])],
                  [float(X[8]), float(X[9]), float(X[10]), float(X[11])],
                  [0, 0, 0, 1]])
    # print("done with getTransform")
    return H

def getDICT(sampledList):
  # print("inside getDICT")
  d = {}
  for items in sampledList:
    key = items[0]
    value = items[1]
    d[key] = value

  # print("done with getDICT")
  return d


def getNumInliers(H, pairs, t):
  # print("inside getNumInliers")
  numInliers = 0
  for prev, cur in pairs.items():
      prev = np.array([[prev[0]],
                       [prev[1]],
                       [prev[2]]])
      cur = np.array([[cur[0]],
                      [cur[1]],
                      [cur[2]],
                      [1]])
      prevHat = H.dot(cur)

      # lets find error
      err = m.sqrt((prev[0]-prevHat[0])**2 + (prev[1]-prevHat[1])**2 + (prev[2]- prevHat[2])**2)

      # print("error is: {} ".format(err))

      if(err <= t):
        numInliers += 1

  # print("done with getNumInliers")
  return numInliers

def getTransformRANSAC(pairs):
  # print("inside getTransformRANSAC")
  # n Minimum number of data points required to estimate model parameters.
  # k Maximum number of iterations allowed in the algorithm.
  # t Threshold value to determine data points that are fit well by model.
  # d Number of close data points required to assert that a model fits well to data.
  k = 100 # number of iterations
  n = 4 # we just need four points because they give us 12 equations
  t = 0.2 # threshold of error
  d = 0 # number of close data points
  bestnumInliers = 0
  Hbest = None
  for i in range(0, k):
    # get n random samples from pairs
    sampledList = random.sample(list(pairs.items()), 4)
    # fit a model
    modelPairs = getDICT(sampledList)
    Hmaybe = getTransform(modelPairs)
    # count number of inliers
    numInliers = getNumInliers(Hmaybe, pairs, t)

    if numInliers >= bestnumInliers and numInliers >= d:
      bestnumInliers = numInliers
      Hbest = Hmaybe

  # print("done with getTransformRANSAC")
  print("number Of inliners are {}".format(bestnumInliers))
  return Hbest
"""
RANSAC ENDS HERE
"""
def transformPoints(X, Y, Z):
    # convert to correct cordinate frame
    R = np.array([[0, 0 , 1],
                  [-1, 0, 0],
                  [0, -1, 0]])
    pt = np.array([[X],[Y],[Z]])
    pt = np.matmul(R, pt)
    return [float(pt[0]), float(pt[1]), float(pt[2])]

def stereo_cb(msg_left, msg_right):
    global cv_left_image_prev, cv_right_image_prev, ini_image_obtained, prev_time, H, transformNum
    global trajVoX, trajVoY, trajActX, trajActY, trajVoTh, trajActTh
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
        print(e)
        return

    if not ini_image_obtained:
        ini_image_obtained = True
    else:
        stereo = cv2.StereoBM_create(numDisparities = 16, blockSize = 15)
        # compute disparities
        disp_prev = stereo.compute(cv_left_image_prev, cv_right_image_prev).astype(np.float32) / 16.0
        disp_cur = stereo.compute(cv_left_image_cur, cv_right_image_cur).astype(np.float32) / 16.0
        # match features in cv_left_image_prev, cv_left_image_cur
        kp_prev, des_prev, prevImgWKp = ORB_detect(cv_left_image_prev, (255, 0, 0))
        kp_cur, des_cur, curImgWKp = ORB_detect(cv_left_image_cur, (0,255,0))
        # generate a point cloud for left image
        # lets compute the 3D point for the left image
        b = 0.064
        f = 238.3515418
        cx = 200.5
        cy = 200.5

        # find the reprojection matrix
        Q = np.array([[1,0,0,-cx],
                      [0,1,0,-cy],
                      [0,0,0,-f],
                      [0,0,-1/b, 0]])

        # we need to find matches and use those matching point and this disparity maps
        # to triangulate
        matches = ORB_draw_matches(cv_left_image_prev, cv_left_image_cur, kp_prev, kp_cur, des_prev, des_cur)

        finIndex = int(1.0*len(matches))
        d = {} # : (prev) : (cur)
        for match in matches[:finIndex]:
            qidx = match.queryIdx
            tidx = match.trainIdx
            # get first img matched keypoints
            p1x, p1y = kp_prev[qidx].pt
            p2x, p2y = kp_cur[tidx].pt
            #populate d
            d[(p1x, p1y)] = (p2x, p2y)

        pairs = {}
        # find the triangulated point from dictionary
        pcl_msg_prev = PointCloud()
        pcl_msg_cur = PointCloud()

        # lets make two arrays source and destination for working for ICP algorithm
        SRC = None
        DST = None
        for pt_prev, pt_cur in d.items():
            x_prev, y_prev = pt_prev
            x_cur, y_cur = pt_cur

            x_prev =  int(round(x_prev))
            y_prev = int(round(y_prev))
            x_cur = int(round(x_cur))
            y_cur = int(round(y_cur))

            d_prev = disp_prev[y_prev,x_prev]
            d_cur = disp_cur[y_cur, x_cur]

            if d_prev == 0 or d_cur == 0:
                continue
            # lets put it int to the dictionary of pairs
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

            # we need to transform the points
            Xprev, Yprev, Zprev = transformPoints(Xprev, Yprev, Zprev)
            Xcur, Ycur, Zcur = transformPoints(Xcur, Ycur, Zcur)
            pairs[(Xprev, Yprev, Zprev)] = (Xcur, Ycur, Zcur)

            # if we have reached here, it means everything went well
            # we can safely add points to the SRC AND DST
            if SRC is None:
                SRC = np.array([[Xprev, Yprev, Zprev]])
                DST = np.array([[Xcur, Ycur, Zcur]])
            else:
                SRC = np.concatenate((SRC, [[Xprev, Yprev, Zprev]]), axis = 0)
                DST = np.concatenate((DST, [[Xcur, Ycur, Zcur]]), axis = 0)

            # lets try visualizing the point cloud
            point_msg_prev = Point32()
            point_msg_prev.x = Xprev
            point_msg_prev.y = Yprev
            point_msg_prev.z = Zprev
            pcl_msg_prev.points.append(point_msg_prev)
            # lets try visualizing the point cloud
            point_msg_cur = Point32()
            point_msg_cur.x = Xcur
            point_msg_cur.y = Ycur
            point_msg_cur.z = Zcur
            pcl_msg_cur.points.append(point_msg_cur)


        pcl_msg_prev.header.stamp.secs = rospy.Time.now().secs
        pcl_msg_prev.header.stamp.nsecs = rospy.Time.now().nsecs
        pcl_msg_prev.header.frame_id = "base_link"
        pcl_pub_prev.publish(pcl_msg_prev)

        # publish current pcl
        pcl_msg_cur.header.stamp.secs = rospy.Time.now().secs
        pcl_msg_cur.header.stamp.nsecs = rospy.Time.now().nsecs
        pcl_msg_cur.header.frame_id = "base_link"
        pcl_pub_cur.publish(pcl_msg_cur)
        # we need to find the transformRANSAC

        # After this we will have the the pairs dictionary
        # In this we will have source points as keys and dst points as values
        H_new, distances, iterations = icp.icp(DST, SRC, max_iterations=20,tolerance=0.0001) #0.00000001
        H = H.dot(H_new)
        # lets see if this compiles
        print(H)

        # what we really need is the yaw angle
        sin_theta = H[1,0];
        cos_theta = H[1,1];
        yaw = m.atan2(sin_theta, cos_theta)
        roll = 0
        pitch = 0

        # print("yaw value is: {}".format(m.degrees(yaw)))

        q = quaternion_from_euler(roll, pitch, yaw)
        x = H[0,3]
        y = H[1,3]
        # create a pose message
        pose = PoseStamped()
        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.position.z = 0
        pose.pose.orientation.x = q[0]#odom_msg.pose.pose.orientation.x
        pose.pose.orientation.y = q[1]#odom_msg.pose.pose.orientation.y
        pose.pose.orientation.z = q[2]#odom_msg.pose.pose.orientation.z
        pose.pose.orientation.w = q[3]#odom_msg.pose.pose.orientation.w
        pose.header.frame_id = "odom"
        pose.header.stamp.secs = rospy.Time().secs
        pose.header.stamp.nsecs = rospy.Time().nsecs
        pose_pub.publish(pose)


        # store values
        trajVoX.append(x)
        trajVoY.append(y)
        trajVoTh.append(yaw)
        # store the odom values
        odom = rospy.wait_for_message("/odom", Odometry)
        Aq = odom.pose.pose.orientation
        ori_list = [Aq.x, Aq.y, Aq.z, Aq.w]
        (_, _, yawAct) = euler_from_quaternion(ori_list)
        xAct = odom.pose.pose.position.x
        yAct = odom.pose.pose.position.y
        trajActX.append(xAct)
        trajActY.append(yAct)
        trajActTh.append(yawAct)

    cv_left_image_prev = cv_left_image_cur
    cv_right_image_prev = cv_right_image_cur
    prev_time = cur_time

    # lets actually store odom and current trajectory


if __name__ == "__main__":
    try:
        left_cam_sub = message_filters.Subscriber("/stereo/head_C_left/image_raw", Image)
        right_cam_sub = message_filters.Subscriber("/stereo/head_C_right/image_raw", Image)
        cam_sub = message_filters.ApproximateTimeSynchronizer([left_cam_sub, right_cam_sub], queue_size = 5, slop = 0.1)
        cam_sub.registerCallback(stereo_cb)
        rospy.spin()

        # after this we will plot
        plt.plot(trajVoX, trajVoY, 'r.', label = "VO")
        plt.plot(trajActX, trajActY, 'b.', label = "GT")
        # plt.plot(trajVoTh)
        # plt.plot(trajActTh)
        plt.legend()
        plt.show()
    except rospy.ROSInterruptException:
        pass
