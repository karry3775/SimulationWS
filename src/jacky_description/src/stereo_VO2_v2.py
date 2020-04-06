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
import random
from tf.transformations import quaternion_from_euler

#Initiate the node
rospy.init_node("stereo_VO_v2_node")

# create an image publisher
image_pub = rospy.Publisher("/features", Image, queue_size = 10)
pose_pub = rospy.Publisher("/jacky_vo_pose", PoseStamped, queue_size = 10)
pcl_pub = rospy.Publisher("/jacky_pcl", PointCloud, queue_size = 10)

# Global variables
cv_left_image_prev = None
cv_right_image_prev = None
ini_image_obtained = False
prev_time = None
H = np.eye(4) # initial pose
transformNum = 0

# Camera calibration matrix
k_calib = np.array([[238.3515418, 0.0, 200.5],
                    [0.0, 238.3515418, 200.5],
                    [0.0, 0.0, 1.0]])

def ORB_detect(img,c):
    orb = cv2.ORB_create() # nfeatures=1000000, scoreType=cv2.ORB_FAST_SCORE
    kp, des = orb.detectAndCompute(img, None)
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

def populate_dict(match, kp1, kp2):
    d = {}
    for m in match:
        query_idx = m.queryIdx
        train_idx = m.trainIdx
        #get first image matched keypoints
        p1_x, p1_y = kp1[query_idx].pt
        #get the second image matched keypoints
        p2_x, p2_y = kp2[train_idx].pt
        # push it to the dict
        d[(p1_x, p1_y)] = (p2_x, p2_y) # (p1_x, p1_y) : (p2_x, p2_y)

    return d

def find_common_matches(match1, match2, match3, kp_prev_left, kp_prev_right, kp_cur_left, kp_cur_right):
    common_matches = []
    # Go through the first match1 list and for every pair check if there is a pair exisiting in the match2
    # to accelerate the process we can make use of a dict which can ensure constant time lookup
    # lets first make a dict1
    dict1 = populate_dict(match1, kp_prev_left, kp_prev_right)
    dict2 = populate_dict(match2, kp_prev_right, kp_cur_left)
    dict3 = populate_dict(match3, kp_cur_left, kp_cur_right)

    for key1, key2 in dict1.items():
        if key2 in dict2:
            key3 = dict2[key2]
            if key3 in dict3:
                key4 = dict3[key3]
                common_matches.append([key1, key2, key3, key4])

    return common_matches

def triangulate(ptL, ptR, b, f, u0, v0):
    uL, vL = ptL
    uR, vR = ptR
    # calculate xL, xR, yL
    xL = uL - u0
    xR = uR - u0
    yL = vL - v0
    # calculate disparity
    d = xL - xR
    if(d == 0):
        return None
    Z = (f * b) / d
    X = (Z * xL) / f
    Y = (Z * yL) / f
    # convert to correct frame
    R = np.array([[0,0,1],
                  [-1,0,0],
                  [0,1,0]])
    pt = np.array([[X],[Y],[Z]])
    pt = R.dot(pt)
    X = float(pt[0])
    Y = float(pt[1])
    Z = float(pt[2])
    return [X, Y, Z]

def triangulateAll(common_matches, stereo_base, focal_length):
    # defining preliminaries
    b = stereo_base
    f = focal_length
    u0 = 200.5
    v0 = 200.5

    pairs = {}
    for matches in common_matches:
        ptL_prev, ptR_prev, ptL_cur, ptR_cur = matches
        retVal_prev = triangulate(ptL_prev, ptR_prev, b, f, u0, v0)
        retVal_cur = triangulate(ptL_cur, ptR_cur, b, f, u0, v0)
        if(retVal_prev == None or retVal_cur == None):
            continue

        X_prev, Y_prev, Z_prev = retVal_prev
        X_cur, Y_cur, Z_cur = retVal_cur

        pairs[(X_prev, Y_prev, Z_prev)] = (X_cur, Y_cur, Z_cur)

    return pairs

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
  k = 30 # number of iterations
  n = 8 # we just need four points because they give us 12 equations
  t = 0.4 # threshold of error
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
  return Hbest

def stereo_cb(msg_left, msg_right):
    print("stereo_cb called\n")
    global cv_left_image_prev, cv_right_image_prev, ini_image_obtained, prev_time, H, transformNum
    try:
        cv_left_image_cur = CvBridge().imgmsg_to_cv2(msg_left, "bgr8")
        cv_right_image_cur = CvBridge().imgmsg_to_cv2(msg_right, "bgr8")
        # convert to gray
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
        # compute disparities
        disparity_prev = stereo.compute(cv_left_image_prev, cv_right_image_prev)
        diparity_cur = stereo.compute(cv_left_image_cur, cv_right_image_cur)
        # compute ORB features for four frames
        kp_prev_left, des_prev_left, prev_img_w_kp_left = ORB_detect(cv_left_image_prev, (255, 0, 0))
        kp_prev_right, des_prev_right, prev_img_w_kp_right = ORB_detect(cv_right_image_prev, (0, 255, 0))
        kp_cur_left, des_cur_left, cur_img_w_kp_left = ORB_detect(cv_left_image_cur, (0, 0, 255))
        kp_cur_right, des_cur_right, cur_img_w_kp_right = ORB_detect(cv_right_image_cur, (255, 255, 0))
        # combined image
        combined_image = cv2.hconcat([prev_img_w_kp_left, prev_img_w_kp_right, cur_img_w_kp_left, cur_img_w_kp_right])

        # get matches for three pair of images
        # match1 - (left_prev, right_prev)
        match1 = ORB_draw_matches(cv_left_image_prev, cv_right_image_prev, kp_prev_left, kp_prev_right, des_prev_left, des_prev_right, True)
        # match2 - (right_prev, left_cur)
        match2 = ORB_draw_matches(cv_right_image_prev, cv_left_image_cur, kp_prev_right, kp_cur_left, des_prev_right, des_cur_left)
        # match3 - (left_cur, right_cur)
        match3 = ORB_draw_matches(cv_left_image_cur, cv_right_image_cur, kp_cur_left, kp_cur_right, des_cur_left, des_cur_right)

        # find common matches
        finIndex = int(0.50 * len(match1))
        common_matches = find_common_matches(match1[:finIndex], match2, match3, kp_prev_left, kp_prev_right, kp_cur_left, kp_cur_right)

        # so this looks good, now we have to triangulate the 3d
        stereo_base = 0.064 # in m
        focal_length = 238.3515418 # in unit pixels
        pairs = triangulateAll(common_matches, stereo_base, focal_length)

        # now we need to run a least squares problem on this
        H_new = getTransformRANSAC(pairs)
        # lets modify H_new
        H_modified = np.eye(4)
        H_modified[0:2, 0:2] = H_new[0:2, 0:2]
        H_modified[0:2, 3:] = H_new[0:2, 3:]

        H_new = H_modified
        translation = np.array([H_new[0,3], H_new[1,3], H_new[2,3]])
        print("translational dx dy dz : {}".format(translation))
        H = H.dot(H_new)

        print("transform number {} is:\n {}\n".format(transformNum,H))
        transformNum += 1

        # lets try displaying this transform
        # what we can do is publish it as a pose
        # for that we will need to convert rotation matrix to a quaternion
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
        pose.pose.orientation.x = q[0]
        pose.pose.orientation.y = q[1]
        pose.pose.orientation.z = q[2]
        pose.pose.orientation.w = q[3]
        pose.header.frame_id = "odom"
        pose.header.stamp.secs = rospy.Time().secs
        pose.header.stamp.nsecs = rospy.Time().nsecs
        pose_pub.publish(pose)

        # print("the difference in time is: {}".format(cur_time - prev_time))


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
