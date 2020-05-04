#!/usr/bin/env python
import rospy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion
import message_filters # for time synchronization
import matplotlib.pyplot as plt
import numpy as np

# Global variables
X_gt = []
Y_gt = []
YAW_gt = []
X_vo = []
Y_vo = []
YAW_vo = []

# Global variables for deltas
DELTA_X_VO = []
DELTA_Y_VO = []
DELTA_X_GT = []
DELTA_Y_GT = []

# firstFlag
firstFlag = True

# init a node
rospy.init_node("compare_vanilla_viso2_node")
# subscribe to both topics and store the X,Y,yaw values for both


def stereo_cb(vo_msg):
    print("getting messages")
    global X_gt, Y_gt, YAW_gt, X_vo, Y_vo, YAW_vo, DELTA_X_VO, DELTA_Y_VO, DELTA_X_GT, DELTA_Y_GT, firstFlag

    # wait for odom message
    gt_msg = rospy.wait_for_message("/ground_truth/state", Odometry)
    # extract pose
    gt_pose = gt_msg.pose.pose
    vo_pose = vo_msg.pose

    gtx = gt_pose.position.x
    gty = gt_pose.position.y
    gtq = gt_pose.orientation
    (_,_,gtyaw) = euler_from_quaternion([gtq.x, gtq.y, gtq.z, gtq.w])

    vox = vo_pose.position.x
    voy = vo_pose.position.y
    voq = vo_pose.orientation
    (_, _, voyaw) = euler_from_quaternion([voq.x, voq.y, voq.z, voq.w])

    if firstFlag:
        firstFlag = False
    else:
        # get the last vo states
        vox_prev = X_vo[-1]
        voy_prev = Y_vo[-1]
        voyaw_prev = YAW_vo[-1]
        # get the last ground_truth states
        gtx_prev = X_gt[-1]
        gty_prev = Y_gt[-1]
        gtyaw_prev = YAW_gt[-1]

        # get the vo rotation matrix
        R_vo = np.array([[np.cos(voyaw_prev), -np.sin(voyaw_prev)],
                         [np.sin(voyaw_prev), np.cos(voyaw_prev)]])
        # get the gt rotation matrix
        R_gt = np.array([[np.cos(gtyaw_prev), -np.sin(gtyaw_prev)],
                         [np.sin(gtyaw_prev), np.cos(gtyaw_prev)]])

        # get the deltas
        delta_vo_vector = np.dot(R_vo, np.array([[vox - vox_prev],
                                                 [voy - voy_prev]]))
        delta_gt_vector = np.dot(R_gt, np.array([[gtx - gtx_prev],
                                                 [gty - gty_prev]]))

        # append
        DELTA_X_VO.append(float(delta_vo_vector[0]))
        DELTA_Y_VO.append(float(delta_vo_vector[1]))
        DELTA_X_GT.append(float(delta_gt_vector[0]))
        DELTA_Y_GT.append(float(delta_gt_vector[1]))


    #append to global variables
    X_gt.append(gtx); Y_gt.append(gty); YAW_gt.append(gtyaw)
    X_vo.append(vox); Y_vo.append(voy); YAW_vo.append(voyaw)

if __name__ == "__main__":

    try:
        vo_sub = rospy.Subscriber("/stereo_odometer/pose", PoseStamped, stereo_cb)
        rospy.spin()

        print(X_gt)
        # plot the variables
        fig, axs = plt.subplots(4)
        axs[0].set_aspect('equal')
        # axs[1].set_aspect('equal')
        fig.suptitle('VO vs Ground Truth')
        axs[0].plot(X_gt, Y_gt,'r-', label = "ground_truth")
        axs[0].plot(X_vo, Y_vo,'b-', label = "vo output")
        axs[0].set_title("Trajectory comparison (x, y)")
        axs[1].plot(YAW_gt,'r-', label = "ground_truth")
        axs[1].plot(YAW_vo,'b-', label = "vo output")
        axs[1].set_title("Yaw comparison")
        axs[2].plot(DELTA_X_VO, 'b-', label = "vo output")
        axs[2].plot(DELTA_X_GT, 'r-', label = "ground_truth")
        axs[2].set_title("Delta x")
        axs[3].plot(DELTA_Y_VO, 'b-', label = "vo output")
        axs[3].plot(DELTA_Y_GT, 'r-', label = "ground_truth")
        axs[3].set_title("Delta y")

        plt.legend()
        plt.show()
    except rospy.ROSInterruptException:
        pass
