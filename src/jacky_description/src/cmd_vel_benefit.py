#!/usr/bin/env python
import rospy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion
import message_filters # for time synchronization
import matplotlib.pyplot as plt
import numpy as np

# Global variables with dely assumption
X_gt = []
Y_gt = []
YAW_gt = []
X_vo = []
Y_vo = []
YAW_vo = []

DELTA_X_VO = []
DELTA_Y_VO = []
DELTA_X_GT = []
DELTA_Y_GT = []

# firstFlag
firstFlag = True

# Global variables without dely assumption
nX_vo = []
nY_vo = []
nYAW_vo = []

nDELTA_X_VO = []
nDELTA_Y_VO = []
nDELTA_X_GT = []
nDELTA_Y_GT = []

# firstFlag
nfirstFlag = True




# init a node
rospy.init_node("cmd_vel_benefit_node")
# subscribe to both topics and store the X,Y,yaw values for both


def stereo_cb_w_cmd_vel(vo_msg):
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

def stereo_cb_wo_cmd_vel(vo_msg):
    print("getting messages")
    global nX_vo, nY_vo, nYAW_vo, nDELTA_X_VO, nDELTA_Y_VO, nfirstFlag

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

    if nfirstFlag:
        nfirstFlag = False
    else:
        # get the last vo states
        vox_prev = nX_vo[-1]
        voy_prev = nY_vo[-1]
        voyaw_prev = nYAW_vo[-1]

        # get the vo rotation matrix
        R_vo = np.array([[np.cos(voyaw_prev), -np.sin(voyaw_prev)],
                         [np.sin(voyaw_prev), np.cos(voyaw_prev)]])

        # get the deltas
        delta_vo_vector = np.dot(R_vo, np.array([[vox - vox_prev],
                                                 [voy - voy_prev]]))

        # append
        nDELTA_X_VO.append(float(delta_vo_vector[0]))
        nDELTA_Y_VO.append(float(delta_vo_vector[1]))


    #append to global variables
    nX_vo.append(vox); nY_vo.append(voy); nYAW_vo.append(voyaw)

if __name__ == "__main__":

    try:
        vo_sub_w_cmd_vel = rospy.Subscriber("/stereo_odometer_with_cmd_vel/pose_with_cmd_vel", PoseStamped, stereo_cb_w_cmd_vel)
        vo_sub_wo_cmd_vel = rospy.Subscriber("/stereo_odometer_without_cmd_vel/pose_without_cmd_vel", PoseStamped, stereo_cb_wo_cmd_vel)
        rospy.spin()

        print(X_gt)
        # plot the variables
        fig, axs = plt.subplots(2, 2)
        axs[0,0].set_aspect('equal')
        axs[0,0].plot(X_gt, Y_gt,'r-', label = "GT")
        axs[0,0].plot(X_vo, Y_vo,'b-', label = "VO with Motion Model")
        axs[0,0].plot(nX_vo, nY_vo, 'm-', label = "VO without Motion Model")
        axs[0,0].set_title("Robot Path")

        axs[0,1].plot(YAW_gt,'r-', label = "GT")
        axs[0,1].plot(YAW_vo,'b-', label = "VO with Motion Model")
        axs[0,1].plot(nYAW_vo, 'm-', label = "VO without Motion Model")
        axs[0,1].set_title("Yaw")


        axs[1,0].plot(DELTA_X_GT, 'r-', label = "GT")
        axs[1,0].plot(DELTA_X_VO, 'b-', label = "VO with Motion Model")
        axs[1,0].plot(nDELTA_X_VO, 'm-', label = "VO without Motion Model")
        axs[1,0].set_title("Delta x")

        axs[1,1].plot(DELTA_Y_GT, 'r-', label = "GT")
        axs[1,1].plot(DELTA_Y_VO, 'b-', label = "VO with Motion Model")
        axs[1,1].plot(nDELTA_Y_VO, 'm-', label = "VO without Motion Model")
        axs[1,1].set_title("Delta y")

        lines, labels = fig.axes[-1].get_legend_handles_labels()
        fig.legend(lines, labels, loc = 'upper center')
        plt.show()
    except rospy.ROSInterruptException:
        pass
