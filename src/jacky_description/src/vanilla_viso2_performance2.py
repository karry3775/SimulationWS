#!/usr/bin/env python
import rospy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion
import message_filters # for time synchronization
import matplotlib.pyplot as plt

# Global variables
X_gt = []
Y_gt = []
YAW_gt = []
X_vo = []
Y_vo = []
YAW_vo = []

# init a node
rospy.init_node("compare_vanilla_viso2_node")
# subscribe to both topics and store the X,Y,yaw values for both


def stereo_cb(vo_msg):
    print("getting messages")
    global X_gt, Y_gt, YAW_gt, X_vo, Y_vo, YAW_vo

    # wait for odom message
    gt_msg = rospy.wait_for_message("/odom", Odometry)
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

    #append to global variables
    X_gt.append(gtx); Y_gt.append(gty); YAW_gt.append(gtyaw)
    X_vo.append(vox); Y_vo.append(voy); YAW_vo.append(voyaw)

if __name__ == "__main__":

    try:
        vo_sub = rospy.Subscriber("/stereo_odometer/pose", PoseStamped, stereo_cb)
        rospy.spin()

        print(X_gt)
        # plot the variables
        fig, axs = plt.subplots(2)
        axs[0].set_aspect('equal')
        # axs[1].set_aspect('equal')
        fig.suptitle('vanilla vo comparison')
        axs[0].plot(X_gt, Y_gt,'r-')
        axs[0].plot(X_vo, Y_vo,'b-')
        axs[1].plot(YAW_gt,'r-')
        axs[1].plot(YAW_vo,'b-')

        plt.show()
    except rospy.ROSInterruptException:
        pass