#!/usr/bin/env python
import rospy
from sensor_msgs.msg import Image
from geometry_msgs.msg import PoseStamped
from tf.transformations import quaternion_from_euler
from cv_bridge import CvBridge, CvBridgeError
import cv2
import numpy as np
import math as m

# global variables
ori = None
ori_init = False

# intialize the node
rospy.init_node("line_node")

# create a pose_publisher
pose_pub = rospy.Publisher("/abs_orientation", PoseStamped, queue_size = 10)
image_pub = rospy.Publisher("/features", Image, queue_size = 10)

def wrap2Pi(theta):
    return m.atan2(m.sin(theta), m.cos(theta))

def image_cb(msg):
    global ori, ori_init
    HOUGH_P = False
    """
    need to store prev frame
    """
    try:
        cv_image = CvBridge().imgmsg_to_cv2(msg, "bgr8")
    except CvBridgeError as e:
        print(e)
    else:
        gray = cv2.cvtColor(cv_image,cv2.COLOR_BGR2GRAY)
        edges = cv2.Canny(gray,50,150,apertureSize = 3)
        minLineLength = 100
        maxLineGap = 10
        if HOUGH_P:
            lines = cv2.HoughLinesP(edges,1,np.pi/180,100,minLineLength,maxLineGap)
            for x1,y1,x2,y2 in lines[0]:
                cv2.line(cv_image,(x1,y1),(x2,y2),(0,255,0),2)
        else:
            lines = cv2.HoughLines(edges,1,np.pi/180,200)

            for rho,theta in lines[0]:
                a = np.cos(theta)
                b = np.sin(theta)
                x0 = a*rho
                y0 = b*rho
                x1 = int(x0 + 1000*(-b))
                y1 = int(y0 + 1000*(a))
                x2 = int(x0 - 1000*(-b))
                y2 = int(y0 - 1000*(a))

                cv2.line(cv_image,(x1,y1),(x2,y2),(0,0,255),2)

            if not ori_init:
                ori_init = True
                _, ori = lines[0][0]
            else:
                _, f_ori = lines[0][0]

                f_ori = wrap2Pi(f_ori)

                # print(f_ori , ori)
                print("prev_ori : {}".format(m.degrees(ori)))
                # this might also happen that f_ori is off by 90 degrees
                f_ori1 = wrap2Pi(f_ori + m.radians(90))
                f_ori2 = wrap2Pi(f_ori + m.radians(-90))
                f_ori3 = wrap2Pi(f_ori + m.radians(180))
                # we need to find which has the smallest difference
                # f_ori, f_ori1 or f_ori2
                if(abs(wrap2Pi(ori - f_ori)) < abs(wrap2Pi(ori - f_ori1)) and abs(wrap2Pi(ori - f_ori)) < abs(wrap2Pi(ori - f_ori2)) and abs(wrap2Pi(ori - f_ori)) < abs(wrap2Pi(ori - f_ori3))):
                    ori = f_ori
                elif(abs(wrap2Pi(ori - f_ori1)) < abs(wrap2Pi(ori - f_ori)) and abs(wrap2Pi(ori - f_ori1)) < abs(wrap2Pi(ori - f_ori2)) and abs(wrap2Pi(ori - f_ori1)) < abs(wrap2Pi(ori - f_ori3))):
                    ori = f_ori1
                elif(abs(wrap2Pi(ori - f_ori2)) < abs(wrap2Pi(ori - f_ori)) and abs(wrap2Pi(ori - f_ori2)) < abs(wrap2Pi(ori - f_ori1)) and abs(wrap2Pi(ori - f_ori2)) < abs(wrap2Pi(ori - f_ori3))):
                    ori = f_ori2
                else:
                    ori = f_ori3

                print("Orientaions --> f_ori : {}, f_ori1 : {}, f_ori2 : {}, f_ori3 : {}, ORI : {}".format(m.degrees(f_ori), m.degrees(f_ori1), m.degrees(f_ori2), m.degrees(f_ori3), m.degrees(ori)))

        # create euler angles
        roll = 0
        pitch = 0
        yaw = -ori

        # convert to quaternion
        q = quaternion_from_euler(roll, pitch, yaw)
        # create a pose message
        pose = PoseStamped()
        pose.pose.position.x = 0
        pose.pose.position.y = 0
        pose.pose.position.z = 0
        pose.pose.orientation.x = q[0]
        pose.pose.orientation.y = q[1]
        pose.pose.orientation.z = q[2]
        pose.pose.orientation.w = q[3]
        pose.header.frame_id = "odom"
        pose.header.stamp.secs = rospy.Time().secs
        pose.header.stamp.nsecs = rospy.Time().nsecs
        pose_pub.publish(pose)

        # cv2.imshow("frame", cv_image)
        # cv2.waitKey(1)
        rosimg = CvBridge().cv2_to_imgmsg(cv_image, "bgr8")
        image_pub.publish(rosimg)


def camera_feed_sub():
    rospy.Subscriber('upward_camera/image_raw',Image,image_cb)

if __name__ == "__main__":
    try:
        camera_feed_sub()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
