#!/usr/bin/env python
import rospy
import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import message_filters #for time synchronization

rospy.init_node("Image_writer_node")

def stereo_cb(msg_left, msg_right):
    try:
        cv_left_image = CvBridge().imgmsg_to_cv2(msg_left, "bgr8")
        cv_right_image = CvBridge().imgmsg_to_cv2(msg_right, "bgr8")
    except CvBridgeError as e:
        print(e)
    else:
        # we need to save the images
        left_gray = cv2.cvtColor(cv_left_image, cv2.COLOR_BGR2GRAY)
        right_gray = cv2.cvtColor(cv_right_image, cv2.COLOR_BGR2GRAY)

        cv2.imwrite("left_gray.png", left_gray)
        cv2.imwrite("right_gray.png", right_gray)
        cv2.imwrite("left_color.png", cv_left_image)
        cv2.imwrite("right_color.png", cv_right_image)

if __name__ == "__main__":
    try:
        left_cam_sub = message_filters.Subscriber("/stereo/head_C_left/image_raw", Image)
        right_cam_sub = message_filters.Subscriber("/stereo/head_C_right/image_raw", Image)
        cam_sub = message_filters.ApproximateTimeSynchronizer([left_cam_sub, right_cam_sub], queue_size = 5, slop = 0.1)
        cam_sub.registerCallback(stereo_cb)
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
