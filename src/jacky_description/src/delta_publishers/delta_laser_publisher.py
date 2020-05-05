#!/usr/bin/env python
import rospy
from nav_msgs.msg import Odometry
from jacky_description.msg import DeltaMsgStamped
from tf.transformations import euler_from_quaternion
import math as m

rospy.init_node("delta_laser_publisher_node")
# create a publisher
delta_pub = rospy.Publisher("/laser_odometer/delta_topic", DeltaMsgStamped, queue_size = 10)


## Global Variables
firstRun = True
prevx = 0
prevy = 0
prevyaw = 0

def wrapToPi(theta):
    return m.atan2(m.sin(theta), m.cos(theta))

def laser_odom_cb(msg):
    global firstRun, prevx, prevy, prevyaw
    # extract pose
    pose = msg.pose.pose
    x = pose.position.x
    y = pose.position.y
    q = pose.orientation
    (_, _, yaw) = euler_from_quaternion([q.x, q.y, q.z, q.w])
    print("prevyaw : {}, cur_yaw : {}".format(prevyaw, yaw))
    if firstRun:
        firstRun = False
        delta_yaw = wrapToPi(yaw - prevyaw)
        delta_x = x - prevx
        delta_y = y - prevy
    else:
        # follow the derivation from overleaf
        beta_1 = m.atan2(prevy, prevx)
        beta_2 = m.atan2(y, x)
        beta_dash = wrapToPi(beta_2 - beta_1)
        F1F2 = m.sqrt((x - prevx)**2 + (y - prevy)**2)
        delta_yaw = wrapToPi(yaw - prevyaw)

        if F1F2 < 0.000001:
            delta_x = 0.0
            delta_y = 0.0
        else:
            OF1 = m.sqrt(prevx**2 + prevy**2)
            OF2 = m.sqrt(x**2 + y**2)
            # OF1 might be zero for the case when prevx and prevy were actually equal to zero
            if OF1 < 0.000001:
                beta_3 = wrapToPi(yaw - prevyaw)
                delta_x = OF2 * m.cos(beta_3)
                delta_y = OF2 * m.sin(beta_3)
            else:
                # from cosine rule
                b = F1F2
                a = OF1
                c = OF2
                cos_alpha = (a**2 + b**2 - c**2) / (2*a*b)
                # using sine rule
                sin_alpha = c * m.sin(beta_dash) / b
                alpha = m.atan2(sin_alpha, cos_alpha)

                # then beta_3 can be determined
                beta_3 = wrapToPi(m.pi - alpha - wrapToPi(prevyaw - beta_1))

                # then we can safely determine the three delta values
                delta_x = F1F2 * m.cos(beta_3)
                delta_y = F1F2 * m.sin(beta_3)
    # reset prev values
    prevx = x
    prevy = y
    prevyaw = yaw

    print("printing : {}, {}, {}".format(delta_x, delta_y, delta_yaw))
    # publish
    delta_msg = DeltaMsgStamped()
    delta_msg.header.stamp = rospy.Time.now()
    delta_msg.data = [delta_x, delta_y, delta_yaw]
    delta_pub.publish(delta_msg)


if __name__ == "__main__":
    try:
        laser_odom_sub = rospy.Subscriber("/odom_rf2o", Odometry, laser_odom_cb)
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
