import roslib
roslib.load_manifest('phx_vision')
import sys
import os
import rospy
import cv2
import numpy as np
import tf
import tf2_ros
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from geometry_msgs.msg import TwistStamped
from nav_msgs.msg import Odometry


class OpticalFlowOdometry:
    def __init__(self):
        self.position = [0, 0]
        self.factor = [1, 1]
        self.last_time = None

        self.twist_sub = rospy.Subscriber("opticalFlow", TwistStamped, self.callback)
        self.odom_pub = rospy.Publisher("opticalFlowOdom", Odometry, queue_size=1)
        self.odom_msg = Odometry()

    def callback(self, twist_data=TwistStamped()):
        new_time = twist_data.header.stamp.secs()
        dx = self.factor[0] * twist_data.twist.linear.x
        dy = self.factor[1] * twist_data.twist.linear.y

        self.position[0] += dx
        self.position[1] += dy

        self.odom_msg.header.stamp = rospy.Time.now()
        self.odom_msg.pose.pose.position.x = self.position[0]
        self.odom_msg.pose.pose.position.y = self.position[1]

        self.odom_pub.publish(self.odom_msg)


def main(args):
    of = OpticalFlowOdometry()
    rospy.init_node('OpticalFlowOdometry', anonymous=True)
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print ("Shutting down")


if __name__ == '__main__':
    main(sys.argv)

