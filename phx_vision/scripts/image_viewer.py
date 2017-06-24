'''
Created on May 10, 2016

@author: mykyta_denysov
'''

#!/usr/bin/env python
import roslib
import sys
import rospy
import cv2
import tf2_ros
import numpy as np
import geometry_msgs.msg
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError


class image_viewer:
    def __init__(self, image_topic):
        self.window_name = image_topic
        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber(image_topic, Image, self.image_callback, queue_size=1)
        self.image = None

    def image_callback(self, data=Image()):
        print 'image format h,w', data.height, data.width
        self.image = self.bridge.imgmsg_to_cv2(data, "mono8")
        cv2.imshow(self.window_name, self.image)
        cv2.waitKey(1000)


def main():
  rospy.init_node('image_view', anonymous=True)
  front_cam = image_viewer('/cam_front/image_rect')
  #bottom_cam = image_viewer('/cam_bottom/image_rect')
  while not rospy.is_shutdown():
    rospy.spin()

  cv2.destroyAllWindows()

if __name__ == '__main__':
    main()
