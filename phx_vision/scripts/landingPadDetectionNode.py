#!/usr/bin/env python
import roslib
import sys
import rospy
import cv2
import numpy as np
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
roslib.load_manifest('phx_vision')


class image_converter(object):
    def __init__(self):
        cv2.namedWindow("Image window", 1)
        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber("image_mono", Image, self.callback)

    def callback(self, data):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, "mono8")
            resizedGreyImage = cv_image
        except CvBridgeError, e:
            print(e)

    # Read from file for debug purposes
    # image = cv2.imread("pad.png")
    # greyImage = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)

    # resizedGreyImage = cv2.resize(greyImage, (0,0), fx=0.125, fy=0.125)
    (_, thresholdedImage) = cv2.threshold(resizedGreyImage, 200, 255,
                                          cv2.THRESH_TOZERO)

    lower = 245
    upper = 260

    edgedImage = cv2.Canny(resizedGreyImage, lower, upper)
    cv2.imshow("Image", resizedGreyImage)
    cv2.imshow("edgedImage", edgedImage)
    # cv2.imshow("ThresholdedImage", thresholdedImage)
    cv2.waitKey(1)


def main(args):
    image_converter()
    rospy.init_node('image_converter', anonymous=True)
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")
        cv2.destroyAllWindows()


if __name__ == '__main__':
    main(sys.argv)
