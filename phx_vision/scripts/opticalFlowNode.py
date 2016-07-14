#!/usr/bin/env python
"""
@author: tatsch
"""

import roslib
roslib.load_manifest('phx_vision')
import sys
import os
import rospy
import cv2
import numpy as np
import tf
import tf2_ros
from sensor_msgs.msg import Image, CameraInfo
import image_geometry
from cv_bridge import CvBridge, CvBridgeError
from geometry_msgs.msg import TwistStamped
from phx_uart_msp_bridge.msg import Altitude

class OpticalFlow:
    def __init__(self):
        self.prev = None
        self.img = None
        self.flow = None
        self.distance = 0.2
        self.runningOnPhoenix = True
	self.cameraModel = image_geometry.PinholeCameraModel()
        
        if not self.runningOnPhoenix:
            cv2.namedWindow("flow", 1)
            
        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber("image_mono", Image, self.imageCallback)
	self.info_sub = rospy.Subscriber('camera_info', CameraInfo, self.cameraInfoCallback)
	self.altitude_sub = rospy.Subscriber("phx/altitude", Altitude, self.altitudeCallback)
        self.twist_pub = rospy.Publisher("opticalFlow", TwistStamped, queue_size=10)
        self.image_pub = rospy.Publisher("flowImage", Image, queue_size=1)
        self.twist = TwistStamped()

    def altitudeCallback(self, data):
	self.distance = data.estimated_altitude

    def cameraInfoCallback(self, data):
        self.cameraModel.fromCameraInfo(data)

    def imageCallback(self, data):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, "mono8")
            
            if self.prev is None:
                self.prev = cv2.resize(cv_image, (0, 0), fx=0.125, fy=0.125)
                self.img = self.prev
            
            self.img = cv2.resize(cv_image, (0, 0), fx=0.125, fy=0.125)
            self.flow = cv2.calcOpticalFlowFarneback(self.prev, self.img, None, 0.5, 3, 15, 3, 5, 1.2, 0)
            self.prev = self.img
            if self.runningOnPhoenix:
                try:
                    self.image_pub.publish(self.bridge.cv2_to_imgmsg(self.draw_flow(self.img, self.flow), "rgb8"))
                except CvBridgeError as e:
                    print(e)
            else:
                cv2.imshow('flow', self.draw_flow(self.img, self.flow))

            fx = np.median(self.flow[:, :, 0])
            fy = np.median(self.flow[:, :, 1])
            vx = fx * self.distance / self.cameraModel.fx()
            vy = fy * self.distance / self.cameraModel.fy()

            self.twist.header.stamp = rospy.Time.now()
            self.twist.twist.linear.x = vx
            self.twist.twist.linear.y = vy
            self.twist_pub.publish(self.twist)
                
        except CvBridgeError as e:
            print(e)

    def draw_flow(self, img, flow, step=16):
        h, w = img.shape[:2]
        y, x = np.mgrid[step/2:h:step, step/2:w:step].reshape(2,-1).astype(int)
        fx, fy = flow[y,x].T
        lines = np.vstack([x, y, x+fx, y+fy]).T.reshape(-1, 2, 2)
        lines = np.int32(lines + 0.5)
        vis = cv2.cvtColor(img, cv2.COLOR_GRAY2BGR)
        cv2.polylines(vis, lines, 0, (0, 255, 0))
        for (x1, y1), (x2, y2) in lines:
            cv2.circle(vis, (x1, y1), 1, (0, 255, 0), -1)
        return vis


def main(args):
    of = OpticalFlow()
    rospy.init_node('OpticalFlow', anonymous=True)
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print ("Shutting down")
    cv2.destroyAllWindows()


if __name__ == '__main__':
    main(sys.argv)

