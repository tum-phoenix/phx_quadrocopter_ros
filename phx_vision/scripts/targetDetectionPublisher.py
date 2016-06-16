#!/usr/bin/env python

'''
Created on May 10, 2016

@author: mykyta_denysov
'''

import roslib
roslib.load_manifest('phx_vision')
import os
import sys
import rospy
import cv2
import tf
import tf2_ros
import numpy as np
import geometry_msgs.msg
import math
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError


class image_converter:
    def __init__(self):
        self.counter = 0
        if (os.uname()[4][:3] == 'arm'):
            self.runningOnPhoenix = True
        else:
            self.runningOnPhoenix = False
        if (not self.runningOnPhoenix):
            cv2.namedWindow("Image window", 1)
        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber("image_mono", Image, self.callback)
        self.image_pub = rospy.Publisher("image_topic_2", Image)
        self.br = tf2_ros.TransformBroadcaster()
        self.t = geometry_msgs.msg.TransformStamped()

        self.pts = [[None], [None], [None], [None], [None], [None]]
        self.Pnt_nxt = [[None], [None], [None], [None], [None], [None]]
        self.past_pts = 3
        self.maxLength = self.past_pts + 2
        self.counter = 0
        self.dX, self.dY = 0, 0
        self.knownWidth = 0.033  # m
        self.focalLength_x = 335.874937  # Test
        self.focalLength_y = 335.599836

        self.dist_x = 0
        self.dist_y = 0
        self.dist_camera = 0

    def callback(self, data):
        self.t.header.stamp = rospy.Time.now()
        self.t.header.seq = self.counter
        self.t.header.frame_id = "camera"
        self.t.transform.translation.x = 0.0
        self.t.transform.translation.y = 0.0
        self.t.transform.translation.z = 0.0
        q = tf.transformations.quaternion_from_euler(0, 0, 0)
        self.t.transform.rotation.x = q[0]
        self.t.transform.rotation.y = q[1]
        self.t.transform.rotation.z = q[2]
        self.t.transform.rotation.w = q[3]

        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, "mono8")
            frame = cv_image
        except CvBridgeError as e:
            print(e)

        # convert the frame to grayscale, blur it, and detect edges
        # gray = cv2.cvtColor(resizedGreyImage, cv2.COLOR_BGR2GRAY)
        if (not self.runningOnPhoenix):
            cv2.imshow("Frame", frame)

        blurred = cv2.GaussianBlur(frame, (7, 7), 0)
        edged = cv2.Canny(blurred, 50, 150)

        heigth = 0.200  # mm
        precision_heigth = 0.030  # Hoehenangabe auf 3 cm genau

        # find contours in the edge map
        cnts = cv2.findContours(edged.copy(), cv2.RETR_EXTERNAL,
                                cv2.CHAIN_APPROX_SIMPLE)[-2]

        expected_Width_low = (self.focalLength_y * self.knownWidth) / (heigth + precision_heigth)
        expected_Width_high = (self.focalLength_y * self.knownWidth) / (heigth - precision_heigth)

        # loop over the contours
        for c in cnts:

            # approximate the contour
            peri = cv2.arcLength(c, True)
            approx = cv2.approxPolyDP(c, 0.01 * peri, True)

            # compute the solidity of the original contour
            area = cv2.contourArea(c)
            hullArea = cv2.contourArea(cv2.convexHull(c))

            rect = cv2.minAreaRect(approx)  # minimales Rechteck um Figure
            (Circle_x, Circle_y), radius = cv2.minEnclosingCircle(approx)

            Area_rect = rect[1][0] * rect[1][1]
            Area_circle = math.pi * radius ** 2
            center = (int(Circle_x), int(Circle_y))
            radius = int(radius)

            if Area_rect > 0.0 and Area_circle > 0.0:
                V_area_rect = area / Area_rect
                V_area_circle = area / Area_circle

            if len(approx) >= 3 and len(approx) < 5 and V_area_rect > 0.5 and V_area_rect < 0.58:
                shape = "Triangle"
                self.t.child_frame_id = "Triangle"
                list_id = 0


            elif len(approx) >= 4 and len(approx) < 6 and V_area_rect > 0.93 and V_area_rect < 1.05:
                # compute the bounding box of the contour and use the
                # bounding box to compute the aspect ratio
                shape = "Square"
                self.t.child_frame_id = "Square"
                list_id = 1



            elif len(approx) >= 5 and len(approx) < 8 and V_area_rect > 0.7 and V_area_rect < 0.78:
                shape = "Pentagon"
                self.t.child_frame_id = "Pentagon"
                list_id = 2


            elif len(approx) > 6 and len(approx) < 14 and V_area_rect > 0.7 and V_area_rect < 0.78:
                shape = "Heart"
                self.t.child_frame_id = "Heart"
                list_id = 3


            elif len(approx) >= 9 and len(approx) < 30 and V_area_rect > 0.79 and V_area_rect < 0.85:

                shape = "Circle"
                self.t.child_frame_id = "Circle"
                list_id = 4


            elif len(approx) >= 10 and V_area_rect > 0.35 and V_area_rect < 0.45:
                shape = "Star"
                self.t.child_frame_id = "Star"
                list_id = 5

            else: 
                list_id = None

            # ensure that the approximated contour is "roughly" rectangular
            if list_id is not None:
                # compute the bounding box of the approximated contour and
                # use the bounding box to compute the aspect ratio
                (x, y, w, h) = cv2.boundingRect(approx)
                aspectRatio = w / float(h)

                # compute the solidity of the original contour
                area = cv2.contourArea(c)
                hullArea = cv2.contourArea(cv2.convexHull(c))
                solidity = area / float(hullArea)

                # Plausibilisierung mit Hoehe
                marker = cv2.minAreaRect(approx)
                perWidth = marker[1][0]

                val_heigth = perWidth > expected_Width_low and perWidth < expected_Width_high

                # compute whether or not the width and height, solidity, and
                # aspect ratio of the contour falls within appropriate bounds
                keepDims = w > 25 and h > 25
                keepSolidity = solidity > 0.5
                keepAspectRatio = aspectRatio >= 0.8 and aspectRatio <= 1.2

                # ensure that the contour passes all our tests
                if keepDims and keepSolidity and keepAspectRatio and val_heigth:
                    # draw an outline around the target and update the status
                    # text
                    cv2.drawContours(frame, [approx], -1, (0, 0, 255), 4)

                    # compute the center of the contour region and draw the
                    # crosshairs
                    M = cv2.moments(approx)
                    (cX, cY) = (int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"]))
                    (startX, endX) = (int(cX - (w * 0.15)), int(cX + (w * 0.15)))
                    (startY, endY) = (int(cY - (h * 0.15)), int(cY + (h * 0.15)))
                    cv2.line(frame, (startX, cY), (endX, cY), (0, 0, 255), 3)
                    cv2.line(frame, (cX, startY), (cX, endY), (0, 0, 255), 3)
                    center = (cX, cY)

                    if self.Pnt_nxt[list_id][0] is not None:
                        cX_est = self.Pnt_nxt[list_id][0]
                        cY_est = self.Pnt_nxt[list_id][1]
                        (startX, endX) = (int(cX_est - (w * 0.03)), int(cX_est + (w * 0.03)))
                        (startY, endY) = (int(cY_est - (h * 0.03)), int(cY_est + (h * 0.03)))
                        cv2.line(frame, (startX, cY_est), (endX, cY_est), (0, 255, 0), 1)
                        cv2.line(frame, (cX_est, startY), (cX_est, endY), (0, 255, 0), 1)
                    # verlgeichen wie gross der Abstand zw. cX_est und cX (bzw cY) erst wenn dieser
                    # einen Grenzwert +/- 20 UNTERschreitet Contour zeichnen falls nicht cX ist cX_est
                    # print(cX-cX_est,cY-cY_est)


                    self.pts[list_id].insert(0, center)
                    if len(self.pts[list_id]) > self.maxLength:
                        self.pts[list_id] = self.pts[list_id][:self.maxLength]

                    # draw the detected shape type on the frame
                    cv2.putText(frame, shape, (cX, cY), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)

                    # Distance to Camera


                    dist_z = (self.knownWidth * self.focalLength_x) / perWidth

                    # self.dist_x / self.dist_y
                    self.dist_x_p = 0.5 * frame.shape[1] - cX  # Distance to Center (x-direction) in pixel
                    self.dist_y_p = cY - 0.5 * frame.shape[0]

                    self.dist_x = (dist_z * self.dist_x_p) / self.focalLength_x
                    self.dist_y = (dist_z * self.dist_y_p) / self.focalLength_y

                    cv2.putText(frame, "d_z %.0f mm" % (dist_z), (cX + 5, cY - 20), cv2.FONT_HERSHEY_SIMPLEX, 0.3,
                                (0, 255, 0), 1)
                    cv2.putText(frame, "d_y %.0f mm" % (self.dist_x), (cX + 5, cY - 30), cv2.FONT_HERSHEY_SIMPLEX, 0.3,
                                (0, 255, 0), 1)
                    cv2.putText(frame, "d_x %.0f mm" % (self.dist_y), (cX + 5, cY - 40), cv2.FONT_HERSHEY_SIMPLEX, 0.3,
                                (0, 255, 0), 1)

        # loop over Objects (list_id)
        for j in range(0, 5):
            # loop over the set of tracked points
            for i in np.arange(1, len(self.pts[j])):
                # if either of the tracked points are None, ignore
                # them
                if self.pts[j][i - 1] is None or self.pts[j][i] is None:
                    continue

                # check to see if enough points have been accumulated in
                # the buffer
                if self.counter >= self.past_pts and i == self.past_pts and self.pts[j][i - self.past_pts] is not None:
                    # compute the difference between the x and y
                    # coordinates and re-initialize the direction
                    # text variables
                    self.dX = self.pts[j][self.past_pts][0] - self.pts[j][0][0]
                    self.dY = self.pts[j][self.past_pts][1] - self.pts[j][0][1]
                    X_now = self.pts[j][0][0]
                    Y_now = self.pts[j][0][1]
                    X_next = X_now - int((1 / float(self.past_pts)) * self.dX)
                    Y_next = Y_now - int((1 / float(self.past_pts)) * self.dY)

                    self.Pnt_nxt[j] = (X_next, Y_next)

        # draw direction of motion
        # cv2.line(frame, (X_now,Y_now), (X_next,Y_next), (0, 255, 0), 2)

        # otherwise, compute the thickness of the line and
        # draw the connecting lines
        # cv2.line(frame, pts[j][i - 1], pts[j][i], (0, 0, 255), 1)



        # draw the status text on the frame
        cv2.putText(frame, 'status', (20, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.5,
                    (0, 0, 255), 2)

        self.t.transform.translation.x = self.dist_x
        self.t.transform.translation.y = self.dist_y
        self.t.transform.translation.z = self.dist_camera
        self.br.sendTransform(self.t)
        self.counter += 1

        try:
            self.image_pub.publish(self.bridge.cv2_to_imgmsg(frame, "mono8"))
        except CvBridgeError as e:
            print(e)

        if (not self.runningOnPhoenix):
            # show the frame and record if a key is pressed
            cv2.imshow("Frame", frame)
            cv2.waitKey(1000)

            # optional:
            # cv2.drawContours(image, [approx], -1, (0, 255, 0), 4)

            # cleanup the camera and close any open windows
            cv2.destroyAllWindows()


def main(args):
    ic = image_converter()
    rospy.init_node('image_converter', anonymous=True)
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print ("Shutting down")
    cv2.destroyAllWindows()


if __name__ == '__main__':
    main(sys.argv)
