'''
Created on May 10, 2016

@author: mykyta_denysov
'''

#!/usr/bin/env python
import roslib
roslib.load_manifest('phx_vision')
import sys
import rospy
import cv2
import tf
import tf2_ros
import numpy as np
import geometry_msgs.msg
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

class image_converter:

  def __init__(self):
    #cv2.namedWindow("Image window", 1)
    self.bridge = CvBridge()
    self.image_sub = rospy.Subscriber("image_mono", Image, self.callback)
    self.br = tf2_ros.TransformBroadcaster()
    self.t = geometry_msgs.msg.TransformStamped()

  def callback(self, data):
      

    self.t.header.stamp = rospy.Time.now()
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
    #gray = cv2.cvtColor(resizedGreyImage, cv2.COLOR_BGR2GRAY)
    #cv2.imshow("Frame", frame)

    blurred = cv2.GaussianBlur(frame, (7, 7), 0)
    edged = cv2.Canny(blurred, 50, 150)
    
    knownWidth = 100.0 #mm
    focalLength = 900  #Test
    
    # find contours in the edge map
    cnts = cv2.findContours(edged.copy(), cv2.RETR_EXTERNAL,
    cv2.CHAIN_APPROX_SIMPLE)[-2]
    # loop over the contours
    for c in cnts:
            
        # approximate the contour
        peri = cv2.arcLength(c, True)
        approx = cv2.approxPolyDP(c, 0.01 * peri, True)
        
          
        if len(approx)==3:
            shape = "Triangle"
            self.t.child_frame_id = "Triangle"

    
        elif len(approx)==4:
            # compute the bounding box of the contour and use the
            # bounding box to compute the aspect ratio
            (x, y, w, h) = cv2.boundingRect(approx)
            ar = w / float(h)
    
            # a square will have an aspect ratio that is approximately
            # equal to one, otherwise, the shape is a rectangle
            shape = "Square" if ar >= 0.95 and ar <= 1.05 else "rectangle"
            self.t.child_frame_id = "Square"

                       
        elif len(approx)==5:
            shape = "Pentagon"
            self.t.child_frame_id = "Pentagon"

            
        elif len(approx)>5 and len(approx)<14:
            shape = "Heart"
            self.t.child_frame_id = "Heart"

        
        elif len(approx)>=14 and len(approx)<30:
            shape = "Circle"
            self.t.child_frame_id = "Circle"

            
        elif len(approx)>=30:
            shape = "Star"
            self.t.child_frame_id = "Star"

    
        # ensure that the approximated contour is "roughly" rectangular
        if len(approx) >= 3 and len(approx) <= 30:
            # compute the bounding box of the approximated contour and
            # use the bounding box to compute the aspect ratio
            (x, y, w, h) = cv2.boundingRect(approx)
            aspectRatio = w / float(h)
    
            # compute the solidity of the original contour
            area = cv2.contourArea(c)
            hullArea = cv2.contourArea(cv2.convexHull(c))
            solidity = area / float(hullArea)
    
            # compute whether or not the width and height, solidity, and
            # aspect ratio of the contour falls within appropriate bounds
            keepDims = w > 25 and h > 25
            keepSolidity = solidity > 0.5
            keepAspectRatio = aspectRatio >= 0.8 and aspectRatio <= 1.2
    
            # ensure that the contour passes all our tests
            if keepDims and keepSolidity and keepAspectRatio:
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
                
                #draw the detected shape type on the frame            
                cv2.putText(frame, shape, (cX, cY), cv2.FONT_HERSHEY_SIMPLEX, 
                            0.5, (255, 255, 255), 2)
                            
                #Distance to Camera
                marker = cv2.minAreaRect(Target_Contour)
                perWidth = marker[1][0]
                dist_camera = (knownWidth * focalLength)/perWidth
                
                #dist_x / dist_y
                dist_x_p = 0.5*frame.shape[1] - cX # Distance to Center (x-direction) in pixel
                dist_y_p = cY - 0.5*frame.shape[0]
                
                dist_x= (dist_camera * dist_x_p)/focalLength
                dist_y= (dist_camera * dist_y_p)/focalLength
                
                            
                            
                self.t.transform.translation.x = dist_x
                self.t.transform.translation.y = dist_y
                self.t.transform.translation.z = dist_camera
                self.br.sendTransform(self.t)

                
    # show the frame and record if a key is pressed
    #cv2.imshow("Frame", frame)
    #cv2.waitKey(1000)
        
    #optional:
    #cv2.drawContours(image, [approx], -1, (0, 255, 0), 4)
    
    # cleanup the camera and close any open windows
    #cv2.destroyAllWindows()
 

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
