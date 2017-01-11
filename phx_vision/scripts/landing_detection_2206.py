# -*- coding: utf-8 -*-q
'''
Created on June 22, 2016

@author: hidenobu_matsuki 松木秀伸
'''

# import the necessary packages
import argparse
import cv2
import numpy as np
import math

# load the video
camera = cv2.VideoCapture(0)

# keep looping
while True:
	# grab the current frame and initialize the status text
	(grabbed, frame) = camera.read()
	status = "No Targets"

	# check to see if we have reached the end of the
	# video
	if not grabbed:
		break

	# convert the frame to grayscale, blur it, and detect edge

	gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
	blurred = cv2.GaussianBlur(gray, (7, 7), 0)
	ret,th3 = cv2.threshold(blurred,80,255,cv2.THRESH_BINARY_INV)
	kernel = np.ones((10, 10), np.uint8)
	dilation = cv2.dilate(th3, kernel, iterations=1)

	# find contours in the edge map
	cnts = cv2.findContours(dilation.copy(), cv2.RETR_EXTERNAL,
		cv2.CHAIN_APPROX_SIMPLE)[-2]


	# loop over the contours
	for c in cnts:
		# approximate the contour
		peri = cv2.arcLength(c, True)
		approx = cv2.approxPolyDP(c, 0.01 * peri, True)

		# ensure that the approximated contour is "roughly" rectangular
		if len(approx) >=10 and len(approx) <= 20: #18はあった方が良い
			# compute the bounding box of the approximated contour and
			# use the bounding box to compute the aspect ratio
			(x, y, w, h) = cv2.boundingRect(approx)

			# compute the solidity of the original contour
			area = cv2.contourArea(c)

            #compute the minimum Rectangle, which is constant to rotation.
			rect = cv2.minAreaRect(c)
			box = cv2.boxPoints(rect)

            #compute aspect ratio
			l1 = math.sqrt((box[1,0]-box[0,0])**2+(box[1,1]-box[0,1])**2)
			l2 = math.sqrt((box[2,0]-box[1,0])**2+(box[2,1]-box[1,1])**2)
			box = np.int0(box)
			width = max(l1,l2)
			height = min(l1,l2)
			aspectRatio = width / height

            #compute solidity
			hullArea = cv2.contourArea(box)
			solidity = area / float(hullArea)

			# compute whether or not the width and height, solidity, and
			# aspect ratio of the contour falls within appropriate bounds
			keepSolidity = solidity > 0.5 and solidity < 0.9
			keepAspectRatio = (aspectRatio >= 1.7 and aspectRatio <= 2.0) or (aspectRatio >= 1 and aspectRatio <= 1.1)
			keeparea = area >1300 and area < 120000


			# ensure that the contour passes all our tests
			if keeparea and keepSolidity and keepAspectRatio:
				# draw an outline around the target and update the status
				# text
				cv2.drawContours(frame, [approx], -1, (0, 0, 255), 4)
				status = "Target(s) Acquired"

				# compute the center of the contour region and draw the
				# crosshairs
				M = cv2.moments(approx)
				(cX, cY) = (int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"]))
				(startX, endX) = (int(cX - (w * 0.15)), int(cX + (w * 0.15)))
				(startY, endY) = (int(cY - (h * 0.15)), int(cY + (h * 0.15)))
				cv2.line(frame, (startX, cY), (endX, cY), (0, 0, 255), 3)
				cv2.line(frame, (cX, startY), (cX, endY), (0, 0, 255), 3)


	# draw the status text on the frame
	cv2.putText(frame, status, (20, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.5,
		(0, 0, 255), 2)

	# show the frame and record if a key is pressed
	cv2.imshow("Frame", frame)
	key = cv2.waitKey(1) & 0xFF



	# if the 'q' key is pressed, stop the loop
	if key == ord("q"):
		break

# cleanup the camera and close any open windows
camera.release()
cv2.destroyAllWindows()
