import numpy as np
import numpy.ma as ma
import cv2
from matplotlib import pyplot as plt

#Load previously saved data
with np.load('CameraValues.npz') as X:
    mtx, dist, _, _ = [X[i] for i in ('mtx','dist','rvecs','tvecs')]

MIN_MATCH_COUNT = 10
world_image = '2016-06-30-173050.jpg'
img1 = cv2.imread('Recognition_Items.jpeg',0)          # queryImage
img2 = cv2.imread(world_image,0) # trainImage
img2_color = cv2.imread(world_image,1)

# Initiate ORB detector
orb = cv2.ORB_create(nfeatures = 1000)

# find the keypoints and descriptors with ORB
kp1, des1 = orb.detectAndCompute(img1,None)
kp2, des2 = orb.detectAndCompute(img2,None)

#FLANN_INDEX_KDTREE = 0
#index_params = dict(algorithm = FLANN_INDEX_LSH, table_number = 6, key_size = 12, multi_probe_level = 1)
#search_params = dict(checks = 50)

#flann = cv2.FlannBasedMatcher(index_params, search_params)
#matches = flann.knnMatch(des1,des2,k=2)

brute = cv2.BFMatcher(cv2.NORM_HAMMING, crossCheck = True)
matches = brute.match(des1,des2)

ratio = 0.8

# keep only the reasonable matches
distance = [m.distance for m in matches]
thres_dist = (sum(distance) / len(distance)) * ratio
good = [m for m in matches if m.distance < thres_dist]


if len(good)>MIN_MATCH_COUNT:

    # geet matches
    src_pts = np.float32([ kp1[m.queryIdx].pt for m in good ]).reshape(-1,1,2)
    dst_pts = np.float32([ kp2[m.trainIdx].pt for m in good ]).reshape(-1,1,2)

    # compute homography and the homography mask
    M, mask_homography = cv2.findHomography(src_pts, dst_pts, cv2.RANSAC,2.0)
    src_pts = src_pts.reshape(-1,2)
    dst_pts = dst_pts.reshape(-1,2)
    mask = mask_homography
    mask_homography_reshaped = np.c_[mask_homography,mask_homography]
    # apply homography mask on the matches
    src_pts_masked = ma.array(src_pts,mask=np.logical_not(mask_homography_reshaped))
    dst_pts_masked = ma.array(dst_pts,mask=np.logical_not(mask_homography_reshaped))

    # using all matches from the homography
    src_pts_good = src_pts_masked.compressed().reshape(-1,2)
    src_pts_good = np.c_[src_pts_good, np.zeros((len(src_pts_good),1))]
    dst_pts_good = dst_pts_masked.compressed().reshape(-1,2)
    _,rvecs, tvecs = cv2.solvePnP(src_pts_good, dst_pts_good, mtx, dist)
    matchesMask = mask.ravel().tolist()

    # draw homography boundaries
    h,w = img1.shape
    pts = np.float32([ [0,0],[0,h-1],[w-1,h-1],[w-1,0] ]).reshape(-1,1,2)
    dst = cv2.perspectiveTransform(pts,M)
    img2 = cv2.polylines(img2_color,[np.int32(dst)],True,0,3, cv2.LINE_AA)

    # define 3d coordinate system
    axis = np.float32([[3,0,0], [0,3,0], [0,0,-3], [0,0,0]]).reshape(-1,3)
    axis = axis * 50
    imgpts, _ = cv2.projectPoints(axis.reshape(-1,3), rvecs, tvecs, mtx, dist)

    # coordinate system base
    corner = tuple(imgpts[3].ravel())

    # draw coordinate axis
    img = cv2.line(img2, corner, tuple(imgpts[0].ravel()), (255,0,0), 5)
    img = cv2.line(img2, corner, tuple(imgpts[1].ravel()), (0,255,0), 5)
    img = cv2.line(img2, corner, tuple(imgpts[2].ravel()), (0,0,255), 5)

else:
    print("Not enough matches are found - %d/%d" % (len(matches),MIN_MATCH_COUNT))
    matchesMask = None

draw_params = dict(matchColor = (0,255,0), # draw matches in green color
                   singlePointColor = (255,0,0),
                   matchesMask = matchesMask, # draw only inliers
                   flags = 2)

img3 = cv2.drawMatches(img1,kp1,img2,kp2,good,None,**draw_params)

#cv2.putText(img3, 'Translation Vector:', (20, 30), cv2.FONT_HERSHEY_SIMPLEX, 4,
#        (255, 255, 255), 2)

#calculate world-space coordinates
#rotationMatrix = cv2.Rodrigues(rvecs)
#cameraPosition = -np.matrix(rotationMatrix).T * np.matrix(tvecs)

cv2.imshow('cv2shown',img3)
cv2.waitKey(0)
image_plt = plt.imshow(img3),

plt.show()
