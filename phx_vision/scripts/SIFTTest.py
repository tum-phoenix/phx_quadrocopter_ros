import numpy as np
import cv2
from matplotlib import pyplot as plt

#Load previously saved data
with np.load('CameraValues.npz') as X:
    mtx, dist, _, _ = [X[i] for i in ('mtx','dist','rvecs','tvecs')]
    
# minimum amount of matches for detection
MIN_MATCH_COUNT = 10
# real-world image where detection takes place
world_image = 'DSC_0018.JPG'

# load the recognition image and the real-world image in grayscale and color
img1 = cv2.imread('Recognition_Items.jpeg',0)      
img2 = cv2.imread(world_image,0) 
img2_color = cv2.imread(world_image,1)

# Initiate SIFT detector
sift = cv2.xfeatures2d.SIFT_create()

# find the keypoints and descriptors with SIFT
kp1, des1 = sift.detectAndCompute(img1,None)
kp2, des2 = sift.detectAndCompute(img2,None)

# Use FLANN matcher
FLANN_INDEX_KDTREE = 0
index_params = dict(algorithm = FLANN_INDEX_KDTREE, trees = 5)
search_params = dict(checks = 50)
flann = cv2.FlannBasedMatcher(index_params, search_params)
matches = flann.knnMatch(des1,des2,k=2)

# store all the good matches as per Lowe's ratio test.
good = []
for m,n in matches:
    if m.distance < 0.7*n.distance:
        good.append(m)
        
if len(good)>MIN_MATCH_COUNT:
    # get matches from source and query image
    src_pts = np.float32([ kp1[m.queryIdx].pt for m in good ]).reshape(-1,1,2)
    dst_pts = np.float32([ kp2[m.trainIdx].pt for m in good ]).reshape(-1,1,2)

    # find Homography between the two images
    M, mask = cv2.findHomography(src_pts, dst_pts, cv2.RANSAC,5.0)
    
    #append a zero column and reshape source points and find rotation and translation vectors    
    src_pts_3d = np.c_[src_pts, np.zeros((len(good),1,1))]
    src_pts_3d = src_pts_3d.reshape(-1,3)
    _, rvecs, tvecs, inliers = cv2.solvePnPRansac(src_pts_3d, dst_pts, mtx, dist)
    matchesMask = mask.ravel().tolist()
    
    # draw rectangle around found region
    h,w = img1.shape
    pts = np.float32([ [0,0],[0,h-1],[w-1,h-1],[w-1,0] ]).reshape(-1,1,2)
    dst = cv2.perspectiveTransform(pts,M) 
    img2 = cv2.polylines(img2_color,[np.int32(dst)],True,0,3, cv2.LINE_AA)
    
    #draw coordinate axis in first image and perspective transform it -> not used since projection from 3D space necessary anyway
    pts_axis = np.float32([ [0,0],[0,500],[500,0] ]).reshape(-1,1,2)
    axis_transformed = cv2.perspectiveTransform(pts_axis,M)
    
    #draw the additional lines in the white rectangle -> not used since projection from 3D space necessary anyway
    axis_transformed = axis_transformed.reshape(3,2)
    axis_transformed = tuple(map(tuple,axis_transformed))
    
    cv2.line(img2,axis_transformed[0],axis_transformed[1],(255,0,0),5)
    cv2.line(img2,axis_transformed[0],axis_transformed[2],(0,255,0),5)
    
    #z axis needs to projected from 3d points, since it cannot be projected from the 2d image
    axis = np.float32([[3,0,0], [0,3,0], [0,0,-3], [0,0,0]]).reshape(-1,3)
    axis = axis * 200
    imgpts, jac = cv2.projectPoints(axis, rvecs, tvecs, mtx, dist)

    # get the coordinate origin and axis lines     
    corner = tuple(imgpts[3].ravel())
    
    img = cv2.line(img2, corner, tuple(imgpts[0].ravel()), (255,0,0), 5)
    img = cv2.line(img2, corner, tuple(imgpts[1].ravel()), (0,255,0), 5)
    img = cv2.line(img2, corner, tuple(imgpts[2].ravel()), (0,0,255), 5)
else:
    print("Not enough matches are found - %d/%d" % (len(good),MIN_MATCH_COUNT))
    matchesMask = None
    
draw_params = dict(matchColor = (0,255,0), # draw matches in green color
                   singlePointColor = None,
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

#image_plt = plt.imshow(img3),
#plt.text(100,100,'Rotation vector',ha='center', va='center',fontsize=10)
#plt.show()