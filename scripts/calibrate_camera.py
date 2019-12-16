#!/usr/bin/env python
"""
DESCRIPTION: This script is for finding the calibration matrices of a camera.
             It simply prints the these to the screen as well as the size of the image.
             In order to properly calibrate the camera, you need to load at least
             ten images of the board from different angles.  Once you run this script
             once, you can simply save the matrices for use in other code.
"""
import numpy as np
import cv2
import glob


# termination criteria
criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)

# prepare object points, like (0,0,0), (1,0,0), (2,0,0) ....,(6,5,0)
objp = np.zeros((7*7,3), np.float32)
##print(objp)
objp[:,:2] = np.mgrid[0:7,0:7].T.reshape(-1,2)
##print(objp)

# Arrays to store object points and image points from all the images.
objpoints = [] # 3d point in real world space
imgpoints = [] # 2d points in image plane.

images = glob.glob('bax*.jpg')
print(images)


for fname in images:
    img = cv2.imread(fname)
    gray = cv2.cvtColor(img,cv2.COLOR_BGR2GRAY)

    h,  w = img.shape[:2]


    # Find the chess board corners
    ret, corners = cv2.findChessboardCorners(gray, (7,7),None)

    # If found, add object points, image points (after refining them)
    if ret == True:
        objpoints.append(objp)

        corners2 = cv2.cornerSubPix(gray,corners,(11,11),(-1,-1),criteria)
        imgpoints.append(corners2)


ret, mtx, dist, rvecs, tvecs = cv2.calibrateCamera(objpoints, imgpoints, gray.shape[::-1],None,None)
print ('mtx')
print (mtx)
print ("dist")
print (dist)
print('h')
print(h)
print('w')
print(w)
