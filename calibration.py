import numpy as np
import cv2
import glob

# termination criteria
criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)

# prepare object points, like (0,0,0), (1,0,0), (2,0,0) ....,(6,5,0)
objp = np.zeros((6*7,3), np.float32)
objp[:,:2] = np.mgrid[0:7,0:6].T.reshape(-1,2)

# Arrays to store object points and image points from all the images.
objpoints = [] # 3d point in real world space
imgpoints = [] # 2d points in image plane.
undistorts = [] # undistort map

images = glob.glob('./calib_img/*.jpg')

for fname in images:
    img = cv2.imread(fname)
    gray = cv2.cvtColor(img,cv2.COLOR_BGR2GRAY)
    h, w = gray.shape
    # gray = cv2.resize(gray, (0,0), fx=0.5, fy=0.5 )

    # Find the chess board corners
    ret, corners = cv2.findChessboardCorners(gray, (7,6),None)

    # If found, add object points, image points (after refining them)
    if ret == True:
        objpoints.append(objp)

        cv2.cornerSubPix(gray,corners,(11,11),(-1,-1),criteria)
        imgpoints.append(corners)

        # undist = cv2.undistort(img, mtx, dist, None, newcameramtx)

        # x,y,w,h = roi
        # undist = undist[y:y+h, x:x+w]
        # cv2.imwrite('calibresult.png',undist)

        # Draw and display the corners
        # cv2.drawChessboardCorners(img, (7,6), corners,ret)
        # cv2.imshow('img',img)
        # cv2.imshow('undist', undist)
        # cv2.waitKey()

ret_cal, mtx, dist, rvecs, tvecs = cv2.calibrateCamera(objpoints, imgpoints, gray.shape[::-1],None,None)

h,  w = img.shape[:2]
newcameramtx, roi = cv2.getOptimalNewCameraMatrix(mtx,dist,(w,h),1,(w,h))
print(mtx)
print(dist)
print(newcameramtx)

mean_error = 0
for i in range(len(objpoints)):
    imgpoints2, _ = cv2.projectPoints(objpoints[i], rvecs[i], tvecs[i], mtx, dist)
    error = cv2.norm(imgpoints[i], imgpoints2, cv2.NORM_L2)/len(imgpoints2)
    mean_error += error
print( "total error: {}".format(mean_error/len(objpoints)) )

# Sample Undistort Image
sample_img = cv2.imread('./calib_img/debayer-left26.jpg')
undist = cv2.undistort(sample_img, mtx, dist, None, newcameramtx)

x,y,w,h = roi
undist_crop = undist[y:y+h, x:x+w]
# cv2.imwrite('calibresult.png',undist)

# Draw and display the corners
cv2.drawChessboardCorners(img, (7,6), corners,ret)
cv2.imshow('img',sample_img)
cv2.imshow('undist', undist)
cv2.imshow('undist_crop', undist_crop)
cv2.waitKey()

cv2.destroyAllWindows()