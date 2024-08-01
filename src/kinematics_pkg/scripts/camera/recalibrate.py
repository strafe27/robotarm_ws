import numpy as np
import cv2
import glob

# Termination criteria
criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)

# Prepare object points (0,0,0), (1,0,0), (2,0,0), ...
objp = np.zeros((6*7,3), np.float32)
objp[:,:2] = np.mgrid[0:7,0:6].T.reshape(-1,2)

# Arrays to store object points and image points
objpoints = [] # 3d points in real world space
imgpoints = [] # 2d points in image plane

images = glob.glob('calibration_images/*.jpg')

# Initialize a variable for gray
gray = None

for fname in images:
    img = cv2.imread(fname)
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    
    # Find the chessboard corners
    ret, corners = cv2.findChessboardCorners(gray, (7,6), None)
    
    if ret:
        objpoints.append(objp)
        imgpoints.append(corners)
        
        # Draw and display the corners
        img = cv2.drawChessboardCorners(img, (7,6), corners, ret)
        cv2.imshow('img', img)
        cv2.waitKey(500)

cv2.destroyAllWindows()

# Ensure gray is defined before using it for calibration
if gray is not None:
    # Calibrate the camera
    ret, cameraMatrix, dist, rvecs, tvecs = cv2.calibrateCamera(objpoints, imgpoints, gray.shape[::-1], None, None)

    # Get the optimal new camera matrix
    h, w = gray.shape[:2]
    newCameraMatrix, roi = cv2.getOptimalNewCameraMatrix(cameraMatrix, dist, (w,h), 1, (w,h))

    # Save the calibration results
    np.savez('calibration_data.npz', cameraMatrix=cameraMatrix, dist=dist, newCameraMatrix=newCameraMatrix, roi=roi)
else:
    print("Error: No images found for calibration.")
