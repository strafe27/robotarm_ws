#!/usr/bin/env python
import cv2
import numpy as np
import rospy
from std_msgs.msg import Float64MultiArray

# Define the cosine and sine functions in degrees
def cosd(degrees):
    return np.cos(np.radians(degrees))

def sind(degrees):
    return np.sin(np.radians(degrees))

def find_midpoint(contour):
    x, y, w, h = cv2.boundingRect(contour)
    midpoint_x = x + w // 2
    midpoint_y = y + h // 2
    mm_per_pixel = 126 / 615
    midpoint_x_mm = midpoint_x * mm_per_pixel
    midpoint_y_mm = midpoint_y * mm_per_pixel
    return midpoint_x, midpoint_y, midpoint_x_mm, midpoint_y_mm

# Define matrices A and B
R180X = np.array([[1, 0, 0], [0, np.cos(np.pi), -np.sin(np.pi)], [0, np.sin(np.pi), np.cos(np.pi)]])
matrixB = np.array([[np.cos(0), -np.sin(0), 0], [np.sin(0), np.cos(0), 0], [0, 0, 1]])

# Calculate the product of matrixA and matrixB
R0_C = np.dot(R180X, matrixB)
D0_C = np.array([[-60.5], [220.35], [0]])
H0_C = np.concatenate((R0_C, D0_C), axis=1)
H0_C = np.concatenate((H0_C, [[0, 0, 0, 1]]), axis=0)

# Load the calibration data from the npz file
calibration_data = np.load('calibration_data.npz')

# Retrieve the calibration parameters
cameraMatrix = calibration_data['cameraMatrix']
dist = calibration_data['dist']
newCameraMatrix = calibration_data['newCameraMatrix']
roi = calibration_data['roi']

# Open a connection to the camera (0 is usually the built-in webcam)
cap = cv2.VideoCapture(0)

def main():
    rospy.init_node('contour_coordinates_publisher', anonymous=True)
    rate = rospy.Rate(10)  # 10 Hz, adjust as needed

    # Create ROS publisher for coordinates as an array
    coordinates_pub = rospy.Publisher('real_world_coordinates', Float64MultiArray, queue_size=10)

    # Create a flag for debugging
    debug_mode = True

    while not rospy.is_shutdown():
        # Capture frame-by-frame
        ret, frame = cap.read()
        if not ret:
            rospy.logerr("Failed to capture frame")
            break

        # Undistort the frame using the loaded calibration parameters
        frame = cv2.undistort(frame, cameraMatrix, dist, None, newCameraMatrix)

        # Crop the frame based on the region of interest (ROI)
        x, y, w, h = roi
        frame = frame[y:y + h, x:x + w]
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        # Apply Gaussian blur
        blurred = cv2.GaussianBlur(gray, (5, 5), 0)

        # Apply Canny edge detection
        edges = cv2.Canny(blurred, 50, 150)

        # Find contours in the edges
        contours, _ = cv2.findContours(edges, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        coordinates_array = []
        for contour in contours:
            if 1000 < cv2.contourArea(contour) < 35000:
                # Get the midpoint coordinates and real-world coordinates
                midpoint_x, midpoint_y, midpoint_x_mm, midpoint_y_mm = find_midpoint(contour)

                # Perform the homogeneous transformation
                matrix = np.array([[midpoint_x_mm], [midpoint_y_mm], [0], [1]])
                P0 = np.dot(H0_C, matrix)

                # Append coordinates to the array
                coordinates_array.append(P0[0][0])
                coordinates_array.append(P0[1][0])

                # Draw the bounding box and midpoint on the original image
                cv2.drawContours(frame, [contour], -1, (0, 255, 0), 2)  # Green bounding box
                cv2.circle(frame, (midpoint_x, midpoint_y), 5, (0, 0, 255), -1)  # Red midpoint
                # Draw imaginary vertical line
                cv2.line(frame, (midpoint_x, 0), (midpoint_x, frame.shape[0]), (255, 0, 0), 2)
                # Draw imaginary horizontal line
                cv2.line(frame, (0, midpoint_y), (frame.shape[1], midpoint_y), (255, 0, 0), 2)

                # Publish the coordinates as an array
                coordinates_msg = Float64MultiArray(data=coordinates_array)
                coordinates_pub.publish(coordinates_msg)

                # Log information to ROS
                rospy.loginfo("Real world coordinates: %s", coordinates_array)

        if debug_mode:
            # Display the result
            cv2.imshow('Original Frame', frame)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

        rate.sleep()

    cap.release()
    cv2.destroyAllWindows()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
