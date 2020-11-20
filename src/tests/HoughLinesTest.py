##### Hough Line Transform Test #####

### Import required libraries ###
# This is the vision library OpenCV
import cv2
# This is a library for mathematical functions for python (used later)
import numpy as np
# This is a library to get access to time-related functionalities
import time
# Import arUco code library
import cv2.aruco as aruco

import geom

### Setup camera ###
# Read and store the calibration information from Sample_Calibration
CALIBRATION = np.load('tests/Calibration.npz') # Load the camera calibration values 
CAMERA_MATRIX = CALIBRATION['CM'] # Camera matrix
DIST_COEFFS = CALIBRATION['dist_coef'] # Distortion coefficients from the camera

# Select the first camera (0) that is connected to the machine
# in Laptops should be the build-in camera
video_capture = cv2.VideoCapture(0)

# Set the width and heigth of the camera to 640x480
video_capture.set(3,640)
video_capture.set(4,480)

### Setup viewing windows ###
# Create two opencv named windows
cv2.namedWindow("frame-image", cv2.WINDOW_AUTOSIZE)

# Position the windows next to each other
cv2.moveWindow("frame-image",100,100)

### Run Camera ###
# Execute this continuously
while(True):
    
    # Start the performance clock
    #start = time.perf_counter()
    
    # Capture current frame from the camera
    ret, frame = video_capture.read()

    # Convert the image from the camera to Gray scale
    out = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

    # Edge detection
    out = cv2.Canny(out, 50, 150, apertureSize = 3)

    # Hough line transform
    lines = cv2.HoughLinesP(out, 1, np.pi/180, 100, minLineLength = 100, maxLineGap = 10)

    if lines is not None: # If we found some lines
        # Loop through the lines and draw them
        for line in lines:
            x1, y1, x2, y2 = line[0]
            frame = cv2.line(frame, (x1, y1), (x2, y2), (0, 255, 0), 2)

    # Display the original frame in a window
    cv2.imshow('frame-image',frame)

    # Stop the performance counter
    #end = time.perf_counter()

    # If the button q is pressed in one of the windows
    if cv2.waitKey(20) & 0xFF == ord('q'):
        # Exit the While loop
        break

# When everything done, release the capture
cap.release()
# close all windows
cv2.destroyAllWindows()
# exit the kernel
exit(0)