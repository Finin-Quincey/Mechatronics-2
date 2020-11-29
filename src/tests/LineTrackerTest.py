##### Hough-transform-based Line Tracker Test #####

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
import vision
from vision import line_tracker

### Setup camera ###
# Read and store the calibration information from Sample_Calibration
CALIBRATION = np.load('tests/Calibration.npz') # Load the camera calibration values 
CAMERA_MATRIX = CALIBRATION['CM'] # Camera matrix
DIST_COEFFS = CALIBRATION['dist_coef'] # Distortion coefficients from the camera

# Select the first camera (0) that is connected to the machine
# in Laptops should be the build-in camera
video_capture = cv2.VideoCapture(1)

# Set the width and heigth of the camera to 640x480
video_capture.set(3, 1280)
video_capture.set(4, 720)

### Setup viewing windows ###
# Create two opencv named windows
cv2.namedWindow("frame-image", cv2.WINDOW_AUTOSIZE)
# cv2.namedWindow("frame-edges", cv2.WINDOW_AUTOSIZE)

# Position the windows next to each other
cv2.moveWindow("frame-image", 100, 100)
# cv2.moveWindow("frame-edges", 800, 100)

n = 0
t = 40 # Threshold for tuning

### Run Camera ###
# Execute this continuously
while(True):
    
    # Start the performance clock
    #start = time.perf_counter()
    
    # Capture current frame from the camera
    ret, frame = video_capture.read()

    # Unlike the aruco marker detection, HLT doesn't do this automatically so we need to do it first
    frame = cv2.undistort(frame, CAMERA_MATRIX, DIST_COEFFS)

    frame = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

    frame[frame > t] = 255 # Tune threshold

    display_frame = cv2.cvtColor(frame, cv2.COLOR_GRAY2BGR)
    
    #frame = cv2.adaptiveThreshold(frame, 255, cv2.ADAPTIVE_THRESH_GAUSSIAN_C, cv2.THRESH_BINARY, 11, t)

    # Edge detection
    edges = cv2.Canny(frame, 50, 200, apertureSize = 3)

    if n > 50: # Camera does weird stuff for the first few frames

        # Hough line transform
        # This returns lines in a 3D matrix of the form [[[x1, y1, x2, y2]], [...], ...]
        lines = cv2.HoughLinesP(edges, 1, np.pi/180, 80, minLineLength = 25, maxLineGap = 10)

        if lines is not None: # If we found some lines
            lines = lines[:, 0]
            # Loop through the lines and draw them
            i = 0
            for line in lines:
                frame = cv2.line(frame, (int(line[0]), int(line[1])), (int(line[2]), int(line[3])), (255, 255, 0), 2)
                i += 1
                if i > 50: break

        # Update line tracker
        lines = line_tracker.update(frame)

        if lines is not None: # If we found some lines
            # Loop through the lines and draw them
            for line in lines:
                display_frame = cv2.line(display_frame, (int(line[0]), int(line[1])), (int(line[2]), int(line[3])), (255, 0, 0), 2)

        centrelines = line_tracker.locate_centrelines(lines)

        if centrelines is not None: # If we found some lines
            # Loop through the lines and draw them
            for line in centrelines:
                display_frame = cv2.line(display_frame, (int(line[0]), int(line[1])), (int(line[2]), int(line[3])), (0, 0, 255), 2)
    
    cv2.putText(display_frame, f"Threshold: {t} (W to increase, S to decrease)", (20, 700), cv2.FONT_HERSHEY_PLAIN, 2, (255, 0, 0), 2)

    # Display the original frame in a window
    cv2.imshow('frame-image', display_frame)
    # cv2.imshow('frame-edges', edges)

    # Stop the performance counter
    #end = time.perf_counter()

    # If the button q is pressed in one of the windows
    key = cv2.waitKey(20) & 0xFF
    if key == ord('q'):
        # Exit the While loop
        break
    elif key == ord('w') and t < 255:
        t += 1
    elif key == ord('s') and t > 0:
        t -= 1

    n += 1

# When everything done, release the capture
video_capture.release()
# close all windows
cv2.destroyAllWindows()
# exit the kernel
exit(0)