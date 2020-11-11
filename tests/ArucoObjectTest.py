import vision
from vision import ArucoMarker

### Import required libraries ###
# This is the vision library OpenCV
import cv2
# This is a library for mathematical functions for python (used later)
import numpy as np
# This is a library to get access to time-related functionalities
import time
# Import arUco code library
import cv2.aruco as aruco

### Manual Inputs ###
# ArUco code width
MARKER_WIDTH = 76

### Setup camera ###
# Read and store the calibration information from Sample_Calibration
CALIBRATION = np.load('tests/Calibration.npz') # Load the camera calibration values 
CAMERA_MATRIX = CALIBRATION['CM'] # Camera matrix
DIST_COEFFS = CALIBRATION['dist_coef'] # Distortion coefficients from the camera

marker = ArucoMarker.ArucoMarker(2, MARKER_WIDTH, 20, CAMERA_MATRIX, DIST_COEFFS)

marker.print()

while True: # Forever

    marker.update(frame)