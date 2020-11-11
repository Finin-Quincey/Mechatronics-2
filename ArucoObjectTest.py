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

marker = ArucoMarker.ArucoMarker(2, MARKER_WIDTH, 20)

marker.print()

while True: # Forever

    

    marker.update()