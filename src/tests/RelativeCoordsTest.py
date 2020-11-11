##### Calibration_Checker with Objects #####
#### This code ouputs ArUco code tracking data based on the Jypyter calibration to check ####

### Import required libraries ###
# This is the vision library OpenCV
import cv2
# This is a library for mathematical functions for python (used later)
import numpy as np
# This is a library to get access to time-related functionalities
import time
# Import arUco code library
import cv2.aruco as aruco

import vision
from vision import Tickable
from vision import ArucoMarker
from vision import Arena

### Manual Inputs ###
# ArUco code width
MARKER_WIDTH = 77

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

MARKER_WIDTH = 76 # Marker width in mm (black region)

# Set up aruco marker object
origin = ArucoMarker.ArucoMarker(2, MARKER_WIDTH, 20, CAMERA_MATRIX, DIST_COEFFS)
marker = ArucoMarker.ArucoMarker(5, MARKER_WIDTH, 20, CAMERA_MATRIX, DIST_COEFFS)

arena = Arena.Arena(origin) # Create a new arena with aruco marker id 2 as the origin

### Run Camera ###
# Execute this continuously
while(True):
    
    # Start the performance clock
    #start = time.perf_counter()
    
    # Capture current frame from the camera
    ret, frame = video_capture.read()

    # Convert the image from the camera to Gray scale
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

    Tickable.Tickable.update_all(gray) # Update all markers and the arena

    # Display the original frame in a window
    cv2.imshow('frame-image',frame)

    # Stop the performance counter
    #end = time.perf_counter()
    
    sys.stdout.write("\033[K") # Clear to the end of line
    
    # Marker relative position readout
    if(marker.visible):
        print(f"Found marker at {arena.get_arena_coords(marker.position)}", end="\r") # end="\r" makes it overwrite the previous line
    else:
        print("Did not find marker", end="\r") # Sad times :(

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