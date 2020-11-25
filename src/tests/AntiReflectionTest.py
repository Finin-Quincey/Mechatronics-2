##### Calibration_Checker #####
#### This code ouputs ArUco code tracking data based on the Jypyter calibration to check ####

### Manual Inputs ###
# ArUco code width
width = 77

### Import required libraries ###
# This is the vision library OpenCV
import cv2
# This is a library for mathematical functions for python (used later)
import numpy as np
# This is a library to get access to time-related functionalities
import time
# Import arUco code library
import cv2.aruco as aruco

### Setup ArUco code ###
# Load the ArUco Dictionary 4x4_50 and set the detection parameters
aruco_dict = aruco.Dictionary_get(aruco.DICT_4X4_50)
pa = aruco.DetectorParameters_create()

### Setup camera ###
# Read and store the calibration information from Sample_Calibration
Camera=np.load('tests/Calibration.npz') #Load the camera calibration values 
CM=Camera['CM'] #camera matrix
dist_coef=Camera['dist_coef']# distortion coefficients from the camera

# Select the first camera (0) that is connected to the machine
# in Laptops should be the build-in camera
cap = cv2.VideoCapture(1)

# Set the width and heigth of the camera to 640x480
cap.set(3, 1280)
cap.set(4, 720)

### Setup viewing windows ###
#Create two opencv named windows
cv2.namedWindow("frame-image", cv2.WINDOW_AUTOSIZE)
cv2.namedWindow("gray-image", cv2.WINDOW_AUTOSIZE)

#Position the windows next to eachother
cv2.moveWindow("frame-image", 0, 100)
cv2.moveWindow("gray-image", 750, 100)

clahe = cv2.createCLAHE(clipLimit=2.0, tileGridSize=(8, 8)) # Adaptive histogram equalisation object

k3 = np.ones(3, dtype = np.uint8)

k5 = np.array([
    [0, 1, 1, 1, 0],
    [1, 1, 1, 1, 1],
    [1, 1, 1, 1, 1],
    [1, 1, 1, 1, 1],
    [0, 1, 1, 1, 0]
], dtype = np.uint8)

k7 = np.array([
    [0, 0, 1, 1, 1, 0, 0],
    [0, 1, 1, 1, 1, 1, 0],
    [1, 1, 1, 1, 1, 1, 1],
    [1, 1, 1, 1, 1, 1, 1],
    [1, 1, 1, 1, 1, 1, 1],
    [0, 1, 1, 1, 1, 1, 0],
    [0, 0, 1, 1, 1, 0, 0]
], dtype = np.uint8)

### Run Camera ###
# Execute this continuously
while(True):
    
    # Start the performance clock
    start = time.perf_counter()
    
    # Capture current frame from the camera
    ret, frame = cap.read()

    # Increase contrast
    frame = cv2.convertScaleAbs(frame, alpha = 1.8, beta = -220)

    # Mono mixer
    r_mult = -0.7
    g_mult =  1.7
    b_mult = -0.7
    gray = frame[:, :, 2] * r_mult + frame[:, :, 1] * g_mult + frame[:, :, 0] * b_mult
    gray[gray < 0] = 0 # Remove negative values
    # gray[gray > 50] = 255 # Threshold
    gray = np.uint8(gray)

    # Morphological closing to reconstruct the white regions
    gray = cv2.dilate(gray, k3)
    gray = cv2.erode(gray, k3)

    gray = cv2.convertScaleAbs(gray, alpha = 3, beta = -60)

    # Morphological opening with a bigger mask to patch up the black regions
    gray = cv2.erode(gray, k5)
    gray = cv2.dilate(gray, k5)

    gray[gray < 180] = 0 # Threshold

    # Convert the image from the camera to Gray scale
    #gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

    # Histogram equalisation
    #gray = clahe.apply(gray)

    ### Detect ArUco Markers ###
    # After the convertion to gray (line 35 of the original) run the detection function
    corners, ids, rP = aruco.detectMarkers(gray, aruco_dict)

    # Draw the detected markers as an overlay on the original frame
    out = aruco.drawDetectedMarkers(frame, corners, ids)
    # Draw the detected markers as an overlay on the grayscale image
    # out = aruco.drawDetectedMarkers(gray, corners, ids)

    s = 0.8
    frame = cv2.resize(frame, (int(s * 1280), int(s * 720)))
    gray = cv2.resize(gray, (int(s * 1280), int(s * 720)))

    # Display the original frame in a window
    cv2.imshow('frame-image', frame)
    # Display the grey image in another window
    cv2.imshow('gray-image', gray)

    # Calculate the pose of the marker based on the Camera calibration using:
    rvecs, tvecs, _objPoints = aruco.estimatePoseSingleMarkers(corners, width, CM, dist_coef)

    # Overlay the axis on the image
    try:
        out = aruco.drawAxis(out, CM, dist_coef, rvecs, tvecs, 10)
    except:
        out = out

    # Stop the performance counter
    end = time.perf_counter()
    
    # Print to console the exucution time in FPS (frames per second)
    print ('{:4.1f}'.format(1/(end - start)))
    
    # Iaonnis' code
    # # Print tvecs
    # if ids is not None:
    #     for ind, id in enumerate(ids):
    #         print(id,tvecs[ind])

    ##### TODO !!!! This 2D array thing may cause error later. CHECK !!!! #####
    # Print tvecs
    if ids is not None:
        sorted_indices = np.argsort(ids,axis = None) # Flattens 2D array from ArUco ids output
        for i in sorted_indices:
            print(ids[i], tvecs[i])

    #print(sorted_indices)
    # print(ids)
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