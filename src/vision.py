import cv2
from cv2 import aruco

import numpy as np

import math

import geom
import line_tracker

DEBUG = False

### Constants ###

# TODO: Measure the actual sizes of things!
ARENA_SIZE = [2400, 1800]
MARKER_WIDTH = 70
INITIAL_SCAN_FRAMES = 20 # Number of frames to scan for when route planning

ROBOT_MARKER_ID = 10
ORIGIN_MARKER_ID = 1 # Markers representing the origin of the arena

# These are in CLOCKWISE ORDER!
ARENA_CORNER_COORDS = np.array([[0, 0], [0, ARENA_SIZE[1]], ARENA_SIZE, [ARENA_SIZE[0], 0]])

KERNEL_3 = np.ones(3, dtype = np.uint8)

KERNEL_5 = np.array([
    [0, 1, 1, 1, 0],
    [1, 1, 1, 1, 1],
    [1, 1, 1, 1, 1],
    [1, 1, 1, 1, 1],
    [0, 1, 1, 1, 0]
], dtype = np.uint8)

### Initialisation ###

# Load ArUco dictionary
ARUCO_DICT = aruco.Dictionary_get(aruco.DICT_4X4_50)

# Load calibration file
CALIBRATION = np.load('tests/Calibration.npz') # Load the camera calibration values 
CAMERA_MATRIX = CALIBRATION['CM'] # Camera matrix
DIST_COEFFS = CALIBRATION['dist_coef'] # Distortion coefficients from the camera

# Initialise the camera
video_capture = cv2.VideoCapture(1)

# Set the width and height of the camera to the maximum resolution it can do
video_capture.set(3, 1280)
video_capture.set(4, 720)

VIDEO_SCALE = 0.8

if DEBUG:
    cv2.namedWindow("frame-image", cv2.WINDOW_AUTOSIZE)

### Variables ###

origin_tvec = np.empty((0, 0))

markers = [] # Will store the detected marker positions
walls = [] # Will store the detected wall positions

robot_pos = []
robot_angle = 0

# ROSIE: CALL THIS!
def scan_and_locate_objects():
    """
    Performs a scan of the arena for several frames and returns the arena size, walls and marker positions (not orientations)
    """
    # Process n frames
    for n in range(INITIAL_SCAN_FRAMES):
        next_frame()
    # Return the located objects
    return get_object_positions()

def next_frame():
    """
    Processes the next frame from the camera and updates the positions of all the objects in it
    """
    # Capture current frame from the camera
    ret, frame = video_capture.read()

    # Convert the image from the camera to greyscale
    out = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

    # Scan for the markers
    scan_for_markers(out)

    # Scan for the robot using the original image rather than the greyscale, it needs the colour information
    scan_for_robot(frame)

    # Unlike the aruco marker detection, HLT doesn't do this automatically so we need to do it first
    out = cv2.undistort(out, CAMERA_MATRIX, DIST_COEFFS)

    out = cv2.resize(out, (int(VIDEO_SCALE * 1280), int(VIDEO_SCALE * 720)))

    #walls = line_tracker.update(out) # Update the line tracker

    return out

def get_object_positions():



    return ARENA_SIZE, walls, markers

def scan_for_markers(frame):

    # Detect aruco markers
    # N.B. The frame should already have been converted to greyscale at this point
    corners, ids, rP = aruco.detectMarkers(frame, ARUCO_DICT)

    if ids is None: return

    # Calculate the pose of the marker based on the camera calibration
    rvecs, tvecs, _objPoints = aruco.estimatePoseSingleMarkers(corners, MARKER_WIDTH, CAMERA_MATRIX, DIST_COEFFS)

    # Sort everything into order of IDs
    idx = np.argsort(ids, axis=0)
    tvecs = tvecs[idx]
    rvecs = rvecs[idx]
    ids = ids[idx]

    corner_indices = np.nonzero(ids == ORIGIN_MARKER_ID)[0]
    other_indices = np.nonzero(ids != ORIGIN_MARKER_ID)[0]

    # Global is required to access these from inside a function
    global markers
    global origin_tvec

    # Split tvecs into corners and actual object markers
    if corner_indices.size > 0: origin_tvec = tvecs[corner_indices[0]] # Get tvecs of arena corner markers
    if other_indices.size > 0: tvecs = tvecs[other_indices]

    # Transform the rest of the points
    if origin_tvec.size != 0 and tvecs.size != 0:
        markers = tvecs[:, :, 0, 0:2] - origin_tvec[:, :, 0:2]
        markers[:, 0, 1] = -markers[:, 0, 1] # Flip y axis

def scan_for_robot(frame):
    """
    Scans for the robot's aruco marker (coloured green), filtering the input image it to remove reflections from the hamster ball
    The imput image should be COLOUR
    """
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

    # Morphological opening to patch up the black regions
    gray = cv2.erode(gray, KERNEL_3)
    gray = cv2.dilate(gray, KERNEL_3)

    gray = cv2.convertScaleAbs(gray, alpha = 3, beta = -60)

    gray = cv2.erode(gray, KERNEL_3)
    # gray = cv2.dilate(gray, KERNEL_5)

    gray[gray < 170] = 0 # Threshold

    corners, ids, rP = aruco.detectMarkers(gray, ARUCO_DICT)

    global robot_pos
    global robot_angle

    if ids is not None and ROBOT_MARKER_ID in ids:

        idx = np.array([np.nonzero(ids == ROBOT_MARKER_ID)[0][0]]) # For some reason it occasinally detects the marker twice

        rvecs, tvecs, _objPoints = aruco.estimatePoseSingleMarkers(corners, MARKER_WIDTH, CAMERA_MATRIX, DIST_COEFFS)
        
        # Transform the rest of the points
        if origin_tvec.size != 0 and tvecs.size != 0:

            robot_pos = tvecs[idx, :, 0:2] - origin_tvec[:, :, 0:2]
            robot_pos[:, :, 1] = -robot_pos[:, :, 1] # Flip y axis

            rmat, jacobian = cv2.Rodrigues(rvecs[idx])

            # Extract yaw angle from rotation matrix
            # When the acos and asin expressions have different signs, the angle is negative
            robot_angle = math.degrees(math.acos(rmat[0][0]))
            if math.acos(rmat[0][0]) * math.asin(rmat[1][0]) < 0:
                robot_angle = -robot_angle
            # robot_angle = math.degrees(math.asin(rmat[1][0]))

def relativise(coords):
    """
    Converts the given coordinate vector to be relative to the origin
    """


### DEBUGGING ###
if DEBUG:
    while(True):
        frame = next_frame()
        #print(robot_pos)
        print(robot_angle)
        cv2.imshow('frame-image', frame)
        # If the button q is pressed in one of the windows
        if cv2.waitKey(20) & 0xFF == ord('q'):
            # Exit the While loop
            break
        