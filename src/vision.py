"""
Vision System

Main file for the vision system, containing most of the vision code. Responsible for setting up the camera,
image filtering, detecting the robot, aruco markers and walls, annotating the display with detected objects,
and handling transformations between world and image coordinates.

See line_tracker.py for line tracking implementation details.
"""

import cv2
from cv2 import aruco

import numpy as np

import math

import geom
import line_tracker

DEBUG = __name__ == '__main__' # True if we ran this file directly, false if it was imported as a module

FALLBACK_MODE = False # True to use pre-programmed walls, false to detect them normally

### Constants ###

FRAME_WIDTH = 1280          # Horizontal resolution of the camera in pixels
FRAME_HEIGHT = 720          # Vertical resolution of the camera in pixels

CAMERA_HEIGHT = 2250        # Height of the camera above the floor (used to find wall world coords)
ARENA_SIZE = [2400, 1400]   # Size of the arena in mm
MARKER_WIDTH = 70           # Width of the aruco markers in mm

ROBOT_MARKER_ID = 10        # ID of the aruco marker attached to the robot
ORIGIN_MARKER_ID = 1        # ID of the aruco marker representing the origin of the arena
PICKUP_MARKER_ID = 2        # ID of the aruco marker representing the pickup point
DROPOFF_1_MARKER_ID = 3     # ID of the aruco marker representing the 1st dropoff point
DROPOFF_2_MARKER_ID = 4     # ID of the aruco marker representing the 2nd dropoff point
DROPOFF_3_MARKER_ID = 5     # ID of the aruco marker representing the 3rd dropoff point
WALL_E_MARKER_ID = 6        # ID of the aruco marker attached to Wall-E

MORPH_KERNEL = np.ones(3, dtype = np.uint8) # Kernel used for morphological filtering as part of the anti-glare filter

### Initialisation ###

# Load ArUco dictionary
ARUCO_DICT = aruco.Dictionary_get(aruco.DICT_4X4_50)

# Load calibration file
#CALIBRATION = np.load('calibrations/fq_laptop_builtin.npz')
CALIBRATION = np.load('calibrations/webcam.npz')
CAMERA_MATRIX = CALIBRATION['CM'] # Camera matrix
DIST_COEFFS = CALIBRATION['dist_coef'] # Distortion coefficients from the camera
INV_CAMERA_MATRIX = np.linalg.inv(CAMERA_MATRIX)

# Initialise the camera
video_capture = cv2.VideoCapture(1)

# Set the width and height of the camera to the maximum resolution it can do
video_capture.set(3, FRAME_WIDTH)
video_capture.set(4, FRAME_HEIGHT)

if DEBUG:
    cv2.namedWindow("frame-image", cv2.WINDOW_AUTOSIZE)

### Variables ###

origin_tvec = np.empty((0, 0))
# origin_rvec = np.empty((0, 0))
origin_rmat = np.empty((0, 0))      # Transforms IMAGE (u, v) to WORLD (x, y, z)
origin_rmat_inv = np.empty((0, 0))  # Transforms WORLD (x, y, z) to IMAGE (u, v)
# transform_matrix = np.empty((0, 0))     # Transforms IMAGE (u, v) to WORLD (x, y, z)
# inv_transform_matrix = np.empty((0, 0)) # Transforms WORLD (x, y, z) to IMAGE (u, v)

markers = [] # Will store the detected marker positions
walls = np.empty((1, 4)) # Will store the detected wall positions

robot_pos = []
robot_angle = 0

pickup_pos = []
dropoff_1_pos = []
dropoff_2_pos = []
dropoff_3_pos = []

wall_e_pos = [0, 0]
wall_e_angle = 0

wall_E_here = 0
wall_E_moving = 0

def get_object_coords(): 
    """
    Returns the positions of all the objects being tracked, in the following order:
    - arena_size = [xEnd, yEnd]
        where: 
        - xEnd = distance (mm) between the origin and the furthest x coordinate of arena
        - yEnd = distance (mm) between the origin and the furthest y coordinate of arena
    - markers = [x0, y0, ID]
        where:
        - Row 0, must be [M_O's x coordinate, M_O's y coordinate, 0] MUST BE 0 FOR ID (ARTIFICALLY DO THIS IF NOT!)
        - Row 1, must be [Pick-up x coordinate, Pick-up y coordiante, 1] AGAIN, MUST BE 1 FOR ID
        - Row 2, must be [Delivery point A x coordinate, Delivery point A y coordinate, 2] AGAIN, MUST BE 2 FOR ID
        - Row 3, must be [Delivery point B x coordinate, Delivery point B y coordinate, 3] AGAIN, MUST BE 3 FOR ID
        - Row 4, must be [Delivery point C x coordinate, Delivery point C y coordinate, 4] AGAIN, MUST BE 4 FOR ID
        - Row 5, must be [Wall_E centre x coordinate, Wall_E centre y coordinate, 5] AGAIN, MUST BE 5 FOR ID
        - Row 6, must be [Wall_E offset x coordinate, Wall_E offset y coordinate, 6] AGAIN, MUST BE 6 FOR ID
        ^ !!!! And if Wall_E is not there, only output a list of 0-4 (exclude the Wall-E rows) !!!!
    - walls = [x0, y0, x1, y1]
        where:
            - Row 0 is the info of one wall
            - Row 1 is the info of the next wall etc. 
    - wall_E_here
        where: 
        - This is a 0 if Wall_E's centre not in the arena bounds
        - This is a 1 if Wall_E's centre is in the arena bounds
    - wall_E_moving
        where:
        - This is a 0 if Wall_E is stopped
        - This is a 1 if Wall_E is moving
    """

    global markers
    global pickup_pos
    global dropoff_1_pos
    global dropoff_2_pos
    global dropoff_3_pos
    global wall_e_pos
    global wall_e_angle
    global wall_E_here
    global wall_E_moving

    markers = []
    
    markers.append([int(robot_pos[0]),       int(robot_pos[1]),       0])
    markers.append([int(pickup_pos[0]),      int(pickup_pos[1]),      1])
    markers.append([int(dropoff_1_pos[0]),   int(dropoff_1_pos[1]),   2])
    markers.append([int(dropoff_2_pos[0]),   int(dropoff_2_pos[1]),   3])
    markers.append([int(dropoff_3_pos[0]),   int(dropoff_3_pos[1]),   4])
    markers.append([int(wall_e_pos[0]),      int(wall_e_pos[1]),      5])

    offset = 240

    pos = [wall_e_pos[0] + math.sin(wall_e_angle) * offset, wall_e_pos[1] + math.cos(wall_e_angle) * offset]

    markers.append([int(pos[0]), int(pos[1]), 6])
    
    global walls

    # Dummy walls for testing
    # walls = [
    #     [2000, 300, 2000, 900],
    #     [1500, 900, 2000, 900],
    #     [300, 300, 2000, 300],
    #     [700, 600, 700, 1200],
    #     [500, 800, 1100, 800],
    # ]

    if FALLBACK_MODE:
        walls = [
            [450, 300, 450, 1000],
            [450, 450, 1000, 450],
            [450, 1000, 950, 1000],
            [1550, 300, 1550, 900],
            [1550, 300, 1950, 300],
            [1950, 300, 1950, 900]
        ]

    # Return fake outputs of Vision
    return ARENA_SIZE, markers, walls, wall_E_here, wall_E_moving

def next_frame(scan_walls):
    """
    Processes the next frame from the camera and updates the positions of all the objects in it
    """
    # Capture current frame from the camera
    ret, frame = video_capture.read()

    # Make a copy of the frame to add annotations to
    display_frame = frame

    # Convert the image from the camera to greyscale
    grey = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

    # Scan for the markers
    # This MUST be done first to get the origin
    display_frame = scan_for_markers(grey, display_frame)

    # Scan for the robot using the original image rather than the greyscale, it needs the colour information
    display_frame = scan_for_robot(frame, display_frame)

    if scan_walls and not FALLBACK_MODE: display_frame = scan_for_walls(grey, display_frame)

    return display_frame

def scan_for_markers(grey, display_frame):
    """
    Scans the input greyscale frame for markers, updates the relevant variables and returns an annotated copy of display_frame
    """

    # Detect aruco markers
    # N.B. The frame should already have been converted to greyscale at this point
    corners, ids, rP = aruco.detectMarkers(grey, ARUCO_DICT)

    if ids is None: return display_frame

    # Calculate the pose of the marker based on the camera calibration
    rvecs, tvecs, _objPoints = aruco.estimatePoseSingleMarkers(corners, MARKER_WIDTH, CAMERA_MATRIX, DIST_COEFFS)

    # Draw the detected markers as an overlay on the original frame
    display_frame = aruco.drawDetectedMarkers(display_frame, corners, ids)

    # Sort everything into order of IDs
    idx = np.argsort(ids, axis=0)
    tvecs = tvecs[idx]
    rvecs = rvecs[idx]
    ids = ids[idx]

    corner_indices = np.nonzero(ids == ORIGIN_MARKER_ID)[0]
    other_indices = np.nonzero(ids != ORIGIN_MARKER_ID)[0]

    # Global is required to access these from inside a function
    global markers
    global pickup_pos
    global dropoff_1_pos
    global dropoff_2_pos
    global dropoff_3_pos
    global wall_e_pos
    global wall_e_angle
    global wall_E_here
    global wall_E_moving
    global origin_tvec
    global origin_rmat
    global origin_rmat_inv
    # global transform_matrix
    # global inv_transform_matrix

    # Split tvecs and rvecs into origin and actual object markers

    if corner_indices.size > 0:
        origin_tvec = tvecs[corner_indices[0]] # Get tvec of arena origin marker
        origin_rvec = rvecs[corner_indices[0]] # Get rvec of arena origin marker
        origin_rmat, jacobian = cv2.Rodrigues(origin_rvec) # Convert rvec to full 3x3 rotation matrix
        origin_rmat_inv = np.linalg.inv(origin_rmat) # Invert rotation matrix for later
        # transform_matrix = np.hstack((origin_rmat, origin_tvec[0].T)) # Assemble transform matrix
        # inv_transform_matrix = np.linalg.inv(np.vstack((transform_matrix, np.array([0, 0, 0, 1])))) # Compute inverse transform matrix
    
    if other_indices.size > 0:
        tvecs = tvecs[other_indices]
        rvecs = rvecs[other_indices]
        ids = ids[other_indices]

    # Transform the rest of the points
    if origin_tvec.size != 0 and tvecs.size != 0:

        markers = tvecs[:, :, 0, 0:2] - origin_tvec[:, :, 0:2]
        markers[:, 0, 1] = -markers[:, 0, 1] # Flip y axis

        # This is utterly disgusting but it'll work well enough for now

        i = np.nonzero(ids == PICKUP_MARKER_ID)[0]
        if i.size > 0: pickup_pos = np.squeeze(markers[i])

        i = np.nonzero(ids == DROPOFF_1_MARKER_ID)[0]
        if i.size > 0: dropoff_1_pos = np.squeeze(markers[i])

        i = np.nonzero(ids == DROPOFF_2_MARKER_ID)[0]
        if i.size > 0: dropoff_2_pos = np.squeeze(markers[i])

        i = np.nonzero(ids == DROPOFF_3_MARKER_ID)[0]
        if i.size > 0: dropoff_3_pos = np.squeeze(markers[i])

        i = np.nonzero(ids == WALL_E_MARKER_ID)[0]
        if i.size > 0:

            wall_E_here = 1

            new_pos = np.squeeze(markers[i])

            if len(wall_e_pos) > 0:
                wall_E_moving = geom.length([wall_e_pos[0], wall_e_pos[1], new_pos[0], new_pos[1]]) > 10

            if wall_E_moving:
                wall_E_moving = 1
            else:
                wall_E_moving = 0

            wall_e_pos = new_pos

        else:
            wall_E_here = 0

    return display_frame

def scan_for_robot(frame, display_frame):
    """
    Scans for the robot's aruco marker (coloured green), filtering the input image it to remove reflections from the hamster ball
    The imput frame should be COLOUR
    """

    ### Anti-glare filter ###
    # Removes reflections (which are generally white) but retains green parts of the aruco code, and reconstructs the shape
    # This relies on the white parts of aruco code being physically coloured green, but that's a small price to pay!
    # Works best with less complex aruco codes, i.e. ones without single-square black or white regions (10 is particularly good)

    # Mono mixer
    r_mult = -0.9
    g_mult =  1.6
    b_mult = -0.4
    gray = frame[:, :, 2] * r_mult + frame[:, :, 1] * g_mult + frame[:, :, 0] * b_mult
    gray[gray < 0] = 0 # Remove negative values

    # Increase contrast
    gray = cv2.convertScaleAbs(gray)

    # Adaptive threshold to separate out the marker shape
    # N.B. I am aware that the aruco detector also does this but we need to do it now before the morphological operations
    gray = cv2.adaptiveThreshold(gray, 255, cv2.ADAPTIVE_THRESH_GAUSSIAN_C, cv2.THRESH_BINARY, 11, -3)

    # Morphological opening to patch up the black regions
    gray = cv2.dilate(gray, MORPH_KERNEL)
    gray = cv2.erode(gray, MORPH_KERNEL)

    # Finally dilate again to give it more white to find
    gray = cv2.dilate(gray, MORPH_KERNEL)

    gray = np.uint8(gray)

    corners, ids, rP = aruco.detectMarkers(gray, ARUCO_DICT)

    global robot_pos
    global robot_angle

    if ids is not None and ROBOT_MARKER_ID in ids:

        idx = np.array([np.nonzero(ids == ROBOT_MARKER_ID)[0][0]]) # For some reason it occasinally detects the marker twice

        # Compute the positions and orientations of the detected aruco markers
        rvecs, tvecs, _objPoints = aruco.estimatePoseSingleMarkers(corners, MARKER_WIDTH, CAMERA_MATRIX, DIST_COEFFS)

        # Draw the detected markers as an overlay on the original frame
        display_frame = aruco.drawDetectedMarkers(display_frame, corners, ids)
        
        # Transform the rest of the points
        if origin_tvec.size != 0 and tvecs.size != 0:

            robot_pos = tvecs[idx, :, 0:2] - origin_tvec[:, :, 0:2]
            robot_pos[:, :, 1] = -robot_pos[:, :, 1] # Flip y axis
            robot_pos = np.squeeze(robot_pos)

            # Convert rotation vector to matrix form
            rmat, jacobian = cv2.Rodrigues(rvecs[idx])

            # Extract yaw angle from rotation matrix
            # When the acos and asin expressions have different signs, the angle is negative
            robot_angle = math.degrees(math.acos(rmat[0][0]))
            if math.acos(rmat[0][0]) * math.asin(rmat[1][0]) < 0:
                robot_angle = -robot_angle
            # robot_angle = math.degrees(math.asin(rmat[1][0]))

    return display_frame

def scan_for_walls(grey, display_frame):
    """
    Scans the input greyscale frame for walls, updates the relevant variables and returns an annotated copy of display_frame
    """
    # Unlike the aruco marker detection, HLT doesn't do this itself so we need to do it first
    grey = cv2.undistort(grey, CAMERA_MATRIX, DIST_COEFFS)
    display_frame = cv2.undistort(display_frame, CAMERA_MATRIX, DIST_COEFFS)

    global walls

    lines_uv = line_tracker.update(grey) # Update the line tracker

    for line_uv in lines_uv:
        # Draw line on display frame (might as well use existing loop)
        display_frame = cv2.line(display_frame, (int(line_uv[0]), int(line_uv[1])), (int(line_uv[2]), int(line_uv[3])), (255, 0, 0), 2)

    walls_uv = line_tracker.locate_centrelines(lines_uv) # Find centrelines

    walls = np.empty((1, 4))

    # If origin exists
    if origin_tvec.size != 0 and origin_rmat_inv.size != 0:

        # Transform walls to world coordinates
        for wall_uv in walls_uv:
            
            start = transform_to_world_coords(wall_uv[0], wall_uv[1])
            end   = transform_to_world_coords(wall_uv[2], wall_uv[3])
            walls = np.append(walls, [[start[0], start[1], end[0], end[1]]], axis = 0)
            
            # Draw wall on display frame (might as well use existing loop)
            display_frame = cv2.line(display_frame, (int(wall_uv[0]), int(wall_uv[1])), (int(wall_uv[2]), int(wall_uv[3])), (255, 255, 0), 2)

    return display_frame

def transform_to_world_coords(u, v):
    """
    Transforms the given image coordinates (uv) to world coordinates (xy). The image coordinates should be undistorted first.
    """
    # Do the reverse of what we have below
    z = CAMERA_HEIGHT
    pt = [u * z, v * z, z]
    image_pt = INV_CAMERA_MATRIX.dot(pt)
    tvec = np.squeeze(origin_tvec)
    xyz = origin_rmat_inv.dot(image_pt - tvec).T
    return xyz[0:2] # We just want x and y (z should be 0 anyway)

def transform_to_image_coords(x, y):
    """
    Transforms the given world coordinates (xy) to image coordinates (uv)
    """
    # See https://stackoverflow.com/questions/46363618/aruco-markers-with-opencv-get-the-3d-corner-coordinates?rq=1
    tvec = np.squeeze(origin_tvec)
    image_pt = origin_rmat.dot(np.array([x, y, 0]).T) + tvec
    pt = CAMERA_MATRIX.dot(image_pt)
    # p = np.array([[x - tvec[0], tvec[1] - y, -tvec[2], 1]]) # Here we flip the y coordinate
    # pts = CAMERA_MATRIX.dot(transform_matrix).dot(p.T)
    # This inbuilt opencv function is supposed to do the above but I can't get it to work
    # pts, _ = cv2.projectPoints(np.array([[[x, y, z]]], dtype=np.float32), origin_rvec, origin_tvec, CAMERA_MATRIX, DIST_COEFFS)
    return [pt[0]/pt[2], pt[1]/pt[2]] # Divide by z to get values at z=1, which is where the camera is

### DEBUGGING ###
if DEBUG:

    n = 0
    
    while(True):
        # Camera does weird stuff for the first few frames so don't track lines until then
        frame = next_frame(n > 50)
        #print(robot_pos)
        #print(robot_angle)
        #print(walls)
        cv2.imshow('frame-image', frame)
        # If the button q is pressed in one of the windows
        if cv2.waitKey(20) & 0xFF == ord('q'):
            # Exit the While loop
            break
        n += 1
        