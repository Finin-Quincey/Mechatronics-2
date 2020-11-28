import geom
import vision
import comms

import time
import math

import numpy as np
import cv2

### Constants ###

MAX_CONNECTION_ATTEMPTS = 5     # Maximum number of tries to get a response from the arduino
MAX_DETECTION_ATTEMPTS = 200    # Maximum number of tries to find the robot in frame

UPDATE_PERIOD = 0.02            # Time between aruco rescans
FEEDBACK_START_DELAY = 1.5      # Delay between sending destination message and first update message
FEEDBACK_INTERVAL = 1.5         # Delay between consecutive update messages

CORRECTION_ANGLE_LIMIT = 30     # Maximum magnitude of the error angle (in degrees) sent in update messages

DESTINATION_REACHED_DIST = 100  # Radius around destination point within which robot is considered to have reached it

DEBUG = __name__ == '__main__' # True if we ran this file directly, false if it was imported as a module

SHOW_WINDOW = True

if SHOW_WINDOW:
    cv2.namedWindow("frame-image", cv2.WINDOW_AUTOSIZE)

def nudge_forward():
    """
    Makes the robot drive forward a short distance
    """
    comms.send_forward()
    time.sleep(1)
    comms.send_stop()

def go_to(dest):
    """
    Tells the robot to drive to the specified position (in mm)
    """

    attempts = 0

    while len(vision.robot_pos) == 0 and attempts < MAX_DETECTION_ATTEMPTS:
        out = vision.next_frame(False)
        if SHOW_WINDOW:
            # print(vision.robot_pos)
            cv2.imshow('frame-image', out)
            cv2.waitKey(20)
        attempts += 1
        
    if len(vision.robot_pos) == 0:
        raise IOError(f"Could not locate robot after {MAX_DETECTION_ATTEMPTS} attempts")

    response = False
    attempts = 0

    while attempts < MAX_CONNECTION_ATTEMPTS:
        # Remember this takes an angle from 0 (NORTH) to 360
        response = comms.send_destination_and_wait(vision.robot_pos, vision.robot_angle % 360, dest)
        print(f"Sent destination message to robot: \nCurrent: {vision.robot_pos}\nBearing: {vision.robot_angle % 360}\nDestination: {dest}")
        if response: break # Move on if we got a response
        print("Failed attempt to connect")
        attempts += 1

    if not response:
        raise IOError(f"Failed to connect after {MAX_CONNECTION_ATTEMPTS} attempts")

    time.sleep(FEEDBACK_START_DELAY)

    start = -1

    while True:

        time.sleep(UPDATE_PERIOD)

        out = vision.next_frame(False)
        if SHOW_WINDOW:
            cv2.imshow('frame-image', out)
            cv2.waitKey(20)

        robot_dest_line = [vision.robot_pos[0], vision.robot_pos[1], dest[0], dest[1]]

        if geom.length(robot_dest_line) < DESTINATION_REACHED_DIST: break # Stop looping if robot is near enough to dest

        # print(vision.robot_pos)

        # N.B. geom uses east = 0, ACW is positive whereas vision uses north = 0, CW is positive
        # This needs to be in -180 to 180 range
        error_angle = ((90 - math.degrees(geom.angle(robot_dest_line))) - vision.robot_angle + 180) % 360 - 180

        # Account for robot travelling forward or reverse
        if error_angle > 90:
            error_angle = error_angle - 180
        elif error_angle < -90:
            error_angle = error_angle + 180

        # Limit error correction to +/- 15 degrees
        if abs(error_angle) > CORRECTION_ANGLE_LIMIT: error_angle *= CORRECTION_ANGLE_LIMIT/abs(error_angle)

        # No point correcting by less than a degree
        if abs(error_angle) > 1 and time.perf_counter() - start > FEEDBACK_INTERVAL:
            comms.send_update(error_angle)
            start = time.perf_counter()
            #print("Update sent")
            #print(error_angle)

# DEBUG
if DEBUG:
    route = [[1200, 800], [600, 400], [500, 900], [2000, 1200]] 
    while True:
        for dest in route:
            go_to(dest)
            print("Reached destination")
            time.sleep(7)