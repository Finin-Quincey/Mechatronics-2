import geom
import vision
import comms

import time
import math
import sys

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

LOITERING_TIME_LIMIT = 12       # Maximum time the robot can wait in one place during a move, before we resend the destination
LOITERING_RADIUS = 100          # Distance within which the robot is considered to be loitering

DEBUG = __name__ == '__main__' # True if we ran this file directly, false if it was imported as a module

SHOW_WINDOW = True
VIDEO_SCALE = 0.8

if SHOW_WINDOW:
    cv2.namedWindow("frame-image", cv2.WINDOW_AUTOSIZE)

def nudge_forward():
    """
    Makes the robot drive forward a short distance, without feedback
    """
    print("Nudging forward...")
    vision.wall_e_angle
    comms.send_forward()
    time.sleep(1)
    comms.send_stop()

def go_to(dest):
    """
    Tells the robot to drive to the specified position (in mm)
    """
    #time.sleep(2) # Wait for robot to settle, helps with overshoot on the turn

    attempt_locate_robot() # First, try and find the robot

    attempt_send_destination(dest) # Then try to send it the destination

    # Store start pos of move for loitering check later
    start_pos = vision.robot_pos
    last_check_in_time = time.perf_counter()

    time.sleep(FEEDBACK_START_DELAY)

    start = -1

    # Position feedback loop
    while True:

        time.sleep(UPDATE_PERIOD)

        update_display(dest)

        # Calculate the distance from the robot to the destination
        robot_dest_line = [vision.robot_pos[0], vision.robot_pos[1], dest[0], dest[1]]
        if geom.length(robot_dest_line) < DESTINATION_REACHED_DIST: break # Stop looping if robot is near enough to dest

        # Loitering check
        if time.perf_counter() - last_check_in_time > LOITERING_TIME_LIMIT:
            
            # Calculate the distance from where the robot was last time we checked to where it is now
            move_since_check_in = [start_pos[0], start_pos[1], vision.robot_pos[0], vision.robot_pos[1]]

            # If it's still there, it is loitering, which is not allowed!
            if geom.length(move_since_check_in) < LOITERING_RADIUS:
                print("Robot loitered in one place for too long! Resending previous destination")
                attempt_send_destination(dest) # Resend the destination

            # Either way, update the stored position to check against next time and reset the timer
            start_pos = vision.robot_pos
            last_check_in_time = time.perf_counter()

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
    
    print("Destination reached")

def update_display(dest):
    """
    Fetches the next frame from the camera, processes it and displays it on-screen
    """
    out = vision.next_frame(False)

    # Plot destination if given
    if dest is not None:
        pt = vision.transform_to_image_coords(dest[0], dest[1])
        out = cv2.circle(out, (int(pt[0]), int(pt[1])), radius = 6, color = (0, 127, 255), thickness = -1)

    if SHOW_WINDOW:
        cv2.imshow('frame-image', cv2.resize(out, (int(VIDEO_SCALE * 1280), int(VIDEO_SCALE * 720))))
        if cv2.waitKey(20) & 0xFF == ord('q'):
            sys.exit() # Exit the program if Q (quit) is pressed

def attempt_locate_robot():
    """
    Continually updates the camera feed until the robot marker is detected, or throws an error if it cannot be found after a
    predefined number of frames
    """
    attempts = 0

    while len(vision.robot_pos) == 0 and attempts < MAX_DETECTION_ATTEMPTS:
        update_display(None)
        attempts += 1
        
    if len(vision.robot_pos) == 0:
        raise IOError(f"Could not locate robot after {MAX_DETECTION_ATTEMPTS} attempts")

def attempt_send_destination(dest):
    """
    Attempts to send a destination (move) packet to the robot, and throws an error if no response is received after a
    predefined number of tries
    """
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

# DEBUG
if DEBUG:
    route = [[1200, 800], [600, 400], [500, 900], [2000, 1200]] 
    while True:
        for dest in route:
            go_to(dest)
            time.sleep(7)