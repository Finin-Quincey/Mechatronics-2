import geom
import vision
import comms

import time
import math

import numpy as np
import cv2

MAX_CONNECTION_ATTEMPTS = 5

UPDATE_PERIOD = 0.02
FEEDBACK_INTERVAL = 2

DESTINATION_REACHED_DIST = 100

DEBUG = True

if DEBUG:
    cv2.namedWindow("frame-image", cv2.WINDOW_AUTOSIZE)
    cv2.namedWindow("bw-image", cv2.WINDOW_AUTOSIZE)


# TODO: Function that just drives forward a bit

def go_to(dest):
    """
    Tells the robot to drive to the specified position (in mm)
    """

    attempts = 0

    while len(vision.robot_pos) == 0 and attempts < 200:
        out, bw_out = vision.next_frame()
        if DEBUG:
            # print(vision.robot_pos)
            cv2.imshow('frame-image', out)
            cv2.imshow('bw-image', bw_out)
            cv2.waitKey(20)
        attempts += 1
        
    if len(vision.robot_pos) == 0:
        raise IOError("Could not locate robot after 200 attempts")

    response = False
    attempts = 0

    while not response and attempts < MAX_CONNECTION_ATTEMPTS:
        # Remember this takes an angle from 0 (NORTH) to 360
        response = comms.send_destination_and_wait(vision.robot_pos, vision.robot_angle % 360, dest)
        print("Failed attempt to connect")
        attempts += 1

    if not response:
        raise IOError("Failed to connect after 5 attempts")

    time.sleep(2)

    start = -1

    while True:

        time.sleep(UPDATE_PERIOD)

        out, bw_out = vision.next_frame()
        if DEBUG:
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
        if abs(error_angle) > 15: error_angle *= 15/abs(error_angle)

        # No point correcting by less than a degree
        if abs(error_angle) > 1 and time.perf_counter() - start > FEEDBACK_INTERVAL:
            comms.send_update(error_angle)
            start = time.perf_counter()
            #print("Update sent")
            print(error_angle)

# DEBUG
if DEBUG:
    while True:
        go_to([1200, 800])
        print("Reached destination")
        go_to([400, 400])
        print("Reached destination")
        go_to([500, 900])
        print("Reached destination")
        go_to([2000, 1200])
        print("Reached destination")