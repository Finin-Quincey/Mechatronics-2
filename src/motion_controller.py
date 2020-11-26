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

DESTINATION_REACHED_DIST = 50

DEBUG = True

if DEBUG:
    cv2.namedWindow("frame-image", cv2.WINDOW_AUTOSIZE)

def go_to(dest):
    """
    Tells the robot to drive to the specified position (in mm)
    """

    attempts = 0

    while len(vision.robot_pos) == 0 and attempts < 200:
        out = vision.next_frame()
        if DEBUG:
            print(vision.robot_pos)
            cv2.imshow('frame-image', out)
            cv2.waitKey(20)
        attempts += 1
        
    if len(vision.robot_pos) == 0:
        raise IOError("Could not locate robot after 200 attempts")

    response = False
    attempts = 0

    pos = np.squeeze(vision.robot_pos)

    while not response and attempts < MAX_CONNECTION_ATTEMPTS:
        # Remember this takes an angle from 0 (NORTH) to 360
        response = comms.send_destination_and_wait(pos, vision.robot_angle % 360, dest)
        print("Failed attempt to connect")
        attempts += 1

    if not response:
        raise IOError("Failed to connect after 5 attempts")

    print("Destination sent")

    time.sleep(3)

    start = -1

    while True:

        time.sleep(UPDATE_PERIOD)

        out = vision.next_frame()
        if DEBUG:
            cv2.imshow('frame-image', out)
            cv2.waitKey(20)

        pos = np.squeeze(vision.robot_pos)

        robot_dest_line = [pos[0], pos[1], dest[0], dest[1]]

        if geom.length(robot_dest_line) < DESTINATION_REACHED_DIST: break # Stop looping if robot is near enough to dest

        error_angle = vision.robot_angle - math.degrees(geom.angle(robot_dest_line))
        # if error_angle > 180:
        #     error_angle = error_angle - 360

        if error_angle > 90:
            error_angle = error_angle - 180
        elif error_angle < -90:
            error_angle = error_angle + 180

        if time.perf_counter() - start > FEEDBACK_INTERVAL:
            comms.send_update(error_angle)
            start = time.perf_counter()
            print("Update sent")
            print(error_angle)

# DEBUG
if DEBUG:
    while True:
        go_to([0, 0])