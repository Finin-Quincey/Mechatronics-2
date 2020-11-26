import geom
import vision
import comms

import time

import numpy as np

MAX_CONNECTION_ATTEMPTS = 5
MAX_CONNECTION_ATTEMPTS = 5

UPDATE_PERDIOD = 0.02

DESTINATION_REACHED_DIST = 50

def go_to(dest):
    """
    Tells the robot to drive to the specified position (in mm)
    """
    response = False
    attempts = 0

    while not response and attempts < MAX_CONNECTION_ATTEMPTS:
        response = comms.send_destination_and_wait(vision.robot_pos, vision.robot_angle, dest)
        attempts += 1

    if not response:
        raise IOError("Failed to connect after 5 attempts")

    while True:

        time.pause(UPDATE_PERDIOD)

        vision.next_frame()

        robot_dest_line = [vision.robot_pos[0], vision.robot_pos[1], dest[0], dest[1]]

        if geom.length(robot_dest_line) < DESTINATION_REACHED_DIST: break # Stop looping if robot is near enough to dest

        error_angle = vision.robot_angle - math.degrees(geom.angle(robot_dest_line))

        comms.send_update(error_angle)