"""
Communications

Code for communicating with the Arduino on board the robot. Responsible for setting up the connection,
converting input parameters into byte (uint8) representation, and sending/receiving messages.
"""

import socket           # Network communication library
import time             # System clock for pause/sleep/delay actions
import logging

from enum import Enum   # Enumeration types

DEBUG = False           # True to print actual bytes sent for debugging purposes

### Constants ###

# This is the IP address of the machine that the data will be send to
# N.B. when on legacy at uni, run simulink to get the IP of the arduino
ROBOT_UDP_IP = "192.168.137.169" # Rovertime hotspot

ROBOT_UDP_PORT = 50001  # Remote port the arduino will reply on

PC_UDP_IP = socket.gethostbyname(socket.gethostname()) # Computer local IP address
PC_UDP_PORT = 50002     # Local receive port

TIMEOUT_LIMIT = 3       # Time in seconds after which the 'wait for response' functions will time out

SPATIAL_RANGE = 2550    # Maximum x and y coordinates in mm, should be at least the size of the arena
ANGLE_RANGE = 360       # Maximum angle in degrees

BUFFER_SIZE = 1024      # Maximum size of the buffer used to receive data

### Initialisation ###

logging.basicConfig(level=logging.INFO)

# Create the sockets for the UDP communication
send_socket = socket.socket(socket.AF_INET,     # Family of addresses, in this case IP type 
                            socket.SOCK_DGRAM)  # What protocol to use

recieve_socket = socket.socket(socket.AF_INET,     # Family of addresses, in this case IP type 
                               socket.SOCK_DGRAM)  # What protocol to use

recieve_socket.bind((PC_UDP_IP, PC_UDP_PORT))
recieve_socket.settimeout(TIMEOUT_LIMIT) # Set a timeout so the program won't just freeze if it can't connect

logging.info('Sockets successfully created')

### Message Types ###

# For ease of implementation in Simulink, each type of message has its own UDP port

class MessagePort(Enum):

    DESTINATION = 50001     # Destination message with current xy, current bearing and destination xy
    UPDATE = 50002          # Update message with correction angle
    MANUAL = 50003          # Manual control message with command ID and speed

### Functions ###

def send_destination_and_wait(pos, bearing, dest):
    """
    Sends a position for the robot to move towards and waits for a response
    Parameters:
        - pos: The current position vector of the robot
        - bearing: The current orientation of the robot, in degrees (0-360)
        - dest: The destination vector to move the robot to
    Returns: True if the robot sent a 1 (confirm) in response, false if the robot sent an error (0) or if the connection timed out
    """
    # Send message to robot
    send_destination(pos, bearing, dest)
    # Wait for response before returning from function
    return wait_for_response()

def send_destination(pos, bearing, dest):
    """
    Sends a position for the robot to move towards, specified as a vector of x and y coordinates in mm
    Parameters:
        - pos: The current position vector of the robot
        - bearing: The current orientation of the robot, in degrees (0-360)
        - dest: The destination vector to move the robot to
    """
    # Convert mm coordinates to 8-bit encoded values for sending
    x = int((pos[0] / SPATIAL_RANGE) * 255)
    y = int((pos[1] / SPATIAL_RANGE) * 255)

    # Shouldn't be outside the arena to start with
    if x < 0 or x > 255: raise ValueError(f"Current x coordinate {pos[0]} out of bounds!")
    if y < 0 or y > 255: raise ValueError(f"Current y coordinate {pos[1]} out of bounds!")

    angle = int((bearing / ANGLE_RANGE) * 255)
    
    # All angles should be within 0-360
    if angle < 0 or angle > 255: raise ValueError(f"Bearing angle {bearing} out of bounds!")

    # Convert mm coordinates to 8-bit encoded values for sending
    dx = int((dest[0] / SPATIAL_RANGE) * 255)
    dy = int((dest[1] / SPATIAL_RANGE) * 255)

    # Don't try and drive outside the arena!
    if dx < 0 or dx > 255: raise ValueError(f"Destination x coordinate {dest[0]} out of bounds!")
    if dy < 0 or dy > 255: raise ValueError(f"Destination y coordinate {dest[1]} out of bounds!")

    # Send the data to the arduino
    send_socket.sendto(bytes([x, y, angle, dx, dy]), (ROBOT_UDP_IP, MessagePort.DESTINATION.value))

    if DEBUG: print(f"Sent destination message: {[x, y, angle, dx, dy]}")

def send_update(angle_correction):
    """
    Sends an orientation adjustment update message to the Arduino
    """
    angle = int(((angle_correction + 180) / ANGLE_RANGE) * 255)
    
    # All angles should be within -180 to 180
    if angle < 0 or angle > 255: raise ValueError(f"Bearing angle {angle_correction} out of bounds!")

    # Even when there's only one value, it MUST BE IN SQUARE BRACKETS!
    send_socket.sendto(bytes([angle]), (ROBOT_UDP_IP, MessagePort.UPDATE.value))

    if DEBUG: print(f"Sent update message: {[angle]}")

def wait_for_response():
    """
    Waits for a response from the Arudino and returns true if a 'confirm' (byte 1) response was received
    """
    # Wait for response
    try:
        data, addr = recieve_socket.recvfrom(BUFFER_SIZE)
    except socket.timeout:
        logging.warn("Connection to arduino timed out!")
        return False # Return false if the connection timed out
        
    if not addr == (ROBOT_UDP_IP, ROBOT_UDP_PORT): raise IOError("Received data from an unknown address!")

    return data[0] == 1 # Return true if we received a 1, false if we received a 0

def send_forward():
    """
    Sends a forward command to the robot
    """
    send_socket.sendto(bytes([1, 30]), (ROBOT_UDP_IP, MessagePort.MANUAL.value))

def send_stop():
    """
    Sends a stop command to the robot
    """
    send_socket.sendto(bytes([0, 0]), (ROBOT_UDP_IP, MessagePort.MANUAL.value))