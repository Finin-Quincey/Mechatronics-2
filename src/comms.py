## Code for communicating with Arduino

import socket   # This library will allow you to communicate over the network
import time     # This library will allow us to access the system clock for pause/sleep/delay actions
import logging  # This library will offer us a different method to print information on the terminal (better for debugging purposes)

from enum import Enum # Enumeration types

### Constants ###

# This is the IP address of the machine that the data will be send to
# N.B. when on legacy at uni, run simulink to get the IP of the arduino
#ROBOT_UDP_IP = "138.38.228.186"
#ROBOT_UDP_IP = "138.38.228.190"
ROBOT_UDP_IP = "138.38.229.150" # Wall-e at uni
#ROBOT_UDP_IP = "192.168.0.169"  # Wall-e at uni

# This is the REMOTE port the machine will reply on (on that machine this is the value for the LOCAL port)
ROBOT_UDP_PORT = 50001

# First we need to get the LOCAL IP address of your system
PC_UDP_IP = socket.gethostbyname(socket.gethostname())
 
# This is the LOCAL port I am expecting data (on the sending machine this is the REMOTE port)
PC_UDP_PORT = 50002

# Time in milliseconds after which the 'wait for response' functions will time out
TIMEOUT_LIMIT = 10000

# Resolution of the x and y coordinates in mm
# This determines the maximum coordinate values, which should be at least the size of the arena
SPATIAL_RESOLUTION = 10

BUFFER_SIZE = 1024

### Initialisation ###

# setting the logging level to INFO
logging.basicConfig(level=logging.INFO)

# Create the sockets for the UDP communication
send_socket = socket.socket(socket.AF_INET,     # Family of addresses, in this case IP type 
                            socket.SOCK_DGRAM)  # What protocol to use

recieve_socket = socket.socket(socket.AF_INET,     # Family of addresses, in this case IP type 
                               socket.SOCK_DGRAM)  # What protocol to use

recieve_socket.bind((PC_UDP_IP, PC_UDP_PORT))

logging.info('Sockets successfully created')

### Message Types ###

class MessageType(Enum):

    STOP = 0
    DESTINATION = 1
    UPDATE = 2

### Functions ###

def send_destination_and_wait(dest):
    """
    Sends a position for the robot to move towards and waits for a response
    Parameters:
        - dest: The destination vector to move the robot to
    Returns: True if the robot sent a 1 (confirm) in response, false if the robot sent an error (0) or if the connection timed out
    """
    # Send message to robot
    send_destination(dest)
    # Wait for response
    return wait_for_response()

def send_destination(dest):
    """
    Sends a position for the robot to move towards, specified as a vector of x and y coordinates in mm
    """
    # Convert mm coordinates to 8-bit encoded values for sending
    x = (dest[0] / SPATIAL_RESOLUTION) * 255
    y = (dest[1] / SPATIAL_RESOLUTION) * 255

    # Don't try and drive outside the arena!
    if x < 0 or x > 255 raise ValueError("x coordinate out of bounds!")
    if y < 0 or y > 255 raise ValueError("y coordinate out of bounds!")

    # Send the data to the arduino
    send_socket.sendto(bytes([MessageType.DESTINATION, x, y]), (ROBOT_UDP_IP, ROBOT_UDP_PORT))

def send_stop_and_wait():
    """
    Sends a stop command to the robot and waits for a response
    """
    # Send message to robot
    send_stop()
    # Wait for response
    return wait_for_response()

def send_stop():
    """
    Sends a stop command to the robot
    Returns: True if the robot sent a 1 (confirm) in response, false if the robot sent an error (0) or if the connection timed out
    """
    # No additional data for this one
    send_socket.sendto(bytes([MessageType.STOP]), (ROBOT_UDP_IP, ROBOT_UDP_PORT))

def send_pos_update(pos, rotation):
    """
    Sends a position and orientation update message to the Arduino
    """
    pass # TODO: Implement position updates

def wait_for_response():
    """
    Waits for a response from the Arudino and returns true if a 'confirm' (byte 1) response was received
    """
    # Wait for response
    try
        data, addr = recieve_socket.recvfrom(BUFFER_SIZE)
    except socket.timeout:
        logging.warn("Connection to arduino timed out!")
        return False # Return false if the connection timed out
        
    if not addr == (ROBOT_UDP_IP, ROBOT_UDP_PORT) raise IOError("Received data from an unknown address!")

    return data[0] == bytes(1) # Return true if we received a 1, false if we received a 0