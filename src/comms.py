## Code for communicating with Arduino

import socket   # This library will allow you to communicate over the network
import time     # This library will allow us to access the system clock for pause/sleep/delay actions
import logging  # This library will offer us a different method to print information on the terminal (better for debugging purposes)

from enum import Enum # Enumeration types

### Constants ###

# This is the IP address of the machine that the data will be send to
#ROBOT_UDP_IP = "138.38.228.186"
#ROBOT_UDP_IP = "138.38.228.190"
ROBOT_UDP_IP = "192.168.0.169"

# This is the RENOTE port the machine will reply on (on that machine this is the value for the LOCAL port)
ROBOT_UDP_PORT = 50001

# First we need to get the LOCAL IP address of your system
PC_UDP_IP = socket.gethostbyname(socket.gethostname())
 
# This is the LOCAL port I am expecting data (on the sending machine this is the REMOTE port)
PC_UDP_PORT = 50002

# Time in milliseconds after which the 'wait for response' functions will time out
TIMEOUT_LIMIT = 10000

### Initialisation ###

# setting the logging level to INFO
logging.basicConfig(level=logging.INFO)

# Create the socket for the UDP communication
s = socket.socket(socket.AF_INET,        # Family of addresses, in this case IP type 
                     socket.SOCK_DGRAM)  # What protocol to use
logging.info('Socket successfully created')

s.sendto(bytes([0, 0]), (ROBOT_UDP_IP, ROBOT_UDP_PORT))

### TESTING STUFF ###
# while True:
#     s.sendto(bytes([1, 50]), (ROBOT_UDP_IP, ROBOT_UDP_PORT))
#     time.sleep(2)
#     s.sendto(bytes([3, 20]), (ROBOT_UDP_IP, ROBOT_UDP_PORT))
#     time.sleep(2)

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
    """
    send_destination(dest)
    # Wait for response

def send_destination(dest):
    """
    Sends a position for the robot to move towards
    """
    pass

def send_stop_and_wait():
    """
    Sends a stop command to the robot and waits for a response
    Timeout after ...
    """
    send_stop()
    # Wait for response

def send_stop():
    """
    Sends a stop command to the robot
    """
    pass

def send_pos_update(pos, rotation):
    """
    Sends a position and orientation update message to the Arduino
    """
    pass