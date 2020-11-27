### WASD control for the robot ###

import socket   # This library will allow you to communicate over the network
import time     # This library will allow us to access the system clock for pause/sleep/delay actions
import logging  # This library will offer us a different method to print information on the terminal (better for debugging purposes)
import cv2      # Literally just using this for key presses
import sys

### Constants ###

REFRESH_PERIOD = 80
DRIVE_SPEED = 40
TURN_SPEED = 25

# ASCII key codes
KEY_W = ord('w')
KEY_A = ord('a')
KEY_S = ord('s')
KEY_D = ord('d')

KEY_Q = ord('q')

# This is the IP address of the machine that the data will be send to
# N.B. when on legacy at uni, run simulink to get the IP of the arduino
#ROBOT_UDP_IP = "138.38.228.186"
#ROBOT_UDP_IP = "138.38.228.190"
#ROBOT_UDP_IP = "138.38.229.150" # Robot at uni
#ROBOT_UDP_IP = "192.168.0.169"  # Robot at home
ROBOT_UDP_IP = "192.168.137.169"  # Robot on rovertime

# This is the REMOTE port the machine will reply on (on that machine this is the value for the LOCAL port)
ROBOT_UDP_PORT = 50003

### Initialisation ###

# setting the logging level to INFO
logging.basicConfig(level = logging.INFO)

# Create the socket for the UDP communication
s = socket.socket(socket.AF_INET,        # Family of addresses, in this case IP type 
                     socket.SOCK_DGRAM)  # What protocol to use
logging.info('Socket successfully created')

# Create a blank window so we can monitor keypresses (yeah this is a dumb way of doing it but meh)
cv2.namedWindow("robot-controller", cv2.WINDOW_AUTOSIZE)

prevkey = 0

### Main loop ###

while(True): # Forever

    key = cv2.waitKey(REFRESH_PERIOD) & 0xFF

    sys.stdout.write("\033[K") # Clear to the end of line

    if key == KEY_Q:
        break # Exit when Q is pressed

    elif key == KEY_W:
        if not key == prevkey:
            s.sendto(bytes([1, DRIVE_SPEED]), (ROBOT_UDP_IP, ROBOT_UDP_PORT))
            time.sleep(0.6) # Ignore the momentary off bit
        print("Forwards", end="\r")

    elif key == KEY_S:
        if not key == prevkey:
            s.sendto(bytes([2, DRIVE_SPEED]), (ROBOT_UDP_IP, ROBOT_UDP_PORT))
            time.sleep(0.6) # Ignore the momentary off bit
        print("Backwards", end="\r")

    elif key == KEY_A:
        if not key == prevkey:
            s.sendto(bytes([3, TURN_SPEED]), (ROBOT_UDP_IP, ROBOT_UDP_PORT))
            time.sleep(0.6) # Ignore the momentary off bit
        print("Left", end="\r")

    elif key == KEY_D:
        if not key == prevkey:
            s.sendto(bytes([4, TURN_SPEED]), (ROBOT_UDP_IP, ROBOT_UDP_PORT))
            time.sleep(0.6) # Ignore the momentary off bit
        print("Right", end="\r")
        
    else:
        if not key == prevkey: s.sendto(bytes([0, 0]), (ROBOT_UDP_IP, ROBOT_UDP_PORT))
        print("Stopped", end="\r")

    prevkey = key