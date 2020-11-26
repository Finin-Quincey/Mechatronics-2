import time     # This library will allow us to access the system clock for pause/sleep/delay actions
import logging  # This library will offer us a different method to print information on the terminal (better for debugging purposes)
import sys
import cv2

import comms

### Constants ###
REFRESH_PERIOD = 80

# ASCII key codes
KEY_Q = ord('q')

### Initialisation ###

# setting the logging level to INFO
logging.basicConfig(level = logging.INFO)

### Setup viewing windows ###
# Create two opencv named windows
# cv2.namedWindow("frame-image", cv2.WINDOW_AUTOSIZE)

prevkey = 0

response = comms.send_destination_and_wait([0, 0], 0, [0, 1000])

if response:
     print("Robot replied with a 1")
else:
     print("Something went wrong!")

time.sleep(4)

comms.send_update(10)

### Main loop ###

# while(True): # Forever

#     frame = vision.next_frame()

#     cv2.imshow('frame-image', frame)

#     key = cv2.waitKey(REFRESH_PERIOD) & 0xFF

#     if key == KEY_Q:
#         break # Exit when Q is pressed