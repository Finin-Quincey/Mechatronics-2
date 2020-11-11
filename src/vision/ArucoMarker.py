# External imports
import time
import cv2
from cv2 import aruco

# Internal imports
from vision import Tickable
import Constants

class ArucoMarker(Tickable.Tickable):
    """Instances of this class represent a single ArUco marker"""

    # Load the ArUco Dictionary 4x4_50 and set the detection parameters
    # This is done statically so it is shared between all markers, it's just convenient to keep it in here
    ARUCO_DICT = aruco.Dictionary_get(aruco.DICT_4X4_50)
    pa = aruco.DetectorParameters_create() # What does this do, Ioannis? It's never used...

    def __init__(self, i, size, refresh_rate, camera_matrix, dist_coef):
        """Creates a new ArUco marker object with the given parameters
        
        Parameters:
        - i: The ID of the ArUco marker (as defined in the dictionary)
        - size: The width of the marker in mm
        - refresh_rate: The number of times to update the marker position per second, cannot be more than the camera fps
        - 
        """
        super().__init__() # Call super constructor to add it to list of things we can update
        self.id = i # The ID of the ArUco code to track
        self.size = size # The width of the marker in mm
        self.refresh_rate = refresh_rate # The refresh rate of the marker in Hz
        self.position = [0, 0] # Position of the marker
        self.camera_matrix = camera_matrix
        self.dist_coef = dist_coef
        self.visible = False # Whether the marker is currently visible
        self._prev_update_time = 0 # Used internally to store the time when the marker was last updated

    def print(self):
        """Prints out information about this marker to the console"""
        print(f"ArUco marker with ID {self.id}, size {self.size}mm and refresh rate {self.refresh_rate}Hz")

    def update(self, frame):
        """Rescans the current frame for this marker and updates its position accordingly"""

        t = time.time() # Get current time in seconds
        # Do nothing if the time elapsed since the last update is less than the refresh period
        if t - self._prev_update_time < 1.0 / self.refresh_rate: return
        
        self._prev_update_time = t # If we are updating, set the new prev update time

        ### Detect ArUco Markers ###
        # After the convertion to gray (line 35 of the original) run the detection function
        corners, ids, rP = aruco.detectMarkers(frame, ArucoMarker.ARUCO_DICT)

        self.visible = False # Assume it's not visible until we find it

        # If no markers were found (list is None) then stop
        # N.B. The position will stay the same as it was last frame so if we momentarily lose the marker it's no biggie
        if ids is None: return

        ids = ids.flatten().tolist() # Convert 2D Numpy array to Python list

        # If marker was not found, don't update position
        if ids.count(self.id) == 0: return

        # Calculate the pose of the marker based on the camera calibration
        rvecs, tvecs, _objPoints = aruco.estimatePoseSingleMarkers(corners, self.size, self.camera_matrix, self.dist_coef)

        i = ids.index(self.id) # Retrieve index of this marker's ID
        self.position = tvecs[i][0][0:2] # Update position to be the corresponding corner coordinates

        self.visible = True # Found the marker yay