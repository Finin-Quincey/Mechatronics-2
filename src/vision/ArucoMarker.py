import cv2
from cv2 import aruco

from vision import Tickable

class ArucoMarker(Tickable.Tickable):
    """Instances of this class represent a single ArUco marker"""

    # Load the ArUco Dictionary 4x4_50 and set the detection parameters
    ARUCO_DICT = aruco.Dictionary_get(aruco.DICT_4X4_50)
    pa = aruco.DetectorParameters_create()

    def __init__(self, i, size, refresh_rate, camera_matrix, dist_coef):
        """Creates a new ArUco marker object with the given parameters"""
        super().__init__() # Call super constructor to add it to list of things we can update
        self.id = i # The ID of the ArUco code to track
        self.size = size # The width of the marker in mm
        self.refresh_rate = refresh_rate # The refresh rate of the marker in Hz
        self.position = [0, 0] # Position of the marker
        self.camera_matrix = camera_matrix
        self.dist_coef = dist_coef
        self.visible = False # Whether the marker is currently visible

    def print(self):
        """Prints out information about this marker to the console"""
        print(f"ArUco marker with ID {self.id}, size {self.size}mm and refresh rate {self.refresh_rate}Hz")

    def update(self, frame):
        """Rescans the current frame for this marker and updates its position accordingly"""
        ### Detect ArUco Markers ###
        # After the convertion to gray (line 35 of the original) run the detection function
        corners, ids, rP = aruco.detectMarkers(frame, ArucoMarker.ARUCO_DICT)

        self.visible = False

        if ids is None: return # If no markers were found (list is None) then stop

        ids = ids.flatten().tolist() # Convert 2D Numpy array to Python list

        # If marker was not found, don't update position
        if ids.count(self.id) == 0: return

        # Calculate the pose of the marker based on the camera calibration
        rvecs, tvecs, _objPoints = aruco.estimatePoseSingleMarkers(corners, self.size, self.camera_matrix, self.dist_coef)

        i = ids.index(self.id) # Retrieve index of this marker's ID
        self.position = tvecs[i][0][0:2] # Update position to be the corresponding corner coordinates

        self.visible = True # Found the marker