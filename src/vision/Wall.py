# External imports
import time
import cv2
from cv2 import aruco

# Internal imports
from vision import Tickable
import Constants

class Wall(Tickable.Tickable):
    """Instances of this class represent a straight 'wall' (strip of black/white paper)"""

    THICKNESS = 50 # Wall thickness in mm

    # Edge detector parameters
    LOWER_THRESHOLD = 50
    UPPER_THRESHOLD = 150
    APERTURE_SIZE = 3

    # Hough line transform parameters
    RHO = 1
    THETA = np.pi/180
    THRESHOLD = 100
    MIN_LINE_LENGTH = 100
    MAX_LINE_GAP = 10

    def __init__(self, refresh_rate, camera_matrix, dist_coef):
        """Creates a new wall object with the given parameters
        
        Parameters:
        - refresh_rate: The number of times to update the wall position per second, cannot be more than the camera fps
        - camera_matrix: The camera matrix from the camera calibration file
        - dist_coef: The distortion coefficients from the camera calibration file
        """
        super().__init__() # Call super constructor to add it to list of things we can update
        self.refresh_rate = refresh_rate # The refresh rate of the wall in Hz
        self.position = [[0, 0], [0, 0]] # Start and end coordinates of the wall
        self.camera_matrix = camera_matrix
        self.dist_coef = dist_coef
        self.visible = False # Whether the wall is currently visible
        self._prev_update_time = 0 # Used internally to store the time when the wall was last updated

    def print(self):
        """Prints out information about this wall to the console"""
        print(f"Wall with refresh rate {self.refresh_rate}Hz")

    def intersect(self, start, end, safety_margin)
        """Returns true if the line joining the given start and end points would cross within the given safety margin of this wall"""
        # Some geometry happens
        return false

    def update(self, frame):
        """Rescans the current frame for this wall and updates its position accordingly"""

        t = time.time() # Get current time in seconds
        # Do nothing if the time elapsed since the last update is less than the refresh period
        if t - self._prev_update_time < 1.0 / self.refresh_rate: return
        
        self._prev_update_time = t # If we are updating, set the new prev update time

        # Edge detection
        out = cv2.Canny(out, LOWER_THRESHOLD, UPPER_THRESHOLD, apertureSize = APERTURE_SIZE)

        # Hough line transform
        lines = cv2.HoughLinesP(out, RHO, THETA, THRESHOLD, minLineLength = MIN_LINE_LENGTH, maxLineGap = MAX_LINE_GAP)

        self.visible = False # Assume it's not visible until we find it

        # Wall detection stuff

        self.visible = True # Found the marker yay