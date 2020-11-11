import cv2
from cv2 import aruco

from vision import ArucoMarker
from vision import Tickable

class Arena(Tickable.Tickable):
    """Represents the area in which the robot can move"""

    def __init__(self, origin):
        super().__init__()
        self.origin = origin
        
    def update(self, frame):
        pass
        # Do nothing for now but eventually we'll update the perspective transform matrix in here

    def get_arena_coords(self, point):
        """Converts the given xyz coordinate to an (x, y) coordinate relative to the origin marker"""
        if(not self.origin.visible): print("Origin marker not visible!")
        result = [point[0] - self.origin.position[0], point[1] - self.origin.position[1]]
        return result