"""
This class implements line tracking using a Hough line transform based on my original implementation in Java
(see https://github.com/Finin-Quincey/Flexiweld/blob/master/src/main/java/uob/flexiweld/geom/LineTracker.java)
The implementation is done a bit differently here to better conform to the Python style of programming, particularly
because we have access to array operations with numpy
"""
# Disclaimer: I am nowhere near as versed in Pythonic style as I am in Java idioms. Things will probably get ugly.
# (Heck, I've noticed about 3 inefficient things in my Java code already, so even that's far from perfect!)

import cv2

from collections import deque
import numpy as np
import math

import geom

### Constants ###

INTERP_FRAMES = 15 # The number of frames over which lines will be merged

ANGLE_THRESHOLD = math.radians(5) # Lines within this angle of each other are considered parallel
PROXIMITY_THRESHOLD = 8 # Parallel lines within this distance of each other are considered conincident
WIDTH_THRESHOLD = 20 # Parallel, non-coincident lines within this distance of each other are considered two sides of a wall

LINE_COUNT_LIMIT = 100 # We shouldn't detect more than this, if we do then something is wrong

### Variables ###

# A queue of arrays of lines from previous frames
# We can't use a 3D numpy array for this because each frame doesn't necessarily have the same number of lines
# A deque is the next most efficient way of doing it, see https://docs.python.org/3.8/tutorial/datastructures.html#using-lists-as-queues
prev_lines = deque(maxlen = INTERP_FRAMES) # Fixed length makes it discard elements from the other end when full

def update(frame):
    """
    Updates the line tracker with the next frame from the camera and returns the merged lines from the last n frames
    """
    # Optional thresholding step (was needed for the kitchen floor since it's tiled!)
    # Use LineTrackerTest
    frame[frame > 80] = 255

    # Edge detection
    frame = cv2.Canny(frame, 50, 150, apertureSize = 3)

    # Hough line transform
    # This returns lines in a 3D matrix of the form [[[x1, y1, x2, y2]], [...], ...]
    lines = cv2.HoughLinesP(frame, 1, np.pi/180, 80, minLineLength = 25, maxLineGap = 10)

    if lines is not None: # If we found some lines

        # For some reason the HLT seems to return a 3D array that's only one layer deep, so let's remove the unnecessary 3rd dimension
        lines = lines[:, 0]

        # If there are more than the maximum number, remove some so it doesn't go super slowly
        if lines.shape[0] > LINE_COUNT_LIMIT:
            lines = lines[0:LINE_COUNT_LIMIT, :]
            print("Too many lines, removing excess!")

        # Sort them by angle
        angles = []
        for line in lines: angles.append(geom.angle(line)) # Construct a list of the angles of each line
        indices = np.argsort(angles) # TODO: We may need to reverse this, can't remember if it matters
        lines = lines[indices]

    # Add the sorted array of lines to the queue (to the left end so that iterations return most recent first)
    # If there were no lines then this will add an empty array
    prev_lines.appendleft(lines)

    averaged_lines = np.zeros((0, 4)) # Will store averaged lines (this is what ultimately gets returned)

    # Use Python's 'inherent booleanness' to check if prev_lines is empty
    if not prev_lines: return averaged_lines # If so, return the empty array since there's nothing more we can do

    prev_lines_excl_none = [a for a in prev_lines if a is not None] # Exclude None values from prev_lines
    # https://stackoverflow.com/questions/16096754/remove-none-value-from-a-list-without-removing-the-0-value

    if not prev_lines_excl_none: return averaged_lines

    # Construct an array of all the lines in the prev_lines queue (in other words, a flattened copy)
    # N.B. We do NOT need to reverse this because we added the most recent lines to the left-hand end earlier
    all_prev_lines = np.concatenate(prev_lines_excl_none)

    # Fuzzy averaging
    # 1. Take the first line in the list as a reference
    # 2. Compare the rest of the lines in the list to the reference line
    # 3. Remove both the reference line and all coincident lines from the list and merge them by averaging the endpoints
    # 4. Repeat with the remaining lines in the list until all lines have been processed
    while len(all_prev_lines) > 0:

        ref_line = all_prev_lines[0, :] # Take the next remaining line as a reference to compare the rest to
        all_prev_lines = np.delete(all_prev_lines, 0, axis = 0)

        # Choose which axis to compare in based on the gradient of the reference line (avoids issues with vertical lines)
        # We need not actually calculate the gradient though, we only care whether the line is more horizontal or vertical
        compare_y = abs(ref_line[3] - ref_line[1]) > abs(ref_line[2] - ref_line[0])

        coincident = np.zeros((0, 4)) # Initialise array to store all the lines that are coincident

        # Since lines here are just rows in the array rather than objects, they're not pass-by-reference so we need to do
        # this a bit differently
        # This way of iterating is hideously un-pythonic but it avoids having to store indices and remove rows afterwards
        i = 0
        while i < len(all_prev_lines): # Query length of all_prev_lines every loop iteration so it stops at the correct point
            
            # Retrieve the next line and rectify it based on which direction we're comparing in
            line = all_prev_lines[i]
            if compare_y:
                line = geom.y_rectify(line)
            else:
                line = geom.x_rectify(line)

            if check_coincidence(ref_line, line): # Test whether the line is approximately coincident with ref_line
                # Remove the line from the main list and add it to the coincident lines list
                all_prev_lines = np.delete(all_prev_lines, i, axis = 0)
                coincident = np.append(coincident, [line], axis = 0)
                # DO NOT increment i here! We just removed the element at index i so everything will shift back automatically
            else:
                i += 1 # Skip to the next line

        coincident = np.append(coincident, [ref_line], axis = 0) # Add the reference line so it can be averaged with all the coincident lines

        # Now average the start and end points of all the coincident lines - with array operations this is EASY
        # We already rectified them so they should all be pointing in roughly the same direction
        avg_line = np.mean(coincident, axis = 0) # OH MY GOODNESS IT'S A ONE-LINER
        
        # Find the indices of the most extreme start and end points of all the lines
        if compare_y:
            start_index = np.argmin(coincident[:, 1])
            end_index   = np.argmax(coincident[:, 3])
        else:
            start_index = np.argmin(coincident[:, 0])
            end_index   = np.argmax(coincident[:, 2])

        # Retrieve the actual coordinates and find nearest points on the line to each of them
        start = geom.nearest_point_to(avg_line, (coincident[start_index, 0], coincident[start_index, 1]))
        end   = geom.nearest_point_to(avg_line, (coincident[end_index,   2], coincident[end_index,   3]))

        # Store the resulting extended average line
        averaged_lines = np.append(averaged_lines, [[start[0], start[1], end[0], end[1]]], axis = 0)

    return averaged_lines

def check_coincidence(line1, line2):
    """
    Returns true if the two lines are approximately coincident (within the thresholds), false otherwise
    """
    return geom.perpendicular_distance(line1, geom.midpoint(line2)) < PROXIMITY_THRESHOLD and check_parallelness(line1, line2)

def check_parallelness(line1, line2):
    """
    Returns true if the two lines are approximately parallel, false otherwise
    """
    return geom.acute_angle_between(line1, line2) < ANGLE_THRESHOLD

def locate_centrelines(lines):
    """
    Locates pairs of parallel lines and returns an array of the centrelines between each pair
    """
    centrelines = np.zeros((0, 4)) # Initialise array to store centrelines

    # This iteration logic works much like the logic above
    # This time all the lines are rectified already
    while len(lines) > 0:

        ref_line = lines[0, :] # Take the next remaining line as a reference to compare the rest to
        lines = np.delete(lines, 0, axis = 0)

        closest_line = None
        closest_distance = WIDTH_THRESHOLD

        i = 0
        while i < len(lines):
            
            line = lines[i]

            if check_parallelness(ref_line, line) and geom.perpendicular_distance(ref_line, geom.midpoint(line)) < closest_distance:
                closest_line = line
                lines = np.delete(lines, i, axis = 0)
                break

            i += 1

        if closest_line is not None:
            centrelines = np.append(centrelines, [geom.equidistant(ref_line, closest_line)], axis = 0)

    return centrelines