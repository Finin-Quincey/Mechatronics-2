"""
Geometry Functions

Contains a variety of utility functions that perform geometry operations, with inputs and outputs in the
format expected by OpenCV and the rest of the code.
"""

import math
import numpy as np

# N.B. in Python OpenCV, there *is* a Point object but it is implemented as a named tuple
# (see https://docs.python.org/3.8/library/collections.html#collections.namedtuple)
# I'm still in two minds as to whether to implement a Line object or not - in Java the
# choice was obvious because *everything* is an object, that's just how it is - but in
# Python we have array operations and stuff, so I can't help but think it would be overkill

# In theory all the operations in here will work fine for both lists and tuples as inputs

### Unary operations ###

def length(line):
    """
    Returns the length of the given line
    """
    return math.sqrt((line[2] - line[0]) ** 2 + (line[3] - line[1]) ** 2)

def angle(line):
    """
    Returns the angle between the given line and the positive x axis, in the range -pi <= angle < pi (anticlockwise is positive)
    """
    return math.atan2(line[3] - line[1], line[2] - line[0])

def gradient(line):
    """
    Returns the gradient of the given line
    """
    return (line[3] - line[1]) / (line[2] - line[0])

def midpoint(line):
    """
    Returns the midpoint of the given line
    """
    return ((line[0] + line[2]) / 2, (line[1] + line[3]) / 2) # Make this a tuple because that's how OpenCV represents points

# Rectification operations make it easier to compare different lines
# In a lot of cases we could just add/subtract pi from the angle but rectification is more useful in general (and is also cheap)

def x_rectify(line):
    """
    Returns a new line with the start and end swapped if necessary so that the x component is positive
    """
    if line[2] < line[0]: return np.array([line[2], line[3], line[0], line[1]])
    else: return line

def y_rectify(line):
    """
    Returns a new line with the start and end swapped if necessary so that the y component is positive
    """
    if line[3] < line[1]: return np.array([line[2], line[3], line[0], line[1]])
    else: return line

def rectify(line):
    """
    Returns a new line with the start and end swapped if necessary so that x or y component with the largest magnitude is positive
    """
    dx = abs(line[2] - line[0])
    dy = abs(line[3] - line[1])
    if dy > dx: return y_rectify(line)
    elif dx > dy: return x_rectify(line)
    else: return line # If the line is exactly diagonal then leave it as-is

### Binary operations ###

def angle_between(line1, line2):
    """
    Returns the angle between the direction vectors of the given lines, in the range 0 <= angle < pi
    """
    angle_diff = abs(angle(line1) - angle(line2))
    return min(angle_diff, 2 * np.pi - angle_diff)

def acute_angle_between(line1, line2):
    """
    Returns the acute angle between the two given lines, in the range 0 <= angle <= pi/2
    """
    angle_diff = angle_between(line1, line2)
    return min(angle_diff, np.pi - angle_diff)

def obtuse_angle_between(line1, line2):
    """
    Returns the obtuse angle between the two given lines, in the range pi/2 <= angle <= pi
    """
    angle_diff = angle_between(line1, line2)
    return max(angle_diff, np.pi - angle_diff)

def equidistant(line1, line2):
    """
    Returns the coordinates of the line equidistant from the two given lines
    """
    # Need to rectify the lines first or we'll get the wrong result when the lines are in opposite directions
    line1 = rectify(line1)
    line2 = rectify(line2)
    return (line1 + line2) / 2 # Mmmmmm that is neat

def perpendicular_distance(line, point):
    """
    Returns the distance of the given point from the given line
    """
    # Calculate lambda (A) in vector equation of line between wall and node
    start = np.array([line[0], line[1]])
    direction = np.array([line[2] - line[0], line[3] - line[1]])

    numerator = np.dot(np.array(point) - start, direction)
    denominator = np.linalg.norm(direction) ** 2
    A = np.array(numerator/denominator)

    # Using calculated lambda (A) in vector equation, state coordinate of point on wall
    nearest_pt = start + A * direction

    # Look at line equation of point on wall to a node (perpendicular - shortest distance)
    perpendicular_line = [nearest_pt[0] - point[0], nearest_pt[1] - point[1]]

    # Calculate length of perpendicular line between wall point and node
    return np.linalg.norm(perpendicular_line)

def nearest_point_to(line, point):
    """
    Returns the point on the given line that is nearest to the given point. The returned point may be beyond the end of the line.
    """
    hypot = [line[0], line[1], point[0], point[1]] # Imaginary line between line start and given point
    l = length(line)
    if l == 0: # Prevent divide by zero errors, somehow these seem to happen occasionally
        f = 0 # If the point *is* the start of the line, then the nearest point on the line is simply that point!
    else:
        f = (length(hypot) * math.cos(angle_between(line, hypot))) / l

    return (line[0] + f * (line[2] - line[0]), line[1] + f * (line[3] - line[1]))