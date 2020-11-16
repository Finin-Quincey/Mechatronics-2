##### RoutePlanningIdeas #####
#### Initial attempt at Route Planning using A* method ####

### Import required libraries ###
import matplotlib.pyplot as plt
import numpy as np

### Manually write inputs - these will be output by vision code ###
SAFETY_RADIUS = 150 # 150mm Acceptable distance between centre of sphere (robot) and wall/obstacles
arena_size = [2500, 1500] # Arbitrary arena size 2500mm = 2.5m by 1500mm = 1.5m
arena_step = 50 # Increment grid at 50mm = 5cm

wall_count = 2 # Number of walls to test code
walls = [[0, 0, 0, 0]] * wall_count # Initialise walls list
# Load test walls, dependent on how many are being tested
if wall_count == 0:
    pass
if wall_count >= 1:
    walls[0] = ([0, 0, 2500, 1500])
if wall_count >= 2:
    walls[1] = ([0, 1500, 2500, 0])
if wall_count >= 3:
    walls[2] = ([2, 2, 2498, 1498])   

print(walls)    

#### Actual code ####
### Establish grid ###
## Plot arena ##

# Setup
ARENA_COUNT = 4 # Number of walls to test code
arena_lines = [[0, 0, 0, 0]] * ARENA_COUNT # Initialise arena_points list

# Assign arena grid points
arena_lines[0] = [0, 0, 0, arena_size[1]] # Bottom left corner
arena_lines[1] = [0, arena_size[1], arena_size[0], arena_size[1]] # Top left corner
arena_lines[2] = [arena_size[0], arena_size[1], arena_size[0], 0] # Top right corner
arena_lines[3] = [arena_size[0], 0, 0, 0] # Bottom right corner

# Plot arena lines (borders) between 2 corners
for line in arena_lines:
    x = [line[0], line[2]]
    y = [line[1], line[3]]
    plt.plot(x, y, 'b')

## Find safe travel zones given arena borders ##

# Setup grid point dimensions
x_arena = list(range(0 + SAFETY_RADIUS, arena_size[0] + 1 - SAFETY_RADIUS, arena_step)) # Apply safety distance to arena edge in x
y_arena = list(range(0 + SAFETY_RADIUS, arena_size[1] + 1 - SAFETY_RADIUS, arena_step)) # Apply safety distance to arena edge in y

arena_nodes = [] # Initialise arena nodes (grid points)
# Make grid points
for x in x_arena:
    for y in y_arena:
        arena_nodes.append([x, y])
arena_nodes = np.array(arena_nodes) # Make list an array for use in subsequent functions

# Plot safe travel gridpoints given arena borders
for node in arena_nodes: # Look at each coodinate in turn
    plt.plot(node[0],node[1],'o', color = 'black') # scatter plot of all nodes

available_nodes = []

## Consider distances between node and wall
for node in arena_nodes: 

    true_vector = []

    for wall in walls:

        ## For reference, equation of a vector in the form: vector equation = position vector + A * direction vector
        #                                                   wall_vector     = wall_position   + A * wall_direction

        ## Calculate direction vector of wall
        # For reference, direction vector = [(x1 - x0), (y1 - y0)]
        wall_direction = [wall[2]-wall[0], wall[3]-wall[1]]
        #print(wall_direction)
        ## Establish x range of wall
        # Find lower limit of x in wall: Look at x0 and x1, and find which is lower
        if wall[0] <= wall[2]:
            x_lower_boundary = wall[0] - SAFETY_RADIUS
            x_upper_boundary = wall[2] + SAFETY_RADIUS
        else:
            x_lower_boundary = wall[2] - SAFETY_RADIUS
            x_upper_boundary = wall[1] + SAFETY_RADIUS

        # Repeat above for y range of wall
        # Find lower limit of y in wall: Look at y0 and y1, and find which is lower
        if wall[1] <= wall[3]:
            y_lower_boundary = wall[1] - SAFETY_RADIUS
            y_upper_boundary = wall[3] + SAFETY_RADIUS
        else:
            y_lower_boundary = wall[3] - SAFETY_RADIUS
            y_upper_boundary = wall[1] + SAFETY_RADIUS   

        # Calculate lambda (A) in vector equation of line between wall and node
        wall_start = [wall[0], wall[1]]
        numerator = np.dot((node - wall_start), wall_direction)
        denominator = (np.linalg.norm(wall_direction))**2
        A = np.array(numerator/denominator)

        # Using calculated lambda (A) in vector equation, state coordinate of point on wall
        point = wall_start + A * wall_direction

        # Look at line equation of point on wall to a node (perpendicular - shortest distance)
        perpendicular_line = [point[0]-node[0], point[1]-node[1]]

        # Calculate length of perpendicular line between wall point and node
        perpendicular_line_length = np.linalg.norm(perpendicular_line)

        # If the node is in the safety range of the wall in terms of x and y, check its not too close to wall
        if node[0] >= x_lower_boundary and node[0] <= x_upper_boundary:
            if node[1] >= y_lower_boundary and node[1] <= y_upper_boundary:
                if perpendicular_line_length <= SAFETY_RADIUS:
                    true_vector.append(0)
                else:
                    true_vector.append(1)
            else:
                true_vector.append(1)       
        else:
            true_vector.append(1)     

    if all(true_vector):
        available_nodes.append(node) 

for node in available_nodes: # Look at each coodinate in turn
    plt.plot(node[0],node[1],'o', color = 'red') # scatter plot of all nodes

for wall in walls:
    wall_x = [wall[0], wall[2]]
    wall_y = [wall[1], wall[3]]
    plt.plot(wall_x, wall_y) # Plot wall

### Plot all above ###
plt.show() # Show plot in new figure

##### NOTE TO SELF #####
# When doing A*, will need to make sure it only allows movements between adjacent nodes, not a distant available node.