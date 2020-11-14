##### RoutePlanningIdeas #####
#### Initial attempt at Route Planning using A* method ####

### Import required libraries ###
import matplotlib.pyplot as plt
import numpy as np

### Manually write input values - these will be outputted from other code later on ###
SAFETY_RADIUS = 1 # Arbitrary safety radius
arena_size = [20, 10] # Arbitrary arena size
wall_1 = [2, 1, 12, 5] # Arbitrary wall size [x0, y0, x1, y1]
wall_2 = [5, 7, 2, 4]
wall_3 = [0, 10, 0, 2]

## Create list of walls
walls = [] # Initialise list of walls
walls.append(wall_1) # Add walls to list of walls
walls.append(wall_2) # Add walls to list of walls
walls.append(wall_3) # Add walls to list of walls
print(walls)

### Step One - Establish grid ###
arena_nodes = [] # Initialise list of grid nodes
for x in range(arena_size[0] + 1): # For all intervals in x
    for y in range(arena_size[1] + 1): # For all intervals in y
        arena_nodes.append([x, y]) # Record x,y node coordinates
arena_nodes = np.array(arena_nodes) # Make list an array for use in subsequent functions

### Step Two - Consider walls, therefore make new list of available nodes to move in

bad_nodes = []

for wall in walls:
    ## For reference, equation of a vector in the form: vector equation = position vector + A * direction vector
    #                                                   wall_vector     = wall_position   + A * wall_direction

    ## Calculate direction vector of wall
    # For reference, direction vector = [(x1 - x0), (y1 - y0)]
    wall_direction = [wall[2]-wall[0], wall[3]-wall[1]]
    print(wall_direction)
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

    ## Consider distances between node and wall
    for node in arena_nodes: 
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

        # # Remove nodes that are too close to line within the x, y range of line (not infinite line)
        # if node[0] >= x_lower_boundary and node[0] <= x_upper_boundary:
        #     if node[1] >= y_lower_boundary and node[1] <= y_upper_boundary:
        #         if perpendicular_line_length <= SAFETY_RADIUS:
                    
        #            # Find the node[0] and node[1] x,y coordinate in the available nodes array and remove it

        # If the node is in the safety range of the wall in terms of x and y, check its not too close to wall
        if node[0] >= x_lower_boundary and node[0] <= x_upper_boundary:
            if node[1] >= y_lower_boundary and node[1] <= y_upper_boundary:
                if perpendicular_line_length <= SAFETY_RADIUS:
                    bad_nodes.append([node[0], node[1]])
                    ###### THIS IS WHERE I MADE A LIST OF BAD NODES FOR EACH WALL ######
        
        #     else: 
        #         bad_nodes.append([node[0], node[1]])       
        # else:
        #     bad_nodes.append([node[0], node[1]])    

    wall_x = [wall[0], wall[2]]
    wall_y = [wall[1], wall[3]]
    plt.plot(wall_x, wall_y) # Plot wall

#### HERE I WANT TO SAY:
#available_nodes = arena_nodes - bad_nodes

# for node in available_nodes: # Look at each coodinate in turn
#    plt.plot(node[0],node[1],'o', color = 'black') # scatter plot of all nodes

for node in bad_nodes: # Look at each coordinate in turn
    plt.plot(node[0],node[1],'o', color = 'red') # scatter plot of available nodes

plt.show() # Show plot in new figure















#### Mess below


# available_nodes = arena_nodes
# X = []
# for node in bad_nodes_no_duplicates:
#     X.append(np.where((arena_nodes[:,0] == node[0]) & (arena_nodes[:,1]==node[1])))
# print(X)

#print(available_nodes)

# X = np.where((arena_nodes == (0, 1)).all(axis=1))
# print(X)
# for node in arena_nodes:
#     for i in bad_nodes_no_duplicates:
#         if (node[0] == i[0]) and (node[1] == i[1]):
#             pass
#         else:
#             available_nodes.append(node)

# available_nodes = np.array(available_nodes) # Make list an array for use in subsequent functions

# for node in arena_nodes:
#     for i in bad_nodes_no_duplicates:
#         if (node[0] is not i[0]) and (node[1] is not i[1]):
#             available_nodes.append(node)

# available_nodes = np.array(available_nodes) # Make list an array for use in subsequent functions

# print(available_nodes)
# print(A)
# print(node)
# print(arena_nodes)
# print(bad_nodes)


#available_nodes = arena_nodes - bad_nodes