## Note, everything in arena coordinates ##
# This code is given target location:

# target = [x,y]

# This code will call up other functions to supply:

# start_postion = [x,y]
# pickup = [x,y]
# mars_drop = [x,y]
# galaxy_drop = [x,y]
# milkyway_drop = [x,y]

# wall1 = object
# wall1 = [[x1,y1],[x2,y2]]

# Plot a route between start and target

##### Start of actual code #####
import matplotlib.pyplot as plt
import numpy as np

SAFETY_RADIUS = 1 # For now...

#### First step is creating a grid in graph form ####
## Manually define inputs from vision ##

# Create arbitrary arena
x_arena = 20 # Set x distance of arena to ...
y_arena = 10 # Set y distance of arena to ...

# Create arbitrary wall
wall_start = [2,1] 
wall_end = [12,5]

# Create grid of nodes 
arena_nodes = []
for x in range(x_arena):
    for y in range(y_arena):
        arena_nodes.append([x,y])
arena_nodes = np.array(arena_nodes)      
#print(arena_nodes[0])
print(arena_nodes)

# Calculate direction vector of wall
wall_direction = [wall_end[0]-wall_start[0], wall_end[1]-wall_start[1]]
print(wall_direction)

available_nodes = []

for node in arena_nodes:
    # Calculate lambda (A) in vector equation of line between wall and node
    numerator = np.dot((node - wall_start) , wall_direction)
    denominatior = (np.linalg.norm(wall_direction))**2
    A = np.array(numerator/denominatior)
    #print(A)

    # Using calculated lambda (A) in vector equation, state coordinate of point on wall
    point = wall_start + A * wall_direction
    #print(point)

    # Look at line equation of point on wall to a node (perpendicular - shortest distance)
    perpendicular_line = [point[0]-node[0], point[1]-node[1]]
    #print(perpendicular_line)

    # Calculate length of perpendicular line between wall point and node
    perpendicular_line_length = np.linalg.norm(perpendicular_line)
    #print(perpendicular_line_length)

    #print(f"node is {node}")

    # Problem - this only works if the wall_start is the smallest x & y values, and wall_end is largest x & y value
    # If the node is in the safety range of the wall in terms of x and y, check its not too close to wall
    if node[0] >= (wall_start[0] - SAFETY_RADIUS) and node[0] <= (wall_end[0] + SAFETY_RADIUS):
        if node[1] >= (wall_start[1] - SAFETY_RADIUS) and node[1] <= (wall_end[1] + SAFETY_RADIUS):
            if perpendicular_line_length >= SAFETY_RADIUS:
                available_nodes.append([node[0],node[1]])
        else: 
            available_nodes.append([node[0],node[1]])       
    else:
        available_nodes.append([node[0],node[1]])       
        
#print(f"available nodes are {available_nodes}")

for node in arena_nodes: # Look at each coodinate in turn
    plt.plot(node[0],node[1],'o', color = 'black') # scatter plot of all nodes

for node in available_nodes: # Look at each coordinate in turn
    plt.plot(node[0],node[1],'o', color = 'red') # scatter plot of available nodes

wall_x = [wall_start[0], wall_end[0]] # Get wall x values
wall_y = [wall_start[1], wall_end[1]] # Get wall y values
plt.plot(wall_x, wall_y) # Plot wall

plt.show() # Show plot in new figure







# # Plot all intiial arena nodes
# for node in arena_nodes: # Look at each coodinate in turn
#     plt.plot(node[0],node[1],'o', color = 'black') # scatter plot the nodes
# #plt.show() # show plot in new figure

# # Create wall_1 vector
# # wall_vector = array([x2-x1, y2-y1])
# wall_direction_vector = np.array([wall_end[0]-wall_start[0], wall_nd[1]-wall_start[1]])
# print(wall_direction_vector)
# #wall_vector = wall_1_start + A * wall_direction_vector
# #print(wall_direction_vector)
# #print(wall_vector)

# #numerator = (###)
# #numerator = (node - startPoint) dot wall direction vecoty
# numerator = np.dot((arena_nodes[0] - wall_1_start) , wall_direction_vector)
# denominatior = (np.linalg.norm(wall_direction_vector))**2
# A = numerator/denominatior
# print(A)


# Find neighbours of nodes
def neighbours(node): # Node is input
    directions = [[1, 0], [0, 1], [-1, 0], [0, -1]] # Create list of directions: Step right, step up, step left, step down
    result = [] # Initalise result
    for direction in directions: # For each option in directions list
        neighbour = [node[0] + directions[0], node[1] + directions[1]] # Look at node, and add direction x,y 
        if neighbour in arena_nodes: # If the neighbour nodes are actually within the arena boundaries
            result.append(neighbour) # Add them to the list off nodes
    return result # Return result (list of node, now including neighbours)








