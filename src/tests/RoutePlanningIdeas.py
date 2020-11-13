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

#### First step is creating a grid in graph form ####
## Manually define inputs from vision ##

# Create arbitrary arena
x_arena = 20 # Set x distance of arena to ...
y_arena = 10 # Set y distance of arena to ...

# Create arbitrary horizonal wall
wall_1_start = [2,1] 
wall_1_end = [12,5]

# Create grid of nodes 
arena_nodes = []
for x in range(x_arena):
    for y in range(y_arena):
        arena_nodes.append([x,y])
arena_nodes = np.array(arena_nodes)      

print(arena_nodes[0])
# Plot all intiial arena nodes
for node in arena_nodes: # Look at each coodinate in turn
    plt.plot(node[0],node[1],'o', color = 'black') # scatter plot the nodes
#plt.show() # show plot in new figure

# Create wall_1 vector
# wall_vector = array([x2-x1, y2-y1])
wall_direction_vector = np.array([wall_1_end[0]-wall_1_start[0], wall_1_end[1]-wall_1_start[1]])
print(wall_direction_vector)
#wall_vector = wall_1_start + A * wall_direction_vector
#print(wall_direction_vector)
#print(wall_vector)

#numerator = (###)
#numerator = (node - startPoint) dot wall direction vecoty
numerator = np.dot((arena_nodes[0] - wall_1_start) , wall_direction_vector)
denominatior = (np.linalg.norm(wall_direction_vector))**2
A = numerator/denominatior
print(A)


# Find neighbours of nodes
def neighbours(node): # Node is input
    directions = [[1, 0], [0, 1], [-1, 0], [0, -1]] # Create list of directions: Step right, step up, step left, step down
    result = [] # Initalise result
    for direction in directions: # For each option in directions list
        neighbour = [node[0] + directions[0], node[1] + directions[1]] # Look at node, and add direction x,y 
        if neighbour in arena_nodes: # If the neighbour nodes are actually within the arena boundaries
            result.append(neighbour) # Add them to the list off nodes
    return result # Return result (list of node, now including neighbours)








