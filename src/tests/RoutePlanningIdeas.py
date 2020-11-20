##### Route Planning Code #####
#### Import libraries ####
import matplotlib.pyplot as plt
import numpy as np
from warnings import warn
import heapq

################################################## Fake Vision Outputs ##################################################
### Arena ###
## Arbitrary test setup ## –> EDIT THIS
x_length = 2500 # x - dimension
y_length = 1500 # y - dimension
arena_size = [x_length, y_length] # Create list

### M_O ###
M_O = [2150, 1150] # Position of robot M_O found by Vision

### Markers ###
marker_count = 3 # Number of markers to test
markers = [[0, 0, 0]] * marker_count # Initialise list
if marker_count == 0: # If no markers
    markers = [] # Empty list
if marker_count >= 1: # If 1 or more marker
    markers[0] = [1203, 600, 1] # Assign x0, y0, ID 
if marker_count >= 2: # If 1 or more marker
    markers[1] = [490, 790, 2] # Assign x1, y1, ID
if marker_count >= 3: # If 1 or more marker
    markers[2] = [2289, 300, 3] # Assign x2, y2, ID  
# ...

### Walls ###
wall_count = 3 # Number of walls to test
walls = [[0, 0, 0, 0]] * wall_count # Initialise list
if wall_count == 0: # If no walls
    walls = [] # Empty list
if wall_count >= 1: # If 1 or more wall
    walls[0] = [150, 150, 500, 500] # Assign x[0][0], y[0][0], x[0][1], y[0][1] 
if wall_count >= 2: # If 1 or more wall
    walls[1] = [890, 450, 620, 989] # Assign x[1][0], y[1][0], x[1][1], y[1][1] 
if wall_count >= 3: # If 1 or more wall
    walls[2] = [0, 0, 3, 3] # Assign x[2][0], y[2][0], x[2][1], y[2][1] 
# ...

### Fake Vision Output Function ###
def get_coords(): 
    """Route Planner will call up this function (in Vision) to return cords:
        - Arena_size [xEnd, yEnd]
        - M_O [x0,y0]
        - Markers [x0,y0]
        - Walls [x0,y0,x1,y1]
        ALSO NEEDS TO CHECK IF WALL_E HAS ARRIVED ON SCENE
        Also averages frames to see if WallE is moving"""
    ### Arena & Markers & Walls ###
    return arena_size, M_O, markers, walls

################################################## Actual Code ##################################################
### Set constants ###
STEP = 50 # Nodes are 50mm apart
SAFETY_RADIUS = 150 # Centre of sphere (M_O) always at least 150mm away from centre of obstacle

################################################## Mission Control Kick-Off ##################################################
def wheres_M_O():    
    """Mission Control will call up this function to kick off logic.
        Function needs to communicate with Vision to get coords:
        - Arena_size [xEnd, yEnd]
        - M_O [x0,y0]
        - Markers [x0,y0]
        ~ Walls [x0,y0,x1,y1] - Uneeded for this, but standard Vision output
        Function then returns:
        - M_O in limbo
        - OR, M_O at marker "X" """ 

    ### Get coordinates from Vision ### 
    arena_size, M_O, markers, walls = get_coords()

    ### Establish grid ### 
    nodes, grid = establish_grid(arena_size)

    ### Round markers to nodes ###


    ### Process coordinates ###
    ## If M_O close to a marker ##
    # M_O at marker "X"

    ## If M_O is not close to marker ##
    # M_O in limbo
    return limbo # Set as this for now

################################################## Mission Control General Command ######################################################
def M_O_go_here(ID):
    """Mission Control will call up this function to give M_O commands.
        Function needs to communicate with Vision to get coords:
        - M_O [x0,y0]
        - Markers [x0,y0]
        - Walls [x0,y0,x1,y1]
        Function calculates route in steps
        Function then tells Comms to move M_O in steps:
        - M_O here (constantly updating)
        - Go to step 1 
        - Confirm at step 1
        - Go to step 2, etc. till at target
        Function then returns location of M_O:
        - Marker "X" """
    
    ### Get coordinates from Vision ### 
    arena_size, M_O, markers, walls = get_coords()

    ### Establish grid ### 
    nodes, grid = establish_grid(arena_size)

    ### Round markers to grid ###
    markers = round_to_node(markers)
    
    ### Find safe travel zones ###
    nodes_safe, grid = find_safe_zone(arena_size, walls, grid)

    ### Convert ID to target location ###
    target = [0, 0]
    #print(markers)

    for marker in markers:
        #print(marker)
        print(ID)
        print(marker[2])
        if marker[2] == ID:
            
            target[0] = marker[0]
            target[1] = marker[1]
            #print('X')

    ### Find route between M_O and target ###
    route = a_star(M_O, target, grid)

    for node in nodes:
        plt.plot(node[0], node[1], '.', color='yellow')

    for node in nodes_safe:
        plt.plot(node[0], node[1], '.', color='blue')

    for marker in markers:
        plt.plot(marker[0], marker[1], 'o', color='orange')

    plt.plot(M_O[0], M_O[1], 'x', color='black')

    plt.plot(target[0], target[1], 'x', color='red')
    
    for step in route:
        plt.plot(step[0], step[1], '*', color='magenta')

    plt.show()

################################################## Internal Functions ##################################################
################################################## Establish grid ##################################################

def establish_grid(arena_size):
    """This function takes Vision's arena_size dimensions and turns this into a 
    list of nodes (establishing the coordinates) and then the grid (this is a 
    matrix of the nodes with a value of 0 or 1. 0 = Safe zone. 1 = Blockage.
    At this point, a matrix of ones is created. The find_safe_zone function will 
    assess where the safe grid points (nodes) are."""

    ### Setup list of nodes (x,y) ###
    x_nodes = list(range(0, arena_size[0] + 1, STEP)) # Linspace x by step size
    y_nodes = list(range(0, arena_size[1] + 1, STEP)) # Linspace y by step size

    ### Create list of nodes (x,y) ###
    nodes = [] # Initialise list
    for x in x_nodes:
        for y in y_nodes:
            nodes.append([x, y])

    ### Setup matrix of grid point values ###
    x_grid = len(x_nodes)
    y_grid = len(y_nodes)

    ### Create matrix of grid point values ###
    grid = np.ones([x_grid, y_grid])
    grid = grid.tolist() # Needs to be in list format for the A Star pathfinder

    return nodes, grid

################################################## Round Markers to Nodes ##################################################
def round_to_node(markers):
    """ """
    new_markers = []
    print(markers)
    #int(arena_step * round(float(x)/arena_step))
    for marker in markers:
        new_marker = []
        new_marker = marker[0]//STEP * STEP, marker[1]//STEP*STEP, marker[2]
        new_markers.append(new_marker)

    return new_markers
    
# def round_to_node(x):
#     return int(arena_step * round(float(x)/arena_step))

################################################## Find Safe Zone ##################################################

def find_safe_zone(arena_size, walls, grid):
    """This function takes the established grid, and assigns a 0 value to all the safe travel zones.

    Step One: Ignore all the nodes closest to the arena borders, thus leaving them as 1 (blocked)
    Step Two: In remaining nodes, look if nodes far enough away from walls, assign 0 (safe)
    Output list of safe nodes and updated grid values"""

    ### Establish list of nodes away from arena borders ###
    x_nodes_border_safe = list(range(0 + SAFETY_RADIUS, arena_size[0] + 1 - SAFETY_RADIUS, STEP)) # Apply safety distance to arena edge in x
    y_nodes_border_safe = list(range(0 + SAFETY_RADIUS, arena_size[1] + 1 - SAFETY_RADIUS, STEP)) # Apply safety distance to arena edge in y

    ### Create list of nodes away from arena borders (x,y) ###
    nodes_border_safe = [] # Initialise list
    for x in x_nodes_border_safe:
        for y in y_nodes_border_safe:
            nodes_border_safe.append([x, y])

    ### Create list of nodes away from wall and assign corresponding grid values to 0 ###
    nodes_safe = [] # Initialise list of safe nodes
    
    ## For each node in nodes safe list ##
    for node in nodes_border_safe:

        true_vector = [] # Initialise list: Safe or too close to wall check (0 = too close, 1 = safe)

        ## For each wall in walls list ##
        for wall in walls:

            ## Calculate direction vector of wall ##
            # For reference, equation of a vector in the form: vector equation = position vector + A * direction vector
            #                                                  wall_vector     = wall_position   + A * wall_direction
            #                                                 Direction vector = [(x1 - x0), (y1 - y0)]
            wall_direction = [wall[2]-wall[0], wall[3]-wall[1]] # Calculates direction vector of wall
            wall_direction = np.array(wall_direction) # Turns list into an array for later calcs.

            ## Establish x range of wall ##
            # Find lower limit of x in wall: Look at x0 and x1, and find which is lower
            if wall[0] <= wall[2]:
                x_lower_boundary = wall[0] - SAFETY_RADIUS
                x_upper_boundary = wall[2] + SAFETY_RADIUS
            else:
                x_lower_boundary = wall[2] - SAFETY_RADIUS
                x_upper_boundary = wall[0] + SAFETY_RADIUS

            ## Repeat above for y range of wall ##
            # Find lower limit of y in wall: Look at y0 and y1, and find which is lower
            if wall[1] <= wall[3]:
                y_lower_boundary = wall[1] - SAFETY_RADIUS
                y_upper_boundary = wall[3] + SAFETY_RADIUS
            else:
                y_lower_boundary = wall[3] - SAFETY_RADIUS
                y_upper_boundary = wall[1] + SAFETY_RADIUS   

            ## If end of walls outside arena (nodes list), bring range (the section we will assess) inside ##
            if x_lower_boundary < 0:
                x_lower_boundary = 0
            if x_upper_boundary > arena_size[0]:
                x_upper_boundary = arena_size[0] 
            if y_lower_boundary < 0:
                y_lower_boundary = 0
            if y_upper_boundary > arena_size[1]:
                y_upper_boundary = arena_size[1]     

            ## Calculate lambda (A) in vector equation of line between wall and node ##
            wall_start = [wall[0], wall[1]]
            wall_start = np.array(wall_start)
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
            nodes_safe.append(node) 
            point = [node[0],node[1]]
            new_point = convert_from_nodes(point)
            grid[new_point[0]][new_point[1]] = 0

    return nodes_safe, grid

################################################## A_Star ##################################################
################################################## Create Grid Point Class ##################################################

class grid_point:
    """ """

    ### Create class object ###
    def __init__(self, parent=None, position=None):

        ## Assign parent and position coordinate of grid_point ##
        self.parent = parent
        self.position = position

        ## Initialise parameters ##
        self.g = 0 # Distance between the current grid point and start node
        self.h = 0 # Heuristic - estimated distance between current grid point and end grid point
        self.f = 0 # Total cost of grid point - f = g + h

    def __eq__(self, other):
        return self.position == other.position
    
    def __repr__(self):
        return f"{self.position} - g: {self.g} h: {self.h} f: {self.f}"

    # defining less than for purposes of heap queue
    def __lt__(self, other):
        return self.f < other.f
        
    # defining greater than for purposes of heap queue
    def __gt__(self, other):
        return self.f > other.f
        
################################################## Find Route ##################################################
        
### Returns route between M_O and target ###
def return_route(current_grid_point):

    ## Initialise route ##
    route_grid = [] 
    route_nodes = []

    ## Backtrack grid points passed on route (parent of current grid point) and add to route list ##
    current = current_grid_point
    while current is not None:
        route_grid.append(current.position)
        current = current.parent
    route_grid[::-1]  # Return reversed path 

    for step in route_grid:
        step = convert_from_grid(step)
        route_nodes.append(step)
    
    return route_nodes

def a_star(M_O, target, grid):
    """ """

    M_O = convert_from_nodes(M_O)
    target = convert_from_nodes(target)

    print(M_O)
    print(target)
    
    ### Create start grid point = M_O and End grid point = target ####
    start_grid_point = grid_point(None, M_O) # M_O has no parent
    start_grid_point.g = start_grid_point.h = start_grid_point.f = 0 # Initalise parameters
    end_grid_point = grid_point(None, target) # target has no parent
    end_grid_point.g = end_grid_point.h = end_grid_point.f = 0 # Initalise parameters
       
    ### Initialize Open List and Closed List ###
    open_list = [] # Stores grid points to look at 
    closed_list = [] # Stores grid points already assessed

    ### Make Open list a priority queue by heapifying it ###
    heapq.heapify(open_list) # The open list will now be ordered with smallest value at [0] position

    ### Add the first grid point to the Priority Queue ###
    heapq.heappush(open_list, start_grid_point) # Heapush adds the start grid point and keeps the priority (ascending order)

    ### Adding a stop condition ###
    iterations = 0 # Initalise number of iterations completed
    max_iterations = len(grid[0]) * len(grid) // 2 # If every node has been checked and no solution found, stop
        #max_iterations = (len(grid[0]) * len(grid) // 2)

    #allow_diagonal_movement
    ### Establish directions of travel ###
    #directions = ((0, -1), (0, 1), (-1, 0), (1, 0)) # All possible 90deg. directions 
    #if allow_diagonal_movement:
    directions = ((0, -1), (0, 1), (-1, 0), (1, 0), (-1, -1), (-1, 1), (1, -1), (1, 1)) # All possible 45deg. directions 

    ### Search grid points until target (end grid point) found ###
    while len(open_list) > 0:

        ## Update iteration number ##
        iterations += 1

        if iterations > max_iterations:
            ## Return route found up to this point ## - CHANGE THIS LATER, IF FAILURE, SHOULD TRY TO MOVE SOMEWHERE ELSE
            warn("Route not found, too many iterations")
            return return_route(current_grid_point)   

        ## Remove the current grid point from the Open List (to assess list) ##
        current_grid_point = heapq.heappop(open_list)

        ## Add the current gird point to the Closed List (already assessed list) ##
        closed_list.append(current_grid_point)

        ## Check if target reached, if so return the route ##
        if current_grid_point == end_grid_point:
            return return_route(current_grid_point)

        ## Generate children (adjacent grid points to current based on direction options) ##
        children = [] # Initilise list of grid points
        
        ### Create children grid points from current grid points ###
        for direction in directions: # For all adjacent grid points (all direction options from current grid point)

            ## Update the grid point coordinate ##
            grid_point_position = (current_grid_point.position[0] + direction[0], current_grid_point.position[1] + direction[1])

            ## If the grid point coordinate is outside the list, move on to next grid point ##
            if grid_point_position[0] > (len(grid) - 1) or grid_point_position[0] < 0 or grid_point_position[1] > (len(grid[len(grid)-1]) -1) or grid_point_position[1] < 0:
                continue

            ## If the grid point is blocked, e.g. by wall, move on to next grid point ##
            if grid[grid_point_position[0]][grid_point_position[1]] != 0:
                continue

            ## Create new grid point ##
            new_grid_point = grid_point(current_grid_point, grid_point_position)

            ## Append list of children ##
            children.append(new_grid_point)

        ### Assess the quality of each child grid point, and add to open list in order of quality ###
        for child in children: # For all adjacent grid points identified which were in a safe location
            
            ## If child already on the closed list, move on to next grid point ##
            if len([closed_child for closed_child in closed_list if closed_child == child]) > 0:
                continue

            ## Calculate the f, g, and h values for children grid points ##
            child.g = current_grid_point.g + 1
            child.h = ((child.position[0] - end_grid_point.position[0]) ** 2) + ((child.position[1] - end_grid_point.position[1]) ** 2)
            child.f = child.g + child.h

            ## If child is already on the open list, move on to next grid point ##
            if len([open_grid_point for open_grid_point in open_list if child.position == open_grid_point.position and child.g > open_grid_point.g]) > 0:
                continue

            ## Add the child to the open list ##
            heapq.heappush(open_list, child) # Heapush adds the child grid point and keeps the priority (ascending order)

    warn("Route has not been found")
    return None

################################################## Convert from grid to nodes ##################################################

def convert_from_grid(point):
    
    converted_point = (int(point[0] * STEP), int(point[1] * STEP))
    return converted_point

################################################## Convert from nodes to grid ##################################################    

def convert_from_nodes(point):

    converted_point = (point[0] // STEP, point[1]// STEP)
    return converted_point

################################################## Testing ##################################################

ID = 2
M_O_go_here(ID)

