#####  #####  Import required libraries  ##### #####
# import matplotlib.pyplot as plt
import numpy as np
import math
import heapq
from warnings import warn

import vision
import motion_controller

# !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!! # 
# !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!! # 
# !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!! # 
# !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!! # 
# !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!! # 

########## FOR REAL CODE, COMMENT OUT ALL THE BELLOW UP TO NEXT !!!!! SECTION. THESE VALUES SHOULD ALL COME FROM A 
########## VISION FUNCTION. THEN SEARCH THROUGH ACTUAL CODE AND COMMENT OUT ANY GET_COORDS LINES FOR THE VISION.GET_CORDS
########## DONT DELETE THE LINES, JUST COMMENT OUT!!! 

# !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!! # 
# !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!! # 
# !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!! # 
# !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!! # 
# !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!! # 


#####  ##### Fake Vision #####  #####
#####  Arena  #####
# x_length = 2500 # x dimension of floorplan - EDIT THIS
# y_length = 2500 # y dimension of floorplan - EDIT THIS
# arena_size = [x_length, y_length] # Create list in Vision output form

# #####  M_O  #####
# M_O = [2300, 250, 0] # M_O's initial position coord and ID - EDIT THIS

#####  Markers  #####
# marker_count = 7 # Number of markers to test - EDIT THIS
# markers = [[0, 0, 0]] * marker_count # Initialise list
# if marker_count == 0: # If not markers...
#     markers = [] # Empty list
# if marker_count >= 1: # If 1 or more marker
#     markers[0] = M_O # Initiate list with M_O's coords and ID
# if marker_count >=2: # If 2 or more marker
#     markers[1] = [500, 2000, 1] # Assign [x,y,ID] of marker - EDIT THIS
# if marker_count >=3: # If 3 or more marker
#     markers[2] = [1500, 1200, 2] # Assign [x,y,ID] of marker - EDIT THIS
# if marker_count >=4: # If 2 or more marker
#     markers[3] = [2250, 1500, 3] # Assign [x,y,ID] of marker - EDIT THIS
# if marker_count >=5: # If 3 or more marker
#     markers[4] = [1300, 1750, 4] # Assign [x,y,ID] of marker - EDIT THIS
    
#     ## WALL_E
# if marker_count >= 7: # If 3 or more marker
#     markers[5] = [500, 1000, 5] # Assign [x,y,ID] of marker - EDIT THIS
#     markers[6] = [500, 1250, 6]

#####  Walls  #####
# wall_count = 5 # Number of walls to test - EDIT THIS
# walls = [[0, 0, 0, 0]] * wall_count # Initialise list
# if wall_count == 0: # If no walls...
#     walls = [] # Empty list
# if wall_count >= 1: # If 1 or more wall
#     walls[0] = [500, 1500, 1500, 1500] # Assign x[0][0], y[0][0], x[0][1], y[0][1]  [500, 300, 2500, 500]
# if wall_count >= 2: # If 1 or more wall
#     walls[1] = [500, 500, 2000, 500] # Assign x[1][0], y[1][0], x[1][1], y[1][1]   [0, 500, 1750, 1000]
# if wall_count >= 3: # If 1 or more wall
#     walls[2] = [1000, 1000, 1000, 2000] # Assign x[2][0], y[2][0], x[2][1], y[2][1]    [400, 1000, 2300, 1900]
# if wall_count >= 4: # If 1 or more wall
#     walls[3] = [1500, 2000, 2250, 2000] # Assign x[2][0], y[2][0], x[2][1], y[2][1]    [300, 2200, 2300, 1700]
# if wall_count >= 5: # If 1 or more wall
#     walls[4] = [2000, 2000, 2000, 500] # Assign x[2][0], y[2][0], x[2][1], y[2][1]    [300, 2200, 2300, 1700]
# # ... if more walls to test

##### wall_E? #####
# wall_E_here = 1
# wall_E_moving = 0

#####  Fake Vision Outputs  #####
# def get_coords(): 
#     """
#     I need: 
#     - arena_size = [xEnd, yEnd]
#         where: 
#         - xEnd = distance (mm) between the origin and the furthest x coordinate of arena
#         - yEnd = distance (mm) between the origin and the furthest y coordinate of arena
#     - markers = [x0, y0, ID]
#         where:
#         - Row 0, must be [M_O's x coordinate, M_O's y coordinate, 0] MUST BE 0 FOR ID (ARTIFICALLY DO THIS IF NOT!)
#         - Row 1, must be [Pick-up x coordinate, Pick-up y coordiante, 1] AGAIN, MUST BE 1 FOR ID
#         - Row 2, must be [Delivery point A x coordinate, Delivery point A y coordinate, 2] AGAIN, MUST BE 2 FOR ID
#         - Row 3, must be [Delivery point B x coordinate, Delivery point B y coordinate, 3] AGAIN, MUST BE 3 FOR ID
#         - Row 4, must be [Delivery point C x coordinate, Delivery point C y coordinate, 4] AGAIN, MUST BE 4 FOR ID
#         - Row 5, must be [Wall_E centre x coordinate, Wall_E centre y coordinate, 5] AGAIN, MUST BE 5 FOR ID
#         - Row 6, must be [Wall_E offset x coordinate, Wall_E offset y coordinate, 6] AGAIN, MUST BE 6 FOR ID
#         ^ !!!! And if Wall_E is not there, only output a list of 0-4 (exclude the Wall-E rows) !!!!
#     - walls = [x0, y0, x1, y1]
#         where:
#             - Row 0 is the info of one wall
#             - Row 1 is the info of the next wall etc. 
#     - wall_E_here
#         where: 
#         - This is a 0 if Wall_E's centre not in the arena bounds
#         - This is a 1 if Wall_E's centre is in the arena bounds
#     - wall_E_moving
#         where:
#         - This is a 0 if Wall_E is stopped
#         - This is a 1 if Wall_E is moving
#                 """

#     # Return fake outputs of Vision
#     return arena_size, markers, walls, wall_E_here, wall_E_moving

# !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!! # 
# !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!! # 
# !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!! # 
# !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!! # 
# !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!! # 





#####  #####  Actual Code  #####  #####
#####  Set constants  #####  
STEP = 50 # Distance between nodes (mm)
SAFETY_RADIUS = 140 # Distance between centre of M_O and obstacle (mm)
PROXIMITY_RADIUS = 150 #math.sqrt(2 * STEP ** 2) # Centre of M_O to marker (mm) - made so can be any adjacent node
WALL_E_CLEARANCE_RADIUS = 240 # Clearance between M_O and Wall_E centres

        # #####  Main Function Structure #####  
        # def M_O_go_here(ID):

        #     # Initialise target_reached
        #     target_reached = 0

        #     # If target has not been reached
        #     while target_reached == 0:

        #         # Initialise wall_E_check
        #         wall_E_check = 0

        #         # If wallE not here, go to target
        #         while wall_E_check == 0:
        #             # Move to target
        #             # Return target_reached == 1

        #         # If wallE here
        #         while wall_E_check == 1:
                    
        #             # And if wallE is stopped
        #             while wall_E_moving == 0:
        #                 # Move to wall_E offset
        #                 # Move to wall_E
    
#####  Main Function  #####  
def M_O_go_here(ID):

    # Initialise target_reached
    target_reached = 0

    # If target has not been reached
    while target_reached == 0:

        # Initialise wall_E_check
        wall_E_here = 0

        # If wallE not here, go to target
        while wall_E_here == 0:

            # Get outputs from Vision - Same for wall_E
            arena_size, markers, walls, wall_E_here, wall_E_moving = vision.get_coords()

            # Establish nodes/grid - Same for wall_E
            nodes, grid = establish_grid(arena_size)

            # Round markers to nodes/grid-points - Same for wall_E
            rounded_markers = round_markers(markers)

            # Assess where M_O is (needs to be within area for marker, but at node for wall_E) - Same for wall_E
            location = where_is_M_O(rounded_markers) 

            # If M_O already at target ID, target_reached = 1 - Change this for wall_E
            if location == ID:
                target_reached = 1
                print("Route completed")
                break

            # Work out safe travel nodes - Same for wall_E
            nodes_safe, grid = find_safe_zones(nodes, grid, arena_size, walls, rounded_markers)

            # Work out route to target (ID for normal, ID = 6 for Wall_E)
            route = route_plan(rounded_markers, ID, grid, nodes_safe)
            
            if route == []: 
                target_reached = 1 # If no route was found, skip this destination (exit the M_O_go_here function)
                print(f"Cannot find route, skipping target {ID}")
                break

            # Turn the route into steps 
            steps = get_steps(route)
            
            # Move to each step
            for step in steps:
                _, _, _, wall_E_here, wall_E_moving = vision.get_coords()
                if wall_E_here == 1: break
                # Move to this step
                motion_controller.go_to(step)

        print("Wall-E detected!")

        flag = True

        # If wallE here
        while wall_E_here == 1 and flag:
            
            _, _, _, wall_E_here, wall_E_moving = vision.get_coords()

            # And if wallE is stopped
            while wall_E_moving == 0:
                
                # Get outputs from Vision - Same for wall_E
                arena_size, markers, walls, wall_E_here, wall_E_moving = vision.get_coords()

                # Establish nodes/grid - Same for wall_E
                nodes, grid = establish_grid(arena_size)

                # Round markers to nodes/grid-points - Same for wall_E
                rounded_markers = round_markers(markers)

                # Assess where M_O is (needs to be within area for marker, but at node for wall_E) - Same for wall_E
                location = where_is_M_O(rounded_markers) 

                # If M_O already at target ID, target_reached = 1 - Change this for wall_E
                if location == 6: # If at Wall_E offset
                    motion_controller.nudge_forward()
                    flag = False
                    break

                # Work out safe travel nodes - Same for wall_E
                nodes_safe, grid = find_safe_zones(nodes, grid, arena_size, walls, rounded_markers)

                # Work out route to target (ID for normal, ID = 6 for Wall_E)
                target = 6
                route = route_plan(rounded_markers, target, grid, nodes_safe)

                ##### NOT SURE WHAT TO DO HERE IF CANNOT GET TO WALLE
                if route == []: 
                    target_reached = 1 # If no route was found, skip this destination (exit the M_O_go_here function)
                    print(f"Cannot find route, skipping target {ID}")
                    flag = False
                    break

                # Turn the route into steps - Same for wall_E
                steps = get_steps(route)

                # Move to each step - Same for wall_E
                for step in steps:
                    _, _, _, wall_E_here, wall_E_moving = vision.get_coords()
                    # Move to this step
                    motion_controller.go_to(step)

                # Move M_O to hit Wall_E

                # for node in nodes:
                #     plt.plot(node[0], node[1], 's', color='black')
                # for node in nodes_safe:
                #     plt.plot(node[0], node[1], 's', color='white')
                # for rounded_marker in rounded_markers[1:]:
                #     plt.plot(rounded_marker[0], rounded_marker[1], 'x', color='green')   
                # plt.plot(rounded_markers[0][0], rounded_markers[0][1], 'o', color='magenta') 
                # for node in route:
                #     plt.plot(node[0], node[1], '.', color='red')   
                # for step in steps:
                #     plt.plot(step[0], step[1], '.', color='blue')  
                # plt.show()

#####  #####  Mini Functions  #####  #####
##### Turn arena into list of nodes and matrix of gridpoints #####
def establish_grid(arena_size):

    # Create list of nodes
    x_nodes = list(range(0, arena_size[0] + 1, STEP)) # Linspace x by step size
    y_nodes = list(range(0, arena_size[1] + 1, STEP)) # Linspace x by step size
    nodes = []
    for x in x_nodes:
        for y in y_nodes:
            nodes.append([x,y])

    # Create corresponding grid
    x_grid = len(x_nodes)
    y_grid = len(y_nodes)
    grid = np.ones([x_grid, y_grid])

    return nodes, grid

#####  Round markers to closest nodes/grid-points  #####
def round_markers(markers):

    # Initialise list of rounded markers
    rounded_markers = []

    for marker in markers:
        rounded_marker = STEP * round(marker[0] / STEP), STEP * round(marker[1] / STEP), marker[2]
        rounded_markers.append(rounded_marker)

    return rounded_markers

#####  Check distance between M_O and target  #####
def where_is_M_O(rounded_markers): # In wall_E, change ID to Wall_E ID

    # For the markers 1-4 (pick-up and delivery), need to be within proximity radius
    location = []
    index = list(range(1, 5))
    for i in index:
        distance = math.sqrt(((rounded_markers[i][0] - rounded_markers[0][0]) ** 2) + ((rounded_markers[i][1] - rounded_markers[0][1]) ** 2))
        if distance <= PROXIMITY_RADIUS: # Is M_O close enough to a marker?
            location = rounded_markers[i][2] 
        if location == []: # If not close enough to any markers, location is simply M_O's coords. 
            location = rounded_markers[0][2]
        
    # For the Wall_E marker (6 offset), need to be actually at correct node
    if len(rounded_markers) == 7: # If Wall_E actually in list outputted by Vision
        distance = math.sqrt(((rounded_markers[6][0] - rounded_markers[0][0]) ** 2) + ((rounded_markers[6][1] - rounded_markers[0][1]) ** 2))
        if distance <= 1 * STEP: # M_O needs to be at actual node, so within half a step of Wall_E offset
            location = rounded_markers[6][2] # If M_O close enough to Wall_E
    
    return location

#####  Find safe nodes/grid points based on borders, walls and Wall_E  #####  
def find_safe_zones(nodes, grid, arena_size, walls, rounded_markers):

    print(f"Processing arena safe zones...")

    # Find nodes which avoid arena borders
    nodes_border_safe = []
    for node in nodes:
        if node[0] >= SAFETY_RADIUS and node[0] <= (arena_size[0] - SAFETY_RADIUS):
            if node[1] >= SAFETY_RADIUS and node[1] <= (arena_size[1] - SAFETY_RADIUS):
                nodes_border_safe.append(node)

    # Find nodes which avoid crashing into Wall_E
    if len(rounded_markers) == 7: # If Wall_E actually on the map

        wall_E_safe_nodes = []
        # Take the centre of wall_E (5 - centre)
        # Take the surrounding nodes as unsafe (WALL_E_CLEARANCE_RADIUS)
        for node in nodes_border_safe:
            distance = math.sqrt(((rounded_markers[5][0] - node[0]) ** 2) + ((rounded_markers[5][1] - node[1]) ** 2))
            if distance > WALL_E_CLEARANCE_RADIUS:
                wall_E_safe_nodes.append(node)
                # point = [node[0],node[1]]
                # new_point = convert_from_nodes(point)
                # grid[new_point[0]][new_point[1]] = 0 # Set grid point to 0 = safe
    else:
        wall_E_safe_nodes = nodes_border_safe

    # Find nodes which avoid walls 
    nodes_safe = []
    for node in wall_E_safe_nodes:

        true_vector = [] # Initialise list: Safe or too close to wall check (0 = too close, 1 = safe)

        # For each wall in walls list
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

        if all(true_vector): # If node satisfies all above safety conditions, add to list of safe nodes
            nodes_safe.append(node) 
            point = [node[0],node[1]]
            new_point = convert_from_nodes(point)
            grid[new_point[0]][new_point[1]] = 0 # Set grid point to 0 = safe

    return nodes_safe, grid

#####  Convert a grid point to node coords  #####  
def convert_from_grid(point):
    
    converted_point = (int(point[0] * STEP), int(point[1] * STEP))
    return converted_point  

#####  Convert a node coord to gridpoint position  #####  
def convert_from_nodes(point):

    converted_point = (point[0] // STEP, point[1]// STEP)
    return converted_point

def route_plan(rounded_markers, target, grid, nodes_safe):

    # Target = ID when normal mode
    # Target = 6 when Wall_E - offset node

    M_O_node = [rounded_markers[0][0], rounded_markers[0][1]]
    end_node = [rounded_markers[target][0], rounded_markers[target][1]] # Turn into a point, as a_star function reused for simple coords later

    print(f"Planning route to target point {target} at position {end_node}...")

    route = a_star(M_O_node, end_node, grid)

    if route == "Fail": # If unable to find route, find nearest safe corner

        warn("Cannot find direct route, attempting to navigate to corner...")
        
        safe_corners = get_corners(nodes_safe) # Find coord of safe corners
        # closest_corner = get_closest_corner(safe_corners, rounded_markers) # Closest corner coord

        route_found = 0

        for corner in safe_corners:

            if route_found == 0: # If no route found

                route_to_corner = a_star(M_O_node, corner, grid)

                if route_to_corner != "Fail": # If route to corner found

                    route_to_target = a_star(corner, end_node, grid) # Calculate route to target

                    if route_to_target != "Fail": # If route to target found

                        route_found = 1 # Set check value to true, ending for loop
                        arrived = 1 # Let Mission Control know it worked 
                        route = route_to_corner + route_to_target
    
    if route != "Fail" and route != []:

        route_found = 1

    if route_found == 1:
        return route
    if route_found == 0:
        route = []
        return route

#####   Get route to node   #####  
def a_star(M_O_node, end_node, grid):

    M_O = convert_from_nodes(M_O_node)
    end = convert_from_nodes(end_node)

    ### Create start grid point = M_O and End grid point = target ####
    start_grid_point = grid_point(None, M_O) # M_O has no parent
    start_grid_point.g = start_grid_point.h = start_grid_point.f = 0 # Initalise parameters
    end_grid_point = grid_point(None, end) # target has no parent
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
    # max_iterations = len(grid[0]) * len(grid) # If every node has been checked and no solution found, stop
    max_iterations = (len(grid[0]) * len(grid) // 2)

    ### Establish directions of travel ###
    # Non-diagonal option:
    #directions = ((0, -1), (0, 1), (-1, 0), (1, 0)) # All possible 90deg. directions 
    # Diagonal option:
    directions = ((0, -1), (0, 1), (-1, 0), (1, 0), (-1, -1), (-1, 1), (1, -1), (1, 1)) # All possible 45deg. directions 

    ### Search grid points until target (end grid point) found ###
    while len(open_list) > 0:

        ## Update iteration number ##
        iterations += 1

        if iterations > max_iterations:
            ## Return route found up to this point ## - CHANGE THIS LATER, IF FAILURE, SHOULD TRY TO MOVE SOMEWHERE ELSE
            warn("Unable to find route at mid-point of code")
            route = "Fail"
            return route

        ## Remove the current grid point from the Open List (to assess list) ##
        current_grid_point = heapq.heappop(open_list)

        ## Add the current gird point to the Closed List (already assessed list) ##
        closed_list.append(current_grid_point)

        ## Check if target reached, if so return the route ##
        if current_grid_point == end_grid_point:
            route = return_route(current_grid_point)   
            return route

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

    warn("Unable to find route at end of code")
    route = "Fail"
    return route

#####   Creat grid-point class to store parameter values   #####  
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
    #print(f"route_grid = {route_grid}")

    for step in route_grid:
        step = convert_from_grid(step)
        route_nodes.append(step)
    route_nodes.reverse()

    return route_nodes

#####  Find safe corners for M_O to move to ##### 
def get_corners(nodes_safe):
    
    x_list = [] # Initialise list
    y_list = [] # Initialise list
    
    for node in nodes_safe:
        x_list.append(node[0])

    x_min = min(x_list)

    for node in nodes_safe: # Look at each node
        if node[0] == x_min: # If on LHS
            y_list.append(node[1]) # Get list of y values

    y_min = min(y_list) # Find minimum LHS y value
    y_max = max(y_list) # Find maximum LHS y value

    bottom_left = [x_min, y_min]
    top_left = [x_min, y_max]

    y_list = [] # Initialise list

    x_max = max(x_list) # Find maximum x value
    for node in nodes_safe: # Look at each node
        if node[0] == x_max: # If on RHS
            y_list.append(node[1]) # Get list of y values

    y_min = min(y_list) # Find minimum RHS y value
    y_max = max(y_list) # Find maximum RHS y value
    
    bottom_right = [x_max, y_min]
    top_right = [x_max, y_max]

    safe_corners = bottom_left, top_left, bottom_right, top_right

    return safe_corners

#####  Find closest safe corner to travel to ##### 
def get_closest_corner(safe_corners, rounded_markers):

    index = list(range(0, len(safe_corners)))
    corner_distances = []
    distances = []

    for i in index:
        # Calculate distance between M_O and each safe corner
        distance = math.sqrt(((safe_corners[i][0] - rounded_markers[0][0]) ** 2) + ((safe_corners[i][1] - rounded_markers[0][1]) ** 2))
        corner_distances.append([safe_corners[i][0], safe_corners[i][1], distance]) # Store info of corner coord and distance
        distances.append(distance)

    for corner in corner_distances:
        if min(distances) == corner[2]: # If smallest distance, is the specific corner's distance
            closest_corner = [corner[0], corner[1]]  # Set this as the closest corner

    return closest_corner

def get_steps(route):
    
    steps = []
    index = list(range(1, (len(route) - 2))) # 1 because we're ignoring the 1st point (the start point)

    for i in index:
        # Calculate magnitude of x and y current jumps
        x_current_jump = route[i][0] - route[i-1][0] 
        y_current_jump = route[i][1] - route[i-1][1] 

        # Calculate magnitude of x and y next jumps
        x_next_jump = route[i+1][0] - route[i][0]
        y_next_jump = route[i+1][1] - route[i][1]

        # If direction changes between current and next jump, output a step
        if x_current_jump != x_next_jump or y_current_jump != y_next_jump: 
            step = route[i][0], route[i][1]
            steps.append(step)

    steps.append([route[-1][0], route[-1][1]])

    return steps

################################################
#####  #####  Fake Mission Control  #####  #####
destinations = [1, 2, 1, 3, 1, 4, 1]
#destinations = [3]
for destination in destinations:
    M_O_go_here(destination)

print("Finished!")


