### Responsible for planning a route and monitoring/updating the Arduino whilst it is following the route

# A bunch of functions go here that interact with vision etc.

def start_route(target)
    """Starts moving the robot towards the given target"""
    # Initialise route object, find objects, calculate route

class Route:
    """Instances of this class represent a route to be followed"""

    def __init__(self, target)
        """Creates a new route that ends at the given target coordinates"""
        self.target = target
        self.current_step = 0 # Keeps track of which step in the route is currently being followed
        self.steps = [] # A list of coordinates describing the route (excludes start point)
        # Stuff

    def calculate_steps(self, start_point)
        # Work out the points along the route, fill in self.steps
        # Just do a straight line for now
        self.steps = [self.target]

    def advance(self)
        """Move this route to the next step"""
        self.current_step += 1

    def get_current_destination(self)
        """Returns the point at the end of the current step of the route"""
        return self.steps[self.current_step]

    def is_finished(self)
        """Returns true if all steps have been completed, false otherwise"""
        return self.current_step >= len(self.steps)