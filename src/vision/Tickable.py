class Tickable:
    """Anything that extends this class will process the current frame every update cycle"""

    # Global list of things that can be updated
    tickables = []

    def __init__(self):
        Tickable.tickables.append(self)
    
    def update(self, frame):
        pass
        # Does nothing here, subclasses will use this to do whatever they do

    @staticmethod
    def update_all(frame):
        """Updates all tickable objects in the application"""
        for t in Tickable.tickables:
            t.update(frame)
