

class search_Node:
    """
    A node class that represents a node in the search graph for
    the search project in 470. Needs to contain a location of the
    node, and a set of pointers to other nodes it is connected to.
    """

    def __init__(self, location, neighbors=None, obstacle=False, goal=False):
        """
        param: location -- 2d vector denoting the top left corner of the node tile
        param: neighbors -- list of nodes this node is connected to
        param: obstacle -- True is there is an obstacle in this node, False otherwise
        param: goal -- True is the goal is in in this node, False otherwise
        
        """
        self.location = location
        self.neighbors = neighbors
        self.hasObstacle = obstacle 
        self.hasGoal = goal 
        self.isVisited = False
        self.searchParent = None
        self.searchPathLength = 0

    def expandPath(self):
        if self.searchParent is None:
            return [self.center]
        else:
            return self.searchParent.expandPath().append(self.center)
