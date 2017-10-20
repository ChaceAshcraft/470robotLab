import numpy as np

class search_Node:
    """
    A node class that represents a node in the search graph for
    the search project in 470. Needs to contain a location of the
    node, and a set of pointers to other nodes it is connected to.
    """

    def __init__(self, location, neighbors=[], goal=False):
        """
        param: location -- 2d vector denoting the top left corner of the node tile
        param: neighbors -- list of nodes this node is connected to
        param: goal -- True is the goal is in in this node, False otherwise
        
        """
        self.location = location
        self.neighbors = neighbors
        self.hasGoal = goal 
        self.isVisited = False
        self.searchParent = None
        self.searchPathLength = np.inf 

        self.priority = np.inf

    def expandPath(self):
        print("Expanding ", self.location) 
        if self.searchParent is None:
            print("got to root? ", self.location)
            return [self.location]
        else:
            temp = self.searchParent.expandPath()
            temp.append(self.location)
            print("not root", temp)
            return temp

    def addNeighbor(self, new_neighbor):
        self.neighbors.append(new_neighbor)
        new_neighbor.neighbors.append(self)

    def __lt__(self, other):
        return self.priority < other.priority
