import numpy as np


class search_Node:
    """
    A node class that represents a node in the search graph for
    the search project in 470. Needs to contain a location of the
    node, and a set of pointers to other nodes it is connected to.
    """

    def __init__(self, location, neighbors=None, obstacle=False, goal=False):
        self.location = location
        self.neighbors = neighbors
        self.hasObstacle = obstacle 
        self.hasGoal = goal 
