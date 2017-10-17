import numpy as np
from node import search_node

def distance(point1, point2):
    return np.sqrt((point1[0] - point2[0]) ** 2 + (point1[1] - point2[1]) ** 2)

class Robot_AStar_Graph:
    """
    The A* graph for our 470 search project. This class should
    contain the nodes of the graph, a pointer to the 'root'
    (probably the robot location in this case), and the methods
    to do the A* search.
    """

    def __init__(self, nodes, root, goal_location, heuristic_function=None):
        """
        Class constructor. 
        param: nodes -- list (or dictionary?) of search_node objects
        param: root -- search_node object representing the location of the robot
        param: goal_location -- 2d vector (list) indicating x,y location of the goal
        param: heuristic_function -- function to be called on a search_node object
                representing the h function of the A* search, default is distance to
                goal_location
        """
        self.nodes = nodes
        self.root = root
        self.goal_location = goal_location
        if heuristic_function is not None:
            self.H = heuristic_function
        else:
            # Assign the heuristic function to be the distance between the node
            # location and the goal location
            self.H = lambda x: distance(x.location, self.goal_location)

    def search(self):
        """
        Starting from root node, perform an A* search of the graph until a node
        with the value hasGoal is true. Returns the path to the goal if one is 
        found.
        returns: list of nodes with the 0 index being the root and the -1 index
                being the goal node.
        """
        pass
