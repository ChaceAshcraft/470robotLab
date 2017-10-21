import numpy as np
import queue
from node import search_Node

def mh_distance(point1, point2):
    return abs(point1[0] - point2[0]) + abs(point1[1] - point2[1])

class Robot_AStar_Graph:
    """
    The A* graph for our 470 search project. This class should
    contain the nodes of the graph, a pointer to the 'root'
    (probably the robot location in this case), and the methods
    to do the A* search.
    """

    def __init__(self, root, goal_location, heuristic_function=None):
        """
        Class constructor.
        param: root -- search_node object representing the location of the robot
        param: goal_location -- 2d vector (list) indicating x,y location of the goal
        param: heuristic_function -- function to be called on a search_node object
                representing the h function of the A* search, default is distance to
                goal_location
        """
        self.root = root
        self.goal_location = goal_location
        if heuristic_function is not None:
            self.H = heuristic_function
        else:
            # Assign the heuristic function to be the distance between the node
            # location and the goal location
            self.H = lambda x: mh_distance(x.location, self.goal_location)

    def search(self):
        """
        Starting from root node, perform an A* search of the graph until a node
        with the value hasGoal is true. Returns the path to the goal if one is 
        found.
        returns: list of nodes with the 0 index being the root and the -1 index
                being the goal node.
        """
        goalNode = None
        if self.root.hasGoal:
            return [self.root.center]
        PQ = queue.PriorityQueue()
        self.root.isVisited = True
        self.root.searchPathLength = 0
        self.root.priority = 0
        PQ.put(self.root)
        while not PQ.empty() and goalNode is None:
            n = PQ.get() # PQ.get() returns a tuple (priority, node)
            for neighbor in n.neighbors:
                if not neighbor.isVisited:
                    print("visited ", neighbor.isVisited)
                    print(neighbor.location)
                    neighbor.isVisited = True
                    neighbor.searchParent = n
                    neighbor.searchPathLength = (n.searchPathLength + 
                                        mh_distance(n.location, neighbor.location))
                    if neighbor.hasGoal:
                        goalNode = neighbor
                        print("search found a goal node!")
                        break
                    else:
                        neighbor.priority = neighbor.searchPathLength + self.H(neighbor)
                        PQ.put(neighbor)

        if goalNode is not None:
            print("not none!")
            return goalNode.expandPath()
        else:
            return None
