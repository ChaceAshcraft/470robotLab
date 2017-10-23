import numpy as np

class search_Node:
    """
    A node class that represents a node in the search graph for
    the search project in 470. Needs to contain a location of the
    node, and a set of pointers to other nodes it is connected to.
    """

    def __init__(self, location, neighbors=None, goal=False):
        """
        param: location -- 2d vector denoting the top left corner of the node tile
        param: neighbors -- list of nodes this node is connected to
        param: goal -- True is the goal is in in this node, False otherwise

        """
        self.location = location
        if neighbors is None:
            self.neighbors = []
        else:
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
        #print("New Add neighbor call")
        #print("Self: ", self.location)
        #print("neighbor: ", new_neighbor.location)
        #print("\n")
        #print("Before: \n", self.neighbors, '\n', new_neighbor.neighbors)
        self.neighbors.append(new_neighbor)
        #print("Between: \n", self.neighbors, '\n', new_neighbor.neighbors)
        new_neighbor.neighbors.append(self)
        #print("After: \n", self.neighbors, '\n', new_neighbor.neighbors)
        #input("Enter to continue")

    def __lt__(self, other):
        return self.priority < other.priority

class rrt_Node:

    def __init__(self, location, dist_from_root, children=None, goal=False):
        self.location = location
        self.distance_from_root = dist_from_root
        if children is None:
            self.children = []
        else:
            self.children = children
        self.goal = goal

    def distance_to_location(self, other_location):
        return np.sqrt((self.location[0] - other_location[0]) ** 2 + (self.location[1] - other_location[1]) ** 2)

    def get_separation_from_root(self):
        return self.distance_from_root

    def get_growth_factor(self):
        return .9**self.distance_from_root

    def get_location(self):
        return self.location

    def add_child(self, other_node):
        self.children.append(other_node)

    def get_path(self):
        if self.goal is True:
            print("Reached the goal!")
            return [self.location]
        else:
            for child in self.children:
                path = child.get_path()
                if path is not None:
                    print("Path not none: ", path, " Here's self: ", self)
                    path.append(self.location)
                    print("New path: ", path)
                    return path
            return None
