import numpy as np
import queue
from node import search_Node
from node import rrt_Node
from random import randint

def mh_distance(point1, point2):
    return abs(point1[0] - point2[0]) + abs(point1[1] - point2[1])

def distance(point1, point2):
    return np.sqrt((point1[0] - point2[0]) ** 2 + (point1[1] - point2[1]) ** 2)

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


class Robot_RRT_Graph:

    def __init__(self, root, object_size, obstacle_locations, goal_location, width, height):
        self.root = root
        self.object_size = object_size
        self.obstacle_locations = obstacle_locations
        self.goal_location = goal_location
        self.width = width
        self.height = height
        self.k_val = 10000
        self.nodes = [root]
        self.baseline_growth = np.sqrt(width**2 + height**2) / 4.0

    def search(self):
        goal_found = False
        for k in range(self.k_val):
            rand = randint(0, self.width * self.height)
            q_rand_location = [rand % self.width, rand / self.width]
            q_near = self.find_closest_node(q_rand_location)
            if q_near.distance_to_location(q_rand_location) is not 0:
                new_point = self.find_new_point(q_rand_location, q_near)
                if self.is_available(new_point):
                    if distance(self.goal_location, new_point) <= self.object_size:
                        q_near.add_child(rrt_Node(new_point, q_near.get_separation_from_root() + 1), goal=True)
                        goal_found = True
                        break
                    else:
                        q_near.add_child(rrt_Node(new_point, q_near.get_separation_from_root() + 1))
                else:
                    k -= 1
            else:
                k -= 1
        if goal_found:
            return self.get_path()
        else:
            return None


    def find_closest_node(self, other_location):
        closest_node = self.root
        dist = closest_node.distance_to_location(other_location)
        for node in self.nodes:
            new_dist = node.distance_to_location(other_location)
            if new_dist < dist:
                dist = new_dist
                closest_node = node
        return closest_node

    def find_new_point(self, location, node):
        normalized_vector = (node.get_location() - location) / np.abs(node.get_location() - location)
        relative_position = normalized_vector * self.baseline_growth * node.get_growth_factor()
        return (node.get_location + relative_position)

    def is_available(self, location):
        for obstacle_location in self.obstacle_locations:
            if distance(obstacle_location, location) < self.object_size:
                return False
        return True

    def get_path(self):
        x = self.root.get_path()
        if x is None:
            return None
        else:
            return x[::-1]
