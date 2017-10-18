import asyncio
import sys
import json
from math import atan2, sin, cos, pi
from time import sleep
import numpy as np
from graph import Robot_AStar_Graph
from node import search_Node

def main(host='localhost', port=55555, goal="I0"):
    """
    This file is a copy of 'guide.py' in the examples, modified to have the
    robot navigate from a starting position to a goal through a field of
    obstacles.

    Note that we always expect arugments to the main function from the
    command line. In particular, the goal value should be the marker
    label that the robot is seeking.

    """

    # Setup asyncio and open the connection
    loop = asyncio.get_event_loop()
    reader, writer = loop.run_until_complete(
        asyncio.open_connection(host, port))

    # Simple method to process a command. It's defined inside the main
    # method so reader and writer are in its namespace
    def do(command):
        print('>>>', command)

        # Send the command -- write() expect bytes, so we call encode()
        writer.write(command.strip().encode())

        # This is a lot in one line, but here's what's happening:
        #   reader.readline() starts an asyncio coroutine that reads
        #     until it gets a complete line (ending with \n) and then
        #     returns that coroutine.
        #   run_until_complete() runs the coroutine until it terminates
        #   decode() turns the bytes object into a string
        #   strip() removes whitespace at the beginning or end, like \n
        res = loop.run_until_complete(reader.readline()).decode().strip()
        print('<<<', res)
        try:
            # The response is a json encoded string, so we decode it
            res = json.loads(res)
        except json.decoder.JSONDecodeError:
            # If an error occurred, handle it gracefully
            print('Error decoding response')
            res = {}
        print()
        # Return the resulting dict
        return res

    def calc_angle(x, y):
        return atan2(-y, x)  # Reversed for upside-down y

    def calc_angle2(x, y):
        return atan2(y, x)

    # Sometimes, when taking differences between angles, you end up
    # with something out of usable range. This fixes that by
    # constraining x to be between +/- pi.
    def normalize_angle(x):
        return ((x + 3*pi) % (2*pi)) - pi

    def dot_product(a, b):
        return sum(map(lambda x: x[0]*x[1], zip(a, b)))

    def distance(point1, point2):
        return np.sqrt((point1[0] - point2[0]) ** 2 + (point1[1] - point2[1]) ** 2)

    def mh_distance(point1, point2):
        return abs(point1[0] - point2[0]) + abs(point1[1] - point2[1])

    # PID is given a list of errors where the last error is the current error, as well as three proportionality constants
    # PID returns a number
    def PID(errors, KP, KD, KI):
        print("errors, ", errors)
        term1 = KP * errors[-1]
        term2 = 0
        term3 = 0
        if len(errors) > 1:
            term2 = KD * derivative(errors[-2:])
            term3 = KI * np.trapz(errors)
        print("Proportional term: ", term1, " Derivative term: ", term2, " Integral term: ", term3)
        return term1 + term2 + term3

    def derivative(x):
        d = x[1] - x[0]
        return d

    def integrate(x):
        i = 0
        # Loop through x, adding vals to i, then divide i by the length of the array x minus 1
        for j in range(len(x)):
            i += x[j]
        i /= len(x)
        i *= len(x) - 1
        return i

    def our_round(x):
        if x >= 1:
            return round(x)
        if x >=.2:
            return 1
        return 0

    def make_A_star_grid(width, height, tile_length, obstacles, goal_location, root_location):
        num_tiles_x = int(width/tile_length)
        num_tiles_y = int(height/tile_length)
        graph_guts = [[None for i in range(num_tiles_x)] for i in range(num_tiles_y)]
        root = None
        for i in range(num_tiles_x):
            for j in range(num_tiles_y):
                is_valid = True
                x = i * tile_length
                y = height - j * tile_length
                node_center = [x + tile_length / 2.0, y + tile_length / 2.0]
                for obstacle in obstacles.values():
                    is_valid = is_valid and not overlap(node_center, obstacle, tile_length, tile_length, 1==1)
                if is_valid:
                    if overlap(node_center, goal_location, tile_length, tile_length, True):
                        new_node = search_Node(node_center, goal=True)
                    elif overlap(node_center, root_location, tile_length, 0, True):
                        new_node = search_Node(node_center)
                        root = new_node
                    else:
                        new_node = search_Node(node_center)
                    graph_guts[i][j] = new_node
        for x in range(num_tiles_x):
            for y in range(num_tiles_y):
                if graph_guts[x][y] is not None:
                    if graph_guts[x + 1][y] != None:
                        graph_guts[x][y].addNeighbor(graph_guts[x + 1][y])
                    if y < num_tiles_y:
                        if graph_guts[x + 1][y + 1] is not None:
                            graph_guts[x][y].addNeighbor(graph_guts[x + 1][y + 1])
                        if graph_guts[x][y + 1] is not None:
                            graph_guts[x][y].addNeighbor(graph_guts[x][y + 1])
                        if x != 0:
                            graph_guts[x][y].addNeighbor(graph_guts[x - 1][y + 1])
        return Robot_AStar_Graph(root, goal_location)


    def overlap(center1, center2, length1, length2, other_const_just_to_annoy_him):
        half_avg_length = (length1 + length2) / 4.0
        inX = abs(center1[0] - center2[0]) <= half_avg_length
        inY = abs(center1[1] - center2[1]) <= half_avg_length
        return inX and inY and other_const_just_to_annoy_him

    # not sure these will be useful for our implementation
    angle_target = calc_angle(0, -1)
    position_target = 1080 / 2


    lost_count = 0

    '''
    One way to do this is to get all of the markers at the beginning
    and assign a potential field to each. Then, during the while
    loop, we assume that the markers do not move and only do field
    computations on the set fields and the current robot postion.
    '''
    good_nodes = {}
    width = 1500
    height = 1200

    obstacles = {} 
    max_rob_speed = 2.3 #adjust as needed
    trans_err_list = np.array([0,0,0,0,0])
    angle_err_list = np.array([0,0,0,0,0])

    k_trans = [0.01, 0.0, 0.0]
    k_angle = [0.2, 0.4, 0.01]
    A_star_is_born = True
    markers = do('where others')
    marker_side_length =  distance(markers[goal]['corners'][0],
                                        markers[goal]['corners'][1])

    goal_location = markers[goal]['center']
    goal_location[1] *= -1
    print("Goal= ", goal)

    for key in markers.keys():
        if key != 'time':
            if key != goal:
                location = markers[key]['center']
                location[1] *= -1
                obstacles[key] = location

    if A_star_is_born:
        grid = make_A_star_grid(width, height, marker_side_length, obstacles, goal_location)

    # Running loop
    try:

        you_did_it = False
        turn_coefficient = 4
        cur_direction = None

        while not you_did_it:
            # Get the position of the robot. The result should be a
            # dictionary with four corners, a center, an orientation
            # vector, and a timestamp that isn't very useful yet.
            res = do('where robot')

            # Check that the operation succeeded before proceeding
            if 'orientation' in res:
                lost_count = 0
                cur_robot_angle = calc_angle(*res['orientation'])
                '''
                # Calculate an error in the angle, which gives a
                # direction (sign) to turn and also an idea of what
                # speed to go (the magnitude). Note that this is the
                # same as the P term in a PID controller. A PD or PID
                # controller would do even better (hint hint).
                angle_error = normalize_angle(angle_target - angle)

                # Also calculate an error in the position; we want it
                # to be in the center of the image (on a centered
                # horizontal line). I compensate for the reversed
                # coordinates by negating the result.
                position_error = -(position_target - res['center'][1])

                # These two errors tell us how much we want to turn
                # and how much we want to move. We can use both of
                # these at the same time if we're clever.
                turn = 5 * angle_error
                drive = 0.05 * cos(angle_error) * position_error
                do('speed {} {}'.format(round(drive-turn), round(drive+turn)))
                '''

                # accrue effect of potential fields on the robot
                cur_rob_loc = np.array(res['center'])
                cur_rob_loc[1] *= -1 # adjust for y being down

                #print("robot loc, ", cur_rob_loc)
                #print("goal loc: ", goal_location)

                if distance(cur_rob_loc, goal_location) < 2.75*marker_side_length:
                    you_did_it = True
                    do('speed 0 0')
                else:
                    cur_trans_err = distance(cur_rob_loc, cur_rob_loc + field_effect)
                    if abs(field_effect.sum()) > 0:
                        #print("field anglei ", calc_angle2(*field_effect))  
                        #print('cur_robot_angle, ', cur_robot_angle)
                        #print('difference, ',calc_angle2(*field_effect) - cur_robot_angle) 
                        #print('mod difference, ',(calc_angle2(*field_effect) - cur_robot_angle) % (2 * np.pi))
                        cur_angle_err = (calc_angle2(*field_effect) - cur_robot_angle) % (2*np.pi)
                        #print("end_cur_anglei, ", cur_angle_err)
                        if cur_angle_err > np.pi:
                            cur_angle_err -= 2*np.pi
                        #print("end_cur_anglei, ", cur_angle_err)
                    else:
                        cur_angle_err = 0
                    trans_err_list = trans_err_list[1:]
                    angle_err_list = angle_err_list[1:]
                    trans_err_list = np.append(trans_err_list, cur_trans_err)
                    angle_err_list = np.append(angle_err_list, cur_angle_err)

                    drive = PID(trans_err_list, k_trans[0], k_trans[1], k_trans[2])
                    turn = PID(angle_err_list, k_angle[0], k_angle[1], k_angle[2])

                    print("original drive,turn= {},{}".format(drive, turn))

                    if drive > max_rob_speed:
                        drive = max_rob_speed

                    #if abs(turn_coefficient*turn) > max_rob_speed:
                    #    new_turn = np.sign(turn_coefficient*turn)*max_rob_speed
                    #    left_wheel =  int(our_round(drive - new_turn))
                    #    right_wheel = int(our_round(drive + new_turn))
                    #else:
                    left_wheel = int(our_round(drive - turn_coefficient*turn))
                    right_wheel = int(our_round(drive + turn_coefficient*turn))

                    print("instruct: left_wheel,right_wheel={},{}".format(left_wheel,right_wheel))
                    do('speed {} {}'.format(left_wheel, right_wheel))
                    sleep(0.5)
                    #input("press enter")

            else:
                # Sometimes the camera fails to find the robot, and it
                # will return a mostly empty response. I handle this
                # by trying a number of times, and if it fails too
                # much, stopping the robot (in case its movement is
                # blurring the image).
                lost_count += 1
                if lost_count == 10:
                    do('speed 0 0')
                sleep(0.05)

    # Stop with ^C
    except KeyboardInterrupt:
        print('\nStopping')

    # Close the connection (wait for the last command to return)
    sleep(0.5)
    writer.close()
    loop.close()


if __name__ == '__main__':
    from sys import argv
    main(*argv[1:])
