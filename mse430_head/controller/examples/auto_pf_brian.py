import asyncio
import json
from math import atan2, cos, pi
from time import sleep
import numpy as np
from potentialField import PotentialField

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

    # Sometimes, when taking differences between angles, you end up
    # with something out of usable range. This fixes that by
    # constraining x to be between +/- pi.
    def normalize_angle(x):
        return ((x + 3*pi) % (2*pi)) - pi

    def dot_product(a, b):
        return sum(map(lambda x: x[0]*x[1], zip(a, b)))

    def distance(point1, point2):
        return np.sqrt((point1[0] - point2[0]) ** 2 + (point1[1] - point2[1]) ** 2)

    # PID is given a list of errors where the last error is the current error, as well as three proportionality constants
    # PID returns a number
    def PID(errors, KP, KD, KI):
        term1 = KP * errors[-1]
        term2 = 0
        term3 = 0
        if len(errors) > 1:
            term2 = KD * derivative(errors[-2:-1])
            term3 = KI * np.trapz(errors)
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
    fields = {}
    goalField = None
    max_rob_speed = 20 #adjust as needed
    attractor_strength = 0.35
    repulsor_strength = 0.35
    tangent_strength = 0.35
    trans_err_list = np.zeros(5)
    angle_err_list = np.zeros(5)

    k_trans = [0.5, 0.5, 0.5]
    k_angle = [0.5, 0.5, 0.5]

    markers = do('where others')
    marker_radius =  distance(markers[markers.keys()[0]]['corners'][0], 
                                        markers[markers.keys()[0]]['center']) 
    for key in markers.keys():
        if key == goal:
            location = markers[key]['center']
            location[1] *= -1  # adjust for y being down
            goalField = PotentialField(location, 10000, marker_radius/2, attractor_strength, 
                                                                        type='attracter')
        else:
            location = markers[key]['center']
            location[1] *= -1  # adjust for y being down
            if np.random.randint(10) < 1: # 1/10 probability of tangent field
                fields[key] = PotentialField(location, 2*marker_radius, marker_radius, repulsor_strength, 
                                                                        type='tangent')
            else:
                fields[key] = PotentialField(location, 2*marker_radius, marker_radius, tangent_strength, 
                                                                        type='repulsor')

    # Running loop
    try:

        while True:
            # Get the position of the robot. The result should be a
            # dictionary with four corners, a center, an orientation
            # vector, and a timestamp that isn't very useful yet.
            res = do('where robot')

            # Check that the operation succeeded before proceeding
            if 'orientation' in res:
                lost_count = 0
                cur_robot_angle = calc_angle(*res['orientation'])
                '''
                #Not sure we need this for our implementation
                
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
                field_effect = np.array([0, 0])
                field_effect += goalField.effect(cur_rob_loc)
                for field in fields.items():
                    field_effect += field.effect(cur_rob_loc)

                cur_trans_err = distance(cur_rob_loc, cur_rob_loc + field_effect)
                cur_angle_err = cur_robot_angle - calc_angle(field_effect)
                trans_err_list = trans_err_list[:-1]
                angle_err_list = angle_err_list[:-1]
                np.append(trans_err_list, cur_trans_err)
                np.append(angle_err_list, cur_angle_err)

                drive = PID(trans_err_list, k_trans[0], k_trans[1], k_trans[2])
                turn = PID(angle_err_list, k_angle[0], k_angle[1], k_angle[2])

                do('speed {} {}'.format(round(drive - turn), round(drive + turn)))

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
