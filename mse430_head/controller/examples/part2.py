import asyncio
import sys
import json
from math import atan2, sin, cos, pi
from time import sleep
import numpy as np
from potentialField import PotentialField
from matplotlib import pyplot as plt
import select

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


    def repulsor_effect(robot_location, marker_location, radius, spread, field_strength, infinity):
        d = distance(robot_location, marker_location)
        if d > radius + spread:
            return [0, 0]
        theta = atan2(marker_location[1] - robot_location[1], marker_location[0] - robot_location[0])
        #  a constant to give them some angle
        if d >= self.radius:
            return [-field_strength*(spread + radius - d)*cos(theta),
                         -field_strength * (spread + radius - d) * sin(theta)]
        else:
            return [-np.sign(cos(theta))*infinity, -np.sign(sin(theta))*infinity]
 

    def sonar(robot_location, direction, granularity, dist, obst_radius):
        n = int(dist/granularity)
        for i in range(n):
            check_point = [robot_location[0] + granularity*cos(direction), robot_location[1] + granularity*sin(direction)] 
            for obstacle in obstacles.values():
                d = distance(check_point, obstacle)
                if d  <= obst_radius:
                    return check_point 
        return None
                    
    
    def check_sonars(robot_location, granularity, distance, obst_radius, spread, field_strength, infinity):
        field_effect = np.array([0, 0])
        for i in range(16):
            angle = i*np.pi/8
            res = sonar(robot_location, angle, granularity, distance, obst_radius)
            if res is not None:
                field_effect += repulsor_effect(robot_location, res, obst_radius, spread, field_strength, infinity) 
        return field_effect

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
    obstacles = {} 
    max_rob_speed = 2.3 #adjust as needed
    attractor_strength = 3.53
    repulsor_strength = 0.32
    tangent_strength = 0.78
    trans_err_list = np.array([0,0,0,0,0])
    angle_err_list = np.array([0,0,0,0,0])

    tangs_fields = {'74':'counter', '42':'counter', '72':'counter', '32':'counter',
                     '20':'clock', '97':'clock', '99':'clock'}

    k_trans = [0.01, 0.01, 0.01]
    k_angle = [0.2, 0.3, 0.01]

    markers = do('where others')
    marker_radius =  distance(markers[goal]['corners'][0],
                                        markers[goal]['center'])

    goal_location = markers[goal]['center']

    for key in markers.keys():
        if key != 'time':
            if key == goal:
                location = markers[goal]['center']
                location[1] *= -1
                obstacles[goal] = location
            else:
                location = markers[key]['center']
                location[1] *= -1
                obstacles[key] = location
                
                
    # Running loop
    try:

        you_did_it = False
        turn_coefficient = 4
        cur_direction = None
        attractor = PotentialField([0,0], 1, 2, 0, field_type='attractor')


        vx = 0;
        vy = 0;

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

                print("robot loc, ", cur_rob_loc)
                if distance(cur_rob_loc, goal_location) < 2.75*marker_radius:
                    you_did_it = True
                    do('speed 0 0')
                else:

                    if sys.stdin in select.select([sys.stdin], [], [], 0)[0]:
                        new_direction = sys.stdin.read(1)
                        print('new_direction', new_direction)
                        sleep(0.05)
                        if new_direction == cur_direction:
                            attractor.field_strength = 0
                        else:
                            if new_direction == 'w':
                                print("Turn up")
                                cur_direction = new_direction
                                att_loc = [cur_rob_loc[0], 10000]
                                attractor = PotentialField(att_loc, 1, 10, attractor_strength, field_type='attractor')
                            elif new_direction == 's':
                                cur_direction = new_direction
                                print("Turn down")
                                att_loc = [cur_rob_loc[0], -10000]
                                attractor = PotentialField(att_loc, 1, 10, attractor_strength, field_type='attractor')
                            elif new_direction == 'd':
                                cur_direction = new_direction
                                print("Turn right")
                                att_loc = [10000, cur_rob_loc[1]]
                                attractor = PotentialField(att_loc, 1, 10, attractor_strength, field_type='attractor')
                            elif new_direction == 'a':
                                cur_direction = new_direction
                                print("Turn left")
                                att_loc = [-10000, cur_rob_loc[1]]
                                attractor = PotentialField(att_loc, 1, 10, attractor_strength, field_type='attractor')


                    field_effect = np.array([0., 0.])
                    att_effect = attractor.o_effect(cur_rob_loc)
                    obst_effect = check_sonars(cur_rob_loc, marker_radius/2.0, marker_radius*16.0, marker_radius, marker_radius*5,repulsor_strength, 10)
                    # check_sonars(robot_location, granularity, distance, obst_radius, spread, field_strength, infinity):

                    print("attract effect: ", att_effect)
                    print("obstacle effect: ", obst_effect)
                    
                    field_effect += att_effect + obst_effect

                    cur_trans_err = distance(cur_rob_loc, cur_rob_loc + field_effect)
                    cur_angle_err =  calc_angle2(*field_effect) - cur_robot_angle
                    trans_err_list = trans_err_list[1:]
                    angle_err_list = angle_err_list[1:]
                    trans_err_list = np.append(trans_err_list, cur_trans_err)
                    angle_err_list = np.append(angle_err_list, cur_angle_err)

                    drive = PID(trans_err_list, k_trans[0], k_trans[1], k_trans[2])
                    turn = PID(angle_err_list, k_angle[0], k_angle[1], k_angle[2])

                    if turn > max_rob_speed:
                        turn = max_rob_speed

                    '''
                    ax = field_effect[0]
                    ay = field_effect[1]

                    vx = ax
                    vy = ay

                    drive = np.sqrt(vx**2 + vy**2)
                    angle_diff = atan2(vy,vx) - cur_robot_angle
                    turn = np.sign(angle_diff) * turn_coefficient * np.sqrt(np.abs(angle_diff))
                    '''

                    left_wheel = int(our_round(drive - turn_coefficient*turn))
                    right_wheel = int(our_round(drive + turn_coefficient*turn))
                    if drive > max_rob_speed:
                        left_wheel =  int(our_round(max_rob_speed - turn_coefficient * turn))
                        right_wheel = int(our_round(max_rob_speed + turn_coefficient * turn))

                    do('speed {} {}'.format(left_wheel, right_wheel))
                    sleep(0.5)

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
        # t = np.linspace(len(left_wheel_list))
        # plt.plot(t, np.array(left_wheel_list), label='left_wheel')
        # plt.plot(t, np.array(right_wheel_list), label='right_wheel')
        # plt.legend()
        # plt.show()
        # plt.savefig("wheels.jpg")
        # plt.figure()
        # plt.plot(t, np.array(x_vec_list), label='x_mag')
        # plt.plot(t, np.array(y_vec_list), label='y_mag')
        # plt.legend()
        # plt.show()
        # plt.savefig("vecs.jpg")

    # Close the connection (wait for the last command to return)
    sleep(0.5)
    writer.close()
    loop.close()


if __name__ == '__main__':
    from sys import argv
    main(*argv[1:])
