import asyncio
import json
from math import atan2, pi
from time import sleep


def main(host='localhost', port=55555, marker='50'):
    """This example is just for fun. Using a secondary marker, we can
    drive the robot like an RC car.

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

    def normalize_angle(x):
        return ((x + 3*pi) % (2*pi)) - pi
    
    def distance(a, b):
        return sum(map(lambda x: x*x, map(lambda y: y[1]-y[0], zip(a,b))))**0.5

    lost_count = 0

    # Running loop
    try:
        while True:
            # Get the parameters 
            res = do('where others')

            # Check that the operation succeeded before proceeding
            if marker in res:
                lost_count = 0

                # Extract info about the marker
                angle = normalize_angle(calc_angle(
                    *res[marker]['orientation']) - 0.5*pi)
                corners = res[marker]['corners']
                tilt = (distance(corners[3], corners[2]) /
                        distance(corners[1], corners[0])) - 1

                # Use it to control the robot
                turn = 5 * angle
                drive = 150 * tilt
                do('speed {} {}'.format(round(drive-turn), round(drive+turn)))

            else:
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
