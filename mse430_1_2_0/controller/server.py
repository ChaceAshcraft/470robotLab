import asyncio
import json
import argparse
from robot import Robot
from vision import Vision


class Server():

    def __init__(self, robot, *args, **kwargs):
        self.loop = asyncio.get_event_loop()
        self.robot = Robot(robot, self.loop, **kwargs)
        self.vision = Vision(self.loop, self.robot.num, **kwargs)
        self.server = None
        self.port = kwargs['port'] or 55555
        self.futures = []
        self.commands = {
            'where': self.where,
            'speed': self.speed,
            'power': self.power,
            'param': self.param,
            'battery': self.battery,
            'shutdown': self.shutdown,
            'help': self.help,
        }
        
    def run(self):
        try:
            self.loop.run_until_complete(self.robot.connect())
            self.futures.append(asyncio.ensure_future(self.vision.run()))
            self.server = self.loop.run_until_complete(
                self.loop.create_server(lambda: ServerProtocol(self),
                                        port=self.port))
            print('Serving on {}'.format(
                self.server.sockets[0].getsockname()))
            self.loop.run_forever()
        except KeyboardInterrupt:
            print()
        print('Exiting')
        self.stop()

    def stop(self):
        self.robot.stop()
        self.vision.stop()
        self.loop.run_until_complete(self.futures[0])  # Not quite
        self.server.close()
        self.loop.run_until_complete(self.server.wait_closed())
        self.loop.stop()
        self.robot.close()
        self.loop.close()

    async def where(self, what=None):
        """where ['robot' | 'others'] -- return locations of tracked objects
        
        The server continually runs a camera and computer vision in
        the background, looking for the robot and other objects marked
        with ArUco tags. This command returns the results of this
        tracking as a dictionary.  The results can be extensive, but
        it's fairly self-explanatory.

        The command can optionally be followed by either 'robot' or
        'others' to filter results to either just the robot or all
        tags that are not the robot. There's no performance gain, this
        is just a convenience.

        Sometimes, the camera may lose tracking of an object for one
        reason or another. If that occurs when this command is called,
        the object won't be in the dictionary, so watch out for
        that. I would like to fix that, but don't count on it
        happening too soon.

        Note: For now, the coordinates are simply image coordinates,
        so the y-axis goes from top to bottom, and the numbers are
        representative of the image dimensions (nominally
        1920x1080). This WILL change.

        """

        if what is None:
            return self.vision.objects
        elif what == 'robot':
            if 'robot' in self.vision.objects:
                return self.vision.objects['robot']
            else:
                return {}
        elif what == 'others':
            return {k: v for k, v in self.vision.objects.items() if k!='robot'}
            

    async def speed(self, *args):
        """speed [speed_a speed_b] -- Get or set motor speed

        With arguments (speed_a and speed_b), this will set targets
        for the speed controller on the robot. Without arguments it
        can be used to just check on the actual speed. This command
        will always return the current speeds of both motors.

        Top speed is somewhere around 80, but there is an artificial
        limit on the robot so it will go in straight lines at top
        speed. This value can be changed as the parameter ms.

        """
        if len(args) in [0, 2]:
            res = await self.robot.speed(tuple(map(int, args)))
        else:
            raise TypeError('speed() takes 0 or 2 arguments ({} given)'.format(
                len(args)))
        return res

    async def power(self, *args):
        """power [power_a power_b] -- Directly get or set motor power

        Motor power is (currently) between -512 and 512. Power doesn't
        correspond directly with speed, but this might help to see
        what the PID is doing internally.

        """
        if len(args) in [0, 2]:
            res = await self.robot.power(tuple(map(int, args)))
        else:
            raise TypeError('power() takes 0 or 2 arguments ({} given)'.format(
                len(args)))
        return res

    async def param(self, name, value=None):
        """param name [value] -- Get or set controller parameters on robot

        If a value is provided, the specified parameter will be set to
        that value. This command always returns the actual value of
        the parameter as well. Due to fixed-point encodings, this may
        sometimes be different than the value passed to it.

        Parameters:
        kp: Proportional term gain
        ki: Integral term gain
        kd: Derivative term gain
        ic: Integral term cap
        ms: Max speed

        """
        return await self.robot.param(name, value and float(value))

    async def battery(self):
        """battery -- Get the current battery voltage

        If the battery gets too low, the robot might shut down when
        trying to drive the motors too hard. I'm not sure yet what to
        have as a threshold, but try to keep it above 5 V or so when
        at rest. It will dip when moving, but theoretically it should
        be stable until you hit about 3.7 V.

        """
        return await self.robot.battery()

    async def help(self, command=None):
        """help [command] -- Display all commands or show details of one"""
        if command:
            return self.commands[command.lower()].__doc__
        else:
            return '\n'.join([x.__doc__.split('\n')[0] for x in
                              self.commands.values()])

    async def shutdown(self):
        """shutdown -- Shutdown the server

        The server will keep running after a client disconnects, but
        this command will shut it down.

        """
        self.loop.call_soon(self.loop.stop)
        return 'Shutting down'
        
    async def handle_command(self, data, write): 
        cmd = data.split()[0].lower()
        args = data.split()[1:]
        if cmd in self.commands:
            try:
                res = await self.commands[cmd](*args)
                write(json.dumps(res) if type(res) is dict else res)
            except Exception as e:
                write('{} occurred while handling "{}": {}'.format(
                    type(e).__name__, data, str(e)))
                raise e
        else:
            write('Unknown command: "{}"'.format(cmd))


class ServerProtocol(asyncio.Protocol):
    def __init__(self, server):
        self.server = server
        self.transport = None
        
    def connection_made(self, transport):
        print('Connection made')
        self.transport = transport
        
    def connectionn_lost(self, exc):
        print('Connection lost')
        self.transport = None  # Do I need to close it myself?
        
    def write(self, data):
        data = str(data)
        print('Sent: {}'.format(data))
        self.transport.write((data+'\n').encode())

    def data_received(self, data):
        data = data.strip().decode()
        if not data:
            return
        print('Received: {}'.format(data))
        asyncio.ensure_future(self.server.handle_command(data, self.write))

    def eof_received(self):
        return False


def main():
    parser = argparse.ArgumentParser(description='MSE430 robot server '
                                     'for CS 470')
    parser.add_argument('robot', help='Name or number of the robot to control')
    parser.add_argument('--port', type=int)

    Vision.cli_arguments(parser)
    Robot.cli_arguments(parser)

    args = parser.parse_args()

    Server(**vars(args)).run()


if __name__ == '__main__':
    main()
