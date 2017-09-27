#!/usr/bin/env python3

import bluetooth as bt
from struct import pack


class Robot:
    def __init__(self, dev, log=False):
        self.socket = None
        self.addr, self.name = dev
        if self.name[:7] != 'MSE430-':
            raise ValueError('Invalid device name', self.name)
        self.log = log

    def connect(self):
        while not self.socket:
            try:
                if self.log:
                    print('[CONNECT ]', self.addr, flush=True)
                self.socket = bt.BluetoothSocket(bt.RFCOMM)
                self.socket.connect((self.addr, 1))
            except:  # !!!
                if self.socket:
                    self.socket.close()
                self.socket = None

    def set_power(self, power):
        if type(power) is not tuple or len(power) is not 2:
            raise ValueError(
                'Argument \'power\' must be a tuple of length 2')
        try:
            message = pack('<chh', b'P', power[0], power[1])
        except struct.error:
            raise ValueError(
                'Power values must be in range [-2**15, 2**15)')
        if self.log:
            print('[SEND    ]', message.hex())
        self.socket.send(message)

    def shutdown(self):
        if self.log:
            print('[SHUTDOWN]')
        self.set_power((0, 0))
        import time
        time.sleep(3)
        self.socket.close()
        time.sleep(3)

    @staticmethod
    def discover(repeat=None, verbose=False):
        devs = []
        if verbose:
            print('Searching for robots...', end='', flush=True)
        while not devs:
            devs = list(filter(lambda x: 'MSE430' in x[1],
                               bt.discover_devices(lookup_names=True)))
            if repeat:
                repeat -= 1
                if repeat == 0:
                    break
        if verbose:
            print(devs)
        return devs


def main():
    dev = Robot.discover(verbose=True)[0]
    robot = Robot(dev, log=True)
    robot.connect()

    import pygame
    pygame.init()
    screen = pygame.display.set_mode((400, 400))

    target_a = 0
    target_b = 0
    power_a = 0
    power_b = 0

    pygame.time.set_timer(pygame.USEREVENT, 100)
    
    while True:
        for event in pygame.event.get():
            # print('[EVENT   ]', event)
            if event.type == pygame.QUIT:
                pygame.display.quit()
                pygame.quit()
                robot.shutdown()
                return
            elif event.type == pygame.KEYDOWN and event.key == pygame.K_ESCAPE:
                pygame.display.quit()
                pygame.quit()
                robot.shutdown()
                return
            elif event.type == pygame.KEYDOWN and event.key == pygame.K_UP:
                print('[KEY_DOWN]', 'UP')
                target_a += 400
                target_b += 400
            elif event.type == pygame.KEYUP and event.key == pygame.K_UP:
                print('[KEY_UP  ]', 'UP')
                target_a -= 400
                target_b -= 400
            elif event.type == pygame.KEYDOWN and event.key == pygame.K_DOWN:
                print('[KEY_DOWN]', 'DOWN')
                target_a += -400
                target_b += -400
            elif event.type == pygame.KEYUP and event.key == pygame.K_DOWN:
                print('[KEY_UP  ]', 'DOWN')
                target_a -= -400
                target_b -= -400
            elif event.type == pygame.KEYDOWN and event.key == pygame.K_LEFT:
                print('[KEY_DOWN]', 'LEFT')
                target_a += -200
                target_b += 200
            elif event.type == pygame.KEYUP and event.key == pygame.K_LEFT:
                print('[KEY_UP  ]', 'LEFT')
                target_a -= -200
                target_b -= 200
            elif event.type == pygame.KEYDOWN and event.key == pygame.K_RIGHT:
                print('[KEY_DOWN]', 'RIGHT')
                target_a += 200
                target_b += -200
            elif event.type == pygame.KEYUP and event.key == pygame.K_RIGHT:
                print('[KEY_UP  ]', 'RIGHT')
                target_a -= 200
                target_b -= -200
            elif event.type == pygame.USEREVENT:
                print('[TICK    ]', target_a, target_b, power_a, power_b)
                power_a = int(0.5*power_a + 0.5*target_a)
                power_b = int(0.5*power_b + 0.5*target_b)
                robot.set_power((power_a, power_b))


if __name__ == '__main__':
    main()
