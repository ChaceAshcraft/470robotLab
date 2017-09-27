import asyncio
import argparse
from struct import pack, unpack
from btcomm import BTComm


class Robot:
    def __init__(self, name, loop, **kwargs):
        self.name = 'MSE430-' + name.split('-')[-1]
        self.loop = loop
        self.num = (lambda x: int(x) if x.isnumeric()
                    else int(x, 16))(self.name.split('-')[-1])
        try:
            self.addr = ADDRESSES[self.name]
        except KeyError:
            raise ValueError('Unrecognized name: {} interpreted as {}'.format(
                name, self.name))
        self.btcomm = BTComm(self.addr, self.loop)

    async def connect(self):
        self.btcomm.connect()

    def stop(self):
        self.loop.run_until_complete(self.power((0, 0)))

    def close(self):
        self.btcomm.close()

    async def command(self, message):
        assert len(message) == 5
        self.btcomm.write(message)
        res = await self.btcomm.reader.readexactly(5)
        if res[0] != message[0]:
            raise Exception('Bad response from robot: {}'.format(res.decode()))
        return res[1:]
        
    async def power(self, powers):
        message = pack('<chh', b'P', *powers) if powers else b'p\0\0\0\0'
        res = await self.command(message)
        power_a, power_b = unpack('<hh', res)
        return {'power_a': power_a, 'power_b': power_b}

    async def speed(self, speeds):
        message = pack('<chh', b'S', *speeds) if speeds else b's\0\0\0\0'
        res = await self.command(message)
        speed_a, speed_b = unpack('<hh', res)
        return {'speed_a': speed_a, 'speed_b': speed_b}

    PARAMS = {'kp': b'K', 'ki': b'I', 'kd': b'D', 'ic': b'C', 'ms': b'M'}

    async def param(self, name, value=None):
        try:
            tag = self.PARAMS[name]
        except KeyError:
            raise ValueError('Parameter {} not recognized'.format(name))

        if value is None:
            tag = tag.lower()
        elif 'k' in name:
            value = float2fixed(value)
        else:
            value = int(value)

        message = pack('<ci', tag, value or 0)
        res = await self.command(message)
        value, = unpack('<i', res)
        if 'k' in name:
            value = fixed2float(value)
        return {name: value}

    async def battery(self):
        res = await self.command(b'b\0\0\0\0')
        voltage, _ = unpack('<hh', res)

        # The first 2.5 is for the voltage divider, which should be
        # close to exact. The second 2.5 is for the internal reference
        # used to measure the voltage on the msp430. It turns out that
        # this value varies between 2.35 and 2.65, so some calibration
        # might be needed for really accurate measurement. Still, it
        # works well enough for now.
        return {'voltage': voltage * 2.5 * 2.5 / 1023}

    @staticmethod
    def cli_arguments(parser):
        pass
    

def float2fixed(x, bits=10):
    return round(x * 2**bits)


def fixed2float(x, bits=10):
    return x / 2**bits


ADDRESSES = {
    'MSE430-0': '20:16:01:20:14:43',
    'MSE430-1': '20:15:12:22:05:59',
    'MSE430-2': '20:15:12:28:78:04',
    'MSE430-3': '20:15:12:21:90:81',
    'MSE430-4': '20:16:01:19:72:73',
    'MSE430-5': '20:16:01:18:86:01',
    'MSE430-6': '20:15:12:28:78:56',
    'MSE430-7': '98:D3:31:FB:38:4C',
    'MSE430-8': '98:D3:31:FB:38:61',
    'MSE430-9': '98:D3:31:FC:33:82',
    'MSE430-A': '98:D3:31:FB:34:E4'
    }
