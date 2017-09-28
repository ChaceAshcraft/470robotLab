import asyncio
import json


def main(host, port):

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

    # Spin around
    do('speed 20 -20')
    loop.run_until_complete(asyncio.sleep(1.5))

    do('speed 0 0')
    loop.run_until_complete(asyncio.sleep(0.5))

    do('speed -40 40')
    loop.run_until_complete(asyncio.sleep(1.0))

    do('power 0 0')

    # Close the connection
    writer.close()


if __name__ == '__main__':
    from sys import argv
    main(*argv[1:])
