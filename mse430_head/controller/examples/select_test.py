import sys
import select
import tty, termios
from time import sleep
from getch import _Getch

def main():
    inkey = _Getch()
    while True:
        if select.select([sys.stdin], [], [],0): 
            print("found something!")
            print("line, ", sys.stdin.read(1), '\n')

if __name__ == '__main__':
    main()
