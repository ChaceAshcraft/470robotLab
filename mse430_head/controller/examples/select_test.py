import sys
import select
from time import sleep


def main():
    while True:
        (r,w,e) = select.select([sys.stdin], [], [], 0)
        print(r)
        sleep(2)    
