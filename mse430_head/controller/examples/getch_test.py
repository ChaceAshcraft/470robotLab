import sys
import select
import tty, termios
from time import sleep
from getch import _Getch

def main():
    inkey = _Getch()
    while True:
        timer = time.time()
        try:
            '''
            if select.select([sys.stdin.fileno()], [], [],0): 
                print("found something!")
                print("line, ", sys.stdin.read(2), '\n')
            '''
            k1 = inkey()
            
            try:
                k2 = inkey()
            except:
                pass
            if k2:
                if k1 != k2:
                    print("you pressed", k1, k2)
            else:
                print("you pressed", k1)  
            
            if k1 == 'q' or k2 == 'q':
                break
        except KeyboardInterrupt:
            break     

if __name__ == '__main__':
    main()

