import time
import serial
import sys,tty,termios


class _Getch:
    def __call__(self):
        fd = sys.stdin.fileno()
        old_settings = termios.tcgetattr(fd)
        try:
            tty.setraw(sys.stdin.fileno())
            ch = sys.stdin.read(1)
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
        return ch


def get():
    inkey = _Getch()
    while(1):
        k=inkey()
        if k!='':break
    if k == 'L':
        print ("Line")
        s.write("/doLine/run 1\n".encode())
    elif k == 'A':
        print ("Apriltag")
        s.write("/doApriltag/run 1\n".encode())
    
    elif k=='l':
        print("quit Line")
        s.write("/doLine/run 0\n".encode())
    elif k == 'a':
        print("quit Apriltag")
        s.write("/doApriltag/run 0\n".encode())
    elif k=='q':
        print ("quit")
        return 0
    elif k=='B':
        print("Block")
        s.write("/doBlock/run 1\n".encode())
    elif k=='b':
        print("quit Block")
        s.write("/doBlock/run 0\n".encode())
    elif k == 'P':
        print("Parking")
        s.write("/doParking/run 1\n".encode())
    elif k == 'p':
        print("quit Parking")
        s.write("/doParking/run 0\n".encode())
    else:
        print ("not an arrow key!")
    return 1

if len(sys.argv) < 1:
    print ("No port input")
s = serial.Serial(sys.argv[1])
while get():
    i = 0