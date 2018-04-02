import sys
import subprocess
import time
import serial
import threading
from NHK_utils import multi_process

class Serial_data(object):
    def __init__(self):
        self.rcv_msg=0
        self.flag=False

#read char from serial port
class Serial_thread(threading.Thread):
    def __init__(self,ser,msg):
        super().__init__()
        self.ser=ser
        self.msg=msg
    
    def run(self):
        connected=False
        while not connected:
            connected=True
            while True:
                if self.msg.flag: break
                self.msg.rcv_msg=self.ser.read().decode()

def main():
    COMPORT=2
    pole_cmd='NHKrobocon2018.exe r'
    #open serial port
    try:
        ser.close()
        ser=serial.Serial(port=COMPORT,baudrate=9600,timeout=None)
    except:
        ser=serial.Serial(port=COMPORT,baudrate=9600,timeout=None)
    
    msg=Serial_data()
    msg.rcv_msg='x'
    #thread of serial port
    thread=Serial_thread(ser,msg)
    thread.start()

    #get output of NHKrobocon
    for poleangle in  multi_process.get_lines(cmd=pole_cmd):
        print(poleangle)
        print(msg.rcv_msg)
    msg.flag=True

if __name__=='__main__':
    main()
