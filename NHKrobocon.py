import sys
import subprocess
import time
import serial
import threading
import os
import re
from NHK_utils.multi_process import get_lines, Serial_data, Serial_thread, NHK_read_and_send

def main():
    #open serial port
    COMPORT=2
    ser=serial.Serial(port=COMPORT,baudrate=9600,timeout=None)
    data=Serial_data()
    data.rcv_msg='x'
    data.serial_kill=False

    #start thread for serial port
    thread=Serial_thread(ser,data)
    thread.start()
    
    while data.rcv_msg!='t':
        #get output of NHKrobocon
        print(data.rcv_msg)
        NHK_read_and_send(ser,data)
        time.sleep(0.01)
    data.serial_kill=True

if __name__=='__main__':
    main()
