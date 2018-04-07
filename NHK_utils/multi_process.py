import sys
import subprocess
import threading
import re

class Serial_data(object):
    '''
    rcv_msg: message from serial
    serial_kill: flag to terminate serial thread
    pole_angle: angle of pole from auto robot
    success_flag: flag if throwing succeed or not
    '''
    def __init__(self):
        self.rcv_msg=0
        self.serial_kill=False
        self.pole_angle=None
        self.success_flag=False

#read char from serial port
class Serial_thread(threading.Thread):
    '''
    Function to run a thread for serial communication.
    This set rcv_msg which is a char from serial port.
    '''
    def __init__(self,ser,data):
        super().__init__()
        self.ser=ser
        self.data=data
    
    def run(self):
        connected=False
        while not connected and not self.data.serial_kill:
            connected=True
            while not self.data.serial_kill:
                self.data.rcv_msg=self.ser.read().decode()

def get_lines(proc):
    '''
    :param cmd: str 実行するコマンド
    :rtype: generator
    :return: 標準出力（行ごと）
    '''

    while True:
        line=proc.stdout.readline().decode('utf-8')
        if line:
            yield line
        if not line and proc.poll() is not None: #proc.poll() check if child process has terminated
            break

def message_to_pole_success(message):
    '''
    ret: pole angle, success flag (these are None if they're not available)
    '''
    pole_match=re.match('pole angle',message)
    success_match=re.match('success flag',message)
    angle=str(message[pole_match.end():]) if pole_match!=None else None
    scs=str(message[success_match.end():]) if success_match!=None else None
    return angle, scs

#print what recieved from port and send pole_angle and success_flag
def NHK_read_and_send(ser,data):
    print(data.rcv_msg)
    if data.rcv_msg=='r' or data.rcv_msg=='g':
        pole_cmd='NHKrobocon2018.exe k r' if data.rcv_msg=='r' else 'NHKrobocon2018.exe k g'

        proc=subprocess.Popen(pole_cmd, stdout=subprocess.PIPE, stderr=subprocess.STDOUT) #NHK process
        for message in get_lines(proc=proc): #read from stdout
            data.pole_angle,data.success_flag=message_to_pole_success(message)
            #send pole_angle when rcv_msg is r or g
            if data.rcv_msg=='r' or data.rcv_msg=='g': 
                if  data.pole_angle!=None: 
                    print('pole angle is:'+data.pole_angle)
                    ser.write(data.pole_angle.encode('utf-8'))
            #send success_flag when rcv_msg is q
            elif data.rcv_msg=='q':    
                if  data.success_flag!=None:
                    print('success flag is:'+data.success_flag)
                    ser.write(data.success_flag.encode('utf-8'))
            elif data.rcv_msg=='e': break
        proc.kill()

