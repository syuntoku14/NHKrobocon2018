import serial

def main():
    COMPORT=2
    try:
        ser.close()
        ser=serial.Serial(port=COMPORT,baudrate=9600,timeout=None)
    except:
        ser=serial.Serial(port=COMPORT,baudrate=9600,timeout=None)
    
    msg='msg recieved: '
    count=0
    while True:
        count+=1
        c=ser.read()
        print(msg+str(c))
        if count==20: break

if __name__=='__main__':
    main()

