from time import sleep
import sys
if __name__=='__main__':
    count=0
    while True:
        sys.stdout.write('hello world\n')
        count+=1
        sleep(0.5)
        if count==30: break