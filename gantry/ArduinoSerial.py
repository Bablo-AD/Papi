import serial
import time

ard=serial.Serial(port='/dev/ttyACM0',baudrate=115200,timeout=.1)

def sdgcode(gcode):
    ard.write(gcode.encode())
    time.sleep(100)
    res=ard.readline().encode('utf-8').strip()
    return res

