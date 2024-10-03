import serial
import time
try:
    ard=serial.Serial(port='/dev/ttyACM0',baudrate=115200,timeout=.1)
except:
    print("Error Serial")
def sdgcode(gcode):
    try:
        ard.write(gcode.encode())
        time.sleep(100)
        res=ard.readline().encode('utf-8').strip()
    except:
        print("Error")  

