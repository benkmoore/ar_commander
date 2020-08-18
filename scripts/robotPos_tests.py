#!/usr/bin/env python 
import serial
import time
import re



# rospy.init_node('Localization', anonymous=True)


try:
    ser = serial.Serial()
    ser.port = '/dev/ttyACM1'
    ser.baudrate = 115200
    ser.bytesize = serial.EIGHTBITS 
    ser.parity =serial.PARITY_NONE 
    ser.stopbits = serial.STOPBITS_ONE 
    ser.timeout = 1
    ser.open()
    ser.write(b'\r\r')
    time.sleep(1)
    ser.write(b'apg\r')
    ser.close()
except Exception as e:
    print(e)
    pass
print(ser)

ser.open()

while True:
    try:
        ser.write(b'\r\r')
        time.sleep(0.1)
        ser.write(b'apg\r')        
        data=str(ser.readline())
        nos = [int(s) for s in re.findall(r'\b\d+\b', data)]
        if nos:
            print(nos)
        # print(data)
        time.sleep(0.01)

    except Exception as e:
        print(e)
        time.sleep(0.1)
        pass
    except KeyboardInterrupt:
        ser.close()
