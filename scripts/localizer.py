#!/usr/bin/env python 
import serial
import time
import re
import rospy
import numpy as np
from geometry_msgs.msg import Pose2D


RATE = 10
THETA = 0
class Localizer():

    def __init__(self):
        rospy.init_node('localization')

        self.pose2D = Pose2D()
        self.pose2D.x = None
        self.pose2D.y = None
        self.pose2D.theta = None
        self.confidence = None

        # publishers
        self.pub_localize = rospy.Publisher('sensor/decawave_measurement', Pose2D, queue_size=10)



    #check the data from the board using pyserial
    def checkSerial(self):
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
            ser.write(b'lep\r')
            
        except Exception as e:
            print(e)
            pass
        print(ser)

        # ser.open()
        while not rospy.is_shutdown():
            try:
                # ser.write(b'\r\r')

                time.sleep(0.1)
                # ser.write(b'lep\r')  
      
                data=str(ser.readline())
                #print data
                posData = [float(s) for s in re.findall(r'-?\d+\.?\d', data)]
                if len(posData)>1:
                    print(posData)
                    self.pose2D.x = posData[0]
                    self.pose2D.y = posData[1]
                    # self.confidence = posData[3]
                    self.pose2D.theta = THETA    
                    print self.pose2D
                    self.publish()
                # print(data)
                time.sleep(0.01)

            except Exception as e:
                print(e)
                time.sleep(0.1)
                pass
            except KeyboardInterrupt:
                ser.close()



    def publish(self):
        self.pub_localize.publish(self.pose2D)


    def run(self):

        rate = rospy.Rate(RATE) # 10 Hz
        while not rospy.is_shutdown():
            self.checkSerial()                

            rate.sleep()
        # rospy.spin()



if __name__ == '__main__':

    localizer = Localizer()
    localizer.run()
