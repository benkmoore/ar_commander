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

    def startup(self):
        self.ser = serial.Serial()
        self.ser.port = '/dev/ttyACM1'
        self.ser.baudrate = 115200
        self.ser.bytesize = serial.EIGHTBITS
        self.ser.parity =serial.PARITY_NONE
        self.ser.stopbits = serial.STOPBITS_ONE
        self.ser.timeout = 1
        self.ser.open()
        self.ser.write(b'\r\r')
        time.sleep(1)
        self.ser.write(b'lep\r')


    #check the data from the board using pyserial
    def checkSerial(self):

        # ser.open()
        while not rospy.is_shutdown():
            start = time.time()
            try:
                # ser.write(b'\r\r')

                #time.sleep(0.1)
                # ser.write(b'lep\r')  
                #self.ser.write(b'\n')
                data=str(self.ser.readline())
                data = re.sub("[a-zA-Z]", "", data)
                data = re.sub("[\r\n>]","",data)
                print('*****')
                print data
                if len(data) > 1:
                    data = np.array(data.split(','))
                    data = data[1:4]
                    data = data.astype(np.float)
                    print data
                    self.pose2D.x = data[0]
                    self.pose2D.y = data[1]
                    self.pose2D.theta = THETA
                    self.publish()
                # print(data)
                #time.sleep(0.01)

            except Exception as e:
                print(e)
                #self.ser.close()
                #time.sleep(1)
                #self.startup()
                #time.sleep(1)
                #time.sleep(0.1)
                pass
            except KeyboardInterrupt:
                self.ser.close()
            end = time.time()
            #print(1/(end-start))


    def publish(self):
        self.pub_localize.publish(self.pose2D)


    def run(self):
        self.startup()
        rate = rospy.Rate(RATE) # 10 Hz
        while not rospy.is_shutdown():
            self.checkSerial()                

            rate.sleep()
        #rospy.spin()



if __name__ == '__main__':

    localizer = Localizer()
    localizer.run()
