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
        #confidence not used right now but i think we should in future
        self.confidence = None
        self.failConnect = 0
        # publishers
        self.pub_localize = rospy.Publisher('sensor/decawave_measurement', Pose2D, queue_size=10)


    def startup(self):
        self.ser = serial.Serial()
        self.ser.port = '/dev/ttyACM1'
        self.ser.baudrate = 115200
        self.ser.bytesize = serial.EIGHTBITS
        self.ser.parity =serial.PARITY_NONE
        self.ser.stopbits = serial.STOPBITS_ONE
        self.ser.timeout = 0.3
        self.ser.open()
        self.ser.write(b'\r\r')
        time.sleep(1)
        print "startup"

        #here we reset the input buffer to 0, then wait to see if it fills again
        #if the buffer fills, this means that the board is already sending info down the serial line
        #and if we send our command in this case ^ then it will stop the info.
        self.ser.reset_input_buffer()
        time.sleep(0.2)  
        if self.ser.in_waiting == 0:
            self.ser.write(b'lep\r')


    #check the data from the board using pyserial
    def checkSerial(self):
        fails = 0
        #this while should maybe be something else, we have while not rospy.is_shutdown(): in run() as well
        while not rospy.is_shutdown():
            try:
                
                data = str(self.ser.readline())
                data = re.sub("[a-zA-Z]", "", data)
                data = re.sub("[\r\n>]","",data)
                #print data
                #3 is still the length even when we remove all the characters??
                if len(data) > 3:
                    data = np.array(data.split(','))
                    data = data[1:4]
                    data = data.astype(np.float)
                    print data
                    self.pose2D.x = data[0]
                    self.pose2D.y = data[1]
                    self.pose2D.theta = THETA
                    self.publish()
                    print(data)
                    fails = 0
                else:
                    #self.ser.write(b'lep\r')
                    fails += 1
                if fails > 10:
                    print ('Localizer missed {} packages, check connection between boards + orientation of tag.').format(fails)
                    

            except Exception as e:
                print(e)
                # self.ser.write(b'lep\r')
                # self.ser.close()
                #self.startup()
               
                pass


            

    def publish(self):
        self.pub_localize.publish(self.pose2D)


    def closeConnection(self):
        self.ser.write(b'lep\r')
        self.ser.reset_input_buffer()
        self.ser.close()


    def run(self):
        self.startup()
        rate = rospy.Rate(RATE) # 10 Hz
        while not rospy.is_shutdown():
            self.checkSerial()
            # self.closeConnection()     
            rate.sleep()
        #rospy.spin()



if __name__ == '__main__':
    localizer = Localizer()
    localizer.run()