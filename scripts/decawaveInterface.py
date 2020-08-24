#!/usr/bin/env python
import serial
import time
import re
import sys
import rospy
import numpy as np
from geometry_msgs.msg import Pose2D


RATE = 10
THETA = 0
class DecaInterface():

    def __init__(self):
        rospy.init_node('decaInterface')

        # pySerial serial stuff
        self.ser = serial.Serial()
        self.ser.port = '/dev/ttyACM0'
        self.ser.baudrate = 115200
        self.ser.bytesize = serial.EIGHTBITS
        self.ser.parity =serial.PARITY_NONE
        self.ser.stopbits = serial.STOPBITS_ONE
        self.ser.timeout = 0.3

        #Pose stuff, theta not used yet
        self.pose2D = Pose2D()
        self.pose2D.x = None
        self.pose2D.y = None
        self.pose2D.theta = None

        #confidence not published right now but we will soon
        #its the value that the boards give for how confident they are
        self.confidence = None
        self.readFails = 0

        # publishers
        self.pub_decaInterface = rospy.Publisher('sensor/decawave_measurement', Pose2D, queue_size=10)


    def connect(self):
        if not self.ser.is_open:
            self.ser.open()
        #this command gets us into shell mode where we can talk to the board
        self.ser.write(b'\r\r')
        #this sleep is important until we find a better way
        time.sleep(1)
        rospy.loginfo("Decawave serial connecting")

        #here we reset the input buffer to 0, then wait to see if it fills again
        #if the buffer fills, this means that the board is already sending info down the serial line
        #and if we send our command in this case ^ then it will stop the info.
        self.ser.reset_input_buffer()
        time.sleep(0.2)
        if self.ser.in_waiting == 0:
            #when the board receives 'lep' it returns the [x,y,z,confidence] data
            self.ser.write(b'lep\r')


    #check the data from the board using pyserial
    def readSerial(self):

        data = str(self.ser.readline())
        data = re.sub("[a-zA-Z]", "", data)
        data = re.sub("[\r\n>]","",data)
        #3 is still the length even when we remove all the characters??
        if len(data) > 3:
            data = np.array(data.split(','))
            data = data[1:5]
            data = data.astype(np.float)
            self.pose2D.x = data[0]
            self.pose2D.y = data[1]
            self.pose2D.theta = THETA
            self.confidence = data[3]
            self.publish()
            self.readFails = 0
        else:
            self.readFails += 1

            #if we dont read any pose data for this amount of tries then redo the serial connection
        if self.readFails > 30:
            #check connection between boards + orientation of tag.
            rospy.logerr("readSerial failed to read serial data %s times. Check: 1. port name, 2. enough anchors in range, 3. orientation of tag.", self.readFails)
            self.readFails = 0
            self.connect()


    def publish(self):
        self.pub_decaInterface.publish(self.pose2D)


    def closeConnection(self):
        self.ser.write(b'lep\r')
        self.ser.reset_input_buffer()
        self.ser.close()


    def run(self):
        self.connect()
        rate = rospy.Rate(RATE) # 10 Hz
        while not rospy.is_shutdown():
            self.readSerial()
            rate.sleep()
        self.closeConnection()


if __name__ == '__main__':
    decaInterface = DecaInterface()
    decaInterface.run()
