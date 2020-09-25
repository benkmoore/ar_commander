#!/usr/bin/env python
import serial
import time
import re
import sys
import rospy
import numpy as np
import collections
from ar_commander.msg import Decawave

sys.path.append(rospy.get_param("AR_COMMANDER_DIR"))

import configs.hardware_params as params

# timeout in seconds for how long we try to read serial data if no data immediately available
SERIALTIMEOUT = 0.3
RATE = 10
EPSILON = np.finfo(np.float64).eps


class DecaInterface():

    def __init__(self,port):

        # pySerial parameters set to match decawave params
        self.ser = serial.Serial()
        self.ser.port = port
        # baudrate is bits per second expected on the serial channel (the deca boards use 115200)
        self.ser.baudrate = 115200
        self.ser.bytesize = serial.EIGHTBITS
        self.ser.parity =serial.PARITY_NONE
        self.ser.stopbits = serial.STOPBITS_ONE
        self.ser.timeout = SERIALTIMEOUT

        self.x = None
        self.y = None

        #confidence not published right now, its the value that the boards give for how confident they are
        self.confidence = None
        self.readFails = 0
        #flag to check if board has read good data
        self.dataRead = 0


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
        if len(data) > 4:
            data = np.array(data.split(','))
            data = data[1:5]
            data = data.astype(np.float)
            if len(data) > 3:
                self.x = data[0]
                self.y = data[1]
                self.confidence = data[3]
                self.readFails = 0
                self.dataRead = 1
                self.ser.reset_input_buffer()
        else:
            self.readFails += 1
            self.dataRead = 0
            rospy.logerr("on port %s, readSerial failed to read serial data %s times.", self.ser.port, self.readFails)
            #if we dont read any pose data for this amount of tries then redo the serial connection
        if self.readFails > 30:
            #check connection between boards + orientation of tag.
            rospy.logerr("readSerial failed to read serial data %s times. Check: 1. Anchors xy pos is accurate, 2. Enough anchors in range, 3. Orientation of tag.", self.readFails)
            self.readFails = 0
            self.dataRead = 0
            self.connect()


    def closeConnection(self):
        self.ser.write(b'lep\r')
        self.ser.reset_input_buffer()
        self.ser.close()



class GetPose():

    def __init__(self,port1,port2):
        rospy.init_node('decaInterface', anonymous=True)

        # Decawave msgs
        self.measurement_msg = Decawave()
        self.boardY = DecaInterface(port1)
        self.boardY.connect()
        self.boardX = DecaInterface(port2)
        self.boardX.connect()

        # Decawave constants
        self.pos_meas_std = params.pos_measurement_std # pos measurement standard deviation(m)

        # measurements
        self.pos1 = None
        self.pos2 = None
        self.theta = None

        # previous measurements
        self.pos1_prev = None
        self.pos2_prev = None
        self.theta_prev = None

        # covariance of measurements
        self.cov_pos1 = None
        self.cov_pos2 = None
        self.cov_theta = None

        # publishers
        self.pub_decaInterface = rospy.Publisher('sensor/decawave_measurement', Decawave, queue_size=10)


    def publish(self):
        self.pub_decaInterface.publish(self.measurement_msg)


    def calculateCovs(self):
        self.cov_pos1 = ((self.pos_meas_std**2)/self.boardY.confidence)*np.eye(2)
        self.cov_pos2 = ((self.pos_meas_std**2)/self.boardX.confidence)*np.eye(2)

        if self.pos1_prev is None or self.pos2_prev is None or self.theta_prev is None:
            delta_pos1 = delta_pos2 = np.ones(2)
            delta_theta = 1
        else:
            delta_pos1 = self.pos1 - self.pos1_prev
            delta_pos2 = self.pos2 - self.pos2_prev
            delta_theta = self.theta - self.theta_prev

        if abs(delta_pos1) < EPSILON or abs(delta_pos2) < EPSILON: # prevent division by zero
            self.cov_theta = (self.cov_pos1 + self.cov_pos2)/2
        else:
            dth_dp1 = abs(delta_theta/delta_pos1)
            dth_dp2 = abs(delta_theta/delta_pos2)
            self.cov_theta = npl.multi_dot((dth_dp1, self.cov_pos1, dth_dp1)) + npl.multi_dot((dth_dp2, self.cov_pos2, dth_dp2))


    def updateMeasurementMsgData(self):
        self.measurement_msg.x1.data = self.boardY.x # pos 1 on Y axis arm
        self.measurement_msg.y1.data = self.boardY.y
        self.measurement_msg.x2.data = self.boardX.x # pos 2 on X axis arm
        self.measurement_msg.y2.data = self.boardX.y
        self.measurement_msg.theta.data = self.theta
        # covariances
        self.measurement_msg.cov1.data = self.cov_pos1
        self.measurement_msg.cov2.data = self.cov_pos2
        self.measurement_msg.cov_theta.data = self.cov_theta

        # update previous measurements
        self.pos1_prev = self.pos1
        self.pos2_prev = self.pos2
        self.theta_prev = self.theta


    def obtainMeasurements(self):
        self.theta = np.arctan2(-(self.boardY.y-self.boardX.y) ,-(self.boardY.x-self.boardX.x)) + np.pi/4
        if self.theta > np.pi: self.theta = -np.pi + (self.theta % np.pi) # wrap [-pi, pi]
        self.pos1 = np.array([self.boardY.x, self.boardY.y])
        self.pos2 = np.array([self.boardX.x, self.boardX.y])

        # find pos1, pos2 & theta covariances
        self.calculateCovs()
        # add new data to output msg
        self.updateMeasurementMsgData()


    def run(self):
        rate = rospy.Rate(RATE) # 10 Hz
        while not rospy.is_shutdown():
            self.boardY.readSerial()
            self.boardX.readSerial()
            #need something else below to publish the location of 1 board if only 1 receives data due to reasons (bad line of sight, noise etc)
            if self.boardX.dataRead and self.boardY.dataRead:
                self.obtainMeasurements()
                self.publish()

            rate.sleep()



if __name__ == '__main__':
    getPose = GetPose('/dev/ttyACM1','/dev/ttyACM2')
    getPose.run()
