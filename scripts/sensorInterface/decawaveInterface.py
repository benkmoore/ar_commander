#!/usr/bin/env python
import serial
import time
import re
import rospy
import numpy as np
import collections
from ar_commander.msg import Decawave

sys.path.append(rospy.get_param("AR_COMMANDER_DIR"))

import configs.hardware_params as params
from scripts.utils import wrapAngle

PORT1 = params.decawave_ports[0]  # sensor 1 usb port: Y axis arm of robot
PORT2 = params.decawave_ports[1]  # sensor 2 usb port: X axis arm of robot
SERIALTIMEOUT = 0.3        # duration of serial read (s)
NUM_READ_FAILS = 10        # number of fails to read decas print warning
FAILED_CONNECT_LIMIT = 30  # number of read attempts before serial connection is restarted
RATE = 20                  # (Hz)



class DecaInterface():
    def __init__(self,port):
        # pySerial parameters set to match decawave params
        self.ser = serial.Serial()
        self.ser.port = port
        self.ser.baudrate = 115200
        self.ser.bytesize = serial.EIGHTBITS
        self.ser.parity =serial.PARITY_NONE
        self.ser.stopbits = serial.STOPBITS_ONE
        self.ser.timeout = SERIALTIMEOUT

        # data variables
        self.x = None
        self.y = None
        self.confidence = None
        self.readFails = 0
        self.dataRead = None


    def connect(self):
        """
        Connect to decawave board and to request localization data

        Shell mode is entered via a serial write command and the serial
        buffer is reset before data is requested. Before writing 'lep' to
        the board the input buffer is reset to 0. If the buffer fills, this
        means that the board is already sending info down the serial line
        and if we send our command in this case then it will stop the info.
        """

        if not self.ser.is_open:
            self.ser.open()

        self.ser.write(b'\r\r') # enter shell mode to talk to the board
        time.sleep(1) # comms setup time
        rospy.loginfo("Decawave serial connecting")

        self.ser.reset_input_buffer()
        time.sleep(0.2)
        if self.ser.in_waiting == 0:
            self.ser.write(b"lep\r") # board receives 'lep' and returns the [x,y,z,confidence] data


    def readSerial(self):
        """Check the data from the board using pyserial"""
        data = str(self.ser.readline())
        data = re.sub("[a-zA-Z]", "", data)
        data = re.sub("[\r\n>]","",data)

        if len(data) > 4:
            data = np.array(data.split(','))
            data = data[1:5]
            data = data.astype(np.float)
            if len(data) > 3:
                self.x = data[0]
                self.y = data[1]
                self.confidence = data[3]
                self.readFails = 0
                self.dataRead = True
                self.ser.reset_input_buffer()
        else: # if no pose data read more than NUM_READ_FAILS redo the serial connection
            self.readFails += 1
            self.dataRead = False
            if self.readFails > NUM_READ_FAILS:
                rospy.logerr("on port %s, readSerial failed to read serial data %s times.", self.ser.port, self.readFails)

        if self.readFails > FAILED_CONNECT_LIMIT: # check connection between boards + orientation of tag.
            rospy.logerr("readSerial failed to read serial data %s times. Check: 1. Anchors xy pos is accurate, 2. Enough anchors in range, 3. Orientation of tag.", self.readFails)
            self.readFails = 0
            self.dataRead = False
            self.connect()


    def closeConnection(self):
        self.ser.write(b'lep\r')
        self.ser.reset_input_buffer()
        self.ser.close()



class GetPose():
    def __init__(self, port1, port2):
        rospy.init_node('decawaveInterface', anonymous=True)

        # Decawave msgs
        self.measurement_msg = None
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

        # covariance of measurements
        self.cov_pos1 = None
        self.cov_pos2 = None
        self.cov_theta = None

        # publishers
        self.pub_decaInterface = rospy.Publisher('sensor/decawave_measurement', Decawave, queue_size=10)


    def publish(self):
        if self.measurement_msg is not None:
            self.pub_decaInterface.publish(self.measurement_msg)


    def calculateCovs(self):
        """
        Calculate measurement covariances.

        Uses decawave hardware param, standard deviation on position measurement and the outputted confidence in the
        measurement timestamp from the board. Propagates uncertainty by combining the measurement covariances from
        each sensor to find a covariance for the theta measurement with a linear combination. The derivates are
        derived from the heading calculation, the formula relates robot heading to sensor measurements.
        """
        self.cov_pos1 = (self.pos_meas_std**2)*np.eye(2)
        self.cov_pos2 = (self.pos_meas_std**2)*np.eye(2)

        dx = self.pos2[0] - self.pos1[0]
        dy = self.pos2[1] - self.pos1[1]
        div = (dy**2 + dx**2)

        dth_dx1 =  dy / div
        dth_dy1 = -dx / div
        dth_dx2 = -dy / div
        dth_dy2 =  dx / div

        dth_dp1 = np.abs(np.array([dth_dx1, dth_dy1]))
        dth_dp2 = np.abs(np.array([dth_dx2, dth_dy2]))

        std_theta = np.dot(dth_dp1, np.sqrt(self.cov_pos1)) + np.matmul(dth_dp2, np.sqrt(self.cov_pos2))
        self.cov_theta = np.mean(std_theta) ** 2


    def updateMeasurementMsgData(self):
        """ Populate Decawave message with sensor data, measurement covariances and read flags"""
        self.measurement_msg = Decawave()
        self.measurement_msg.x1.data = self.boardY.x # pos 1 on Y axis arm
        self.measurement_msg.y1.data = self.boardY.y
        self.measurement_msg.x2.data = self.boardX.x # pos 2 on X axis arm
        self.measurement_msg.y2.data = self.boardX.y
        self.measurement_msg.theta.data = self.theta

        # covariances
        self.measurement_msg.cov1.data = self.cov_pos1.reshape(-1)
        self.measurement_msg.cov2.data = self.cov_pos2.reshape(-1)
        self.measurement_msg.cov_theta.data = self.cov_theta

        # flags
        self.measurement_msg.new_meas1.data = self.boardY.dataRead
        self.measurement_msg.new_meas2.data = self.boardX.dataRead


    def obtainMeasurements(self):
        if self.boardY.dataRead and self.boardX.dataRead:
            self.theta = np.arctan2(self.boardX.y-self.boardY.y, self.boardX.x-self.boardY.x) + np.pi/4
            self.theta = wrapAngle(self.theta) # wrap [-pi, pi]
        if self.boardY.dataRead:
            self.pos1 = np.array([self.boardY.x, self.boardY.y])
        if self.boardX.dataRead:
            self.pos2 = np.array([self.boardX.x, self.boardX.y])

        if self.pos1 is not None and self.pos2 is not None:
            self.calculateCovs()
            self.updateMeasurementMsgData()


    def run(self):
        rate = rospy.Rate(RATE)
        while not rospy.is_shutdown():
            self.boardY.readSerial()
            self.boardX.readSerial()
            self.obtainMeasurements()
            self.publish()

            rate.sleep()



if __name__ == '__main__':
    getPose = GetPose(PORT1, PORT2)
    getPose.run()
