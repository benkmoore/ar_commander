#!/usr/bin/env python

import rospy
import numpy as np
import sys
import matplotlib.pyplot as plt
import rospkg

sys.path.append(rospkg.RosPack().get_path('ar_commander'))
import configs.robot_v1 as rcfg

from ar_commander.msg import TOF, State
from std_msgs.msg import Float64MultiArray

RATE = 10
Y_ARM_POS = [0.15,0.45] # in m
MID_POS = [0.15,0.15]
X_ARM_POS =[0.45,0.15]
TOF_CONVERSION = 1000.0 # mm to m
TOF_DEFAULT = -9999

class DepthSensor():
    def __init__(self, relativPos, relativAngle):
        self.relativPos = relativPos
        self.relativAngle = relativAngle
        self.absolutPos = None
        self.xyData = None
        self.theta = None
        self.sensorData = None

    def xyCalculate(self, tof_data, theta, pos):
        self.sensorData = tof_data
        self.theta = theta
        self.pos = pos

        if self.theta is not None and self.pos is not None:
            sinVal = np.sin((self.theta+np.arctan2(self.relativPos[1],self.relativPos[0])))
            cosVal = np.cos((self.theta+np.arctan2(self.relativPos[1],self.relativPos[0])))
            self.absDist = np.sqrt(self.relativPos[0]**2 + self.relativPos[1]**2)
            self.absolutPos = self.pos + [cosVal*self.absDist,sinVal*self.absDist]

        if self.absolutPos is not None and self.sensorData is not None:
            self.xyData = [round(self.sensorData*np.cos((self.relativAngle + self.theta)) + self.absolutPos[0],2), round(self.sensorData*np.sin((self.relativAngle + self.theta)) + self.absolutPos[1],2)]
            print(self.xyData, self.theta, self.sensorData)
            return self.xyData



class TOFInterface():
    def __init__(self):
        rospy.init_node('tofInterface', anonymous=True)

        self.sensor1 = DepthSensor(Y_ARM_POS, 0)
        self.sensor2 = DepthSensor(MID_POS, np.pi/4)
        self.sensor3 = DepthSensor(X_ARM_POS, np.pi/2)

        self.tof_data = None
        self.transformed_pts = None

        self.pos = None
        self.vel = None
        self.theta = None
        self.omega = None

        # subscribers
        rospy.Subscriber('sensor/tof_data', TOF, self.tofCallback)
        rospy.Subscriber('estimator/state', State, self.stateCallback)

        # publishers
        self.pub_tof_data = rospy.Publisher('transformed/tof_points', Float64MultiArray, queue_size=10)

    def tofCallback(self, msg):
        self.tof_data = np.array([msg.tof1, msg.tof2, msg.tof3])
        self.transformTOFData()

    def stateCallback(self, msg):
        self.pos = np.array(msg.pos.data)
        self.vel = np.array(msg.vel.data)
        self.theta = msg.theta.data
        self.omega = msg.omega.data

    ## Helper functions
    def transformTOFData(self):
        if self.tof_data is not None and self.pos is not None:
            self.transformed_pts = np.empty((0,2))
            self.tof_data = self.tof_data/TOF_CONVERSION

            if self.tof_data[0] > 0 and self.tof_data[0] != TOF_DEFAULT/TOF_CONVERSION:
                P1 = self.sensor1.xyCalculate(self.tof_data[0], self.theta, self.pos)
                self.transformed_pts = np.vstack((self.transformed_pts, P1))
                print("added1")

            if self.tof_data[1] > 0 and self.tof_data[1] != TOF_DEFAULT/TOF_CONVERSION:
                P2 = self.sensor2.xyCalculate(self.tof_data[1], self.theta, self.pos)
                self.transformed_pts = np.vstack((self.transformed_pts, P2))
                print("added2")

            if self.tof_data[2] > 0 and self.tof_data[2] != TOF_DEFAULT/TOF_CONVERSION:
                P3 = self.sensor3.xyCalculate(self.tof_data[2], self.theta, self.pos)
                self.transformed_pts = np.vstack((self.transformed_pts, P3))
                print("added3")

    ## Process functions
    def publish(self):
        if self.transformed_pts is not None:
            self.tof_pts = Float64MultiArray()
            self.tof_pts.data = self.transformed_pts.flatten()
            self.pub_tof_data.publish(self.tof_pts)

    def run(self):
        rate = rospy.Rate(RATE)
        while not rospy.is_shutdown():
            self.publish()
            # if self.sensor1.xyData is not None:
                # plt.scatter(self.sensor1.xyData[0],self.sensor1.xyData[1],marker = ".")
                #plt.scatter(self.sensor1.absolutPos[0], self.sensor1.absolutPos[1], c = "g", marker = "x")
            if self.sensor2.xyData is not None:
                plt.scatter(self.sensor2.xyData[0],self.sensor2.xyData[1],marker = ".")
                plt.scatter(self.sensor2.absolutPos[0], self.sensor2.absolutPos[1], c = "g", marker = "x")
            # if self.sensor3.xyData is not None:
                # plt.scatter(self.sensor3.xyData[0],self.sensor3.xyData[1],marker = ".")
                #plt.scatter(self.sensor3.absolutPos[0], self.sensor3.absolutPos[1], c = "g", marker = "x")

            plt.show()
            plt.pause(0.0001)
            rate.sleep()


if __name__ == '__main__':
    plt.ion()
    fig = plt.figure()
    plt.axis([-2,2,-2,2])
    plt.xlabel("x value in mm")
    plt.ylabel("y value in mm")
    tofInterface = TOFInterface()
    tofInterface.run()