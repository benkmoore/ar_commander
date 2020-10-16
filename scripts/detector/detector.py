#!/usr/bin/env python

import numpy as np
import rospy
import sys

sys.path.append(rospy.get_param("AR_COMMANDER_DIR"))

from rectangle_detector import RectangleDetector
from scripts.stateMachine.stateMachine import Mode
from ar_commander.msg import Object, TOF, State
from std_msgs.msg import Int8, Float64MultiArray

import matplotlib.pyplot as plt

RATE = 10

class Detector:
    def __init__(self):
        rospy.init_node('detector')

        # input/ouput data
        self.tof_pts = None
        self.point_arr = np.empty((0,2))
        self.corners = None

        # state variables
        self.pos = None
        self.vel = None
        self.theta = None
        self.omega = None
        self.mode = None

        # subscribers
        rospy.Subscriber('transformed/tof_points', Float64MultiArray, self.tofCallback)
        rospy.Subscriber('estimator/state', State, self.stateCallback)
        rospy.Subscriber('state_machine/mode', Int8, self.modeCallback)

        # publishers
        self.pub_object = rospy.Publisher('object', Object, queue_size=10)

    ## Callback functions
    def modeCallback(self,msg):
        self.mode = Mode(msg.data)

    def tofCallback(self, msg):
        self.tof_pts = np.asarray(msg.data).reshape(-1,2)
        if self.mode == Mode.SEARCH:
            self.point_arr = np.vstack((self.point_arr, self.tof_pts))
        else:
            self.point_arr = np.empty((0,2))

    def stateCallback(self, msg):
        self.pos = np.array(msg.pos.data)
        self.vel = np.array(msg.vel.data)
        self.theta = msg.theta.data
        self.omega = msg.omega.data

    ## Process functions
    def publish(self):
        if self.corners is not None:
            print(self.corners)
            for corners in self.corners:
                # plot edges
                plot_points = np.vstack((corners, corners[0,:]))
                plt.plot(plot_points[:,0], plot_points[:,1], "-r")
            # self.object_msg = Object()
            # self.object_msg.data = np.asarray(self.corners).flatten()
            # self.pub_object.publish()

    def detectObjects(self):
        if self.point_arr.size > 0:
            self.rectangleDetector = RectangleDetector(self.point_arr)
            self.corners = self.rectangleDetector.find_rectangles()

    def run(self):
        rate = rospy.Rate(RATE)
        while not rospy.is_shutdown():
            if self.mode == Mode.SEARCH: # only detect in search mode
                self.detectObjects()
                self.publish()
            plt.show()
            plt.pause(0.0001)
            rate.sleep()

if __name__ == "__main__":
    plt.ion()
    fig = plt.figure()
    plt.axis([-2,2,-2,2])
    plt.xlabel("x value in m")
    plt.ylabel("y value in m")
    detector = Detector()
    detector.run()