#!/usr/bin/env python

import numpy as np

from rectangle_detector import RectangleDetector
from ar_commander.msg import Object, TOF, State

RATE = 10

class Detector:
    def __init__(self):
        rospy.init_node('detector')

        # input/ouput data
        self.tof_data = None
        self.point_arr = np.array()
        self.corners = None

        # state variables
        self.pos = None
        self.vel = None
        self.theta = None
        self.omega = None

        self.rectangleDetector = RectangleDetector(self.point_arr)

        # subscribers
        rospy.Subscriber('transformed/tof_data', TOF, self.tofCallback)
        rospy.Subscriber('estimator/state', State, self.stateCallback)

        # publishers
        self.pub_object = rospy.Publisher('object', Object, queue_size=10)

    ## Callback functions
    def tofCallback(self, msg):
        self.tof_data = np.array([msg.tof1, msg.tof2, msg.tof3])
        self.point_arr = np.append(self.point_arr, self.tof_data)

    def stateCallback(self, msg):
        self.pos = np.array(msg.pos.data)
        self.vel = np.array(msg.vel.data)
        self.theta = msg.theta.data
        self.omega = msg.omega.data

    ## Process functions
    def publish(self):
        if self.corners is not None:
            self.object_msg = Object()
            self.object_msg.data = self.corners.flatten()
            self.pub_object.publish()

    def detectObjects(self):
        self.rectangleDetector(self.point_arr)
        self.corners = self.rectangleDetector.find_rectangles()

    def run(self):
        rate = rospy.Rate(RATE)
        while not rospy.is_shutdown():
            if self.mode == Mode.SEARCH: # only detect in search mode
                self.detectObjects()
                self.publish()
            rate.sleep()

if __name__ == "__main__":
    detector = Detector()
    detector.run()