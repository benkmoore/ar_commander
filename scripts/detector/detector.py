#!/usr/bin/env python

import numpy as np

from rectangle_detector import RectangleDetector
from ar_commander.msg import Object, TOF, State

RATE = 3

class Detector:
    def __init__(self):
        rospy.init_node('detector')

        # input/ouput data
        self.tof_data = None
        self.transformed_pts = None
        self.point_arr = np.array()
        self.corners = None

        # state variables
        self.pos = None
        self.vel = None
        self.theta = None
        self.omega = None

        self.rectangleDetector = RectangleDetector(self.point_arr)

        # subscribers
        rospy.Subscriber('sensor/tof_data', TOF, self.tofCallback)
        rospy.Subscriber('estimator/state', State, self.stateCallback)

        # publishers
        self.pub_object = rospy.Publisher('object', Object, queue_size=10)

    ## Callback functions
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
            d1 = np.sqrt(rcfg.L**2 + self.tof_data[0]**2)
            a1 = np.arctan(rcfg.L/self.tof_data[0])
            p1_x = self.pos[0] + d1*np.cos(a1 + self.theta)
            p1_y = self.pos[1] + d1*np.sin(a1 + self.theta)

            d2 = self.tof_data[1]
            p2_x = self.pos[0] + d2*np.cos((np.pi/4) + self.theta)
            p2_y = self.pos[1] + d2*np.sin((np.pi/4) + self.theta)

            d3 = np.sqrt(rcfg.L**2 + self.tof_data[2]**2)
            a3 = np.arctan(rcfg.L/self.tof_data[2])
            p3_x = self.pos[0] - d3*np.cos(a3 + (np.pi/2) - self.theta)
            p3_y = self.pos[1] + d3*np.sin(a3 + (np.pi/2) - self.theta)

            self.trasnformed_pts = np.array([[p1_x, p1_y], [p2_x, p2_y], [p3_x, p3_y]])
            self.point_arr = np.append(self.point_arr, self.trasnformed_pts)

    ## Process functions
    def publish(self)
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