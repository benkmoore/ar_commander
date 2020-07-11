#!/usr/bin/env python

import numpy as np
import rospy

from ar_commander.msg import State
from geometry_msgs.msg import Pose2D

RATE = 10

class Estimator():
    def __init__(self):
        rospy.init_node('estimator')

        # incominng measurements (inputs)
        self.pos_meas = None
        self.theta_meas = None

        # robot state (output)
        self.state = None

        # subscribers
        rospy.Subscriber('/pose', Pose2D, self.poseCallback)

        # publishers
        self.pub_state = rospy.Publisher('estimator/state', State, queue_size=10)

    def poseCallback(self, msg):
        self.pos_meas = np.array([msg.x,msg.y])
        self.theta_meas = msg.theta

    def updateState(self):
        """
        main estimator function
        Uses measurements and previous state to estimate current state
        """

        # wait till we have pos and theta measurements
        if self.pos_meas is None or self.theta_meas is None:
            return

        if self.state is None:   # initialize state
            self.state = State()
            self.state.vel.data = np.zeros(2)
            self.state.omega.data = 0
        else:
            dt = 1. / RATE
            self.state.vel.data = (self.pos_meas - self.state.pos.data)/dt
            self.state.omega.data = (self.theta_meas - self.state.theta.data)/dt

        self.state.pos.data = self.pos_meas
        self.state.theta.data = self.theta_meas

    def publish(self):
        self.pub_state.publish(self.state)

    def run(self):
        rate = rospy.Rate(RATE) # 10 Hz
        while not rospy.is_shutdown():
            self.updateState()
            if self.state is not None:
                self.publish()
            rate.sleep()

if __name__ == '__main__':
    estimator = Estimator()
    estimator.run()
