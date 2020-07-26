#!/usr/bin/env python

import numpy as np
import rospy

from ar_commander.msg import State, ControllerCmd

RATE = 10

class EstimatorOL():
    def __init__(self):
        rospy.init_node('estimator')

        # incominng measurements (inputs)
        self.vel_cmd = None
        self.omega_cmd = None

        # robot state (output)
        self.state = State()
        self.state.pos.data = np.zeros(2)
        self.state.vel.data = np.zeros(2)
        self.state.theta.data = 0
        self.state.omega.data = 0

        # subscribers
        rospy.Subscriber('/controller_cmds', ControllerCmd, self.controllerCmdCallback)

        # publishers
        self.pub_state = rospy.Publisher('estimator/state', State, queue_size=10)


    def controllerCmdCallback(self, msg):
        self.vel_cmd = np.array(msg.robot_vel.data)
        self.omega_cmd = msg.robot_omega.data

    def updateState(self):
        """
        main estimator function
        Uses measurements and previous state to estimate current state
        """

        # wait till we have pos and theta measurements
        if self.vel_cmd is None or self.omega_cmd is None:
            return

        dt = 1. / RATE
        print("cmd")
        print(self.vel_cmd)
        print(self.omega_cmd)
        print("state")
        print(self.state.pos.data)
        print(self.state.theta.data)
        print("")


        self.state.pos.data = self.state.pos.data + self.vel_cmd*dt
        self.state.theta.data = self.state.theta.data + self.omega_cmd*dt

    def publish(self):
        self.pub_state.publish(self.state)

    def run(self):
        rate = rospy.Rate(RATE) # 10 Hz
        while not rospy.is_shutdown():
            self.updateState()
            self.publish()
            rate.sleep()

if __name__ == '__main__':
    estimator = EstimatorOL()
    estimator.run()
