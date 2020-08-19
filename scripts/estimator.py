#!/usr/bin/env python

import numpy as np
import numpy.linalg as npl
import rospy
import sys

from localization_filter import LocalizationFilter

from ar_commander.msg import State, ControllerCmd
from geometry_msgs.msg import Pose2D

env = rospy.get_param("ENV")
sys.path.append(rospy.get_param("AR_COMMANDER_DIR"))

if env == "sim":
    import configs.sim_params as params
elif env == "hardware":
    import configs.hardware_params as params
else:
    raise ValueError("Controller ENV: '{}' is not valid. Select from [sim, hardware]".format(env))

RATE = 10


class Estimator():
    def __init__(self):
        rospy.init_node('estimator')

        # timestep
        self.dt = 1. / RATE

        # incoming measurements (inputs)
        self.pos_meas = None
        self.theta_meas = None

        # robot state (output)
        self.state = None

        # init localization KF
        self.x = None #state
        self.sigma = None #covariance
        A = np.eye(6)
        B = np.eye(6)
        C = np.eye(6)
        Q = np.block([[params.localizationFilterParams['Q'], np.zeros((3,3))], [np.zeros((3,3)), params.localizationFilterParams['Q_d']]])
        R = np.block([[params.localizationFilterParams['R'], np.zeros((3,3))], [np.zeros((3,3)), params.localizationFilterParams['R_d']]])
        self.localization_filter = LocalizationFilter(dt=self.dt, x0=None, sigma0=10*np.eye(6), A=A, B=B, C=C, Q=Q, R=R)

        # subscribers
        rospy.Subscriber('sensor/decawave_measurement', Pose2D, self.localizeCallback)

        # publishers
        self.pub_state = rospy.Publisher('estimator/state', State, queue_size=10)

    def localizeCallback(self, msg):
        self.pos_meas = np.array([msg.x,msg.y])
        self.theta_meas = np.array([msg.theta])

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
            self.state.pos.data = self.pos_meas
            self.state.theta.data = self.theta_meas
            self.state.vel.data = np.zeros(2)
            self.state.omega.data = 0
        else: # run localization filter
            d = np.concatenate(((self.pos_meas - self.state.pos.data),(self.theta_meas - self.state.theta.data)))/self.dt #derviative
            y = np.concatenate((self.pos_meas, self.theta_meas, d)) # measurement
            u = np.concatenate((self.state.vel.data, np.array([self.state.omega.data]), np.zeros(3))).reshape(-1)
            self.x, self.sigma = self.localization_filter.step(u, y)

            self.state.pos.data = self.x[0:2]
            self.state.theta.data = self.x[2]
            self.state.vel.data = self.x[3:5]
            self.state.omega.data = self.x[5]

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
