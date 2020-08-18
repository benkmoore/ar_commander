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

        # incoming measurements (inputs)
        self.pos_meas = None
        self.theta_meas = None

        # robot state (output)
        self.state = None

        # init localization KF
        A = np.eye(3)
        B = np.eye(3)
        C = np.eye(3)
        Q = params.localizationFilterParams['Q'] # noise/uncertainty estimate on predict process
        R = params.localizationFilterParams['R'] # noise/uncertainty estimate on meas. process
        self.localization_filter = LocalizationFilter(rate=RATE, x0=None, sigma0=10*np.eye(3), A=A, B=B, C=C, Q=Q, R=R)

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
            if self.pos_meas is not None and self.state is not None:
                z = np.concatenate((self.pos_meas, self.theta_meas)) # measurement
                u = np.concatenate((self.state.vel.data, np.array([self.state.omega.data]).reshape(-1)))
                self.localization_filter.run(u, z)
            if self.state is not None:
                self.publish()

            if self.pos_meas is not None and self.localization_filter.x is not None:
                print("Truth: {}, {}, Est: {}, {}".format(self.pos_meas[0], self.pos_meas[1], self.localization_filter.x[0], self.localization_filter.x[1]))
                print("Error: {}".format(np.sqrt( (self.pos_meas[0]-self.localization_filter.x[0])**2 + (self.pos_meas[1]-self.localization_filter.x[1])**2 )))

            rate.sleep()

if __name__ == '__main__':
    estimator = Estimator()
    estimator.run()
