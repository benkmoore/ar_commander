#!/usr/bin/env python

import numpy as np
import numpy.linalg as npl
import rospy
import sys

from localization_filter import LocalizationFilter
from ar_commander.msg import State, ControllerCmd, Decawave

env = rospy.get_param("ENV")
sys.path.append(rospy.get_param("AR_COMMANDER_DIR"))

if env == "sim":
    import configs.sim_params as params
elif env == "hardware":
    import configs.hardware_params as params
else:
    raise ValueError("Controller ENV: '{}' is not valid. Select from [sim, hardware]".format(env))

import configs.robot_v1 as rcfg

RATE = 10


class Estimator():

    def __init__(self):
        rospy.init_node('estimator')

        # timestep
        self.dt = 1. / RATE

        # incoming measurements & covariances (inputs)
        self.pos_meas1 = None
        self.pos_meas2 = None
        self.theta_meas = None
        self.cov_pos_meas1 = None
        self.cov_pos_meas2 = None
        self.cov_theta_meas = None

        # new decwave measurement flag
        self.decawave_flag = False

        # controller cmds
        self.vel_cmd = None
        self.omega_cmd = None

        # robot state (output)
        self.state = None

        # init KFs for postion & theta
        self.initPosKF()
        self.initThetaKF()

        # subscribers
        rospy.Subscriber('sensor/decawave_measurement', Decawave, self.decawaveCallback)
        rospy.Subscriber('controller_cmds', ControllerCmd, self.controllerCmdCallback)

        # publishers
        self.pub_state = rospy.Publisher('estimator/state', State, queue_size=10)


    def controllerCmdCallback(self, msg):
        self.vel_cmd = np.array(msg.robot_vel.data)
        self.omega_cmd = msg.robot_omega.data


    def decawaveCallback(self, msg):
        self.decawave_flag = True # received new decwave measurement

        self.theta_meas = msg.theta.data
        self.cov_theta_meas = msg.cov_theta.data

        # transform pos_meas to center corner of robot
        tf_angle = self.state.theta.data if self.state is not None else self.theta_meas
        self.pos_meas1 = np.array([msg.x1.data+rcfg.L*np.sin(tf_angle),
                                    msg.y1.data-rcfg.L*np.cos(tf_angle)]) # sensor on robot Y axis arm
        self.pos_meas2 = np.array([msg.x2.data-rcfg.L*np.cos(tf_angle),
                                    msg.y2.data-rcfg.L*np.sin(tf_angle)]) # sensor on robot X axis arm
        self.cov_pos_meas1 = np.reshape(msg.cov1.data, (2,2))
        self.cov_pos_meas2 = np.reshape(msg.cov2.data, (2,2))

    def initPosKF(self):
        self.pos_state = None #state: [pos_x, pos_y, vel_x, vel_y]
        self.pos_cov = None #covariance
        A_pos = np.block([[np.eye(2), np.zeros((2,2))], [np.zeros((2,2)), np.zeros((2,2))]])
        B_pos = np.block([[self.dt*np.eye(2)], [np.eye(2)]])
        C_pos = np.block([[np.eye(2), np.zeros((2,2))], [np.eye(2), np.zeros((2,2))]])
        Q_pos = np.block([[params.positionFilterParams['Q'], np.zeros((2,2))], [np.zeros((2,2)), params.positionFilterParams['Q_d']]])
        self.pos_filter = LocalizationFilter(x0=np.zeros(4), sigma0=10*np.eye(4), A=A_pos, B=B_pos, C=C_pos, Q=Q_pos, angle=False)


    def initThetaKF(self):
        self.theta_state = None #state: [theta, theta_dot]
        self.theta_cov = None #covariance
        A_theta = np.array([[1, 0], [0, 0]])
        B_theta = np.array([[self.dt],[1]])
        C_theta = np.array([1, 0]).reshape(1,2)
        Q_theta = np.array([[params.thetaFilterParams['Q'], 0], [0, params.thetaFilterParams['Q_d']]])
        self.theta_filter = LocalizationFilter(x0=np.zeros(2), sigma0=10*np.eye(2), A=A_theta, B=B_theta, C=C_theta, Q=Q_theta, angle=True)


    def updateState(self):
        """
        main estimator function
        Uses measurements and previous state to estimate current state
        """

        # wait till we have pos and theta measurements
        if self.pos_meas1 is None or self.pos_meas2 is None or self.theta_meas is None:
            return

        if self.state is None:   # initialize state
            self.state = State()
            self.state.pos.data = (self.pos_meas1 + self.pos_meas2)/2
            self.state.theta.data = self.theta_meas
            self.state.vel.data = np.zeros(2)
            self.state.omega.data = 0
        else: # run localization filter
            u_pos = self.vel_cmd
            u_theta = np.array([self.omega_cmd])

            if self.decawave_flag: # new measurement
                y_pos = np.concatenate((self.pos_meas1, self.pos_meas2))
                R_pos = np.block([[self.cov_pos_meas1, np.zeros((2,2))], [np.zeros((2,2)), self.cov_pos_meas2]])
                y_theta = np.array([self.theta_meas])
                R_theta = self.cov_theta_meas
            else: # no new measurement
                y_pos = R_pos = y_theta = R_theta = None

            self.pos_state, self.pos_cov = self.pos_filter.step(u_pos, y_pos, R_pos)
            self.theta_state, self.theta_cov = self.theta_filter.step(u_theta, y_theta, R_theta)

            self.state.pos.data = self.pos_state[0:2]
            self.state.vel.data = self.pos_state[2:4]
            self.state.theta.data = self.theta_state[0]
            self.state.omega.data = self.theta_state[1]

            self.decawave_flag = False # reset decawave measurement flag


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
