#!/usr/bin/env python

import numpy as np
import numpy.linalg as npl
import rospy

from localization_filter import LocalizationFilter
from ar_commander.msg import State, ControllerCmd, Decawave
from utils.utils import wrapAngle

env = rospy.get_param("ENV")
if env == "sim":
    import configs.sim_params as params
elif env == "hardware":
    import configs.hardware_params as params
else:
    raise ValueError("Controller ENV: '{}' is not valid. Select from [sim, hardware]".format(env))

import configs.robot_v1 as rcfg

RATE = 100


class Estimator():

    def __init__(self):
        rospy.init_node('estimator')

        # timestep
        self.dt = 1. / RATE

        # incoming measurements, covariances (inputs) & flags
        self.pos_meas1 = None
        self.pos_meas2 = None
        self.loc_meas1_flag = None
        self.loc_meas2_flag = None
        self.theta_meas = None
        self.cov_pos_meas1 = None
        self.cov_pos_meas2 = None
        self.cov_theta_meas = None

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

        # update measurement flags
        self.loc_meas1_flag = msg.new_meas1.data
        self.loc_meas2_flag = msg.new_meas2.data


    def initPosKF(self):
        self.pos_state = None #state: [pos_x, pos_y, vel_x, vel_y]
        self.pos_cov = None #covariance
        A_pos = np.block([[np.eye(2), np.zeros((2,2))], [np.zeros((2,2)), np.zeros((2,2))]])
        B_pos = np.block([[self.dt*np.eye(2)], [np.eye(2)]])
        Q_pos = np.block([[params.positionFilterParams['Q'], np.zeros((2,2))], [np.zeros((2,2)), params.positionFilterParams['Q_d']]])
        self.pos_filter = LocalizationFilter(x0=np.zeros(4), sigma0=10*np.eye(4), A=A_pos, B=B_pos, Q=Q_pos)


    def initThetaKF(self):
        self.theta_state = None #state: [theta, theta_dot]
        self.theta_cov = None #covariance
        A_theta = np.array([[1, 0], [0, 0]])
        B_theta = np.array([[self.dt],[1]])
        Q_theta = np.array([[params.thetaFilterParams['Q'], 0], [0, params.thetaFilterParams['Q_d']]])
        self.theta_filter = LocalizationFilter(x0=np.zeros(2), sigma0=10*np.eye(2), A=A_theta, B=B_theta, Q=Q_theta, is_angle=True)


    def getPosFilterInputs(self):
        """Retreive pos filter step inputs"""
        u_pos = self.vel_cmd
        y_pos = np.array([])
        C_pos = np.empty((0,4))
        R_list = []

        if self.loc_meas1_flag:
            y_pos = np.append(y_pos, self.pos_meas1)
            C_pos = np.vstack((C_pos, np.block([np.eye(2), np.zeros((2, 2))])))
            R_list.extend(np.diag(self.cov_pos_meas1))
        if self.loc_meas2_flag:
            y_pos = np.append(y_pos, self.pos_meas2)
            C_pos = np.vstack((C_pos, np.block([np.eye(2), np.zeros((2, 2))])))
            R_list.extend(np.diag(self.cov_pos_meas2))

        R_pos = np.diag(R_list)

        return u_pos, y_pos, C_pos, R_pos


    def getThetaFilterInputs(self):
        # get theta filter inputs
        u_theta = np.array([self.omega_cmd])
        if self.loc_meas1_flag and self.loc_meas2_flag:
            y_theta = np.array([self.theta_meas])
            C_theta = np.array([1, 0]).reshape(1, 2)
            R_theta = self.cov_theta_meas
        else:
            y_theta = C_theta = R_theta = np.array([])

        return u_theta, y_theta, C_theta, R_theta


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
        else:  # run localization filter
            u_pos, y_pos, C_pos, R_pos = self.getPosFilterInputs()
            # if npl.norm(u_pos) < 0.3:
            #     u_pos = np.zeros(2)
            u_theta, y_theta, C_theta, R_theta = self.getThetaFilterInputs()

            self.pos_state, self.pos_cov = self.pos_filter.step(u_pos, y_pos, C_pos, R_pos)
            self.theta_state, self.theta_cov = self.theta_filter.step(u_theta, y_theta, C_theta, R_theta)

            self.loc_meas1_flag = self.loc_meas2_flag = False  # reset measurement flags

            self.state.pos.data = self.pos_state[0:2]
            self.state.vel.data = self.pos_state[2:4]
            self.state.theta.data = wrapAngle(self.theta_state[0])
            self.state.omega.data = self.theta_state[1]


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
