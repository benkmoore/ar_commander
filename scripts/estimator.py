#!/usr/bin/env python

import numpy as np
import numpy.linalg as npl
import rospy

from ar_commander.msg import State, ControllerCmd
from geometry_msgs.msg import Pose2D

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
        self.x = None
        self.u = None
        self.pos_meas_loc = None
        self.sigma = 10*np.eye(3)
        self.A = np.eye(3)
        self.B = np.eye(3)
        self.C = np.eye(3)
        self.Q = 10*np.eye(3)   # noise/uncertainty estimate on predict process
        self.R = 0.01*np.eye(3) # noise/uncertainty estimate on meas. process

        # subscribers
        rospy.Subscriber('/pose', Pose2D, self.poseCallback)
        rospy.Subscriber('sensor/localize', Pose2D, self.localizeCallback)

        # publishers
        self.pub_state = rospy.Publisher('estimator/state', State, queue_size=10)

        # test
        self.pub_localize = rospy.Publisher('sensor/localize', Pose2D, queue_size=10)

    def poseCallback(self, msg):
        self.pos_meas = np.array([msg.x,msg.y])
        self.theta_meas = msg.theta

        # test
        loc = Pose2D()
        loc.x = msg.x + np.random.uniform(-0.25, 0.25, 1)
        loc.y = msg.y + np.random.uniform(-0.25, 0.25, 1)
        loc.theta = 0
        self.pub_localize.publish(loc)

    def localizeCallback(self, msg):
        self.pos_meas_loc = np.array([msg.x,msg.y]).reshape(-1,1)

    def localizeKF(self):
        if self.x is None:
            if self.pos_meas_loc is not None:
                self.x = np.concatenate((self.pos_meas_loc, np.zeros((1,1))))
            else:
                return

        if self.state is not None and self.pos_meas_loc is not None:
            self.predict()
            self.update()

    def predict(self):
        dt = 1. / RATE
        self.u = np.concatenate((self.state.vel.data,np.array([self.state.omega.data]))).reshape(-1,1)
        self.x_pred = np.matmul(self.A, self.x) + np.matmul(self.B, self.u)*dt
        self.sigma_pred = self.A.dot(self.sigma).dot(self.A.T) + self.Q

    def update(self):
        y_delta = np.concatenate((self.pos_meas_loc - self.x_pred[0:2], np.zeros((1,1)))).reshape(-1,1) # only have 2d localization, no theta measurement yet
        innovation_cov = npl.inv(self.C.dot(self.sigma_pred).dot(self.C.T) + self.R)

        self.x = self.x_pred + self.sigma_pred.dot(self.C.T).dot(innovation_cov).dot(y_delta)
        self.sigma = self.sigma_pred - self.sigma_pred.dot(self.C.T).dot(innovation_cov).dot(self.C).dot(self.sigma_pred)

        if self.pos_meas is not None:
            print("Truth: {}, {}, Est: {}, {}".format(self.pos_meas[0], self.pos_meas[1], self.x[0], self.x[1]))
            print("Error: {}".format(np.sqrt( (self.pos_meas[0]-self.x[0])**2 + (self.pos_meas[1]-self.x[1])**2 )))

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
            self.localizeKF()
            self.updateState()
            if self.state is not None:
                self.publish()
            rate.sleep()

if __name__ == '__main__':
    estimator = Estimator()
    estimator.run()
