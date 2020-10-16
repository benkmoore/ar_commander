#!/usr/bin/env python

import numpy as np
import rospy
import time
from ar_commander.msg import Decawave, State, ControllerCmd

RATE = 3

class Logger():
	def __init__(self):
	    rospy.init_node('logger', anonymous=True)
	    self.state_pos = None
	    self.state_vel = None
	    self.state_theta = None
	    self.state_omega = None

	    self.decawave_x1 = None
	    self.boardY = None
	    self.boardX = None

	    self.decawave_y1 = None
	    self.decawave_x2 = None
	    self.decawave_y2 = None
	    self.decawave_theta = None

            self.cmd_omega_arr = None
            self.cmd_phi_arr = None
            self.cmd_robot_vel = None
            self.cmd_robot_omega = None


	    rospy.Subscriber('sensor/decawave_measurement', Decawave, self.decaCallback)
	    rospy.Subscriber('estimator/state', State, self.stateCallback)
	    rospy.Subscriber('controller_cmds', ControllerCmd, self.cmdCallback)


	def decaCallback(self, msg):
	    self.boardY = np.array([msg.x1.data,msg.y1.data])
	    self.boardX = np.array([msg.x2.data,msg.y2.data])

            # self.decawave_x1 = msg.x1.data
            # self.decawave_y1 = msg.y1.data
            # self.decawave_x2 = msg.x2.data
            # self.decawave_y2 = msg.y2.data
            self.decawave_theta = msg.theta.data
            self.Y_confidence = msg.confidence1.data
            self.X_confidence = msg.confidence2.data

	def stateCallback(self, msg):
            self.state_pos = np.array(msg.pos.data)
            self.state_vel = np.array(msg.vel.data)
	    self.state_theta = msg.theta.data
	    self.state_omega = msg.omega.data


	def cmdCallback(self, msg):
            self.cmd_omega_arr = np.array(msg.omega_arr.data)
            self.cmd_phi_arr = np.array(msg.phi_arr.data)
            self.cmd_robot_vel = np.array(msg.robot_vel.data)
            self.cmd_robot_omega = msg.robot_omega.data


	def run(self):
            rate = rospy.Rate(RATE)
            while not rospy.is_shutdown():
                if not self.state_pos is None:
                    print('Estimator: xy: {}, vel: {}, theta: {}, omega: {}').format(self.state_pos, self.state_vel, self.state_theta, self.state_omega)
                if not self.boardY is None:
                    print('Decawaves: boardY: {}, boardX: {}, theta: {}, conf_Y: {}, conf_X {}').format(self.boardY, self.boardX, self.decawave_theta, self.Y_confidence, self.X_confidence)
                if not self.cmd_omega_arr is None:
		        	print('ControllerCmd: Omega: {}, Phi: {}, robotVel: {}, robotOmega: {}\n').format(self.cmd_omega_arr, self.cmd_phi_arr, self.cmd_robot_vel, self.cmd_robot_omega)

            	rate.sleep()



if __name__ == '__main__':
    #sleep for a few seconds until stack is launched
    time.sleep(4)
    logger = Logger()
    logger.run()
