#!/usr/bin/env python

import rospy
import numpy as np
import numpy.linalg as npl
import sys

from ar_commander.msg import MotorCmd, ControllerCmd, State

env = rospy.get_param("ENV")
sys.path.append(rospy.get_param("AR_COMMANDER_DIR"))

if env == "sim":
    import configs.sim_params as params
elif env == "hardware":
    import configs.hardware_params as params
else:
    raise ValueError("Controller ENV: '{}' is not valid. Select from [sim, hardware]".format(env))

import configs.robot_v1 as rcfg



class ActuatorInterface():
	""" 
	Actuator interface node
	Handles conversion of controller commands to actuator level.
	Outputs commands to actuators via the 'motor_cmds' message.
	"""

	def __init__(self):
		rospy.init_node('actuator_interface')

		# robot state
		self.pos = None
		self.theta = None

		# controller & actuator commands
		self.v_cmd = None
		self.w_cmd = None
		self.wheel_phi_cmd = None
		self.wheel_w_cmd = None

		self.phi_prev = np.zeros(rcfg.N)   # can initialize to 0 as it will only affect first command

		# subscribers
		rospy.Subscriber('controller_cmds', ControllerCmd, self.controllerCmdCallback)
		rospy.Subscriber('estimator/state', State, self.stateCallback)

		# publishers
		self.pub_motor_cmds = rospy.Publisher('motor_cmds', MotorCmd, queue_size=10)


	def stateCallback(self, msg):
		self.pos = np.array(msg.pos.data)
		self.theta = msg.theta.data


	def controllerCmdCallback(self, msg):
		self.v_cmd = np.array(msg.robot_vel.data)
		self.w_cmd = msg.robot_omega.data


	def convert2MotorInputs(self, v_cmd_gf, omega_cmd):
			"""Convert velocity and omega commands to motor inputs"""

			# convert inputs to robot frame velocity commands
			R = np.array([[np.cos(self.theta), np.sin(self.theta)],     # rotation matrix
						[-np.sin(self.theta), np.cos(self.theta)]])
			v_cmd_rf = np.dot(R, v_cmd_gf)[:,np.newaxis]        # convert to robot frame

			v_th1 = np.vstack([-rcfg.R1*omega_cmd, np.zeros(rcfg.N/2)])
			v_th2 = np.vstack([np.zeros(rcfg.N/2), rcfg.R2*omega_cmd])
			v_th_rf = np.hstack([v_th1, v_th2])

			v_xy = v_cmd_rf + v_th_rf

			# Convert to |V| and phi
			v_wheel = npl.norm(v_xy, axis=0)
			phi_cmd = np.arctan2(v_xy[1,:], v_xy[0,:])

			# pick closest phi
			phi_diff = phi_cmd - self.phi_prev
			idx = abs(phi_diff) > np.pi/2
			phi_cmd -= np.pi*np.sign(phi_diff)*idx
			v_wheel *= -1*idx + 1*~idx

			# enforce physical bounds
			idx_upper = phi_cmd > rcfg.phi_bounds[1]     # violates upper bound
			idx_lower = phi_cmd < rcfg.phi_bounds[0]     # violates lower bound

			phi_cmd -= np.pi*idx_upper - np.pi*idx_lower
			v_wheel *= -1*(idx_upper+idx_lower) + 1*~(idx_upper + idx_lower)

			# map to desired omega (angular velocity) of wheels: w = v/r
			w_wheel = v_wheel/rcfg.wheel_radius

			return w_wheel, phi_cmd


	def publish(self):
		cmd = MotorCmd()
		cmd.omega_arr.data = self.wheel_w_cmd
		cmd.phi_arr.data = self.wheel_phi_cmd
		self.pub_motor_cmds.publish(cmd)


	def actuateMotors(self):
		self.wheel_w_cmd, self.wheel_phi_cmd = self.convert2MotorInputs(self.v_cmd, self.w_cmd)
		self.phi_prev = self.wheel_phi_cmd     # store previous command


	def run(self):
		rate = rospy.Rate(params.CONTROLLER_RATE)
		while not rospy.is_shutdown():
			if self.theta is not None:
				self.actuateMotors()
				self.publish()
			rate.sleep()


if __name__ == '__main__':
	motors = ActuatorInterface()
	motors.run()