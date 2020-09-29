#!/usr/bin/env python

import sys
import rospy
import numpy as np
import numpy.linalg as npl

sys.path.append(rospy.get_param("AR_COMMANDER_DIR"))

from scripts.stateMachine.stateMachine import Mode
from ar_commander.msg import Trajectory, ControllerCmd, State
from std_msgs.msg import Int8, Bool

env = rospy.get_param("ENV")
if env == "sim":
    import configs.sim_params as params
elif env == "hardware":
    import configs.hardware_params as params
else:
    raise ValueError("Controller ENV: '{}' is not valid. Select from [sim, hardware]".format(env))

import configs.robot_v1 as rcfg

class ControlLoops():
    """Handle the controller loops"""

    def __init__(self):
        self.theta_error_sum = 0

    def resetController(self):
        self.theta_error_sum = 0

    ## Controller Functions
    def thetaController(self, theta_des, theta, omega):
        kp = params.thetaControllerGains['kp']
        ki = params.thetaControllerGains['ki']
        kd = params.thetaControllerGains['kd']

        theta_err = theta_des - theta
        idx = abs(theta_err) > np.pi
        theta_err -= 2*np.pi*np.sign(theta_err)*idx
        theta_dot_cmd = kp*theta_err + ki*self.theta_error_sum + kd*omega
        return theta_dot_cmd

    def pointController(self, pos_des, pos, vel):
        kp = params.pointControllerGains['kp']
        kd = params.pointControllerGains['kd']

        p_err = pos_des - pos
        v_cmd = kp*p_err + kd*vel
        return v_cmd

    def trajectoryController(self, pos, vel, theta, wp, wp_prev):
        # gains
        kp_pos = params.trajectoryControllerGains['kp_pos']
        kd_pos = params.trajectoryControllerGains['kd_pos']
        kp_th = params.trajectoryControllerGains['kp_th']
        k_ol = params.trajectoryControllerGains['k_ol']

        pos_des = wp[0:2]
        v_ol = (wp-wp_prev)[0:2]

        v_cmd = k_ol*v_ol + kp_pos*(pos_des-pos)
        theta_des = wp[2]
        theta_err = theta_des - theta
        idx = abs(theta_err) > np.pi
        theta_err -= 2*np.pi*np.sign(theta_err)*idx
        omega_cmd = kp_th*theta_err

        # saturate v_cmd
        v_cmd = np.clip(v_cmd, -params.max_vel, params.max_vel)

        return v_cmd, omega_cmd

class ControlNode():
    """
    Main controller node
    Handles inputs, outputs and main control logic
    """

    def __init__(self):
        rospy.init_node('controller', anonymous=True)

        # current state
        self.pos = None
        self.theta = None
        self.vel = 0
        self.omega = 0

        self.mode = None

        # navigation info
        self.trajectory = None
        self.traj_idx = 0

        # initialize controllers
        self.controllers = ControlLoops()

        # output commands
        self.robot_v_cmd = None
        self.robot_omega_cmd = None

        self.last_waypoint_flag = False

        # subscribers
        rospy.Subscriber('estimator/state', State, self.stateCallback)
        rospy.Subscriber('cmd_trajectory', Trajectory, self.trajectoryCallback)
        rospy.Subscriber('state_machine/mode', Int8, self.modeCallback)

        # publishers
        self.pub_cmds = rospy.Publisher('controller_cmds', ControllerCmd, queue_size=10)
        self.last_wp_pub = rospy.Publisher('controller/last_waypoint_flag', Bool, queue_size=10)

    ## Callback Functions
    def trajectoryCallback(self, msg):
        self.trajectory = np.vstack([msg.x.data,msg.y.data,msg.theta.data]).T
        self.traj_idx = 0
        self.controllers.resetController()

    def stateCallback(self, msg):
        self.pos = np.array(msg.pos.data)
        self.vel = np.array(msg.vel.data)
        self.theta = msg.theta.data
        self.omega = msg.omega.data

    def modeCallback(self,msg):
        self.mode = Mode(msg.data)

    ## Helper Functions
    def getWaypoint(self):
        if self.trajectory is not None:
            # determine waypoint
            wp = self.trajectory[self.traj_idx, :]

            # advance waypoints
            if npl.norm(wp[0:2]-self.pos) < params.wp_threshold and np.abs(wp[2]-self.theta) < params.theta_threshold and self.traj_idx < self.trajectory.shape[0]-1:
                self.traj_idx += 1
                wp = self.trajectory[self.traj_idx, :]

            if self.traj_idx == 0:
                wp_prev = np.hstack([self.pos, self.theta])
            else:
                wp_prev = self.trajectory[self.traj_idx-1, :]

            return wp, wp_prev

    ## Main Loops
    def controlLoop(self):
        # default behavior
        self.wheel_w_cmd = np.zeros(rcfg.N)
        self.wheel_phi_cmd = np.zeros(rcfg.N) # rads
        self.robot_v_cmd = np.zeros(2)
        self.robot_omega_cmd = 0

        self.last_waypoint_flag = False

        if self.mode == Mode.TRAJECTORY:
            wp, wp_prev = self.getWaypoint()
            if self.traj_idx < self.trajectory.shape[0]-1:
                v_des, w_des = self.controllers.trajectoryController(self.pos, self.vel, self.theta, wp, wp_prev)
            else:
                v_des = self.controllers.pointController(wp[0:2], self.pos, self.vel)
                w_des = self.controllers.thetaController(wp[2], self.theta, self.omega)
                self.last_waypoint_flag = True

            self.robot_v_cmd = v_des
            self.robot_omega_cmd = w_des

    def publish(self):
        """ publish cmd messages """
        cmd = ControllerCmd()
        cmd.robot_vel.data = self.robot_v_cmd
        cmd.robot_omega.data = self.robot_omega_cmd

        self.pub_cmds.publish(cmd)

        flag = Bool()
        flag.data = self.last_waypoint_flag
        self.last_wp_pub.publish(flag)


    def run(self):
        rate = rospy.Rate(params.CONTROLLER_RATE)
        while not rospy.is_shutdown():
            self.controlLoop()
            self.publish()
            rate.sleep()


if __name__ == '__main__':
    controller = ControlNode()
    controller.run()
