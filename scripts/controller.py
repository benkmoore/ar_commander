#!/usr/bin/env python

import rospy
import numpy as np
import numpy.linalg as npl
from stateMachine import Mode

from ar_commander.msg import Trajectory, ControllerCmd, State
from std_msgs.msg import Int8

## Global variables:
RATE = 10                       # rate in Hz

# TODO: Move these into robot configuration file
N = 4                           # number of wheels
R1 = np.array([0.075, 0.425])   # position of wheels along arm 1
R2 = np.array([0.075, 0.425])   # "" arm 2

class ControlLoops():
    """Handle the controller loops"""

    def __init__(self):
        self.theta_error_sum = 0

    def resetController(self):
        self.theta_error_sum = 0

    ## Controller Functions
    def thetaController(self, theta_des, theta, omega):
        kp = 3
        ki = 0.03
        kd = 5e-8

        theta_err = theta_des - theta
        theta_dot_cmd = kp*theta_err + ki*self.theta_error_sum + kd*omega
        return theta_dot_cmd

    def pointController(self, pos_des, pos, vel):
        kp = 10
        kd = 0

        p_err = pos_des - pos
        v_cmd = kp*p_err + kd*vel
        return v_cmd

    def trajectoryController(self, pos, vel, theta, wp, wp_prev):
        # gains
        kp_pos = 12
        kp_th = 0.75
        kd_pos = 0.5
        v_mag = 6

        # fit line/poly and get derivative
        x = np.array([wp_prev[0], wp[0]])
        y = np.array([wp_prev[1], wp[1]])

        if wp[0] == wp_prev[0]:
            p_y = lambda y: wp[1]
        else:
            p_y = np.poly1d(np.polyfit(x, y, 1)) # desired y

        if wp[1] == wp_prev[1]:
            p_x = lambda x: wp[0]
        else:
            p_x = np.poly1d(np.polyfit(y, x, 1)) # desired x

        x_des = p_x(pos[1])
        y_des = p_y(pos[0])
        pos_des = np.array([x_des,y_des])

        v_des = (wp-wp_prev)[0:2]

        v_cmd = kd_pos*v_des + kp_pos*(pos_des-pos)
        v_cmd = v_mag * v_cmd/npl.norm(v_cmd)

        theta_des = wp[2]
        theta_err = theta_des - theta
        omega_cmd = kp_th*theta_err

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
        self.phi_cmd = None
        self.V_cmd = None

        # subscribers
        rospy.Subscriber('estimator/state', State, self.stateCallback)
        rospy.Subscriber('/cmd_trajectory', Trajectory, self.trajectoryCallback)
        rospy.Subscriber('state_machine/mode', Int8, self.modeCallback)

        # publishers
        self.pub_cmds = rospy.Publisher('/controller_cmds', ControllerCmd, queue_size=10)

    ## Callback Functions
    def trajectoryCallback(self, msg):
        if self.trajectory is None:
            self.trajectory = np.vstack([msg.x.data,msg.y.data,msg.theta.data]).T

    def stateCallback(self, msg):
        self.pos = np.array(msg.pos.data)
        self.vel = np.array(msg.vel.data)
        self.theta = msg.theta.data
        self.omega = msg.omega.data

    def modeCallback(self,msg):
        self.mode = Mode(msg.data)

    ## Helper Functions
    def getWaypoint(self):
        # determine waypoint
        wp = self.trajectory[self.traj_idx, :]

        # advance waypoints
        if npl.norm(wp[0:2]-self.pos) < 0.25 and self.traj_idx < self.trajectory.shape[0]-1:
            self.traj_idx += 1
            wp = self.trajectory[self.traj_idx, :]

        if self.traj_idx == 0:
            wp_prev = np.hstack([self.pos, self.theta])
        else:
            wp_prev = self.trajectory[self.traj_idx-1, :]
        return wp, wp_prev

    def convert2MotorInputs(self, v_cmd_gf, omega_cmd):
        """Convert velocity and omega commands to motor inputs"""

        # convert inputs to robot frame velocity commands
        R = np.array([[np.cos(self.theta), np.sin(self.theta)],     # rotation matrix
                      [-np.sin(self.theta), np.cos(self.theta)]])
        v_cmd_rf = np.dot(R, v_cmd_gf)[:,np.newaxis]        # convert to robot frame

        v_th1 = np.vstack([-R1*omega_cmd, np.zeros(N/2)])
        v_th2 = np.vstack([np.zeros(N/2), R2*omega_cmd])
        v_th_rf = np.hstack([v_th1, v_th2])

        V_cmd = v_cmd_rf + v_th_rf

        # Convert to |V| and phi
        V_norm_cmd = npl.norm(V_cmd, axis=0)
        phi_cmd = np.arctan2(V_cmd[1,:], V_cmd[0,:]) + np.pi/2

        return V_norm_cmd, phi_cmd

    ## Main Loops
    def controlLoop(self):
        # default behavior
        self.V_cmd = np.zeros(N)
        self.phi_cmd = np.zeros(N) # rads

        if self.mode == Mode.TRAJECTORY:
            wp, wp_prev = self.getWaypoint()
            if self.traj_idx < self.trajectory.shape[0]-1:
                v_des, w_des = self.controllers.trajectoryController(self.pos, self.vel, self.theta, wp, wp_prev)
            else:
                v_des = self.controllers.pointController(wp[0:2], self.pos, self.vel)
                w_des = self.controllers.thetaController(wp[2], self.theta, self.omega)

            self.V_cmd, self.phi_cmd = self.convert2MotorInputs(v_des,w_des)

    def publish(self):
        """ publish cmd messages """
        self.cmds = ControllerCmd()
        self.cmds.velocity_arr.data = self.V_cmd
        self.cmds.phi_arr.data = self.phi_cmd
        self.pub_cmds.publish(self.cmds)

    def run(self):
        rate = rospy.Rate(RATE) # 10 Hz
        while not rospy.is_shutdown():
            self.controlLoop()
            self.publish()
            rate.sleep()


if __name__ == '__main__':
    controller = ControlNode()
    controller.run()
