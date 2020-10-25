#!/usr/bin/env python

import sys
import rospy
import time
import numpy as np
import numpy.linalg as npl
import scipy.signal as sps
import scipy.interpolate as spi

sys.path.append(rospy.get_param("AR_COMMANDER_DIR"))

from ar_commander.msg import Trajectory, ControllerCmd, State
from std_msgs.msg import Int8, Bool

env = rospy.get_param("ENV")
if env == "sim":
    import configs.sim_params as params
elif env == "hardware":
    import configs.hardware_params as params
else:
    raise ValueError("Controller ENV: '{}' is not valid. Select from [sim, hardware]".format(env))

from scripts.stateMachine.stateMachine import Mode
import configs.robot_v1 as rcfg

class Controller(object):
    def __init__(self, ctrl_tf):
        self.num = ctrl_tf['num']
        self.den = ctrl_tf['den']
        self.z = None

    def saturateCmds(self, v_cmd):
        return np.clip(v_cmd, -params.max_vel, params.max_vel)

    def controllerTF(self, error):
        if self.z is None:
            self.z = sps.lfiltic(self.num, self.den, y=np.zeros_like(self.den)) # initial filter delays
            self.z = np.tile(self.z, error.shape)
        u, self.z = sps.lfilter(self.num, self.den, error, axis=1, zi=self.z)
        v_pos = u[0:2, 0] # vel cmd due to pos error
        v_vel = u[3:, 0]  # vel cmd due to vel error
        v_cmd = v_pos + v_vel
        omega_cmd = u[2, 0] # omega cmd due to theta error

        return v_cmd, omega_cmd


class TrajectoryController(Controller):
    def __init__(self, ctrl_tf):
        super(TrajectoryController, self).__init__(ctrl_tf)

    def fitSpline2Trajectory(self, trajectory, pos, theta):
        if len(trajectory) == 1:
            default_start_pt = np.hstack((pos, theta, 0))
            trajectory = np.vstack((default_start_pt, trajectory))
        x, y, theta, t = np.split(trajectory.reshape(-1, 4), 4, axis=1)
        t = t.reshape(-1)

        self.x_spline = spi.CubicSpline(t, x, bc_type='clamped', extrapolate='False') # output: x_des, input: t
        self.y_spline = spi.CubicSpline(t, y, bc_type='clamped', extrapolate='False') # output: y_des, input: t
        self.theta_spline = spi.CubicSpline(t, theta, bc_type='clamped', extrapolate='False') # output: theta_des, input: t

        self.v_x = self.x_spline.derivative()
        self.v_y = self.y_spline.derivative()

        self.t = t
        self.init_traj_time = time.time()

    def getControlCmds(self, pos, theta, vel):
        t = time.time() - self.init_traj_time
        if t < self.t[0]: t = self.t[0] # bound calls between start and end time
        if t > self.t[-1]: t = self.t[-1]

        vel_des = np.array([self.v_x(t), self.v_y(t)])
        pt_des = np.array([self.x_spline(t), self.y_spline(t), self.theta_spline(t)])
        state_des = np.vstack((pt_des, vel_des))
        state_curr = np.hstack((pos, theta, vel)).reshape(-1,1)
        error = (state_des-state_curr)

        v_cmd, omega_cmd = self.controllerTF(error)
        v_cmd = self.saturateCmds(v_cmd) # saturate v_cmd

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
        self.trajectoryController = TrajectoryController(params.trajectoryControllerTF)

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
        self.trajectory = np.vstack([msg.x.data,msg.y.data,msg.theta.data,msg.t.data]).T
        self.trajectoryController.fitSpline2Trajectory(self.trajectory, self.pos, self.theta)
        self.traj_idx = 0
        self.new_traj_flag = True

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
        self.robot_v_cmd = np.zeros(2)
        self.robot_omega_cmd = 0

        self.last_waypoint_flag = False

        if self.mode == Mode.TRAJECTORY:
            wp, wp_prev = self.getWaypoint()
            v_des, w_des = self.trajectoryController.getControlCmds(self.pos, self.theta, self.vel)
            if self.traj_idx == self.trajectory.shape[0]-1:
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
