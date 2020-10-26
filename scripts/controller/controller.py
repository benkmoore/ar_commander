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
        if npl.norm(v_cmd) > params.max_vel:
            v_cmd = params.max_vel * v_cmd / npl.norm(v_cmd)
        return v_cmd

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
        print(pt_des, t)
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

        self.phi_prev = np.zeros(rcfg.N)   # can initialize to 0 as it will only affect first command

        # navigation info
        self.trajectory = None
        self.traj_idx = 0

        # initialize controllers
        self.trajectoryController = TrajectoryController(params.trajectoryControllerTF)

        # output commands
        self.wheel_phi_cmd = None
        self.wheel_w_cmd = None
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
            t = time.time() - self.trajectoryController.init_traj_time
            # advance waypoints
            if npl.norm(wp[0:2]-self.pos) < params.wp_threshold and np.abs(wp[2]-self.theta) < params.theta_threshold and self.traj_idx < self.trajectory.shape[0]-1 or t > wp[3]:
                self.traj_idx += 1
                print("reached: ", wp)
                if self.traj_idx < self.trajectory.shape[0] - 1:
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

        v_th1 = np.vstack([-rcfg.R1*omega_cmd, np.zeros(rcfg.N/2)])
        v_th2 = np.vstack([np.zeros(rcfg.N/2), rcfg.R2*omega_cmd])
        v_th_rf = np.hstack([v_th1, v_th2])

        v_xy = v_cmd_rf + v_th_rf

        # Convert to |V| and phi
        v_wheel = npl.norm(v_xy, axis=0)
        phi_cmd = np.arctan2(v_xy[1,:], v_xy[0,:])

        # pick closest phi
        phi_diff = phi_cmd - self.phi_prev
        idx = abs(phi_diff) > 2*np.pi #/2
        phi_cmd -= np.pi*np.sign(phi_diff)*idx
        v_wheel *= -1*idx + 1*~idx

        # enforce physical bounds
        idx_upper = phi_cmd > rcfg.phi_bounds[1]     # violates upper bound
        idx_lower = phi_cmd < rcfg.phi_bounds[0]     # violates lower bound

        phi_cmd -= np.pi*idx_upper - np.pi*idx_lower
        v_wheel *= -1*(idx_upper+idx_lower) + 1*~(idx_upper + idx_lower)

        # map to desired omega (angular velocity) of wheels: w = v/r
        w_wheel = v_wheel # /rcfg.wheel_radius

        return w_wheel, phi_cmd

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
            v_des, w_des = self.trajectoryController.getControlCmds(self.pos, self.theta, self.vel)
            if self.traj_idx == self.trajectory.shape[0]-1:
                self.last_waypoint_flag = True

            self.wheel_w_cmd, self.wheel_phi_cmd = self.convert2MotorInputs(v_des,w_des)
            self.robot_v_cmd = v_des
            self.robot_omega_cmd = w_des
            self.phi_prev = self.wheel_phi_cmd     # store previous command

    def publish(self):
        """ publish cmd messages """
        cmd = ControllerCmd()
        cmd.omega_arr.data = self.wheel_w_cmd
        cmd.phi_arr.data = self.wheel_phi_cmd
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
