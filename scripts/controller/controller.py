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


class TrajectoryController():
    def __init__(self, tf_state, tf_state_dot):
        self.tf_state = tf_state
        self.tf_state_dot = tf_state_dot
        self.z_state = None
        self.z_state_dot = None

        self.trajectory = None

        self.robot_offsets = {
            1: np.array([-params.object_offset["x"], -params.object_offset["y"], 0, 0]),
            2: np.array([params.object_offset["x"], -params.object_offset["y"], np.pi/2, 0]),
            3: np.array([params.object_offset["x"], -params.object_offset["y"], np.pi, 0]),
            4: np.array([-params.object_offset["x"], params.object_offset["y"], -np.pi/2, 0])
        }

    def fitSpline2Trajectory(self, trajectory, pos, theta):
        self.trajectory = trajectory.reshape(-1, 4)
        default_start_pt = np.hstack((pos, theta, 0))
        self.trajectory = np.vstack((default_start_pt, self.trajectory))
        x, y, theta, t = np.split(self.trajectory, 4, axis=1)
        t = t.reshape(-1)

        self.x_spline = spi.CubicSpline(t, x, bc_type='clamped', extrapolate='False') # output: x_des, input: t
        self.y_spline = spi.CubicSpline(t, y, bc_type='clamped', extrapolate='False') # output: y_des, input: t
        self.theta_spline = spi.CubicSpline(t, theta, bc_type='clamped', extrapolate='False') # output: theta_des, input: t

        self.v_x = self.x_spline.derivative()
        self.v_y = self.y_spline.derivative()
        self.omega = self.theta_spline.derivative()

        self.t = t
        self.init_traj_time = time.time()

    def getControlCmds(self, pos, theta, vel, omega, pos1=None, pos2=None, ns=None):
        if self.trajectory is not None:
            t = time.time() - self.init_traj_time
            if t < self.t[0]: t = self.t[0] # bound calls between start and end time
            if t > self.t[-1]: t = self.t[-1]

            state_des = np.vstack((self.x_spline(t), self.y_spline(t), self.theta_spline(t)))
            state_dot_des = np.vstack((self.v_x(t), self.v_y(t), self.omega(t)))
            error = state_des - np.vstack((pos.reshape(-1,1), theta))
            error_dot = state_dot_des - np.vstack((vel.reshape(-1,1), omega))

            v_cmd, omega_cmd = self.runController(error, error_dot, state_dot_des)

            if pos1 is not None and pos2 is not None:
                v_cmd += self.formationController(pos, pos1, pos2, ns)

            v_cmd = self.saturateCmds(v_cmd) # saturate v_cmd
            omega_cmd = np.clip(omega_cmd, -0.175, 0.175)

        else: # default behaviour
            v_cmd = np.zeros(2)
            omega_cmd = 0

        return v_cmd, omega_cmd

    def saturateCmds(self, v_cmd):
        if npl.norm(v_cmd) > params.max_vel:
            v_cmd = params.max_vel * v_cmd / npl.norm(v_cmd)
        return v_cmd

    def formationController(self, pos, pos1=None, pos2=None, ns=None):
        K = 0.1
        if ns == '/robot1/':
            p_des = self.robot_offsets[1][0:2] - self.robot_offsets[2][0:2] + pos2
        elif ns == '/robot2/':
            p_des = self.robot_offsets[2][0:2] - self.robot_offsets[1][0:2] + pos1
        error_formation = p_des - pos
        v_formation_cmd = K*error_formation

        return v_formation_cmd

    def runController(self, error, error_dot, ref_dot_feedfwd):
        if self.z_state is None:
            self.z_state = sps.lfiltic(self.tf_state['num'], self.tf_state['den'], y=np.zeros_like(self.tf_state['den'])) # initial filter delays
            self.z_state = np.tile(self.z_state, error.shape)
            self.z_state_dot = sps.lfiltic(self.tf_state_dot['num'], self.tf_state_dot['den'], y=np.zeros_like(self.tf_state_dot['den']))
            self.z_state_dot = np.tile(self.z_state_dot, error_dot.shape)

        u_state, self.z_state = sps.lfilter(self.tf_state['num'], self.tf_state['den'], error, axis=1, zi=self.z_state)
        u_state_dot, self.z_state_dot = sps.lfilter(self.tf_state_dot['num'], self.tf_state_dot['den'], error_dot, axis=1, zi=self.z_state_dot)
        u = u_state + u_state_dot + ref_dot_feedfwd

        v_cmd = u[0:2, 0]
        omega_cmd = u[2, 0]

        return v_cmd, omega_cmd


class ControlNode():
    """
    Main controller node
    Handles inputs, outputs and main control logic
    """

    def __init__(self):
        rospy.init_node('controller', anonymous=True)
        # namespace
        self.ns = rospy.get_namespace()

        # current state
        self.pos = None
        self.theta = None
        self.vel = 0
        self.omega = 0

        self.pos1 = None
        self.pos2 = None

        self.mode = None

        self.phi_prev = np.zeros(rcfg.N)   # can initialize to 0 as it will only affect first command

        # navigation info
        self.trajectory = None
        self.traj_idx = 0

        # initialize controllers
        self.trajectoryController = TrajectoryController(params.ctrl_tf_state, params.ctrl_tf_state_dot)

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
        rospy.Subscriber('/robot1/estimator/state', State, self.r1StateCallback)
        rospy.Subscriber('/robot2/estimator/state', State, self.r2StateCallback)

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

    def r1StateCallback(self, msg):
        self.pos1 = np.array(msg.pos.data)

    def r2StateCallback(self, msg):
        self.pos2 = np.array(msg.pos.data)

    def modeCallback(self,msg):
        self.mode = Mode(msg.data)

    ## Helper Functions
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
        #phi_diff = phi_cmd - self.phi_prev
        #idx = abs(phi_diff) > np.pi/2
        #phi_cmd -= np.pi*np.sign(phi_diff)*idx
        #v_wheel *= -1*idx + 1*~idx

        # enforce physical bounds
        #idx_upper = phi_cmd > rcfg.phi_bounds[1]     # violates upper bound
        #idx_lower = phi_cmd < rcfg.phi_bounds[0]     # violates lower bound

        #phi_cmd -= np.pi*idx_upper - np.pi*idx_lower
        #v_wheel *= -1*(idx_upper+idx_lower) + 1*~(idx_upper + idx_lower)

        # map to desired omega (angular velocity) of wheels: w = v/r
        w_wheel = v_wheel #/rcfg.wheel_radius

        try:
            t = time.time() - self.trajectoryController.init_traj_time
            if t < 4: w_wheel = np.zeros(rcfg.N)
        except:
           pass

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
            v_des, w_des = self.trajectoryController.getControlCmds(self.pos, self.theta, self.vel, self.omega, self.pos1, self.pos2, self.ns)
            try:
                t = time.time() - self.trajectoryController.init_traj_time
                if npl.norm(self.trajectoryController.trajectory[-1,0:2]-self.pos) < params.wp_threshold and (t > self.trajectoryController.t[-1] or abs(t - self.trajectoryController.t[-1]) < params.time_threshold): # check if near end pos and end time
                    self.last_waypoint_flag = True
            except:
                t = 0

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
