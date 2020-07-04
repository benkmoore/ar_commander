#!/usr/bin/env python

import rospy
import numpy as np
import numpy.linalg as npl

import warnings
warnings.filterwarnings("error")

from ar_commander.msg import Trajectory, ControllerCmd
from geometry_msgs.msg import Pose2D, Vector3

# Global variables:
N = 4                           # number of wheels
R1 = np.array([0.075, 0.425])   # position of wheels along arm 1
R2 = np.array([0.075, 0.425])   # "" arm 2

class Controller():
    def __init__(self):
        rospy.init_node('controller', anonymous=True)

        # current state
        self.pos = None
        self.theta = None
        self.vel = None

        # controller commands (outputs)
        self.phi_cmd = None
        self.V_cmd = None
        self.theta_dot_cmd = 0
        self.theta_error_sum = 0
        self.theta_delta_prev = 0
        self.p_delta_prev = 0

        # navigation info (inputs)
        self.pos_des = None
        self.trajectory = None
        self.traj_idx = 0

        # subscribers
        rospy.Subscriber('/pose', Pose2D, self.poseCallback)
        rospy.Subscriber('/cmd_waypoint', Pose2D, self.waypointCallback)
        rospy.Subscriber('/cmd_trajectory', Trajectory, self.trajectoryCallback)

        # publishers
        self.pub_cmds = rospy.Publisher('/controller_cmds', ControllerCmd, queue_size=10)

    ## Callback Functions
    def waypointCallback(self, msg):
        print("Received waypoint")
        self.pos_des = Vector3()
        self.pos_des.x = msg.x
        self.pos_des.y = msg.y
        self.theta_des = msg.theta

    def trajectoryCallback(self, msg):
        if self.trajectory is None:
            print("Received trajectory")
            self.trajectory = np.concatenate((np.array(msg.x.data).reshape(-1,1), \
                                              np.array(msg.y.data).reshape(-1,1), \
                                              np.array(msg.theta.data).reshape(-1,1)), axis=1)

    def poseCallback(self, msg):
        if self.pos is None:
            self.pos = np.zeros(2)
        self.pos[0] = msg.x
        self.pos[1] = msg.y
        self.theta = msg.theta

    ## Controller Functions
    def pointController(self):
        kp_1 = 10
        kd_1 = 0
        kp_2 = 3
        ki_2 = 0.03
        kd_2 = 5e-8

        p_des = np.array([self.pos_des.x,self.pos_des.y])
        p_delta = p_des-self.pos
        theta_delta = self.theta_des-self.theta
        self.theta_error_sum += theta_delta

        if npl.norm(p_delta) > 0.01:
            v_cmd = kp_1*p_delta + kd_1*(p_delta-self.p_delta_prev)
        else:
            v_cmd = np.array([1e-15,1e-15])
        theta_dot_cmd = kp_2*theta_delta + ki_2*self.theta_error_sum \
                        + kd_2*(theta_delta-self.theta_delta_prev)

        self.theta_delta_prev = theta_delta
        self.p_delta_prev = p_delta

        # Convert to motor inputs
        V_cmd, phi_cmd = self.convert2motorInputs(v_cmd, theta_dot_cmd)

        return V_cmd, phi_cmd, v_cmd

    def trajectoryController(self):
        # gains
        kp_e_p = 12
        kp_e_th = 0.75
        kp_v = 0.5
        V_mag = 6

        if self.traj_idx == 0:
            wp_prev = self.pos
        else:
            wp_prev = self.trajectory[self.traj_idx-1, 0:2]
        wp = self.trajectory[self.traj_idx, 0:2]

        # advance waypoints
        if npl.norm(wp-self.pos) < 0.25:
            if self.traj_idx+1 < self.trajectory.shape[0]:
                self.traj_idx += 1
                wp_prev = wp
                wp = self.trajectory[self.traj_idx, 0:2]
            else:
                print("Path completed")
                self.pos_des = Vector3()
                self.pos_des.x = self.pt_next[0]
                self.pos_des.y = self.pt_next[1]
                self.theta_des = 0
                return self.pointController() #np.zeros(N), 0

        # fit line/poly and get derivative
        x = np.array([wp_prev[0], wp[0]])
        y = np.array([wp_prev[1], wp[1]])
        try:
            p_y = np.poly1d(np.polyfit(x, y, 1)) # desired y
        except np.RankWarning:
            p_y = lambda y: wp[1]
        try:
            p_x = np.poly1d(np.polyfit(y, x, 1)) # desired x
        except np.RankWarning:
            p_x = lambda x: wp[0]
        v_des = np.array([wp[0]-wp_prev[0], wp[1]-wp_prev[1]])

        v_cmd = kp_v*v_des + kp_e_p*(np.array([p_x(self.pos[1])-self.pos[0], p_y(self.pos[0])-self.pos[1]]))
        v_cmd = V_mag*v_cmd/npl.norm(v_cmd)
        self.theta_des = np.arctan2(v_cmd[1], v_cmd[0]) - np.pi/2
        theta_delta = self.theta_des-self.theta
        theta_dot_cmd = kp_e_th*(theta_delta)

        # Convert to motor inputs
        V_cmd, phi_cmd = self.convert2motorInputs(v_cmd, theta_dot_cmd)

        return V_cmd, phi_cmd, v_cmd

    def convert2motorInputs(self, v_cmd_gf, theta_dot_cmd):
        # Arm frame = af, Robot frame = rf, Global frame = gf
        v_th_af = np.concatenate((np.zeros((1,N)), (np.concatenate((R1*theta_dot_cmd, R2*theta_dot_cmd))).reshape(1,-1)))
        v_th_rf = np.concatenate((np.matmul(np.array([[0, -1], [1, 0]]), v_th_af[:,0:N/2]), \
                v_th_af[:,N/2:]), axis=1) # Frame 1 to robot frame: 90 cw, Frame 2 is already aligned
        v_des_rf = np.matmul(np.array([[np.cos(self.theta), np.sin(self.theta)], [-np.sin(self.theta), np.cos(self.theta)]]), v_cmd_gf.reshape(2,1))

        V_cmd = np.repeat(v_des_rf, N, axis=1) + v_th_rf # Command in rf
        V_cmd = np.array(V_cmd, dtype=np.float64)

        V_norm_cmd = npl.norm(V_cmd, axis=0)
        phi_cmd = np.arctan2(V_cmd[1,:], V_cmd[0,:]) + np.pi/2

        return V_norm_cmd, phi_cmd

    ## Main Loops
    def controlLoop(self):
        # default behavior (0 = Vx Vy theta_dot)
        self.V_cmd = np.zeros(N)
        self.phi_cmd = np.zeros(N) # rads

        if self.pos is not None:
            # Point controller
            if self.pos_des is not None:
                self.V_cmd, self.phi_cmd, self.vel = self.pointController()
            # Trajectory controller
            if self.trajectory is not None:
                self.V_cmd, self.phi_cmd, self.vel = self.trajectoryController()

    def publish(self):
        """ publish cmd messages """
        self.cmds = ControllerCmd()
        self.cmds.velocity_arr.data = self.V_cmd
        self.cmds.phi_arr.data = self.phi_cmd
        self.pub_cmds.publish(self.cmds)

    def run(self):
        rate = rospy.Rate(10) # 10 Hz
        while not rospy.is_shutdown():
            self.controlLoop()
            self.publish()
            rate.sleep()


if __name__ == '__main__':
    controller = Controller()
    controller.run()
