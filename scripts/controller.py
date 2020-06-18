#!/usr/bin/env python

import rospy
import numpy as np
import numpy.linalg as npl
import warnings
warnings.filterwarnings("error")

from ar_commander.msg import Trajectory, ControllerCmd
from sensor_msgs.msg import JointState
from gazebo_msgs.msg import ModelStates
from std_msgs.msg import Float64, Float64MultiArray
from geometry_msgs.msg import Pose2D, Twist, Vector3
from tf.transformations import euler_from_quaternion

# Global variables:
h = [0.35, 0.125, 0.35, 0.125]                  # wheel distances along arms (from end) [ L1 L2 R1 R2 ]
N = len(h)                                      # number drive motors
d = np.sqrt(2*(0.475**2))                       # diagonal distance between two arms of robot
r1 = np.array([0.075, 0.425])                   # axis_offset - (arm_width/2) = 0.075
r2 = np.array([0.075, 0.425])                   # distacnce between wheels = 0.35

def pointController(self):
    kp_1 = 10
    kd_1 = 0
    kp_2 = 3
    ki_2 = 0.03
    kd_2 = 5*(10**-8)

    p_des = np.array([self.pos_des.x,self.pos_des.y])
    p_delta = p_des-self.pos
    theta_delta = self.theta_des-self.theta
    self.theta_error_sum += theta_delta

    if npl.norm(p_delta) > 0.01:
        v_cmd = kp_1*p_delta + kd_1*(p_delta-self.p_delta_prev)
    else:
        v_cmd = np.array([10**-15,10**-15])
    theta_dot_cmd = kp_2*theta_delta + ki_2*self.theta_error_sum \
                    + kd_2*(theta_delta-self.theta_delta_prev)

    self.theta_delta_prev = theta_delta
    self.p_delta_prev = p_delta

    # Convert to motor inputs
    V_cmd, phi_cmd = convert2motorInputs(v_cmd, theta_dot_cmd)

    return V_cmd, phi_cmd, v_cmd

def convert2motorInputs(v_cmd, theta_dot_cmd):

    v_th = np.concatenate((r1*theta_dot_cmd, r2*theta_dot_cmd))
    v_des = np.repeat(v_cmd.reshape(2,1), N, axis=1)
    V_cmd = v_cmd + armFrame2RobotFrame(np.concatenate((np.zeros((1,N)), v_th.reshape(1,-1))))
    V_cmd = np.array(V_cmd, dtype=np.float64)
    V_norm_cmd = npl.norm(V_cmd, axis=1)

    phi_cmd = np.arctan2(V_cmd[:,1], V_cmd[:,0]) + np.pi/2

    return V_norm_cmd, phi_cmd

def armFrame2RobotFrame(v_armFrame):
    # Frame 1 to robot frame, frame 2 is already aligned
    v_af1 = v_armFrame[:,0:N/2]
    v_af2 = v_armFrame[:,N/2:]
    R_90_ccw = np.array([[0, -1], [1, 0]])

    v_robotFrame = np.concatenate((np.matmul(R_90_ccw, v_af1), v_af2))

    return v_robotFrame

def trajectoryController(self):
    kp_e_p = 12
    kp_e_th = 0.75
    kp_v = 0.5
    V_mag = 6

    # init
    if self.traj_init == True:
        self.pt_num = 0
        self.pt_prev = self.pos
        self.pt_next = self.solution_path[0,0:2]
        self.traj_init = False

    # advance waypoints
    if npl.norm(self.pt_next-self.pos) < 0.25:
        if self.pt_num+1 < self.solution_path.shape[0]:
            self.pt_prev = self.pt_next
            self.pt_num += 1
            self.pt_next = self.solution_path[self.pt_num,0:2]
        else:
            print("Path completed")
            self.pos_des = Vector3()
            self.pos_des.x = self.pt_next[0]
            self.pos_des.y = self.pt_next[1]
            self.theta_des = 0
            return pointController(self) #np.zeros(N), 0

    # fit line/poly and get derivative
    x = np.array([self.pt_prev[0], self.pt_next[0]])
    y = np.array([self.pt_prev[1], self.pt_next[1]])
    try:
        p_y = np.poly1d(np.polyfit(x, y, 1)) # desired y
    except np.RankWarning:
        p_y = lambda y: self.pt_next[1]
    try:
        p_x = np.poly1d(np.polyfit(y, x, 1)) # desired x
    except np.RankWarning:
        p_x = lambda x: self.pt_next[0]
    v_des = np.array([self.pt_next[0]-self.pt_prev[0], self.pt_next[1]-self.pt_prev[1]])

    v_cmd = kp_v*v_des + kp_e_p*(np.array([p_x(self.pos[1])-self.pos[0], p_y(self.pos[0])-self.pos[1]]))
    v_cmd = V_mag*v_cmd/npl.norm(v_cmd)
    self.theta_des = np.arctan2(v_cmd[1], v_cmd[0]) - np.pi/2
    theta_delta = self.theta_des-self.theta
    theta_dot_cmd = kp_e_th*(theta_delta)

    # Convert to motor inputs
    V_cmd, phi_cmd = convert2motorInputs(v_cmd, theta_dot_cmd)

    return V_cmd, phi_cmd, v_cmd


class Controller():
    def __init__(self):
        rospy.init_node('controller', anonymous=True)

        self.pos = np.array([None, None])
        self.theta = None
        self.vel = np.zeros(2)
        self.phi_cmd = None
        self.V_cmd = None
        self.theta_dot_cmd = 0
        self.theta_error_sum = 0
        self.theta_delta_prev = 0
        self.p_delta_prev = 0

        # pointController desired x_f
        self.pos_des = None

        # trajetory desired path - init
        self.solution_path = np.array([[None,None]]) #np.array([[1,1],[2,3],[3,5],[2,6],[0,4],[0.5,2],[1,0]])
        self.traj_init = True

        # publishers
        self.pub_cmds = rospy.Publisher('/controller_cmds', ControllerCmd, queue_size=10)

        # subscribers
        rospy.Subscriber('/pose', Pose2D, self.poseCallback)
        rospy.Subscriber('/cmd_waypoint', Pose2D, self.waypointCallback)
        rospy.Subscriber('/cmd_trajectory', Trajectory, self.trajectoryCallback)

    def waypointCallback(self, msg):
        print("Received waypoint")
        self.pos_des = Vector3()
        self.pos_des.x = msg.x
        self.pos_des.y = msg.y
        self.theta_des = msg.theta

    def trajectoryCallback(self, msg):
        print("Received trajectory")
        self.solution_path = np.concatenate((np.array(msg.x.data).reshape(-1,1), \
                                             np.array(msg.y.data).reshape(-1,1), \
                                             np.array(msg.theta.data).reshape(-1,1)), axis=1)

    def poseCallback(self, msg):
        self.pos[0] = msg.x
        self.pos[1] = msg.y
        self.theta = msg.theta

    def controlLoop(self):
        # default behavior (0 = Vx Vy theta_dot)
        self.V_cmd = np.zeros(N)
        self.phi_cmd = np.zeros(N) # rads

        if self.pos[0] != None:
            # Point controller
            if self.pos_des != None:
                self.V_cmd, self.phi_cmd, self.vel = pointController(self)
            # Trajectory controller
            if self.solution_path[0][0] != None:
                self.V_cmd, self.phi_cmd, self.vel = trajectoryController(self)

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
