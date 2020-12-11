#!/usr/bin/env python
import numpy as np
import numpy.linalg as npl
import rospy
import sys

from ar_commander.msg import Trajectory, State
from std_msgs.msg import Int8
from stateMachine.stateMachine import Mode
from ar_commander.msg import Trajectory, ControllerCmd, State
import networkx as nx
from std_msgs.msg import Int8, Bool
import scipy.signal as sps
import scipy.interpolate as spi
import time
import matplotlib.pyplot as plt
# env = rospy.get_param("ENV")
# if env == "sim":
#     import sim_params as params
# elif env == "hardware":
#     import hardware_params as params
# else:
#     raise ValueError("Controller ENV: '{}' is not valid. Select from [sim, hardware]".format(env))

import robot_v1 as rcfg

# TODO:
# Abstract this into hiarchical framework so that the tracking controller for 
# a single robot is independent of the consensus control on the formation

class RobotSubscriber:
    def __init__(self, robot_ns):
        # current state
        self.x = None
        self.x_d = None
        self.u = None # cmd position
        self.reference_gain = .3
        self.formation_gain = .7
        self.last_waypoint_flag = None

        self.phi_prev = np.zeros(rcfg.N) 
    

        self.robot_ns = robot_ns
        rospy.Subscriber(self.robot_ns + "/estimator/state", State, self.stateCallback)

        self.pub_cmds = rospy.Publisher(self.robot_ns + '/controller_cmds', ControllerCmd, queue_size=10)
        

    def stateCallback(self, msg):
        pos = np.array(msg.pos.data)
        vel = np.array(msg.vel.data)
        theta = np.array([msg.theta.data])
        omega = np.array([msg.omega.data])
        self.x = np.concatenate((pos, theta)).reshape(-1,1)
        self.x_d = np.concatenate((vel, omega)).reshape(-1,1)

      ## Helper Functions
    def convert2MotorInputs(self, v_cmd_gf, omega_cmd):
        """Convert velocity and omega commands to motor inputs"""

        # convert inputs to robot frame velocity commands
        R = np.array([[np.cos(self.x[2,0]), np.sin(self.x[2,0])],     # rotation matrix
                      [-np.sin(self.x[2,0]), np.cos(self.x[2,0])]])
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
        print("phi cmd ", phi_cmd)
        print("w wheel ", w_wheel)

        return w_wheel, phi_cmd

    def publish(self):
        """ publish cmd messages """

        robot_v_cmd = self.u[0:2].flatten()
        robot_omega_cmd = self.u[2].flatten()
        wheel_w_cmd, wheel_phi_cmd = self.convert2MotorInputs(robot_v_cmd, robot_omega_cmd)
        cmd = ControllerCmd()
        cmd.omega_arr.data = wheel_w_cmd
        cmd.phi_arr.data = wheel_phi_cmd
        cmd.robot_vel.data = robot_v_cmd
        cmd.robot_omega.data = robot_omega_cmd

        self.phi_prev =wheel_phi_cmd   

        self.pub_cmds.publish(cmd)

    def get_reference_gain(self):
        return self.reference_gain

    def get_formation_gain(self):
        return self.formation_gain

    def update_cmd(self, u):
        self.u = u

class CentralizedFormationController():
    def __init__(self, graph):
        rospy.init_node('FormationController', anonymous=True)

        # initialize graph
        self.graph = graph
        self.robot_states = {}
        self.traj_idx = 0

        # when trajectory is published, 
        # self.trajectoryCallback is executed
        # which then calls fitSpline2Trajectory 
        self.trajectory = None
        self.t = None 
        self.init_traj_time = None
        self.new_traj_flag = False # initially there is no trajectory
        rospy.Subscriber("/cmd_trajectory", Trajectory, self.trajectoryCallback)

        # assume same gains across all robots
        self.gamma = 1
        self.formation_gain = 1
        self.reference_gain = 1

        # for each robot set up state subscriber object
        for robot_ns in self.graph.nodes():
            self.robot_states[robot_ns] = RobotSubscriber(robot_ns)


    def fitSpline2Trajectory(self, trajectory):
        self.trajectory = trajectory.reshape(-1, 4)
        x, y, theta, t = np.split(self.trajectory, 4, axis=1)
        t = t.reshape(-1)

        self.x_spline = spi.CubicSpline(t, x, bc_type='clamped', extrapolate='False') # output: x_des, input: t
        self.y_spline = spi.CubicSpline(t, y, bc_type='clamped', extrapolate='False') # output: y_des, input: t
        self.theta_spline = spi.CubicSpline(t, theta, bc_type='clamped', extrapolate='False') # output: theta_des, input: t

        self.v_x = self.x_spline.derivative()
        self.v_y = self.y_spline.derivative()
        self.omega = self.theta_spline.derivative()

        # initialize time
        self.t = t
        

    def trajectoryCallback(self, msg):
        if self.traj_idx ==0:
            self.trajectory = np.vstack([msg.x.data,msg.y.data,msg.theta.data,msg.t.data]).T
            self.fitSpline2Trajectory(self.trajectory)
            self.new_traj_flag = True
            self.init_traj_time = time.time()
        self.traj_idx += 1
        print("traj index = ", self.traj_idx)
        

    def get_reference(self):
        t = time.time() - self.init_traj_time
        if t < self.t[0]: 
            t = self.t[0] # bound calls between start and end time
            print("t[0")
        if t > self.t[-1]: 
            t = self.t[-1]
            print("t[-1")
        state_des = np.vstack((self.x_spline(t), self.y_spline(t), self.theta_spline(t)))
        state_dot_des = np.vstack((self.v_x(t), self.v_y(t), self.omega(t)))
        print("traj time", t)
        return state_des, state_dot_des


    def calc_control(self, robot):
        # should robot states be part of the graph?
        # should robot states be a dictionary held in the top level class
        # top level robot states get updated by callback
        robot_ns = robot.robot_ns
        sum_gain = 0
        formation_control = np.zeros((3, 1))
        reference_control = np.zeros((3, 1))

        # get reference state and state derivate

        xref, xref_d = self.get_reference()
        for neighbor_id in self.graph.predecessors(robot_ns):
            neighbor = self.robot_states[neighbor_id]

            # get gains
            reference_gain = robot.get_reference_gain()
            formation_gain = robot.get_formation_gain()
            edge_gain = self.graph[neighbor.robot_ns][robot.robot_ns]["weight"]

            # get offsets
            neighbor_offset = nx.get_node_attributes(self.graph,"offset")[neighbor_id].copy()
            robot_offset = nx.get_node_attributes(self.graph,"offset")[robot_ns].copy()
            print("offsets", np.sum(neighbor_offset)," ", np.sum(robot_offset))

            th = robot.x[2,0]
            R = np.array([
            [np.cos(th), -np.sin(th)],
            [np.sin(th), np.cos(th)]
            ])

            robot_offset[0:2] = R.dot(robot_offset[0:2])
            neighbor_offset[0:2] = R.dot(neighbor_offset[0:2])


            # calculate errors
            formation_error = (robot.x - neighbor.x) - (robot_offset - neighbor_offset)
            reference_error = robot.x - robot_offset - xref

            # calculate control
            # assumes all robots can access the reference
            formation_control += edge_gain * (neighbor.x_d - formation_gain * formation_error)
            reference_control += reference_gain * (xref_d - formation_gain * reference_error)

            # print(robot_ns, " formation error = ", formation_error)
            # print(robot_ns, " refere error = ", reference_error)
            sum_gain += (edge_gain + reference_gain)
            
        control = (1.0 / sum_gain) * (formation_control + reference_control)

        return control

    def controlLoop(self):
        
        
        # check if reached final  position
        # params.wp_threshold
        # params.time_threshold:
        # if npl.norm(self.trajectory[-1,0:2]-self.pos) < 2 and \
        #     abs(t - self.t[-1]) < 1: # check if near end pos and end time
        #     self.last_waypoint_flag = True

        # TODO verify all robots have data
        for robot_id in self.graph.nodes():
            u = self.calc_control(self.robot_states[robot_id])
            self.robot_states[robot_id].update_cmd(u)

    def run(self):
        #params.CONTROLLER_RATE
        rate = rospy.Rate(10)
        rate.sleep()

        # run forever
        while not rospy.is_shutdown():
            # perform formation control until destination is reached
            if self.new_traj_flag is True:

                # calculate control for each robot
                self.controlLoop()

                # publish control for each robot
                for robot_ns in self.robot_states:
                    self.robot_states[robot_ns].publish()
                self.plot()
                
            rate.sleep()

    def plot(self):
        state_des, state_dot_des = self.get_reference()
        # plt.plot(state_des[0,0], state_des[1,0],"r*")
        th = state_des[2,0]
        R = np.array([
            [np.cos(th), -np.sin(th)],
            [np.sin(th), np.cos(th)]
            ])
        

        for robot_id, robot in self.robot_states.items():
            plt.plot(robot.x[0,0], robot.x[1,0], "b*")
            offset = nx.get_node_attributes(self.graph,"offset")[robot_id].copy()
            offset[0:2] = R.dot(offset[0:2])
            
            
            plt.plot(state_des[0,0]+offset[0,0], state_des[1,0]+offset[1,0], "g*")
            # print("offset = ", offset)

        

        plt.axis("equal")
        plt.draw()
        plt.pause(0.0001)


if __name__ == '__main__':
    G  = nx.DiGraph()

    
    G.add_node("/robot1", offset=np.array([[0.0], [0.0], [0.0]]))
    G.add_node("/robot2", offset=np.array([[2.0], [0.0], [0.0]]))
    G.add_node("/robot3", offset=np.array([[2.0], [2.0], [0.0]]))
    G.add_node("/robot4", offset=np.array([[0.0], [2.0], [0.0]]))

    G.add_edge("/robot1", "/robot2", weight=1)
    # G.add_edge("/robot1", "/robot3", weight=1)
    G.add_edge("/robot1", "/robot4", weight=1)

    # G.add_edge("/robot2", "/robot1", weight=1)
    G.add_edge("/robot2", "/robot3", weight=1)
    # G.add_edge("/robot2", "/robot4", weight=1)
    
    G.add_edge("/robot3", "/robot1", weight=1)
    G.add_edge("/robot3", "/robot2", weight=1)
    # G.add_edge("/robot3", "/robot4", weight=1)

    # G.add_edge("/robot4", "/robot1", weight=1)
    # G.add_edge("/robot4", "/robot2", weight=1)
    # G.add_edge("/robot4", "/robot3", weight=1)

    navigator = CentralizedFormationController(G)
    navigator.run()