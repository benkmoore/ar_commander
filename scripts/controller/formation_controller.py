#!/usr/bin/env python
import numpy as np
import numpy.linalg as npl

import sys

import networkx as nx
import scipy.signal as sps
import scipy.interpolate as spi
import time
import matplotlib.pyplot as plt


# TODO:
# Abstract this into hiarchical framework so that the tracking controller for 
# a single robot is independent of the consensus control on the formation

# both classes need same instance of simulated robot
class SimulatedRobotSubscriber:
    # real robot subscriber could except other configs first and then initialize subscriber
    def __init__(self, robot_id, simulated_robo=None, dt=None):
        self.x = np.zeros((3, 1))
        self.x_d = np.zeros((3, 1))
        self.simulated_robot = simulated_robot(dt)
        self.id = robot_id

        self.buffer_head = 0
        self.buffer_size = 5
        self.x_history = np.zeros((3, self.buffer_size))
    
    def update_history(self):
        self.x = self.get_state()
        self.x_d = self.get_state_dot()
        # update buffer
        self.x_history[:, self.buffer_head, np.newaxis] = self.x
        self.buffer_head =  (self.buffer_head + 1) % self.buffer_size

    def get_state(self):
        return self.simulated_robot.get_state()

    def get_state_dot(self):
        return self.simulated_robot.get_state_dot()

class SimulatedRobotInterface:
    def __init__(self, robot_id, simulated_robot, buffer_size):

        self.u = np.zeros((3, 1))
        self.reference_gain = 0.50
        self.formation_gain = 1.0
        self.id = robot_id
        self.simulated_robot = simulated_robot

    def apply_control(self, u):
        self.x_d = u
        self.simulated_robot.apply_control(u)

    def get_reference_gain(self):
        return self.reference_gain

    def get_formation_gain(self):
        return self.formation_gain

class SplineTrajectory:
    def __init__(self, trajectory):
        self.trajectory = trajectory
        self.fitSpline2Trajectory(trajectory)

    def fitSpline2Trajectory(self, trajectory):
        x, y, theta, t = np.split(trajectory, 4, axis=1)
        t = t.reshape(-1)

        self.x_spline = spi.CubicSpline(t, x, extrapolate='False')  # output: x_des, input: t
        self.y_spline = spi.CubicSpline(t, y, extrapolate='False')  # output: y_des, input: t
        self.theta_spline = spi.CubicSpline(t, theta, bc_type='clamped', extrapolate='False')  # output: theta_des, input: t

        self.v_x = self.x_spline.derivative()
        self.v_y = self.y_spline.derivative()
        self.omega = self.theta_spline.derivative()

    def get_state(self, t):
        return np.vstack((self.x_spline(t), self.y_spline(t), self.theta_spline(t)))
        
    def get_state_dot(self, t):
        return np.vstack((self.v_x(t), self.v_y(t), self.omega(t)))

        
class FormationController():
    def __init__(self, graph, trajectory, robot_interface, robot_subscriber):
        self.robot_subscriber = robot_subscriber
        self.robot_interface = robot_interface
        # initialize graph, graph has nodes of subscriber objects
        self.graph = graph
       
        self.trajectory = SplineTrajectory(trajectory)
        # assume same gains across all robots
        self.gamma = 1.0

    def update_reference(self, trajectory):
        self.trajectory = SplineTrajectory(trajectory)

    def run_control(self, t):
        # should robot states be part of the graph?
        # should robot states be a dictionary held in the top level class
        # top level robot states get updated by callback
      
        sum_gain = 0
        formation_control = np.zeros((3, 1))
        reference_control = np.zeros((3, 1))

        # get reference state and state derivate
        x = self.robot_subscriber.x
        xref = self.trajectory.get_state(t)
        xref_d = self.trajectory.get_state_dot(t)
        T = get_offset_transform(xref)

        robot_offset = nx.get_node_attributes(self.graph, "offset")[self.robot_subscriber].copy()

        # calculate contribution from formation tracking
        for neighbor in self.graph.predecessors(self.robot_subscriber):
            # get gains
            formation_gain = self.robot_interface.get_formation_gain()
            edge_gain = self.graph[neighbor][self.robot_subscriber]["weight"]

            # get offsets
            neighbor_offset = nx.get_node_attributes(self.graph, "offset")[neighbor].copy()
            

            # calculate errors
            formation_error = (x - neighbor.x) - np.dot(T, (robot_offset - neighbor_offset))
            formation_control += edge_gain * (neighbor.x_d - formation_gain * formation_error)
            sum_gain += edge_gain

        # calculate contribution from reference tracking
        # assumes all robots can access the reference
        reference_error = x - np.dot(T, robot_offset) - xref
        reference_gain = self.robot_interface.get_reference_gain()
        reference_control = reference_gain * (xref_d - formation_gain * reference_error)
        sum_gain += reference_gain
            
        control = (1.0 / sum_gain) * (formation_control + reference_control)
        self.robot_interface.apply_control(control)

def get_offset_transform(x):
    th = x[2, 0]
    T = np.array([
        [np.cos(th), -np.sin(th), 0.0],
        [np.sin(th), np.cos(th), 0.0],
        [0.0, 0.0, 1.0]
        ])
    return T

def plot(graph, trajectory,t):
    pos  = {}
    labels = {}
    state_des = trajectory.get_state(t)
    plt.plot(state_des[0,0], state_des[1,0], "rs")
    plt.plot(trajectory.trajectory[:,0], trajectory.trajectory[:,1])
    ref_pos = {}
    xref = trajectory.get_state(t)
    T = get_offset_transform(xref)
    for robot in graph.nodes():
        pos[robot] = (robot.x[0,0], robot.x[1,0])
        offset = nx.get_node_attributes(graph,"offset")[robot].copy()
        ref_pos[robot] = (xref + np.dot(T, offset)).reshape(-1,)[0:2]
        labels[robot]  =  robot.id

    
    nx.draw_networkx(graph, pos=pos, labels=labels, node_color='b')
    nx.draw_networkx(graph, pos=ref_pos, labels=labels, node_color='r')
    plt.axis("equal")
    plt.draw()
    plt.pause(0.0001)
    plt.cla()


def control_loop(graph, trajectory):
    robot_controllers = []
    for node in graph.nodes():
        buffer_size = 5
        simulated_robot = node.simulated_robot
        robot_interface = SimulatedRobotInterface(node.id, simulated_robot, buffer_size)
        controller = FormationController(graph, trajectory, robot_interface, node)
        robot_controllers.append(controller)

    
    t = 0.0
    
    while t < trajectory[-1,3]:
        traj_object = SplineTrajectory(trajectory)
        # start_time = time.time()
        for controller in robot_controllers:
            # t = time.time() - start_time
            controller.run_control(t)
            
        for controller in robot_controllers:
            controller.robot_subscriber.update_history()
        plot(graph, traj_object, t)
        # time.sleep(0.01)
        t += 0.1
        # plt.clf()

def main():
    # setup graph
    G  = nx.DiGraph()
    robot_subscriber = SimulatedRobotSubscriber
    subscriber_kwargs_normal = {"simulated_robot" : SingleIntegrator, "dt": 0.1}
    subscriber_kwargs_slow = {"simulated_robot" : SlowSingleIntegrator, "dt": 0.1}


    node1 = robot_subscriber("/robot1", **subscriber_kwargs_normal)
    node2 = robot_subscriber("/robot2", **subscriber_kwargs_normal)
    node3 = robot_subscriber("/robot3", **subscriber_kwargs_slow)
    node4 = robot_subscriber("/robot4", **subscriber_kwargs_slow)

    G.add_node(node1, offset=np.array([[0.0], [0.0], [0.0]]))
    G.add_node(node2, offset=np.array([[2.0], [0.0], [0.0]]))
    G.add_node(node3, offset=np.array([[2.0], [2.0], [0.0]]))
    G.add_node(node4, offset=np.array([[0.0], [2.0], [0.0]]))

    G.add_edge(node1, node2, weight=1)
    # G.add_edge(node1, node3, weight=1)
    G.add_edge(node1, node4, weight=1)

 
    G.add_edge(node2, node3, weight=1)
    # G.add_edge(node2, node4, weight=1)
    # G.add_edge(node2, node1, weight=1)
    
    G.add_edge(node3, node1, weight=1)
    G.add_edge(node3, node2, weight=1)
    # G.add_edge(node3, node4, weight=1)


    # G.add_edge(node4, node1, weight=1)
    # G.add_edge(node4, node2, weight=1)
    # G.add_edge(node4, node3, weight=1)

    t = np.linspace(0,2*np.pi, 20)[:, np.newaxis]
    a = 10
    x = a*np.sin(t)
    y = a*np.sin(t)*np.cos(t)
    time = np.linspace(1, 40, 20)[:, np.newaxis]
    trajectory = np.hstack([x, y, t, time])

    # num_pts = 50
    # t = np.linspace(0, 2*np.pi, num_pts)[:, np.newaxis]
    # time = np.linspace(1, 100, num_pts)[:, np.newaxis]
    # trajectory = np.hstack([8*np.sin(t), 8*np.cos(t), t, time])

    control_loop(G, trajectory)
        

# if __name__ == '__main__':
#     main()
