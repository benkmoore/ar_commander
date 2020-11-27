#!/usr/bin/env python
import numpy as np
import numpy.linalg as npl
import rospy
import sys

from ar_commander.msg import Trajectory, State
from std_msgs.msg import Int8
from ar_commander.msg import Trajectory, ControllerCmd, State
import networkx as nx
from std_msgs.msg import Int8, Bool
import scipy.signal as sps
import scipy.interpolate as spi
import time

env = rospy.get_param("ENV")
if env == "sim":
    import configs.sim_params as params
elif env == "hardware":
    import configs.hardware_params as params
else:
    raise ValueError("Controller ENV: '{}' is not valid. Select from [sim, hardware]".format(env))

# TODO:
# Abstract this into hiarchical framework so that the tracking controller for
# a single robot is independent of the consensus control on the formation

class RobotSubscriber:
    def __init__(self, robot_ns):
        # current state
        self.x = None
        self.x_d = None
        self.last_waypoint_flag = None
        self.robot_ns = robot_ns

        # subscribers
        rospy.Subscriber(self.robot_ns + "/estimator/state", State, self.stateCallback)

    def stateCallback(self, msg):
        pos = np.array(msg.pos.data)
        vel = np.array(msg.vel.data)
        theta = np.array([msg.theta.data])
        omega = np.array([msg.omega.data])
        self.x = np.concatenate((pos, theta)).reshape(-1,1)
        self.x_d = np.concatenate((vel, omega)).reshape(-1,1)


class FormationController():
    def __init__(self):
        # initialize graph
        self.graph = nx.DiGraph()
        self.robot_states = {}
        self.traj_idx = 0

        # create graph nodes and edges
        self.buildGraph()

        # assume same gains across all robots
        self.gamma = 1
        self.formation_gain = 1

        # set up state subscriber object for each robot
        self.system_online = False
        for robot_ns in self.graph.nodes():
            self.robot_states[robot_ns] = RobotSubscriber(robot_ns)

    def buildGraph(self):
        # TODO: convert to loops and combinations
        self.graph.add_node("/robot1", offset=np.array([[-params.object_offset["x"]], [-params.object_offset["y"]], [0.0]]))
        self.graph.add_node("/robot2", offset=np.array([[ params.object_offset["x"]], [-params.object_offset["y"]], [np.pi/2]]))
        self.graph.add_node("/robot3", offset=np.array([[ params.object_offset["x"]], [ params.object_offset["y"]], [np.pi]]))
        self.graph.add_node("/robot4", offset=np.array([[-params.object_offset["x"]], [ params.object_offset["y"]], [-np.pi/2]]))

        self.graph.add_edge("/robot1", "/robot2", weight=1)
        self.graph.add_edge("/robot1", "/robot3", weight=1)
        self.graph.add_edge("/robot1", "/robot4", weight=1)

        self.graph.add_edge("/robot2", "/robot1", weight=1)
        self.graph.add_edge("/robot2", "/robot3", weight=1)
        self.graph.add_edge("/robot2", "/robot4", weight=1)

        self.graph.add_edge("/robot3", "/robot1", weight=1)
        self.graph.add_edge("/robot3", "/robot2", weight=1)
        self.graph.add_edge("/robot3", "/robot4", weight=1)

        self.graph.add_edge("/robot4", "/robot1", weight=1)
        self.graph.add_edge("/robot4", "/robot2", weight=1)
        self.graph.add_edge("/robot4", "/robot3", weight=1)

    def checkSystemOnline(self):
        self.system_online = True
        for robot_ns in self.graph.nodes():
            if self.robot_states[robot_ns].x is None:
                self.system_online = False

    def getControlCmds(self, robot_ns):
        # should robot states be part of the graph?
        # should robot states be a dictionary held in the top level class
        # top level robot states get updated by callback
        robot = self.robot_states[robot_ns[:-1]]
        robot_ns = robot_ns[:-1]
        sum_gain = 0
        formation_control = np.zeros((3, 1))
        control = np.zeros((3,1))

        self.checkSystemOnline()
        if self.system_online:
            for neighbor_id in self.graph.predecessors(robot_ns):
                neighbor = self.robot_states[neighbor_id]

                # get gains
                edge_gain = self.graph[neighbor.robot_ns][robot.robot_ns]["weight"]

                # get offsets
                neighbor_offset = nx.get_node_attributes(self.graph,"offset")[neighbor_id].copy()
                robot_offset = nx.get_node_attributes(self.graph,"offset")[robot_ns].copy()

                th = robot.x[2,0]
                R = np.array([
                [np.cos(th), -np.sin(th)],
                [np.sin(th), np.cos(th)]
                ])

                robot_offset[0:2] = R.dot(robot_offset[0:2])
                neighbor_offset[0:2] = R.dot(neighbor_offset[0:2])

                # calculate errors
                state_curr = (robot.x - neighbor.x)
                ref = (robot_offset - neighbor_offset)
                formation_error = ref - state_curr

                # add feedforward term
                formation_control += edge_gain * (neighbor.x_d + self.formation_gain * formation_error)

                # print(robot_ns, " formation error = ", formation_error)
                sum_gain += edge_gain

            control = (1.0 / sum_gain) * formation_control * 0.2

        return control.flatten()
