#!/usr/bin/env python
import numpy as np
import numpy.linalg as npl
import rospy

from std_msgs.msg import Int8
from ar_commander.msg import ControllerCmd, State
import networkx as nx
from std_msgs.msg import Int8, Bool
# from ar_commander.scripts.controller.formation_controller import get_offset_transform
import robot_v1 as rcfg
import time
from utils.utils import robotConfig, wrapAngle
from formation_controller import FormationController
from formation_controller import SplineTrajectory
import matplotlib.pyplot as plt



# TODO:
# Abstract this into hiarchical framework so that the tracking controller for 
# a single robot is independent of the consensus control on the formation
def get_offset_transform(x):
    th = x[2, 0]
    T = np.array([
        [np.cos(th), -np.sin(th), 0.0],
        [np.sin(th), np.cos(th), 0.0],
        [0.0, 0.0, 1.0]
        ])
    return T

class RobotSubscriber:
    def __init__(self, id):
        # current state
        self.x = None
        self.x_d = None
        self.id = id

        self.buffer_head = 0
        self.buffer_size = 5
        self.x_history = np.zeros((3, self.buffer_size))

        # create subscriber
        rospy.Subscriber(self.id + "/estimator/state", State, self.state_callback)

    def state_callback(self, msg):
        pos = np.array(msg.pos.data)
        vel = np.array(msg.vel.data)
        theta = np.array([msg.theta.data])
        omega = np.array([msg.omega.data])
        self.x = np.concatenate((pos, theta)).reshape(-1, 1)
        self.x_d = np.concatenate((vel, omega)).reshape(-1, 1)

    def get_state(self):
        return self.x

    def get_state_dot(self):
        return self.x_d

class RobotInterface:
    def __init__(self, robot_id, buffer_size):
        self.u = np.zeros((3,1))
        self.reference_gain = 1.0
        self.formation_gain = 1.5
        self.id = robot_id
        self.rcfg = robotConfig(self.id + "/")
        self.phi_prev = np.array([0, 0 ,0 ,0])

        # create subscriber and publisher
        rospy.Subscriber(self.id + "/estimator/state", State, self.stateCallback)
        self.pub_cmds = rospy.Publisher(self.id + '/controller_cmds', ControllerCmd, queue_size=10)

    def stateCallback(self, msg):
        pos = np.array(msg.pos.data)
        vel = np.array(msg.vel.data)
        theta = np.array([msg.theta.data])
        omega = np.array([msg.omega.data])
        self.x = np.concatenate((pos, theta)).reshape(-1,1)
        self.x_d = np.concatenate((vel, omega)).reshape(-1,1)

    def convert2MotorInputs(self, v_cmd_gf, omega_cmd):
        """Convert velocity and omega commands to motor inputs"""

        # convert inputs to robot frame velocity commands
        R = np.array([[np.cos(self.x[2,0]), np.sin(self.x[2,0])],     # rotation matrix
                      [-np.sin(self.x[2,0]), np.cos(self.x[2,0])]])
        v_cmd_rf = np.dot(R, v_cmd_gf)[:,np.newaxis]        # convert to robot frame

        v_th1 = np.vstack([-self.rcfg.R1*omega_cmd, np.zeros(self.rcfg.N/2)])
        v_th2 = np.vstack([np.zeros(self.rcfg.N/2), self.rcfg.R2*omega_cmd])
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
        idx_upper = phi_cmd > self.rcfg.phi_bounds[1]     # violates upper bound
        idx_lower = phi_cmd < self.rcfg.phi_bounds[0]     # violates lower bound

        phi_cmd -= np.pi*idx_upper - np.pi*idx_lower
        v_wheel *= -1*(idx_upper+idx_lower) + 1*~(idx_upper + idx_lower)

        # map to desired omega (angular velocity) of wheels: w = v/r
        w_wheel = v_wheel/self.rcfg.wheel_radius
        print("phi cmd ", phi_cmd)
        print("w wheel ", w_wheel)

        return w_wheel, phi_cmd

    def apply_control(self, u):
        """ publish cmd messages """

        robot_v_cmd = u[0:2].flatten()
        robot_omega_cmd = u[2].flatten()
        wheel_w_cmd, wheel_phi_cmd = self.convert2MotorInputs(robot_v_cmd, robot_omega_cmd)
        cmd = ControllerCmd()
        cmd.omega_arr.data = wheel_w_cmd
        cmd.phi_arr.data = wheel_phi_cmd
        cmd.robot_vel.data = robot_v_cmd
        cmd.robot_omega.data = robot_omega_cmd

        self.phi_prev = wheel_phi_cmd   
        self.pub_cmds.publish(cmd)

    def get_reference_gain(self):
        return self.reference_gain

    def get_formation_gain(self):
        return self.formation_gain


class FormationControllerNode():
    def __init__(self):
        #  initialize node
        rospy.init_node('FormationController', anonymous=True)

        # create subscriber objects
        robot_subscriber = RobotSubscriber

        # build graph
        G = nx.DiGraph()
        node1 = robot_subscriber("/robot1")
        node2 = robot_subscriber("/robot2")
        node3 = robot_subscriber("/robot3")
        node4 = robot_subscriber("/robot4")

        G.add_node(node1, offset=np.array([[0.0], [0.0], [0.0]]))
        G.add_node(node2, offset=np.array([[2.0], [0.0], [0.0]]))
        G.add_node(node3, offset=np.array([[2.0], [2.0], [0.0]]))
        G.add_node(node4, offset=np.array([[0.0], [2.0], [0.0]]))

        G.add_edge(node1, node2, weight=1)
        G.add_edge(node1, node4, weight=1)
        G.add_edge(node2, node3, weight=1)
        G.add_edge(node3, node1, weight=1)
        G.add_edge(node3, node2, weight=1)

        # initialize trajectory
        t = np.linspace(0, 2*np.pi, 20)[:, np.newaxis]
        a = 10
        x = a*np.sin(t)
        y = a*np.sin(t)*np.cos(t)
        time = np.linspace(1, 40, 20)[:, np.newaxis]
        trajectory = np.hstack([x, y, t, time])
        self.control_loop(G, trajectory)
    
    def control_loop(self, graph, trajectory):
        rate = rospy.Rate(10)
        robot_controllers = []
        for node in graph.nodes():
            buffer_size = 5
            robot_interface = RobotInterface(node.id, buffer_size)
            controller = FormationController(graph, trajectory, robot_interface, node)
            robot_controllers.append(controller)

        rospy.sleep(2) # wait 2 seconds to get robot info, should initialize subscriber objects with current pose
        start_time = rospy.get_rostime()
        t = 0.0
        while t < trajectory[-1,3]:
            traj_object = SplineTrajectory(trajectory)
            for controller in robot_controllers:
                t = (rospy.get_rostime() - start_time).to_sec()
                controller.run_control(t)
            # self.plot(graph, traj_object, t)
            rate.sleep()

        for controller in robot_controllers:
            controller.apply_control(np.zeros(3,1))
           
          

    def plot(self, graph, trajectory, t):
        pos  = {}
        labels = {}
        state_des = trajectory.get_state(t)
        plt.plot(state_des[0,0], state_des[1,0], "rs")
        plt.plot(trajectory.trajectory[:,0], trajectory.trajectory[:,1])
        ref_pos = {}
        xref = trajectory.get_state(t)
        T = get_offset_transform(xref)
        for robot in graph.nodes():
            pos[robot] = (robot.x[0, 0], robot.x[1, 0])
            offset = nx.get_node_attributes(graph, "offset")[robot].copy()
            ref_pos[robot] = (xref + np.dot(T,offset)).reshape(-1,)[0:2]
            labels[robot] =  robot.id

        nx.draw_networkx(graph, pos=pos, labels=labels, node_color='b')
        nx.draw_networkx(graph, pos=ref_pos, labels=labels, node_color='r')
        plt.axis("equal")
        plt.draw()
        plt.pause(0.0001)
        plt.cla()

            

if __name__ == '__main__':
    navigator = FormationControllerNode()
