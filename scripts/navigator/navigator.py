#!/usr/bin/env python

import numpy as np
import numpy.linalg as npl
import rospy
import sys

sys.path.append(rospy.get_param("AR_COMMANDER_DIR"))

from astar import AStar, DetOccupancyGrid2D
from scripts.stateMachine.stateMachine import Mode
from ar_commander.msg import Trajectory, State
from std_msgs.msg import Int8


class Navigator():
    def __init__(self):
        rospy.init_node('navigator')

        self.trajectory_published = False
        self.map_width = 10
        self.map_height = 10
        self.obstacles = [((6,6),(8,7)),((2,0),(8,2)),((2,4),(4,6)),((6,2),(8,4))]

        self.start_pos = (None, None)
        self.end_pos = (8.0,8.0)
        self.pos = np.array([None, None])
        self.theta = None
        self.mode = None

        self.trajectory = None


        # subscribers
        rospy.Subscriber('estimator/state', State, self.stateCallback)
        rospy.Subscriber('state_machine/mode', Int8, self.modeCallback)

        # publishers
        self.pub_trajectory = rospy.Publisher('cmd_trajectory', Trajectory, queue_size=10)


    def stateCallback(self, msg):
        self.pos = np.array(msg.pos.data)
        self.theta = msg.theta.data
        if self.start_pos[0] == None:
            self.start_pos = (round(self.pos[0],1), round(self.pos[1],1))

    def modeCallback(self,msg):
        self.mode = Mode(msg.data)

    def callAstar(self):
        occupancy = DetOccupancyGrid2D(self.map_width, self.map_height, self.obstacles)
        self.astar = AStar((0, 0), (self.map_width, self.map_height), self.start_pos, self.end_pos, occupancy)

        if not self.astar.solve():
            exit(0)

        self.trajectory = Trajectory()
        self.trajectory.x.data = (self.astar.path*self.astar.resolution)[:,0]
        self.trajectory.y.data = (self.astar.path*self.astar.resolution)[:,1]
        self.trajectory.theta.data = np.zeros((len(self.trajectory.x.data),1))
        #astar.plot_path()

    def publish(self):
        self.pub_trajectory.publish(self.trajectory)
        self.trajectory_published = True

    def run(self):
        rate = rospy.Rate(10) # 10 Hz
        while not rospy.is_shutdown() and not self.trajectory_published:
            if self.mode == Mode.IDLE:
                self.callAstar()
                self.publish()
            rate.sleep()

if __name__ == '__main__':
    navigator = Navigator()
    navigator.run()
