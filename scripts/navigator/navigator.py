#!/usr/bin/env python

import numpy as np
import numpy.linalg as npl
import rospy
import sys

sys.path.append(rospy.get_param("AR_COMMANDER_DIR"))

from astar import AStar, DetOccupancyGrid2D
from scripts.stateMachine.stateMachine import Mode
from ar_commander.msg import Trajectory, State, Task, Object
from std_msgs.msg import Int8, Pose2D

SEARCH_ANGLE = np.deg2rad(60.0) # search angle either side of trajectory line


class Navigator():
    def __init__(self):
        rospy.init_node('navigator')

        self.map_width = 10
        self.map_height = 10
        self.obstacles = []

        self.start_pos = (None, None)
        self.end_pos = (None, None)
        self.end_theta = None
        self.pos = np.array([None, None])
        self.theta = None
        self.mode = None

        self.trajectory = None
        self.task_pos = None
        self.object = None

        self.new_pt_flag = False
        self.new_task_flag = False
        self.new_object_flag = False
        self.trajectory_published = False

        # subscribers
        rospy.Subscriber('estimator/state', State, self.stateCallback)
        rospy.Subscriber('state_machine/mode', Int8, self.modeCallback)
        rospy.Subscriber('cmd_pt', Pose2D, self.cmdPtCallback)
        rospy.Subscriber('task', Task, self.taskCallback)
        rospy.Subscriber('object', Object, self.objectCallback)

        # publishers
        self.pub_trajectory = rospy.Publisher('cmd_trajectory', Trajectory, queue_size=10)

    ## Callback functions
    def stateCallback(self, msg):
        self.pos = np.array(msg.pos.data)
        self.theta = msg.theta.data
        if self.start_pos[0] == None:
            self.start_pos = (round(self.pos[0],1), round(self.pos[1],1))

    def modeCallback(self,msg):
        self.mode = Mode(msg.data)

    def cmdPtCallback(self,msg):
        self.new_pt_flag = True
        self.end_pos = (msg.x, msg.y)
        self.end_theta = msg.theta

    def objectCallback(self,msg):
        self.new_object_flag = True
        self.end_pos, self.end_theta = getObjectInfo(msg.corners.data.reshape(-1,2))

    def taskCallback(self,msg):
        self.new_task_flag = True
        self.end_pos = (msg.x.data, msg.y.data)

    ## Helper functions
    def getObjectInfo(self, corners):
        idx_min = np.sqrt((corners[0,:]-self.pos[0])**2 + (corners[1,:]-self.pos[1])**2).argmin()
        closest_corner = corners[idx_min]
        object_orientation = np.arctan2(corners[2,1]-corners[2,0], corners[0,1]-corners[0,0]) #TODO:check with Byron on the output of detector

        return closest_corner, object_orientation

    ## Path planners
    def callAstar(self):
        occupancy = DetOccupancyGrid2D(self.map_width, self.map_height, self.obstacles)
        self.astar = AStar((-self.map_width, -self.map_height), (self.map_width, self.map_height), self.start_pos, self.end_pos, occupancy)

        if not self.astar.solve():
            exit(0)

        self.trajectory = Trajectory()
        self.trajectory.x.data = (self.astar.path*self.astar.resolution)[:,0]
        self.trajectory.y.data = (self.astar.path*self.astar.resolution)[:,1]
        if self.new_task_flag: # search behaviour
            self.trajectory.theta.data = SEARCH_ANGLE*np.sin(self.trajectory.x.data)
        else:
            self.trajectory.theta.data = np.vstack((np.zeros((len(self.trajectory.x.data)-1,1)), self.end_theta))
        #astar.plot_path()

    ## Process functions
    def publish(self):
        if self.trajectory is not None:
            self.pub_trajectory.publish(self.trajectory)
            self.trajectory = None

    def run(self):
        rate = rospy.Rate(10) # 10 Hz
        while not rospy.is_shutdown():

            if self.mode == MODE.IDLE and self.new_pt_flag: # nav to point
                self.callAstar()
                self.new_pt_flag = False

            elif self.mode == MODE.TRAJECTORY and self.new_object_flag: # nav to object corner
                self.callAstar()
                self.new_object_flag = False

            elif self.mode == MODE.IDLE and self.new_task_flag: # nav search for assigned task
                self.callAstar()
                self.new_task_flag = False

            self.publish()
            rate.sleep()

if __name__ == '__main__':
    navigator = Navigator()
    navigator.run()
