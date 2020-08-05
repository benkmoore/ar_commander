#!/usr/bin/env python

import numpy as np
import rospy

from ar_commander.msg import Trajectory
from stateMachine import Mode
from std_msgs.msg import Int8


class Navigator():
    def __init__(self):
        rospy.init_node('navigator_test')

        self.mode = None
        self.trajectory = np.zeros([1,3])
        self.trajectory_published = False

        # subscribers
        rospy.Subscriber('state_machine/mode', Int8, self.modeCallback)

        # publishers
        self.pub_trajectory = rospy.Publisher('/cmd_trajectory', Trajectory, queue_size=10)

    def modeCallback(self,msg):
        self.mode = Mode(msg.data)

    def loadTrajectory(self):
        traj_id = 3  # specify what trajectory we want to use

        if traj_id == 1:    # square (theta=0)
            self.trajectory = np.array([
                [1,0,0],
                [1,1,0],
                [0,1,0],
                [0,0,0],
                [1,0,0]
                ])
        elif traj_id == 2:  # circle
            t = np.linspace(0,2*np.pi)[:, np.newaxis]
            self.trajectory = np.hstack([np.sin(t),np.cos(t),t])
        elif traj_id == 3: # figure of eight
            print("Testing figure of eight trajectory")
            t = np.linspace(0,1.9*np.pi,15)[:, np.newaxis] # cut short due to bug in navigator with same start and end point
            a = 1.2
            x = a*np.sin(t)
            y = a*np.sin(t)*np.cos(t)
            self.trajectory = np.hstack([x,y,np.zeros(t.shape)])

        else:
            raise ValueError("Invalid traj_id")


    def publish(self):
        trajectory = Trajectory()
        trajectory.x.data = self.trajectory[:,0]
        trajectory.y.data = self.trajectory[:,1]
        trajectory.theta.data = self.trajectory[:,2]

        self.pub_trajectory.publish(trajectory)
        self.trajectory_published = True

    def run(self):
        rate = rospy.Rate(10) # 10 Hz
        while not rospy.is_shutdown(): # and not self.trajectory_published:
            if self.mode == Mode.IDLE:
                self.loadTrajectory()
                self.publish()
            rate.sleep()

if __name__ == '__main__':
    navigator = Navigator()
    navigator.run()