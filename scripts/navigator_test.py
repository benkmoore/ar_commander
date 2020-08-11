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
        traj_id = 5  # specify what trajectory we want to use
        
        offset = 0
        length = 1
        if traj_id == 1:    # square (theta=0)
            self.trajectory = np.array([
                [length,0,0],
                [length,length,0],
                [0,length,0],
                [offset,0,0]
                ])
        elif traj_id == 2:    # reverse square (theta=0)
            self.trajectory = np.array([
                [-1.5,0,0],
                [-1.5,-1.5,0],
                [0,-1.5,0],
                [0,0,0],
                [-1.5,0,0]
                ])
        elif traj_id == 3:  # circle
            num_times = 1
            t = np.linspace(-np.pi,num_times*np.pi,30*num_times)[:, np.newaxis]
            r = 0.7 # radius
            x_c = r # force circle edge to start at (0,0)
            y_c = 0
            self.trajectory = np.hstack([-y_c-r*np.sin(t),x_c+r*np.cos(t),np.zeros(t.shape)])
        elif traj_id == 4: # figure of eight
            num_times = 1
            t = np.linspace(0,num_times*2*np.pi,30*num_times)[:, np.newaxis] # cut short due to bug in navigator with same start and end point
            a = 1 #2
            x = a*np.sin(t)
            y = a*np.sin(t)*np.cos(t)
            self.trajectory = np.hstack([x,y,np.zeros(t.shape)])
        elif traj_id == 5:  # sine wave
            A = 0.5
            w = np.pi/1.5  # freq
            t = np.arange(0.0, 3.0, 0.1)[:, np.newaxis] # 1 full period is 3 units: T = 2pi/w
            s = A*np.sin(w*t)
            self.trajectory = np.hstack([t,s,np.zeros(t.shape)])
        elif traj_id == 6:  # rotate on spot
            t = np.linspace(0, -np.pi)[:, np.newaxis]
            self.trajectory = np.hstack([np.zeros(t.shape),np.zeros(t.shape),t])
        elif traj_id == 7:  # rotate theta while moving in straight line along y
            num_pts = 30
            zero2pi = np.linspace(0, np.pi/2, num_pts/2)[:, np.newaxis]
            pi2neg_pi = np.linspace(np.pi/2, 0, num_pts/2)[:, np.newaxis]
            t = np.concatenate((zero2pi,pi2neg_pi),axis=0)
            y_end = 10
            y_pts = np.linspace(0,y_end, num_pts)[:, np.newaxis]
            self.trajectory = np.hstack([np.zeros(t.shape),y_pts,t])
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
