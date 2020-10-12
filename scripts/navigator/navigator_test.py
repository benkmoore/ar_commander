#!/usr/bin/env python

import numpy as np
import rospy
import sys

sys.path.append(rospy.get_param("AR_COMMANDER_DIR"))

from ar_commander.msg import Trajectory
from std_msgs.msg import Int8
from scripts.stateMachine.stateMachine import Mode

class Navigator():
    def __init__(self):
        rospy.init_node('navigator_test')

        self.mode = None
        self.trajectory = np.zeros([1,3])
        self.trajectory_published = False

        # subscribers
        rospy.Subscriber('state_machine/mode', Int8, self.modeCallback)

        # publishers
        self.pub_trajectory = rospy.Publisher('cmd_trajectory', Trajectory, queue_size=10)

    def modeCallback(self,msg):
        self.mode = Mode(msg.data)

    def loadTrajectory(self):
        traj_id = 1  # specify what trajectory we want to use

        if traj_id == 1:    # square (theta=0)
            self.trajectory = np.array([
                [0,0,0,0],
                [1,0,0,1],
                [1,1,0,2],
                [0,1,0,3],
                [0,0,0,4]
                ])
        elif traj_id == 2: # circle
            t = np.linspace(0,2*np.pi, 20)[:, np.newaxis]
            time = np.linspace(1, 10, 20)
            self.trajectory = np.hstack([np.sin(t), np.cos(t), t, time])
        elif traj_id == 3: # sine wave
            num_pts = 100
            t = np.linspace(0.1, 5.0, num_pts)[:, np.newaxis]
            time = np.linspace(1, 10, num_pts)[:, np.newaxis]
            s = np.sin(np.pi * t)
            self.trajectory = np.hstack([t, s, np.zeros(t.shape), time])
        elif traj_id == 4: # figure of eight
            t = np.linspace(0,2*np.pi, 20)[:, np.newaxis]
            a = 3
            x = a*np.sin(t)
            y = a*np.sin(t)*np.cos(t)
            time = np.linspace(1, 10, 20)[:, np.newaxis]
            self.trajectory = np.hstack([x, y, t, time])
        elif traj_id == 5: # rotate on spot
            t = np.linspace(0,2*np.pi, 20)[:, np.newaxis]
            time = np.linspace(1, 10, 20)[:, np.newaxis]
            self.trajectory = np.hstack([np.zeros(t.shape), np.zeros(t.shape), t, time])
        elif traj_id == 6: # rotate along line
            t = np.linspace(0,-2*np.pi,10)[:, np.newaxis]
            Y = np.linspace(0,10, 10)[:, np.newaxis]
            self.trajectory = np.hstack([np.zeros(t.shape), Y, t, t])
        else:
            raise ValueError("Invalid traj_id")


    def publish(self):
        trajectory = Trajectory()
        trajectory.x.data = self.trajectory[:,0]
        trajectory.y.data = self.trajectory[:,1]
        trajectory.theta.data = self.trajectory[:,2]
        trajectory.t.data = self.trajectory[:,3]

        self.pub_trajectory.publish(trajectory)
        self.trajectory_published = True

    def run(self):
        rate = rospy.Rate(10) # 10 Hz
        while not rospy.is_shutdown() and not self.trajectory_published:
            if self.mode is not None and self.mode.value >= Mode.IDLE.value:
                self.loadTrajectory()
                self.publish()
            rate.sleep()

if __name__ == '__main__':
    navigator = Navigator()
    navigator.run()
