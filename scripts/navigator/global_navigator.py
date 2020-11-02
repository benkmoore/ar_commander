#!/usr/bin/env python
import numpy as np
import numpy.linalg as npl
import rospy
import sys

from ar_commander.msg import Trajectory, State
from std_msgs.msg import Int8
from stateMachine.stateMachine import Mode


class GlobalNavigator():
    def __init__(self):
        rospy.init_node('navigator')

        self.pos = np.array([None, None])
        self.theta = None
        self.mode = None

        self.trajectory_published = False
        self.trajectory = None

        # publishers
        # TODO initialize with for loop
        self.pub_trajectory1 = rospy.Publisher('/robot1/cmd_trajectory', Trajectory, queue_size=10)
        self.pub_trajectory2 = rospy.Publisher('/robot2/cmd_trajectory', Trajectory, queue_size=10)
        self.pub_trajectory3 = rospy.Publisher('/robot3/cmd_trajectory', Trajectory, queue_size=10)
        self.pub_trajectory4 = rospy.Publisher('/robot4/cmd_trajectory', Trajectory, queue_size=10)

        self.robot_offsets = {
            1: np.array([-1, -1, 0, 0]),
            2: np.array([1, -1, np.pi/2, 0]),
            3: np.array([5, -5, 0, 0]),
            4: np.array([-5, 5, 0, 0])
        }

        self.robot_publishers = {
            1: self.pub_trajectory1,
            2: self.pub_trajectory2,
            3: self.pub_trajectory3,
            4: self.pub_trajectory4
        }

    def loadTrajectory(self):
        print("loading trajectory")
        traj_id = 1 # specify what trajectory we want to use

        if traj_id == 1:    # square (theta=0)
            self.trajectory = np.array([
                # [0,0,0,0],
                [3,8,0,8]
                # [1,1,0,4],
                # [0,1,0,6],
                # [0,0,0,8]
                ])
        elif traj_id == 2: # circle
            num_pts = 20
            t = np.linspace(0,2*np.pi, num_pts)[:, np.newaxis]
            time = np.linspace(1, 30, num_pts)[:, np.newaxis]
            self.trajectory = np.hstack([np.sin(t), np.cos(t), t, time])
        elif traj_id == 3: # sine wave
            num_pts = 100
            t = np.linspace(0.1, 5.0, num_pts)[:, np.newaxis]
            time = np.linspace(1, 20, num_pts)[:, np.newaxis]
            s = np.sin(np.pi * t)
            self.trajectory = np.hstack([t, s, np.zeros(t.shape), time])
        elif traj_id == 4: # figure of eight
            t = np.linspace(0,2*np.pi, 20)[:, np.newaxis]
            a = 3
            x = a*np.sin(t)
            y = a*np.sin(t)*np.cos(t)
            time = np.linspace(1, 40, 20)[:, np.newaxis]
            self.trajectory = np.hstack([x, y, t, time])
        elif traj_id == 5: # rotate on spot
            num_pts = 20
            t = np.linspace(0,2*np.pi, num_pts)[:, np.newaxis]
            time = np.linspace(1, 10, num_pts)[:, np.newaxis]
            self.trajectory = np.hstack([np.zeros(t.shape), np.zeros(t.shape), t, time])
        elif traj_id == 6: # rotate along line
            num_pts = 20
            time = np.linspace(0, 20, num_pts)[:, np.newaxis]
            t = np.linspace(0,-2*np.pi, num_pts)[:, np.newaxis]
            y = np.linspace(0,10, num_pts)[:, np.newaxis]
            x = np.zeros(t.shape)
            self.trajectory = np.hstack([x, y, t, time])
        else:
            raise ValueError("Invalid traj_id")

    def modeCallback(self, msg):
        self.mode = Mode(msg.data)

    def publish(self):
        print("publishing")
        for robot_id in self.robot_publishers:
            robot_trajectory = self.trajectory + self.robot_offsets[robot_id]
            trajectory = Trajectory()
            trajectory.x.data = robot_trajectory[:,0]
            trajectory.y.data = robot_trajectory[:,1]
            trajectory.theta.data = robot_trajectory[:,2]
            trajectory.t.data = robot_trajectory[:,3]
            self.robot_publishers[robot_id].publish(trajectory)
            #print(robot_id, "pusblished", trajectory)
        self.trajectory_published = True

    def run(self):
        rate = rospy.Rate(10) # 10 Hz
        self.loadTrajectory()
        while not rospy.is_shutdown(): # and not self.trajectory_published:
            print("entered while loop")
            self.publish()
            rate.sleep()

if __name__ == '__main__':
    navigator = GlobalNavigator()
    navigator.run()
