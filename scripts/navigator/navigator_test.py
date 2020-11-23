#!/usr/bin/env python

import numpy as np
import numpy.linalg as npl
import rospy
import sys

from ar_commander.msg import Trajectory, State
from std_msgs.msg import Int8
from stateMachine.stateMachine import Mode

sys.path.append(rospy.get_param("AR_COMMANDER_DIR"))
env = "hardware" #rospy.get_param("ENV")
if env == "sim":
    import configs.sim_params as params
elif env == "hardware":
    import configs.hardware_params as params
else:
    raise ValueError("Controller ENV: '{}' is not valid. Select from [sim, hardware]".format(env))

class Navigator():
    def __init__(self):
        rospy.init_node('navigator_test')

        self.mode = None
        self.trajectory = np.zeros([1,3])
        self.trajectory_published = False

        self.pos = None

        self.start_wp = None
        self.desiredSpeed = 0.4 # m/s

        # subscribers
        rospy.Subscriber('/robot3/state_machine/mode', Int8, self.modeCallback)
        rospy.Subscriber('/robot3/estimator/state', State, self.stateCallback)

        # publishers
        self.pub_trajectory = rospy.Publisher('/robot3/cmd_trajectory', Trajectory, queue_size=10)

    def modeCallback(self,msg):
        self.mode = Mode(msg.data)

    def stateCallback(self,msg):
        self.pos = msg.pos.data

    def calcTime(self):
        if self.pos is not None:
            self.start_wp = self.pos
            prev_time = params.startup_time
            for i in range(0, self.trajectory.shape[0]):
                print(self.start_wp)
                self.trajectory[i,3] = (npl.norm(self.trajectory[i,0:2] - self.start_wp) / self.desiredSpeed) + prev_time
                self.start_wp = self.trajectory[i,0:2]
                prev_time = self.trajectory[i,3]
            print(self.trajectory)

    def loadTrajectory(self):
        traj_id = 1 # specify what trajectory we want to use

        if traj_id == 1:    # square (theta=0)
            self.trajectory = np.array([
                [2.5, 8, np.pi, 20],
                [2.5, 0, np.pi, 40],
                #[2.5, 1, 0, 46],
                #[1,1,0,1],
                #[2.5,1,0,4],
                #[2.5,4,0,9],
                #[1,4,0,12],
                #[1,1,0,17],
                #[2.5,1,0,20]
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
                self.calcTime()
                self.publish()
            rate.sleep()

if __name__ == '__main__':
    navigator = Navigator()
    navigator.run()
