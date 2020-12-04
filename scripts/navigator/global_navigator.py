#!/usr/bin/env python
import numpy as np
import numpy.linalg as npl
import rospy
import sys

from ar_commander.msg import Trajectory, State
from std_msgs.msg import Int8

sys.path.append(rospy.get_param("AR_COMMANDER_DIR"))
env = "hardware" #rospy.get_param("ENV")
if env == "sim":
    import configs.sim_params as params
elif env == "hardware":
    import configs.hardware_params as params
else:
    raise ValueError("Controller ENV: '{}' is not valid. Select from [sim, hardware]".format(env))

from scripts.stateMachine.stateMachine import Mode


class GlobalNavigator():
    def __init__(self):
        rospy.init_node('navigator')

        self.mode = None

        self.trajectory_published = False
        self.trajectory = None

        self.start_wp = None
        self.desiredSpeed = 0.3 # m/s

        self.pos1 = None
        self.pos2 = None
        self.pos3 = None
        self.pos4 = None

        # subscribers
        rospy.Subscriber('/robot1/estimator/state', State, self.state1Callback)
        rospy.Subscriber('/robot2/estimator/state', State, self.state2Callback)
        rospy.Subscriber('/robot3/estimator/state', State, self.state3Callback)
        rospy.Subscriber('/robot4/estimator/state', State, self.state4Callback)

        # publishers
        # TODO initialize with for loop
        self.pub_trajectory1 = rospy.Publisher('/robot1/cmd_trajectory', Trajectory, queue_size=10)
        self.pub_trajectory2 = rospy.Publisher('/robot2/cmd_trajectory', Trajectory, queue_size=10)
        self.pub_trajectory3 = rospy.Publisher('/robot3/cmd_trajectory', Trajectory, queue_size=10)
        self.pub_trajectory4 = rospy.Publisher('/robot4/cmd_trajectory', Trajectory, queue_size=10)

        self.robot_offsets = {
            1: np.array([-params.object_offset["x"], -params.object_offset["y"], 0, 0]), #
            2: np.array([ params.object_offset["x"], -params.object_offset["y"], np.pi/2, 0]),
            3: np.array([ params.object_offset["x"],  params.object_offset["y"], np.pi, 0]),
            4: np.array([-params.object_offset["x"],  params.object_offset["y"], -np.pi/2, 0])
        }

        self.robot_publishers = {
            1: self.pub_trajectory1,
            2: self.pub_trajectory2,
            3: self.pub_trajectory3,
            4: self.pub_trajectory4
        }

    def calcTime(self):
        if self.pos1 is not None and self.pos2 is not None and self.pos3 is not None and self.pos4 is not None:
            self.start_wp = np.mean(np.vstack((self.pos1, self.pos2, self.pos3, self.pos4)))
            prev_time = params.startup_time
            for i in range(0, self.trajectory.shape[0]):
                self.trajectory[i,3] = (npl.norm(self.trajectory[i,0:2] - self.start_wp) / self.desiredSpeed) + prev_time
                self.start_wp = self.trajectory[i,0:2]
                prev_time = self.trajectory[i,3]
            print(self.trajectory)

    def loadTrajectory(self):
        print("loading trajectory")
        traj_id = 1 # specify what trajectory we want to use

        if traj_id == 1:    # square (theta=0)
            self.trajectory = np.array([
                # [0,0,0,0],
                [2.5,6,0,1],
                [2.5,2,0,2]
                #[3,2,0,38],
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

    def state1Callback(self, msg):
        self.pos1 = np.array(msg.pos.data)

    def state2Callback(self, msg):
        self.pos2 = np.array(msg.pos.data)

    def state3Callback(self, msg):
        self.pos3 = np.array(msg.pos.data)

    def state4Callback(self, msg):
        self.pos4 = np.array(msg.pos.data)

    def publish(self):
        print("publishing")
        for robot_id in self.robot_publishers:
            # if robot_id == 2:
            #     pass
            # else:
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
            self.calcTime()
            self.publish()
            rate.sleep()

if __name__ == '__main__':
    navigator = GlobalNavigator()
    navigator.run()
