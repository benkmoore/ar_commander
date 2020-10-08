#!/usr/bin/env python

import rospy
import numpy as np
import sys
import matplotlib.pyplot as plt
import matplotlib.animation as animation
fig = plt.figure()
ax1 = fig.add_subplot(1,1,1)
import rospkg

sys.path.append(rospkg.RosPack().get_path('ar_commander'))
import configs.robot_v1 as rcfg

from ar_commander.msg import TOF, State


class TOFInterface():
    def __init__(self):
        rospy.init_node('tofInterface', anonymous=True)

        self.tof_data = None
        self.transformed_data = None

        self.pos = np.zeros(2)
        self.vel = np.zeros(2)
        self.theta = 0
        self.omega = None
        self.state_flag = False

        rospy.Subscriber('/robot1/sensor/tof_data', TOF, self.tofCallback)
        rospy.Subscriber('/robot1/estimator/state', State, self.stateCallback)

    def tofCallback(self, msg):
        self.tof_data = np.array([msg.tof1, msg.tof2, msg.tof3])

    def stateCallback(self, msg):
        self.pos = np.array(msg.pos.data)
        self.vel = np.array(msg.vel.data)
        self.theta = msg.theta.data
        self.omega = msg.omega.data
        self.state_flag = True

    def getSensorData(self):
        if self.tof_data is not None:
            d1 = np.sqrt(rcfg.L**2 + self.tof_data[0]**2)
            a1 = np.arctan(rcfg.L/self.tof_data[0])
            p1_x = self.pos[0] + d1*np.cos(a1 + self.theta)
            p1_y = self.pos[1] + d1*np.sin(a1 + self.theta)

            d2 = self.tof_data[1]
            p2_x = self.pos[0] + d2*np.cos((np.pi/4) + self.theta)
            p2_y = self.pos[1] + d2*np.sin((np.pi/4) + self.theta)

            d3 = np.sqrt(rcfg.L**2 + self.tof_data[2]**2)
            a3 = np.arctan(rcfg.L/self.tof_data[2])
            p3_x = self.pos[0] - d3*np.cos(a3 + (np.pi/2) - self.theta)
            p3_y = self.pos[1] + d3*np.sin(a3 + (np.pi/2) - self.theta)

            self.transformed_data = np.array([[p1_x, p1_y], [p2_x, p2_y], [p3_x, p3_y]])

    def animate(self):
        ax1.clear()
        ax1.scatter(self.transformed_data[:,0], self.transformed_data[:,1])
        plt.pause(0.05)

    def run(self):
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            self.getSensorData()
            if self.transformed_data is not None:
                self.animate()
            rate.sleep()


if __name__ == '__main__':
    tofInterface = TOFInterface()
    tofInterface.run()