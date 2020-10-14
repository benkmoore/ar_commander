#!/usr/bin/env python

import rospy
import numpy as np
import sys
import rospkg

sys.path.append(rospkg.RosPack().get_path('ar_commander'))
import configs.robot_v1 as rcfg

from ar_commander.msg import TOF, State, Object

RATE = 10


class TOFInterface():
    def __init__(self):
        rospy.init_node('tofInterface', anonymous=True)

        self.tof_data = None
        self.transformed_data = None

        self.pos = None
        self.vel = None
        self.theta = None
        self.omega = None

        # subscribers
        rospy.Subscriber('sensor/tof_data', TOF, self.tofCallback)
        rospy.Subscriber('estimator/state', State, self.stateCallback)

        # publishers
        self.pub_tof_data = rospy.Publisher('transformed/tof_data', TOF, self.tofCallback)

    def tofCallback(self, msg):
        self.tof_data = np.array([msg.tof1, msg.tof2, msg.tof3])
        self.transformTOFData()

    def stateCallback(self, msg):
        self.pos = np.array(msg.pos.data)
        self.vel = np.array(msg.vel.data)
        self.theta = msg.theta.data
        self.omega = msg.omega.data

    ## Helper functions
    def transformTOFData(self):
        if self.tof_data is not None and self.pos is not None:
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

            self.transformed_pts = np.array([[p1_x, p1_y], [p2_x, p2_y], [p3_x, p3_y]])

    ## Process functions
    def publish(self):
        if self.transformed_pts is not None:
            self.tof_msg = TOF()
            self.tof_msg.data = self.transformed_pts
            self.pub_tof_data.publish(self.tof_msg.data)

    def run(self):
        rate = rospy.Rate(RATE)
        while not rospy.is_shutdown():
            self.publish()
            rate.sleep()


if __name__ == '__main__':
    tofInterface = TOFInterface()
    tofInterface.run()