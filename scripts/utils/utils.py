#!/usr/bin/env python

import rospy
import numpy as np

class robotConfig():
    def __init__(self):
        self.N = rospy.get_param("N")
        self.R1 = np.asarray(rospy.get_param("R1"))
        self.R2 = np.asarray(rospy.get_param("R2"))
        self.L = rospy.get_param("L")
        self.phi_bounds = rospy.get_param("phi_bounds")
        self.wheel_radius = rospy.get_param("wheel_radius")