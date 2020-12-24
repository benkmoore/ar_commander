#!/usr/bin/env python

import rospy
import numpy as np

class robotConfig():
    def __init__(self, namespace=""):
        self.N = rospy.get_param(namespace + "N")
        self.R1 = np.asarray(rospy.get_param(namespace + "R1"))
        self.R2 = np.asarray(rospy.get_param(namespace + "R2"))
        self.L = rospy.get_param(namespace + "L")
        self.phi_bounds = rospy.get_param(namespace + "phi_bounds")
        self.wheel_radius = rospy.get_param(namespace + "wheel_radius")
        self.enable_reverse = rospy.get_param(namespace + "enable_reverse")


def wrapAngle(angle):
    """Wraps any angle to [-pi, pi] interval"""

    angle = (angle + np.pi) % (2 * np.pi) - np.pi

    return angle