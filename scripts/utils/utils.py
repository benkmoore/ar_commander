#!/usr/bin/env python
import numpy as np


def wrapAngle(angle):
    """Wraps any angle to [-pi, pi] interval"""

    angle = (angle + np.pi) % (2 * np.pi) - np.pi

    return angle