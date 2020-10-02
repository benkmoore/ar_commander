
import scipy.interpolate as spi
import scipy.signal as sps
import numpy as np
import numpy.linalg as npl

class TrajectoryController():
    def __init__(self, ctrl_tf):
        num = ctrl_tf['num']
        den = ctrl_tf['den']

        self.x_ctrl = PIDController(num, den)
        self.y_ctrl = PIDController(num, den)
        self.theta_ctrl = PIDController(num, den)


    def getControlCmds(self, state, wp):
        # reference inputs
        r_x, r_y, r_theta = np.split(wp - state, 3)

        # control commands
        u_x = self.x_ctrl.getControlCmd(r_x)
        u_y = self.y_ctrl.getControlCmd(r_y)
        u_theta = self.theta_ctrl.getControlCmd(r_theta)
        u = np.concatenate((u_x, u_y, u_theta), axis=None)

        return u



class PIDController():
    def __init__(self, num, den):
        # closed loop transfer function: Y / R = [C] / [1+C]
        self.num = num
        self.den = den


    def getControlCmd(self, r):
        u = sps.lfilter(self.num, self.den, np.array([r]))

        return u

