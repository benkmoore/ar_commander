
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

        self.x_spline = None
        self.y_spline = None
        self.theta_spline = None


    def waypoints2Spline(self, trajectory):
        x, y, theta = np.split(trajectory.reshape(-1, 3), 3, axis=1)
        x = x.reshape(-1)
        y = y.reshape(-1)
        x_uniq, x_idxs = np.unique(x, return_index=True) # drop duplicates for spline fit
        y_uniq, y_idxs = np.unique(y, return_index=True)

        self.x_spline = spi.splrep(y_uniq, x[y_idxs]) # input: y, output: x desired
        self.y_spline = spi.splrep(x_uniq, y[x_idxs]) # input: x, output: y desired
        self.th_spline = spi.interp2d(x, y, theta) # input: x & y, output: theta desired


    def getControlCmds(self, state, wp):
        x, y, theta = np.split(state, 3) 

        # reference inputs
        r_x = spi.splev(y, self.x_spline)
        r_y = spi.splev(x, self.y_spline)
        r_theta = self.th_spline(x,y)

        # control commands
        u_x = self.x_ctrl.getControlCmd(r_x)
        u_y = self.y_ctrl.getControlCmd(r_y)
        u_theta = self.theta_ctrl.getControlCmd(r_theta)
        u = np.hstack((u_x, u_y, np.zeros(1)))

        return u



class PIDController():
    def __init__(self, num, den):
        # closed loop transfer function: Y / R = [C] / [1+C]
        self.num = num
        self.den = den


    def getControlCmd(self, r):
        u = sps.lfilter(self.num, self.den, r)

        return u 

