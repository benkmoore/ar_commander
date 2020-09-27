
import scipy as sp
import numpy as np


class TrajectoryController():
    def __init__(self, num, den):
        self.x_ctrl = PIDController(num, den)
        self.y_ctrl = PIDController(num, den)
        self.theta_ctrl = PIDController(num, den)


    def waypoints2Spline(self, trajectory):
        x_pts, y_pts, theta_pts = np.split(trajectory, 3)

        self.x_spline = sp.interpolate.UnivariateSpline(y_pts, x_pts) # output: x_des, input: y
        self.y_spline = sp.interpolate.UnivariateSpline(x_pts, y_pts) # output: y_des, input: x
        self.theta_spline = sp.interpolate.SmoothBivariateSpline(x_pts, y_pts, theta_pts) # output: theta_des, input: x, y


    def getControlCmds(self, state):
        # reference inputs
        x, y, theta = np.split(state, 3)
        r_x = self.x_spline(y)
        r_y = self.y_spline(x)
        r_theta = self.theta_spline.ev(x, y)

        # control commands
        u_x = self.x_ctrl.getControlCmd(r_x)
        u_y = self.y_ctrl.getControlCmd(r_y)
        u_theta = self.theta_ctrl.getControlCmd(r_theta)
        u = np.hstack((u_x, u_y, u_theta))

        return u



class PIDController():
    def __init__(self, num, den):
        # closed loop transfer function: Y / R = [C] / [1+C]
        self.num = num
        self.den = den


    def getControlCmd(self, r):
        u = sp.signal.lfilter(self.num, self.den, r)

        return u 

