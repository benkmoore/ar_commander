
import scipy.interpolate as spi
import scipy.signal as sps
import numpy as np

class TrajectoryController():
    def __init__(self, num, den):
        self.x_ctrl = PIDController(num, den)
        self.y_ctrl = PIDController(num, den)
        self.theta_ctrl = PIDController(num, den)

        self.x_spline = None
        self.y_spline = None
        self.theta_spline = None


    def waypoints2Spline(self, trajectory):
        x, y, theta = np.split(trajectory, 3, axis=1)
        order_x = np.argsort(x.reshape(-1))
        order_y = np.argsort(y.reshape(-1))

        self.x_spline = spi.UnivariateSpline(y[order_y], x[order_y]) # output: x_des, input: y
        self.y_spline = spi.UnivariateSpline(x[order_x], y[order_x]) # output: y_des, input: x
        self.theta_spline = spi.SmoothBivariateSpline(x, y, theta) # output: theta_des, input: x, y

    def getControlCmds(self, state):
        # reference inputs
        x, y, theta = np.split(state, 3)
        r_x = self.x_spline(y)
        r_y = self.y_spline(x)
        r_theta = self.theta_spline.ev(x, y)
        print(r_x, r_y, r_theta)

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
        print(u)
        return u 

