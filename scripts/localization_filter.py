import numpy as np
import numpy.linalg as npl
import numpy.random as npr


class LocalizationFilter():
    """
    localization filter. Uses statespace model to predict the next state of the robot via the
    dynamics, x(k+1) = Ax(k)+Bu(k) + w_process, with input, u and process noise, w_process.
    Updates the state using measurements y and covariance R via the measurement model
    y(k) = Cx(k). If state is an angle, indicated by bool angle variable, enforce angle wrap 
    to [-pi, pi].
    """

    def __init__(self, x0, sigma0, A, B, C, Q, angle):
        # initial state and covariance
        self.x = x0
        self.sigma = sigma0
        # predicted state and covariance
        self.x_pred = None
        self.sigma_pred = None
        # dynamics model
        self.A = A
        self.B = B
        # measurement model
        self.C = C
        # uncertainty, Q on predict step
        self.Q = Q
        # is state angle
        self.angle = angle
 

    # wrap angle to [-pi, pi] to avoid delta > pi in filter state
    def wrap_angle(self, angle):
        if abs(angle) > np.pi:
            angle = angle - np.sign(angle) * 2 * np.pi

        return angle


    def predict(self, u):
        self.x_pred = np.matmul(self.A, self.x) + np.matmul(self.B, u)
        self.sigma_pred = npl.multi_dot([self.A, self.sigma, self.A.T]) + self.Q


    def update(self, y, R):
        y_delta = y - np.matmul(self.C, self.x_pred)
        if self.angle:  # wrap delta angle along axis
            y_delta = np.apply_along_axis(self.wrap_angle, 0, y_delta)
        innovation_cov = npl.inv(npl.multi_dot([self.C, self.sigma_pred, self.C.T]) + R)

        self.x = self.x_pred + npl.multi_dot([self.sigma_pred, self.C.T, innovation_cov, y_delta])
        self.sigma = self.sigma_pred - npl.multi_dot([self.sigma_pred, self.C.T, innovation_cov, self.C, self.sigma_pred])


    def step(self, u, y, R):
        self.predict(u)
        if y is not None and R is not None:
            self.update(y, R)

        return self.x, self.sigma
