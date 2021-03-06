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

    def __init__(self, x0, sigma0, A, B, C, Q, is_angle=False):
        # predicted and estimated states/covariances
        self.x = x0
        self.sigma = sigma0
        self.x_pred = None
        self.sigma_pred = None

        # state-space model
        self.A = A
        self.B = B
        self.C = C

        self.Q = Q  # covariance on process noise

        self.is_angle = is_angle   # specifies whether state is an angle (and needs to be wrapped)


    def wrap_angle(self, angle):
        """Wraps angle to [-pi, pi] to avoid delta > pi in filter state"""
        if abs(angle) > np.pi:
            angle = angle - np.sign(angle) * 2 * np.pi

        return angle


    def predict(self, u):
        self.x_pred = np.matmul(self.A, self.x) + np.matmul(self.B, u)
        self.sigma_pred = npl.multi_dot([self.A, self.sigma, self.A.T]) + self.Q


    def update(self, y, R):
        y_delta = y - np.matmul(self.C, self.x_pred)
        if self.is_angle:  # wrap delta angle along axis
            y_delta = np.apply_along_axis(self.wrap_angle, 0, y_delta)
        innovation_cov = npl.inv(npl.multi_dot([self.C, self.sigma_pred, self.C.T]) + R)

        self.x = self.x_pred + npl.multi_dot([self.sigma_pred, self.C.T, innovation_cov, y_delta])
        self.sigma = self.sigma_pred - npl.multi_dot([self.sigma_pred, self.C.T, innovation_cov, self.C, self.sigma_pred])


    def step(self, u, y, R):
        self.predict(u)
        if y is not None and R is not None:
            self.update(y, R)

        return self.x, self.sigma
