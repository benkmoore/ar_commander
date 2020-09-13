import numpy as np
import numpy.linalg as npl
import numpy.random as npr


class LocalizationFilter():
    """
    localization filter. Uses statespace model to predict the next state of the robot via the
    dynamics, x(k+1) = Ax(k)+Bu(k) + w_process, with input, u and process noise, w_process.
    Updates the state using measurements y and covariance R via the measurement model
    y(k) = Cx(k) + w_meas, with measurement noise, w_meas.
    """

    def __init__(self, x0, sigma0, A, B, C, Q, w_process):
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
        # uncertainty, Q and noise, w_process on predict step
        self.Q = Q
        self.w_process = w_process


    def predict(self, u):
        self.x_pred = np.matmul(self.A, self.x) + np.matmul(self.B, u) + npr.normal(0.0, self.w_process)
        self.sigma_pred = npl.multi_dot([self.A, self.sigma, self.A.T]) + self.Q


    def update(self, y, R):
        y_delta = y - np.matmul(self.C, self.x_pred)
        innovation_cov = npl.pinv(npl.multi_dot([self.C, self.sigma_pred, self.C.T]) + R)

        self.x = self.x_pred + npl.multi_dot([self.sigma_pred, self.C.T, innovation_cov, y_delta])
        self.sigma = self.sigma_pred - npl.multi_dot([self.sigma_pred, self.C.T, innovation_cov, self.C, self.sigma_pred])


    def step(self, u, y, R):
        self.predict(u)
        if y is not None and R is not None:
            self.update(y, R)

        return self.x, self.sigma
