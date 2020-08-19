import numpy as np
import numpy.linalg as npl


class LocalizationFilter():
    def __init__(self, dt, x0, sigma0, A, B, C, Q, R):
        self.dt = dt
        self.x = x0
        self.x_pred = None
        self.sigma = sigma0
        self.sigma_pred = None
        self.A = A
        self.B = B
        self.C = C
        self.Q = Q
        self.R = R

    def predict(self, u):
        self.x_pred = np.matmul(self.A, self.x) + np.matmul(self.B, u)*self.dt
        self.sigma_pred = npl.multi_dot([self.A, self.sigma, self.A.T]) + self.Q

    def update(self, y):
        y_delta = np.concatenate((y[0:2] - self.x_pred[0:2], np.zeros(1), y[3:5] - self.x_pred[3:5], np.zeros(1))) # only have 2d localization, no theta measurement yet
        innovation_cov = npl.inv(npl.multi_dot([self.C, self.sigma_pred, self.C.T]) + self.R)

        self.x = self.x_pred + npl.multi_dot([self.sigma_pred, self.C.T, innovation_cov, y_delta])
        self.sigma = self.sigma_pred - npl.multi_dot([self.sigma_pred, self.C.T, innovation_cov, self.C, self.sigma_pred])

    def step(self, u, y):
        if self.x is None:
            self.x = y

        self.predict(u)
        self.update(y)

        return self.x, self.sigma
