import numpy as np
import numpy.linalg as npl


class LocalizationFilter():
    def __init__(self, x0, sigma0, A, B, C, Q, R):
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
        self.x_pred = np.matmul(self.A, self.x) + np.matmul(self.B, u)
        self.sigma_pred = npl.multi_dot([self.A, self.sigma, self.A.T]) + self.Q

    def update(self, y):
        y_delta = y - np.matmul(self.C, self.x_pred)
        innovation_cov = npl.inv(npl.multi_dot([self.C, self.sigma_pred, self.C.T]) + self.R)

        self.x = self.x_pred + npl.multi_dot([self.sigma_pred, self.C.T, innovation_cov, y_delta])
        self.sigma = self.sigma_pred - npl.multi_dot([self.sigma_pred, self.C.T, innovation_cov, self.C, self.sigma_pred])

    def step(self, u, y):
        self.predict(u)
        self.update(y)

        return self.x, self.sigma
