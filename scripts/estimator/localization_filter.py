import numpy as np
import numpy.linalg as npl


class LocalizationFilter:
    def __init__(self, x0, sigma0, A, B, Q, R):
        self.x = x0
        self.x_pred = None
        self.sigma = sigma0
        self.sigma_pred = None
        self.A = A
        self.B = B
        self.Q = Q
        self.R = R

    def predict(self, u):
        self.x_pred = np.matmul(self.A, self.x) + np.matmul(self.B, u)
        self.sigma_pred = npl.multi_dot([self.A, self.sigma, self.A.T]) + self.Q

    def update(self, y, C):
        y_delta = y - np.matmul(C, self.x_pred)
        innovation_cov = npl.inv(npl.multi_dot([C, self.sigma_pred, C.T]) + self.R)

        self.x = self.x_pred + npl.multi_dot(
            [self.sigma_pred, C.T, innovation_cov, y_delta]
        )
        self.sigma = self.sigma_pred - npl.multi_dot(
            [self.sigma_pred, C.T, innovation_cov, C, self.sigma_pred]
        )

    def step(self, u, y, C):
        self.predict(u)
        if y is not None:
            self.update(y, C)

        return self.x, self.sigma
