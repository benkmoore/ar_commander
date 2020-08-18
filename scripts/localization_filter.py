import numpy as np
import numpy.linalg as npl


class LocalizationFilter():
    def __init__(self, rate, x0, sigma0, A, B, C, Q, R):
        self.dt = 1. / rate
        self.x = x0
        self.sigma = sigma0
        self.A = A
        self.B = B
        self.C = C
        self.Q = Q
        self.R = R

    def predict(self, u):
        self.x_pred = np.matmul(self.A, self.x) + np.matmul(self.B, u)*self.dt
        self.sigma_pred = self.A.dot(self.sigma).dot(self.A.T) + self.Q

    def update(self, pos_meas):
        y_delta = np.concatenate((pos_meas[0:2] - self.x_pred[0:2], np.zeros(1))) # only have 2d localization, no theta measurement yet
        innovation_cov = npl.inv(self.C.dot(self.sigma_pred).dot(self.C.T) + self.R)

        self.x = self.x_pred + self.sigma_pred.dot(self.C.T).dot(innovation_cov).dot(y_delta)
        self.sigma = self.sigma_pred - self.sigma_pred.dot(self.C.T).dot(innovation_cov).dot(self.C).dot(self.sigma_pred)

    def run(self, u, pos_meas):
        if self.x is None:
            self.x = pos_meas
        if pos_meas is not None and u is not None:
            self.predict(u)
            self.update(pos_meas)
