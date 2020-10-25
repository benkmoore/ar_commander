import numpy as np

CONTROLLER_RATE = 70                       # rate in Hz

max_vel = 3                                # m/s

wp_threshold = 0.05                        # distance threshold at which we transition to next waypoint (m)

theta_threshold = np.pi/8				   # robot theta threshold at which we transition to next waypoint (rad)

# controller gains: Kp = 1.0, Ki = 0.01, Kd = 0.25, rate = 70 Hz
ctrl_tf_state = {'num': np.array([ 0.01418517, -0.01418314]), 'den': np.array([ 1.0 , -1.98581382,  0.98581584])}
ctrl_tf_state_dot = {'num': np.array([0.00356506]]), 'den': np.array([ 1.0, -0.99643494])}

# filter params obtained from data and testing at: AR1-142
# Q : covariance on the process noise
positionFilterParams = {"Q": 0.8 * np.eye(2), "Q_d": 0.8 * np.eye(2)}

thetaFilterParams = {"Q": 0.1, "Q_d": 0.1}

pos_measurement_std = 0.035 # position measurement standard deviation (m)
