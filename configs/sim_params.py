import numpy as np

CONTROLLER_RATE = 10                       # rate in Hz

max_vel = 3                                # m/s

wp_threshold = 0.25                        # distance threshold at which we transition to next waypoint (m)

theta_threshold = np.pi/8				   # robot theta threshold at which we transition to next waypoint (rad)

ctrl_tf_state = {'num': np.array([ 0.13938256, -0.13919684]), 'den': np.array([ 1.0, -1.86052226, 0.86070798])}
ctrl_tf_state_dot = {'num': np.array([0.0009995]), 'den': np.array([ 1.0, -0.9990005])}

# filter params obtained from data and testing at: AR1-142
# Q : covariance on the process noise
positionFilterParams = {"Q": 0.8 * np.eye(2), "Q_d": 0.8 * np.eye(2)}

thetaFilterParams = {"Q": 0.1, "Q_d": 0.1}
