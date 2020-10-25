import numpy as np

CONTROLLER_RATE = 10                       # rate in Hz

max_vel = 3                                # m/s

wp_threshold = 0.25                        # distance threshold at which we transition to next waypoint (m)

theta_threshold = np.pi/8				   # robot theta threshold at which we transition to next waypoint (rad)

# controller gains: Kp = 1.0, Ki = 0.01, Kd = 0.25, rate = 10 Hz
ctrl_tf_state = {'num': np.array([ 0.09520937, -0.09511421]), 'den': np.array([ 1.0 , -1.90474226,  0.90483742])}
ctrl_tf_state_dot = {'num': np.array([0.02469009]), 'den': np.array([ 1.0, -0.97530991])}

# filter params obtained from data and testing at: AR1-142
# Q : covariance on the process noise
positionFilterParams = {"Q": 0.8 * np.eye(2), "Q_d": 0.8 * np.eye(2)}

thetaFilterParams = {"Q": 0.1, "Q_d": 0.1}
