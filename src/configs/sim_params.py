import numpy as np

CONTROLLER_RATE = 10                       # rate in Hz

max_vel = 3                                # m/s

wp_threshold = 0.25                        # distance threshold at which we transition to next waypoint (m)

theta_threshold = np.pi/8				   # robot theta threshold at which we transition to next waypoint (rad)

thetaControllerGains = {'kp':3, 'ki':0.03, 'kd':5e-2}

pointControllerGains = {'kp':0.5, 'kd':0}

trajectoryControllerGains = {'kp_pos':0.25, 'kp_th':0.75, 'kd_pos':0.5, 'k_ol':0.5}

# filter params obtained from data and testing at: AR1-142
# Q : covariance on the process noise
positionFilterParams = {"Q": 0.8 * np.eye(2), "Q_d": 0.8 * np.eye(2)}

thetaFilterParams = {"Q": 0.1, "Q_d": 0.1}
