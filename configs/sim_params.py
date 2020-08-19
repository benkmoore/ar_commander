import numpy as np

CONTROLLER_RATE = 10                       # rate in Hz

max_vel = 3                                # m/s

wp_threshold = 0.25                        # distance threshold at which we transition to next waypoint (m)

thetaControllerGains = {'kp':3, 'ki':0.03, 'kd':5e-2}

pointControllerGains = {'kp':0.5, 'kd':0}

trajectoryControllerGains = {'kp_pos':0.25, 'kp_th':0.75, 'kd_pos':0.5, 'k_ol':0.5}

# noise/uncertainty estimate on predict (Q) meas. (R) process and derivatives (d)
localizationFilterParams = {'Q':10*np.eye(3), 'Q_d':100*np.eye(3), 'R':0.01*np.eye(3), 'R_d':100*np.eye(3)}
