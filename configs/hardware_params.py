import numpy as np

CONTROLLER_RATE = 70                       # rate in Hz

max_vel = 3                                # m/s

wp_threshold = 0.05                        # distance threshold at which we transition to next waypoint (m)

thetaControllerGains = {'kp':175, 'ki':0.03, 'kd':5e-8}

pointControllerGains = {'kp':180, 'kd':0}

trajectoryControllerGains = {'kp_pos':40, 'kp_th':0.75, 'kd_pos':0.5, 'k_ol':50}

# noise/uncertainty estimate on predict (Q) meas. (R) process and derivatives (d)
localizationFilterParams = {'Q':10*np.eye(3), 'Q_d':100*np.eye(3), 'R':0.01*np.eye(3), 'R_d':np.eye(3)}

thetaFilterParams = {'Q':10, 'Q_d':100, 'R':0.01, 'R_d':100}
