
CONTROLLER_RATE = 10                       # rate in Hz

max_vel = 3                                # m/s

wp_threshold = 0.05                        # m, waypoint threshold

thetaControllerGains = {'kp':3, 'ki':0.03, 'kd':5e-2}

pointControllerGains = {'kp':0.5, 'kd':0}

trajectoryControllerGains = {'kp_pos':12, 'kp_th':0.75, 'kd_pos':0.5, 'k_ol':5}
