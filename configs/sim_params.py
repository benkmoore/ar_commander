import numpy as np

CONTROLLER_RATE = 10                       # rate in Hz

max_vel = 3                                # m/s

wp_threshold = 0.25                        # distance threshold at which we transition to next waypoint (m)

theta_threshold = np.pi/8				   # robot theta threshold at which we transition to next waypoint (rad)

thetaControllerGains = {'kp':3, 'ki':0.03, 'kd':5e-2}

pointControllerGains = {'kp':0.5, 'kd':0}

trajectoryControllerTF = {'num': np.array([ 0.63238478, -0.63175271]), 'den': np.array([ 1.        , -1.36724737,  0.36787944])}

# noise/uncertainty estimate on predict (Q) meas. (R) process and derivatives (d)
positionFilterParams = {'Q':10*np.eye(2), 'Q_d':100*np.eye(2), 'R':0.01*np.eye(2), 'R_d':100*np.eye(2)}

thetaFilterParams = {'Q':10, 'Q_d':100, 'R':0.01}
