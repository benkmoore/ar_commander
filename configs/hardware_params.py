import numpy as np

CONTROLLER_RATE = 70                       # rate in Hz

max_vel = 0.75                         # m/s

wp_threshold = 0.4                        # distance threshold at which we transition to next waypoint (m)

theta_threshold = np.pi				   # robot theta threshold at which we transition to next waypoint (rad)

trajectoryControllerTF = {'num': np.array([ 0.2, -0.3722704 ,  0.17246762]), 'den': np.array([ 1., -1.97219114,  0.97238837])}

# filter params obtained from data and testing at: AR1-142
# Q : covariance on the process noise
positionFilterParams = {"Q": 0.01 * np.eye(2), "Q_d": 0.01 * np.eye(2)}

thetaFilterParams = {"Q": 0.01, "Q_d": 0.01}

pos_measurement_std = 0.035 # position measurement standard deviation (m)

decawave_ports = ['/dev/ttyACM1','/dev/ttyACM2']
