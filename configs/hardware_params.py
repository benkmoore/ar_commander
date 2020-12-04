import numpy as np

CONTROLLER_RATE = 70                       # rate in Hz

max_vel = 3                                # m/s

wp_threshold = 0.05                        # distance threshold at which we transition to next waypoint (m)

theta_threshold = np.pi/8				   # robot theta threshold at which we transition to next waypoint (rad)

trajectoryControllerTF = {'num': np.array([ 0.2, -0.3722704 ,  0.17246762]), 'den': np.array([ 1., -1.97219114,  0.97238837])}

# filter params obtained from data and testing at: AR1-142
# Q : covariance on the process noise
positionFilterParams = {"Q": 0.8 * np.eye(2), "Q_d": 0.8 * np.eye(2)}

thetaFilterParams = {"Q": 0.1, "Q_d": 0.1}

pos_measurement_std = 0.035                # position measurement standard deviation (m)

decawave_ports = ['/dev/ttyACM1', '/dev/ttyACM2'] # sensor usb ports: [Y axis arm of robot, X axis arm of robot]
