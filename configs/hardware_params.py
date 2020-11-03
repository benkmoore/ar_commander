import numpy as np

CONTROLLER_RATE = 70                       # rate in Hz

max_vel = 1.0  #0.75                         # m/s

wp_threshold = 0.4                        # distance threshold at which we transition to next waypoint (m)

theta_threshold = np.pi/5				   # robot theta threshold at which we transition to next waypoint (rad)
time_threshold = 1.0
ctrl_tf_state = {'num': np.array([ 0.01418517, -0.01418314]), 'den': np.array([ 1.0 , -1.98581382,  0.98581584])}
ctrl_tf_state_dot = {'num': np.array([0.00356506]), 'den': np.array([ 1.0, -0.99643494])}

# filter params obtained from data and testing at: AR1-142
# Q : covariance on the process noise
positionFilterParams = {"Q": 10 * np.eye(2), "Q_d": 10 * np.eye(2)}

thetaFilterParams = {"Q": 1, "Q_d": 1}

pos_measurement_std = 0.035 # position measurement standard deviation (m)

decawave_ports = ['/dev/ttyACM1','/dev/ttyACM2']

object_offset = {"x": 0.7, "y": 0.5}
