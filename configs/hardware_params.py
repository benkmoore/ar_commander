import numpy as np

CONTROLLER_RATE = 70                       # rate in Hz

startup_time = 6.0                         # s

max_vel = 1.0  #0.75                       # m/s
max_omega = 0.175                          # rad/s

wp_threshold = 0.4                         # distance threshold at which we transition to next waypoint (m)

max_acceleration = 1.0                     # max acceleration (m/s^2)

theta_threshold = np.pi/5				   # robot theta threshold at which we transition to next waypoint (rad)
time_threshold = 1.0

ctrl_tf_state = {'num': np.array([ 0.00853581, -0.00853378 ]), 'den': np.array([1.        , -1.99146317,  0.9914652])}
ctrl_tf_state_dot = {'num': np.array([0.01418416]), 'den': np.array([ 1.        , -0.98581584])}

# filter params obtained from data and testing at: AR1-142
# Q : covariance on the process noise
positionFilterParams = {"Q": 0.002 * np.eye(2), "Q_d": 0.002 * np.eye(2)}

thetaFilterParams = {"Q": 1, "Q_d": 1}

pos_measurement_std = 0.035 # position measurement standard deviation (m)

decawave_ports = ['/dev/ttyACM1','/dev/ttyACM2']

object_offset = {"x": 1.0, "y": 0.75}
