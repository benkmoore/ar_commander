import numpy as np

CONTROLLER_RATE = 10                       # rate in Hz

max_vel = 3                                # m/s

wp_threshold = 0.25                        # distance threshold at which we transition to next waypoint (m)

theta_threshold = np.pi/8				   # robot theta threshold at which we transition to next waypoint (rad)

pointControllerTF = {'num': np.array([ 0.63238478, -0.63175271]), 'den': np.array([ 1.        , -1.36724737,  0.36787944])}

trajectoryControllerTF = {'num': np.array([ 0.63238478, -0.63175271]), 'den': np.array([ 1.        , -1.36724737,  0.36787944])}

# filter params obtained from data and testing at: AR1-142
# Q : covariance on the process noise
positionFilterParams = {"Q": 0.8 * np.eye(2), "Q_d": 0.8 * np.eye(2)}

thetaFilterParams = {"Q": 0.1, "Q_d": 0.1}
