
import numpy as np

N = 4                           # number of wheels
R1 = np.array([0.15, 0.37])   # position of wheels along arm 1 (m)
R2 = np.array([0.15, 0.37])   # "" arm 2 (m)
L = 0.56							# robot arm length from corner (m)

phi_bounds = np.array([-2*np.pi,2*np.pi])  # physical bounds for wheel angle (rads)
wheel_radius = 0.05             	# m

