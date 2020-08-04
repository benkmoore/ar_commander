
import numpy as np

N = 4                           # number of wheels
R1 = np.array([0.075, 0.425])   # position of wheels along arm 1
R2 = np.array([0.075, 0.425])   # "" arm 2

phi_bounds = np.array([-4*np.pi,4*np.pi])  # physical bounds for wheel angle
