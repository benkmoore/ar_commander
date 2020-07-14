#!/usr/bin/env python

RATE = 10                       # rate in Hz

thetaControllerGains = {'kp':3, 'ki':0.03, 'kd':5e-8}

pointControllerGains = {'kp':10, 'kd':0}

trajectoryControllerGains = {'kp_pos':12, 'kp_th':0.75, 'kd_pos':0.5, 'v_mag':6}
