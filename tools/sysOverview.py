#!/usr/bin/env python

"""
Logging script

Outputs all data for each robot requested in a table format.
Robot IDs (namespaces) are added to the --robot_id arg.

Arguments:
---------

static: if true table is cleared at each iteration and updated.
        If false table is printed beneath the most recently printed
        table. Empty string '' evaluates to False.

robot_ids: list of robot IDs (namespaces) to gather data from.

Example usage:
-------------

python sysOverview.py --static '' --robot_ids 'robot1' 'robot2' 'robot3' 'robot4'

python sysOverview.py --static False --robot_ids 'robot1' 'robot2'
"""

import os
import numpy as np
import rospy
import time
import argparse
import sys

from tabulate import tabulate

AR_COMMANDER_DIR = '/' + os.path.join(*(os.getcwd().split('/')[:-1]))
sys.path.append(AR_COMMANDER_DIR)

from scripts.stateMachine.stateMachine import Mode
from ar_commander.msg import Decawave, State, ControllerCmd
from std_msgs.msg import Int8

RATE = 3 # display rate
DECIMALS = 1



class Logger():
    def __init__(self, robot_IDs, static):
        rospy.init_node('logger', anonymous=True)

        self.static = static
        self.table = []
        self.header = ['']
        self.robot_IDs = robot_IDs
        self.robots = {}

        for robot_id in self.robot_IDs:
            self.robots[robot_id] = DataRetriever(robot_id)
            self.header.append(robot_id)


    def retrieveData(self):
        omega_cmd_data = ['omega cmds']
        phi_cmd_data = ['phi cmds']
        omega_data = ['omega']
        mode_data = ['mode']
        boardx_data = ['board x']
        boardy_data = ['board y']
        meas_theta_data = ['theta meas']
        pos_data = ['pos']
        vel_data = ['vel']
        state_theta_data = ['theta state']

        # collect data from robots
        for robot_id in self.robot_IDs:
            if self.robots[robot_id].state_pos is not None:
                pos_data.append(self.robots[robot_id].state_pos)
            if self.robots[robot_id].state_theta is not None:
                state_theta_data.append(self.robots[robot_id].state_theta)
            if self.robots[robot_id].theta is not None:
                meas_theta_data.append(self.robots[robot_id].theta)
            if self.robots[robot_id].state_vel is not None:
                vel_data.append(self.robots[robot_id].state_vel)
            if self.robots[robot_id].state_omega is not None:
                omega_data.append(self.robots[robot_id].state_omega)
            if self.robots[robot_id].boardX is not None:
                boardx_data.append(self.robots[robot_id].boardX)
            if self.robots[robot_id].boardY is not None:
                boardy_data.append(self.robots[robot_id].boardY)
            if self.robots[robot_id].cmd_omega_arr is not None:
                omega_cmd_data.append(self.robots[robot_id].cmd_omega_arr)
            if self.robots[robot_id].cmd_phi_arr is not None:
                phi_cmd_data.append(self.robots[robot_id].cmd_phi_arr)
            if self.robots[robot_id].mode is not None:
                mode_data.append(self.robots[robot_id].mode)

        # populate table
        self.table.append(pos_data)
        self.table.append(state_theta_data)
        self.table.append(vel_data)
        self.table.append(omega_data)
        self.table.append(boardx_data)
        self.table.append(boardy_data)
        self.table.append(meas_theta_data)
        self.table.append(omega_cmd_data)
        self.table.append(phi_cmd_data)
        self.table.append(mode_data)


    def resetTable(self):
        self.table = []
        if self.static:
            os.system('clear')


    def run(self):
        self.retrieveData()
        if len(self.table) > 0:
            print(tabulate(self.table, self.header))



class DataRetriever():
    def __init__(self, robot_id):
        self.state_pos = None
        self.state_vel = None
        self.state_theta = None
        self.state_omega = None

        self.boardY = None
        self.boardX = None
        self.theta = None

        self.cmd_omega_arr = None
        self.cmd_phi_arr = None
        self.cmd_robot_vel = None
        self.cmd_robot_omega = None

        self.mode = None

        # subscribers
        rospy.Subscriber('/'+ robot_id +'/sensor/decawave_measurement', Decawave, self.decawaveCallback)
        rospy.Subscriber('/'+ robot_id +'/estimator/state', State, self.stateCallback)
        rospy.Subscriber('/'+ robot_id +'/controller_cmds', ControllerCmd, self.cmdCallback)
        rospy.Subscriber('/'+ robot_id +'/state_machine/mode', Int8, self.modeCallback)


    def decawaveCallback(self, msg):
        self.boardY = np.around(np.array([msg.x1.data,msg.y1.data]), decimals=DECIMALS)
        self.boardX = np.around(np.array([msg.x2.data,msg.y2.data]), decimals=DECIMALS)
        self.theta = np.around(msg.theta.data, decimals=DECIMALS)


    def stateCallback(self, msg):
        self.state_pos = np.around(np.array(msg.pos.data), decimals=DECIMALS)
        self.state_vel = np.around(np.array(msg.vel.data), decimals=DECIMALS)
        self.state_theta = np.around(msg.theta.data, decimals=DECIMALS)
        self.state_omega = np.around(msg.omega.data, decimals=DECIMALS)


    def cmdCallback(self, msg):
        self.cmd_omega_arr = np.around(np.array(msg.omega_arr.data), decimals=DECIMALS)
        self.cmd_phi_arr = np.around(np.array(msg.phi_arr.data), decimals=DECIMALS)
        self.cmd_robot_vel = np.around(np.array(msg.robot_vel.data), decimals=DECIMALS)
        self.cmd_robot_omega = np.around(msg.robot_omega.data, decimals=DECIMALS)


    def modeCallback(self, msg):
        self.mode = str(Mode(msg.data))[5:]



if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('--static', type=bool, default=True)
    parser.add_argument('--robot_ids', nargs='+', required=True)
    args = parser.parse_args()

    logger = Logger(args.robot_ids, args.static)
    rate = rospy.Rate(RATE)
    while not rospy.is_shutdown():
        logger.run()
        rate.sleep()
        logger.resetTable()


