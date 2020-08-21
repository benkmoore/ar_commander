#!/usr/bin/env python

"""
State Machine Script for Autonomy Stack
Handles the FSM and high level mission logic
"""

from enum import Enum
import numpy as np
import numpy.linalg as npl
import rospy

from ar_commander.msg import State, Trajectory
from std_msgs.msg import Int8, Bool


RATE = 10
INIT_TIME = 5   # Minimum time to remain in init mode
POS_THRESH = 0.05   # Position error threshold to determine if trajectory is finished
THETA_THRESH = 0.1  # theta error threshold...

class Mode(Enum):
    """State machine modes"""
    INIT         = 0
    IDLE         = 1
    TRAJECTORY   = 2

class StateMachine():
    def __init__(self):
        rospy.init_node('state_machine')

        # internal variables
        self.mode = Mode.INIT
        self.prev_mode = None
        self.mode_start_time = None
        self.trajectory = None

        self.new_traj_flag = False
        self.last_wp_flag = False

        self.pos = None
        self.theta = None

        # subscribers
        rospy.Subscriber('estimator/state', State, self.stateCallback)
        rospy.Subscriber('/cmd_trajectory', Trajectory, self.trajectoryCallback)
        rospy.Subscriber('controller/last_waypoint_flag', Bool, self.lastWpCallback)

        # publishers
        self.pub_mode = rospy.Publisher('state_machine/mode', Int8, queue_size=10)

    ## Callbak functions
    def stateCallback(self, msg):
        self.pos = np.array(msg.pos.data)
        self.vel = np.array(msg.vel.data)
        self.theta = msg.theta.data

    def trajectoryCallback(self,msg):
        self.new_traj_flag = True
        self.trajectory = np.vstack([msg.x.data,msg.y.data,msg.theta.data]).T

    def lastWpCallback(self,msg):
        self.last_wp_flag = msg.data

    ## Decision Fuctions
    def hasInitialized(self):
        # TODO: add more checks (is teensy running, do we have a battery measurement etc)
        check1 = rospy.get_rostime() - self.mode_start_time > rospy.Duration.from_sec(INIT_TIME)
        check2 = self.pos is not None

        if check1 and check2:
            return True
        return False

    def newTrajectoryReceived(self):
        if self.new_traj_flag:
            self.new_traj_flag = False
            return True
        return False

    def trajectoryFinished(self):
        wp_final = self.trajectory[-1,:]
        check1 = npl.norm(self.pos - wp_final[0:2]) < POS_THRESH
        check2 = self.theta - wp_final[2] < THETA_THRESH
        check3 = self.last_wp_flag
        if check1 and check2 and check3:
            return True
        return False

    ## Main loop for FSM
    def determineMode(self):
        """ main state machine function """

        if not(self.prev_mode == self.mode):
            self.prev_mode = self.mode
            self.mode_start_time = rospy.get_rostime()

        if self.mode == Mode.INIT:
            if self.hasInitialized():
                self.mode = Mode.IDLE

        elif self.mode == Mode.IDLE:
            if self.newTrajectoryReceived():
                self.mode = Mode.TRAJECTORY

        elif self.mode == Mode.TRAJECTORY:
            if self.trajectoryFinished():
                self.mode = Mode.IDLE

    ## Process Functinos
    def publish(self):
        """ publish the fsm mode """
        msg = Int8()
        msg.data = self.mode.value
        self.pub_mode.publish(msg)

    def run(self):
        rate = rospy.Rate(RATE) # 10 Hz
        while not rospy.is_shutdown():
            self.determineMode()
            self.publish()
            rate.sleep()

if __name__ == '__main__':
    state_machine = StateMachine()
    state_machine.run()

