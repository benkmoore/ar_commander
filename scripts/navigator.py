#!/usr/bin/env python

import rospy
import numpy.linalg as npl

from astar import *
from ar_commander.msg import Trajectory
from std_msgs.msg import Float64MultiArray
from geometry_msgs.msg import Pose2D


class Navigator():
	def __init__(self):
		rospy.init_node('navigator')

		self.trajectory_complete = False
		self.map_width = 10
		self.map_height = 10
		self.obstacles = [((6,6),(8,7)),((2,0),(8,2)),((2,4),(4,6)),((6,2),(8,4))]

		self.start_pos = (None, None)
		self.end_pos = (8.0,8.0)
		self.pos = np.array([None, None])
		self.trajectory = None

		# publishers
		self.pub_trajectory = rospy.Publisher('/cmd_trajectory', Trajectory, queue_size=10)

		# subscribers
		rospy.Subscriber('/pose', Pose2D, self.poseCallback)

	def poseCallback(self, msg):
	    self.pos[0] = msg.x
	    self.pos[1] = msg.y
	    self.theta = msg.theta
	    if self.start_pos[0] == None:
	    	self.start_pos = (round(self.pos[0],1), round(self.pos[1],1))

	def callAstar(self):
		occupancy = DetOccupancyGrid2D(self.map_width, self.map_height, self.obstacles)
		self.astar = AStar((0, 0), (self.map_width, self.map_height), self.start_pos, self.end_pos, occupancy)

		if not self.astar.solve():
		    print("No path found")
		    exit(0)

		self.trajectory = Trajectory()
		self.trajectory.x.data = (self.astar.path*self.astar.resolution)[:,0]
		self.trajectory.y.data = (self.astar.path*self.astar.resolution)[:,1]
		self.trajectory.theta.data = np.zeros((len(self.trajectory.x.data),1))
		#astar.plot_path()

	def testTrajectoryComplete(self):
		if npl.norm(self.end_pos-self.pos) < 0.1:
			self.trajectory_complete = True

	def publish(self):
		self.pub_trajectory.publish(self.trajectory)

	def run(self):
		rate = rospy.Rate(10) # 10 Hz
		while not rospy.is_shutdown() and not self.trajectory_complete:
			if self.trajectory == None and self.start_pos[0] != None:
				self.callAstar()
				print("Path found...")
			if self.trajectory != None:
				self.publish()
				print("Trajectory published...")
				self.testTrajectoryComplete()
			rate.sleep()
		print("Navigator exiting...")

if __name__ == '__main__':
    navigator = Navigator()
    navigator.run()
