#!/usr/bin/env python

import rospy
from astar import *
from ar_commander.msg import Trajectory
from std_msgs.msg import Float64MultiArray

width = 10
height = 10
x_init = (0.0,0.0)
x_goal = (8.0,8.0)
obstacles = [((6,6),(8,7)),((2,0),(8,2)),((2,4),(4,6)),((6,2),(8,4))]
occupancy = DetOccupancyGrid2D(width, height, obstacles)

astar = AStar((0, 0), (width, height), x_init, x_goal, occupancy)

if not astar.solve():
    print("No path found")
    exit(0)

#astar.plot_path()

rospy.init_node('navigator')
pub_trajectory = rospy.Publisher('/cmd_trajectory', Trajectory, queue_size=10)

trajectory = Trajectory()
trajectory.x.data = (astar.path*astar.resolution)[:,0]
trajectory.y.data = (astar.path*astar.resolution)[:,1]
trajectory.theta.data = np.zeros((len(trajectory.x.data),1))

rate = rospy.Rate(10) # 10 Hz
while not rospy.is_shutdown():
	pub_trajectory.publish(trajectory)
	rate.sleep()

