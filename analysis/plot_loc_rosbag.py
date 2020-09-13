"""
Plot localization data from rosbags from a specific folder. Script processes
each rosbag and plots the data, saving the plot to the same folder as a png.

example usage:

python plot_loc_rosbag --dir ~/folder_of_rosbags

"""


import rosbag
import rospy
import numpy as np
import numpy.linalg as npl
import argparse, os
import matplotlib.pyplot as plt

from ar_commander.msg import State, Decawave

parser = argparse.ArgumentParser()
parser.add_argument('--dir')
args = parser.parse_args()

# bag extension names
exts = ('.bag')
processed_ext = '_OUT.bag'

# config plots
m_col1 = 'royalblue'
m_col2 = 'red'
est_col = 'lime'
display = '-'

# decawave sensor arm lengths v1
L1 = 0.41
L2 = 0.425

theta = 0.0

for root, dirs, files in os.walk(args.dir):
	print("Processing ... ")
	for file in files:
		est_x = []
		est_y = []
		est_vx = []
		est_vy = []
		x1 = []
		x2 = []
		y1 = []
		y2 = []
		cov_det_p1 = []
		cov_det_p2 = []
		cov_theta = []
		meas_theta = []
		est_theta = []
		T = []
		T_decawave = []
		T_est = []

		if file.endswith(exts):
			if file[:-8] != processed_ext: # dont reprocess processed bags
				infile = args.dir+file
				outfile = infile[:-4] + processed_ext
				print(infile)

				#with rosbag.Bag(outfile, 'w') as outbag:
				for topic, msg, t in rosbag.Bag(infile).read_messages():
					T.append(t.to_sec())

					if topic == '/estimator/state':
						T_est.append(t.to_sec())
						theta = msg.theta.data
						#outbag.write(topic, msg, t)
						est_x.append(msg.pos.data[0])
						est_y.append(msg.pos.data[1])
						est_vx.append(msg.vel.data[0])
						est_vy.append(msg.vel.data[1])
						est_theta.append(msg.theta.data)

					if topic == '/sensor/decawave_measurement':
						T_decawave.append(t.to_sec())
						msg.x1.data = msg.x1.data+L1*np.sin(theta)
						msg.y1.data = msg.y1.data-L1*np.cos(theta)  # sensor on robot Y axis arm

						msg.x2.data = msg.x2.data-L2*np.cos(theta)
						msg.y2.data = msg.y2.data-L2*np.sin(theta)

						x1.append(msg.x1.data)
						y1.append(msg.y1.data)
						x2.append(msg.x2.data)
						y2.append(msg.y2.data)
						meas_theta.append(msg.theta.data)

						cov_det_p1.append(npl.det(np.asarray(msg.cov1.data).reshape(2,2)))
						cov_det_p2.append(npl.det(np.asarray(msg.cov2.data).reshape(2,2)))
						cov_theta.append(msg.cov_theta.data)
						print(msg.cov_theta.data)
						#outbag.write(topic, msg, t)

				fig, ax = plt.subplots(nrows=6, ncols=1)
				fig.set_figheight(50)
				fig.set_figwidth(30)
				titles = ['x','y','Velocity','theta','Measured covariance determinants P1, P2', 'Covariance theta']
				for i, row in enumerate(ax):
					if i == 0:
						l1, = row.plot(T_est, est_x, display, color=est_col)
						l2, = row.plot(T_decawave, x1, display, color=m_col1)
						l3, = row.plot(T_decawave, x2, display,color=m_col2)
						row.legend((l1,l2,l3), ('est x', 'meas x1', 'meas x2'), loc='upper right', shadow=True)
						row.set_xlabel('Time (s)')
						row.set_ylabel('x (m)')
					elif i == 1:
						l1, = row.plot(T_est, est_y, display, color=est_col)
						l2, = row.plot(T_decawave, y1, display, color=m_col1)
						l3, = row.plot(T_decawave, y2, display, color=m_col2)
						row.legend((l1,l2,l3), ('est y', 'meas y1', 'meas y2'), loc='upper right', shadow=True)
						row.set_xlabel('Time (s)')
						row.set_ylabel('y (m)')
					elif i == 2:
						l1, = row.plot(T_est, est_vx, display, color=est_col)
						l2, = row.plot(T_est, est_vy, display, color=m_col1)
						row.legend((l1,l2), ('est Vx', 'est Vy'), loc='upper right', shadow=True)
						row.set_xlabel('Time (s)')
						row.set_ylabel('Velocity (m/s)')
					elif i == 3:
						l1, = row.plot(T_est, est_theta, display, color=est_col)
						l2, = row.plot(T_decawave, meas_theta, display, color=m_col1)
						row.legend((l1,l2), ('est theta', 'meas theta'), loc='upper right', shadow=True)
						row.set_xlabel('Time (s)')
						row.set_ylabel('Theta (rads)')
					elif i == 4:
						l1, = row.plot(T_decawave, cov_det_p1 , display, color=m_col1)
						l2, = row.plot(T_decawave, cov_det_p1, display, color=m_col2)
						row.legend((l1,l2), ('cov det p1', 'cov det p2'), loc='upper right', shadow=True)
						row.set_xlabel('Time (s)')
						row.set_ylabel('Magnitude')
					elif i == 5:
						l1, = row.plot(T_decawave, cov_theta , display, color=m_col1)
						row.legend( 'cov theta', loc='upper right', shadow=True)
						row.set_xlabel('Time (s)')
						row.set_ylabel('Magnitude')
					row.grid()

					row.set_title(titles[i])
					ax[i].yaxis.set_major_locator(plt.MaxNLocator(30))

				plt.savefig(outfile[:-4]+'.png')




