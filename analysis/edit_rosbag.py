import rosbag
import rospy
import numpy as np
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

# decawave sensor arm lengths v1
L1 = 0.41
L2 = 0.425


for root, dirs, files in os.walk(args.dir):
	print("Processing ... ")
	for file in files:
		est_x = []
		est_y = []
		x1 = []
		x2 = []
		y1 = []
		y2 = []
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
						#outbag.write(topic, msg, t)

				fig, ax = plt.subplots(nrows=3, ncols=1)
				fig.set_figheight(15)
				fig.set_figwidth(20)
				titles = ['x','y','theta']
				for i, row in enumerate(ax):
					if i == 0:
						l1, = row.plot(T_est, est_x, color=est_col)
						l2, = row.plot(T_decawave, x1, color=m_col1)
						l3, = row.plot(T_decawave, x2, color=m_col2)
						row.legend((l1,l2,l3), ('est x', 'meas x1', 'meas x2'), loc='upper right', shadow=True)
						row.set_xlabel('Time (s)')
						row.set_ylabel('x (m)')
					elif i == 1:
						l1, = row.plot(T_est, est_y, color=est_col)
						l2, = row.plot(T_decawave, y1, color=m_col1)
						l3, = row.plot(T_decawave, y2, color=m_col2)
						row.legend((l1,l2,l3), ('est y', 'meas y1', 'meas y2'), loc='upper right', shadow=True)
						row.set_xlabel('Time (s)')
						row.set_ylabel('y (m)')
					elif i == 2:
						l1, = row.plot(T_est, est_theta, color=est_col)
						l2, = row.plot(T_decawave, meas_theta, color=m_col1)
						row.legend((l1,l2), ('est theta', 'meas theta'), loc='upper right', shadow=True)
						row.set_xlabel('Time (s)')
						row.set_ylabel('Theta (rads)')
					row.grid()
					row.set_title(titles[i])

				plt.savefig(outfile[:-4]+'.png')




