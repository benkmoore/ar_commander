
import rospy
import rosbag
import argparse
import numpy as np
import scipy as sp
import os
import matplotlib.pyplot as plt

from scipy import fft, signal

parser = argparse.ArgumentParser()
parser.add_argument('--dir')
args = parser.parse_args()

# bag extension names
exts = ('.bag')
processed_ext = '_OUT.bag'

x1 = []
x2 = []
y1 = []
y2 = []
meas_theta = []
T = []

for root, dirs, files in os.walk(args.dir):
	print("Processing ... ")
	for file in files:
		if file.endswith(exts) and file[:-len(processed_ext)] != processed_ext: # dont reprocess processed bags
			infile = args.dir+file
			outfile = infile[:-4] + processed_ext
			print(infile)

			for topic, msg, t in rosbag.Bag(infile).read_messages():
				if topic == '/sensor/decawave_measurement':
					T.append(t.to_sec())
					x1.append(msg.x1.data)
					y1.append(msg.y1.data)
					x2.append(msg.x2.data)
					y2.append(msg.y2.data)
					meas_theta.append(msg.theta.data)


x1 = np.asarray(x1)
y1 = np.asarray(y1)
x2 = np.asarray(x2)
y2 = np.asarray(y2)
meas_theta = np.asarray(meas_theta)
T = np.asarray(T)

dt = np.mean(np.diff(T))
print(dt)

#x1_fft = np.abs(fft(x1))
f, t, x1_fft = signal.spectrogram(x1, fs=10)


fig, ax = plt.subplots(nrows=1, ncols=1)
fig.set_figheight(50)
fig.set_figwidth(30)

plt.plot(f, x1_fft[:,0], color='b')
plt.legend('x1', loc='upper right', shadow=True)
plt.ylabel('Amplitude')
plt.xlabel('Frequency [Hz]')
plt.show()

