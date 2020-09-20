
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

def extractData(infile):
	for topic, msg, t in rosbag.Bag(infile).read_messages():
		if topic == '/sensor/decawave_measurement':
			T.append(t.to_sec())
			x1.append(msg.x1.data)
			y1.append(msg.y1.data)
			x2.append(msg.x2.data)
			y2.append(msg.y2.data)
			meas_theta.append(msg.theta.data)

for root, dirs, files in os.walk(args.dir): # dir of bag files
	print("Processing ... ")
	for file in files:
		if file.endswith(exts) and file[:-len(processed_ext)] != processed_ext: # dont reprocess processed bags
			infile = args.dir+file
			outfile = infile[:-4] + processed_ext
			print(infile)
			extractData(infile)

if args.dir[-4:] == exts: # file
	infile = args.dir
	print(infile)
	extractData(infile)


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
fig.set_figheight(8)
fig.set_figwidth(20)

plt.plot(f, x1_fft[:,0], color='b')
plt.legend('x1', loc='upper right', shadow=True)
plt.ylabel('Amplitude')
plt.xlabel('Frequency [Hz]')
plt.show()


fig, ax = plt.subplots(nrows=1, ncols=1)
fig.set_figheight(8)
fig.set_figwidth(10)


number_of_bins = 10
plt.hist(x1, density=False, bins=number_of_bins)
plt.ylabel('Counts')
plt.xlabel('Data');
plt.show()

import matplotlib.pyplot as plt
import scipy
import pandas as pd
from sklearn.preprocessing import StandardScaler

print(scipy.stats.shapiro(x1))


sc=StandardScaler()
dd = x1.reshape (-1,1)
sc.fit(dd)
data =sc.transform(dd)
data = data.flatten()

print(pd.DataFrame(x1,columns=['Data']).describe())

dist_names = ['norm','cauchy','chi','gamma','lognorm','beta','pearson3','uniform','triang']
p_values = []
chi_square = []

percentile_bins = np.linspace(0,100,51)
percentile_cutoffs = np.percentile(data, percentile_bins)
observed_frequency, bins = (np.histogram(data, bins=percentile_bins))
cum_observed_frequency = np.cumsum(observed_frequency)


for distribution in dist_names:
    # Set up distribution and get fitted distribution parameters
    dist = getattr(scipy.stats, distribution)
    param = dist.fit(data)

    # Obtain the KS test P statistic, round it to 5 decimal places
    p = scipy.stats.kstest(data, distribution, args=param)[1]
    #p = np.around(p, 5)
    p_values.append(p)

     # This is based on a 'cumulative distrubution function' (cdf)
    cdf_fitted = dist.cdf(percentile_cutoffs, *param[:-2], loc=param[-2],
                          scale=param[-1])
    expected_frequency = []
    for bin in range(len(percentile_bins)-1):
        expected_cdf_area = cdf_fitted[bin+1] - cdf_fitted[bin]
        expected_frequency.append(expected_cdf_area)

    # calculate chi-squared
    expected_frequency = np.array(expected_frequency) * len(data)
    cum_expected_frequency = np.cumsum(expected_frequency)
    ss = sum (((cum_expected_frequency - cum_observed_frequency) ** 2) / cum_observed_frequency)
    chi_square.append(ss)



# Collate results and sort by goodness of fit (best at top)

results = pd.DataFrame()
results['Distribution'] = dist_names
results['chi_square'] = chi_square
results['p_value'] = p_values
results.sort_values(['chi_square'], inplace=True)

# Report results

print ('\nDistributions sorted by goodness of fit:')
print ('----------------------------------------')
print (results)


bin_cutoffs = np.linspace(np.percentile(x1,0), np.percentile(x1,99),number_of_bins)

# Create the plot
h = plt.hist(x1, bins = bin_cutoffs, color='0.75')

parameters = []
dist_name = 'norm'
dist = getattr(scipy.stats, dist_name)
param = dist.fit(x1)
parameters.append(param)

# Get line for each distribution (and scale to match observed data)
pdf_fitted = dist.pdf(np.arange(len(x1)), *param[:-2], loc=param[-2], scale=param[-1])
scale_pdf = np.trapz (h[0], h[1][:-1]) / np.trapz (pdf_fitted, np.arange(len(x1)))
pdf_fitted *= scale_pdf

# Add the line to the plot
plt.plot(pdf_fitted, label=dist_name)

# Set the plot x axis to contain 99% of the data
# This can be removed, but sometimes outlier data makes the plot less clear
plt.xlim(0,np.percentile(x1,99))

# Add legend and display plot

plt.legend()
plt.show()
