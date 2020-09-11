# filter estiamtor msg from rosbag

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
processed_ext = '_FILTERED.bag'

for root, dirs, files in os.walk(args.dir):
    print("Processing ... ")
    for file in files:
        if file.endswith(exts) and file[-len(processed_ext):] != processed_ext: # dont reprocess processed bags
            infile = args.dir+file
            outfile = infile[:-4] + processed_ext
            print(infile)

            with rosbag.Bag(outfile, 'w') as outbag:
                for topic, msg, t in rosbag.Bag(infile).read_messages():
                    if topic == "/sensor/decawave_measurement":
                        outbag.write(topic, msg, t)
