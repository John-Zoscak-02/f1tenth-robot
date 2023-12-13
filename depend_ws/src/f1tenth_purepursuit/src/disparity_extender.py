#!/usr/bin/env python

import rospy
import numpy as np
import math
from sensor_msgs.msg import LaserScan
from race.msg import pid_input
import copy
import threading
import time
# Handle to the publisher that will publish on the extended_ranges topic. messsages of type 'LaserScan'
extended_ranges_pub = rospy.Publisher('extended_ranges', LaserScan, queue_size=1)

gap_threshold = .11 # Threshold for determining if there is a disparity / gap between two ranges
car_half_width = 0.15 # The half width of the car for extending th esmall error 
extender_padding = 0.2 #0.2 # How far beyond car_half_width the extender will project ranges.

def computeExtendedRanges(data):
    start_time = time.time()
    # with callback_lock:
    ranges = np.array(data.ranges)
    extended_ranges = np.array(data.ranges)

    # print("ranges: ", ranges)

    for i in range(1, len(ranges)-1):
        # For dealing with nan values -- we may need to tweak this
        if math.isnan(ranges[i]) or not (data.range_min <= ranges[i] <= data.range_max):
            if (not math.isnan(data.ranges[i+1]) and ((data.range_min < data.ranges[i+1] < data.range_max))):
                # ranges[i] = data.ranges[i+1]
                ranges[i] = 10
                # extended_ranges[i] = data.ranges[i+1]
            elif not math.isnan(data.ranges[i-1]) and ((data.range_min < data.ranges[i-1] < data.range_max)):
                # ranges[i] = data.ranges[i-1]
                ranges[i] = 10
                # extended_ranges[i] = data.ranges[i-1]
            # elif ((not math.isnan(data.ranges[i-1])) or (not math.isnan(data.ranges[i+1]))):
            #     ranges[i] = 10

    # print("ranges2: ", ranges)

    disparities = np.diff(ranges)

    for i in range(len(disparities)):
        # For dealing with nan values -- we may need to tweak this
        # if math.isnan(ranges[i]):
            # continue
        # If there is a gap defined after the right
        if disparities[i] > gap_threshold:
            r = min(data.ranges[i], data.ranges[i+1])
            if not (data.range_min <= r <= data.range_max):
                # extended_ranges[i] = np.NAN
                continue
            # print("atan: ", math.atan((car_half_width + extender_padding) / (r)))
            # print("angles to overwrite: ", math.atan((car_half_width + extender_padding) / (r)) / data.angle_increment, (r))
            # extension_length_s = int(math.atan((car_half_width) / (r)) / data.angle_increment) + 1
            extension_length = int(math.atan((car_half_width + extender_padding) / (r)) / data.angle_increment) + 1
            if i+extension_length > len(ranges):
                extension_length = len(ranges)-i-1
            extended_ranges[i: i+extension_length] = np.where(ranges[i: i+extension_length] < r, ranges[i: i+extension_length], np.linspace(r+.0001, r+.001, num=extension_length))

                

        # If there is a gap defined after the left
        elif disparities[i] < -gap_threshold:
            r = min(data.ranges[i], data.ranges[i+1])
            if not (data.range_min <= r <= data.range_max):
                # extended_ranges[i] = np.NAN
                continue
            # print("atan: ", math.atan((car_half_width + extender_padding) / (r)))
            # print("angles to overwrite: ", math.atan((car_half_width + extender_padding) / (r)) / data.angle_increment, (r))
            # extension_length_s = int(math.atan((car_half_width) / (r)) / data.angle_increment) + 1
            extension_length = int(math.atan((car_half_width + extender_padding) / (r)) / data.angle_increment) + 1
            # extension_length = max(0, i+1-extension_length)
            if i-extension_length < 0:
                extension_length = i
            extended_ranges[i-extension_length+1: i+1] = np.where(ranges[i-extension_length+1: i+1] < r, ranges[i-extension_length+1: i+1], np.linspace(r+.0001, r+.001, num=extension_length))
    
    # print("extend ranges took: ", time.time() - start_time)
    # extended_ranges = data.ranges
    return extended_ranges


def callback(data):
	scan = data
	scan.ranges = computeExtendedRanges(data)
	scan.header.frame_id = "car_3_laser"

	extended_ranges_pub.publish(scan)


if __name__ == '__main__':
	print("Hokuyo LIDAR node started")
	rospy.init_node('disparity_extender',anonymous = True)
	rospy.Subscriber("/car_3/scan",LaserScan,callback)
	rospy.spin()
