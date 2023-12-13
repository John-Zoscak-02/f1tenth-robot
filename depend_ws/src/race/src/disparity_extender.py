#!/usr/bin/env python

import rospy
import numpy as np
import math
from sensor_msgs.msg import LaserScan
from race.msg import pid_input
import copy
import threading

# Handle to the publisher that will publish on the extended_ranges topic. messsages of type 'LaserScan'
pub = rospy.Publisher('/extended_ranges', LaserScan, queue_size=1)

gap_threshold = .4 # Threshold for determining if there is a disparity / gap between two ranges
car_half_width = .15 # The half width of the car for extending th esmall error 
extender_padding = 0.2 # How far beyond car_half_width the extender will project ranges. 

callback_lock = threading.Lock()

def computeExtendedRanges(data):
	# print('len(data.ranges) original: {}'.format(len(data.ranges)))
	# data: single message from topic /scan
	# angle: between -30 to 210 degrees, where 0 degrees is directly to the right, and 90 degrees is directly in front
	# Outputs length in meters to object with angle in lidar scan field of view
	# Make sure to take care of NaNs etc.

	with callback_lock:
		ranges = np.array(data.ranges)
		extended_ranges = np.array(ranges)

		for i in range(1, len(ranges)-1):
			# For dealing with nan values -- we may need to tweak this
			if math.isnan(ranges[i]):
				# ranges[i] = 0 #ranges[i-1]
				if (not math.isnan(ranges[i+1])):
					ranges[i] = 10
				elif not math.isnan(ranges[i-1]):
					ranges[i] = 10
				else:
					ranges[i] = 0

		disparities = np.diff(ranges)
		
		for i in range(len(disparities)):
			# For dealing with nan values -- we may need to tweak this
			if math.isnan(ranges[i]):
				# extended_ranges[i] = 0 #ranges[i-1]
				# if (not math.isnan(ranges[i+1])):
				# 	extended_ranges[i] = ranges[i+1]
				# elif not math.isnan(ranges[i-1]):
				# 	extended_ranges[i] = ranges[i-1]
				# else:
				extended_ranges[i] = 0
				continue
			# If there is a gap defined after the right
			if disparities[i] > gap_threshold:
				# extension_length = min(100, (car_half_width + extender_padding) / 
				# 					(ranges[i]*math.sin(data.angle_increment)))
				extension_length = min(50, math.atan((car_half_width + extender_padding)/ (ranges[i])) / data.angle_increment)
				# extension_length = math.atan2((car_half_width + extender_padding), (ranges[i])) / data.angle_increment
				extension_length = min(len(ranges), extension_length+i)
				#print("{}: ext len".format(extension_length-i))
				for j in range(i,int(extension_length)):
					if extended_ranges[j] >= ranges[i]:
						# extended_ranges[j] = ranges[i]*math.cos(data.angle_increment) + ((j-i)*.001)
						extended_ranges[j] = ranges[i]*math.cos(data.angle_increment)# + ((j-extension_length+1)*.004)
			# If there is a gap defined after the left
			elif disparities[i] < -gap_threshold:
				# extension_length = min(100, (car_half_width + extender_padding) / 
				# 					(ranges[i+1]*math.sin(data.angle_increment)))
				extension_length = min(50, math.atan((car_half_width + extender_padding)/ (ranges[i+1])) / data.angle_increment)
				# extension_length = math.atan2((car_half_width + extender_padding), (ranges[i+1])) / data.angle_increment
				extension_length = max(0, i+1-extension_length)
				#print("{}: ext len".format(i+1-extension_length))
				for j in range(int(extension_length),i+1):
					if extended_ranges[j] >= ranges[i+1]:
						# extended_ranges[j] = ranges[i+1]*math.cos(data.angle_increment) + ((i+1-j)*.001)
						extended_ranges[j] = ranges[i+1]*math.cos(data.angle_increment)# + ((extension_length-j+1)*.004)
		# print("ranges: {}".format(ranges[90:-90]))
		# print("extended: {}".format(extended_ranges[90:-90]))	
		# padding = np.array([0]*90)
		# return np.append(padding, np.append(extended_ranges[90:-90], padding))
		# return extended_ranges[90:-90]
		# print("extended_ranges: ", np.amin(extended_ranges), np.amax(extended_ranges), np.sum(extended_ranges==np.inf), np.sum(extended_ranges==np.nan))
		# print(extended_ranges)
		return extended_ranges

def callback(data):	
	scan = data
	scan.ranges = computeExtendedRanges(data)
	scan.header.frame_id = "car_3_laser"

	pub.publish(scan)


if __name__ == '__main__':
	print("Hokuyo LIDAR node started")
	rospy.init_node('disparity_extender',anonymous = True)
	rospy.Subscriber("/car_3/scan",LaserScan,callback)
	rospy.spin()
