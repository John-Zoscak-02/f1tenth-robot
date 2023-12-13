#!/usr/bin/env python

import rospy
import numpy as np
import math
from sensor_msgs.msg import LaserScan
from race.msg import pid_input

# Some useful variable declarations.
gap_threshold = .2
vel = 15

# Handle to the publisher that will publish on the error topic, messages of the type 'pid_input'
pub = rospy.Publisher('error', pid_input, queue_size=10)


def find_widest_heading(data):
	ranges = np.array(data.ranges)
	disparities = np.diff(ranges)
	
	last_gap_index = 0
	max_width_gap_index = 0
	# max_width_gap_length = 0
	max_avg = 0
	gap_idx = 0
	gap_depth_threshold = 0.5
	deepest = 0
	deepest_ind = 0
	deepest_gap = 0
	deepest_gap_idx = 0
	for i in range(len(disparities)) :
		#if disparities[i] > deepest_gap:
		#	deepest_gap = disparities[i]
		#	deepest_gap_idx = i
		if ranges[i] > deepest and not math.isinf(ranges[i]): # and data[i] < 4:
			deepest = ranges[i]
			deepest_ind = i
		if abs(disparities[i]) > gap_threshold and ranges[i] > gap_depth_threshold:
			avg = np.average(ranges[last_gap_index:i])
			if avg > max_avg and avg > gap_depth_threshold:
				max_avg = avg
				gap_idx = (last_gap_index + i) // 2
			# last_gap_index = i
			# if i-last_gap_index >= max_width_gap_length:
			# 	max_width_gap_index = i
			# 	max_width_gap_length = i-last_gap_index

			last_gap_index = i



	# max_idx = 0 
	# max_avg = 0
	# for i in ranges(len(gaps)):
	# 	avg = np.average(gaps[i])
	# 	if avg > max_avg:
	# 		max_idx = i
	# 		max_avg = avg

	# midpoint_index = ((2*max_width_gap_index) + max_width_gap_length ) / 2
	# heading = (midpoint_index*data.angle_increment) - (math.pi/3) 
	# return heading
	return gap_idx if gap_idx != 0 else deepest_ind


def callback(data):
	msg = pid_input()	# An empty msg is created of the type pid_input
	# this is the error that you want to send to the PID for steering correction.

	# data.ranges = data.ranges[90:-90]
	# lidar_heading = find_widest_heading(data)
	# # lidar_range = data.range_max - data.range_min
	# # relative_heading = (lidar_heading - (math.pi/2))

	# msg.pid_error = relative_heading
	# msg.pid_vel = vel		# velocity error can also be sent.
	# pub.publish(msg)

	middle_index = len(data.ranges) // 2
	gap_ind = find_widest_heading(data)
	# print("{}: data.ranges".format(data.ranges))
	# print("{}: ranges len".format(len(data.ranges)))

	gap_offset = gap_ind - middle_index
	# print("{}: gap_offset".format(gap_offset))
	# this is not working currently
	# want to vary the error depending on how far away the object at the middle index is in comparison to the deepest gap
	# need to change how it is implemented
	# print('distance_ahead: {}'.format(ahead_distance))
	error = gap_offset * data.angle_increment 
	print('error: {}'.format(error))
	msg.pid_error = error
	msg.pid_vel = vel
	pub.publish(msg)


if __name__ == '__main__':
	print("Hokuyo LIDAR node started")
	rospy.init_node('widest_heading_finder',anonymous = True)
	# TODO: Make sure you are subscribing to the correct car_x/scan topic on your racecar
	rospy.Subscriber("extended_ranges",LaserScan,callback)
	rospy.spin()
