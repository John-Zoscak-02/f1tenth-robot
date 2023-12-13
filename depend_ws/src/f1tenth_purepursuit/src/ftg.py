#!/usr/bin/env python

import rospy
import numpy as np
import math
from sensor_msgs.msg import LaserScan
from ackermann_msgs.msg import AckermannDrive
from visualization_msgs.msg import Marker
from collections import deque

# Some useful variable declarations.
angle_range = 240	# Hokuyo 4LX has 240 degrees FoV for scan

pub = rospy.Publisher('/car_3/offboard/command', AckermannDrive, queue_size=10)
sphere_marker_pub = rospy.Publisher('/sphere_marker', Marker, queue_size=2)
extended_ranges_pub = rospy.Publisher('extended_ranges', LaserScan, queue_size=1)

goals = deque(maxlen=5)
goals.append(360)
gap_threshold = .15 # Threshold for determining if there is a disparity / gap between two ranges
car_half_width = .35 # The half width of the car for extending th esmall error 
extender_padding = 0.15 #0.2 # How far beyond car_half_width the extender will project ranges.

prev_error = 0
kp = 80.0
kd = 120.0
ki = 0
global last_goal
last_goal = -1

def computeExtendedRanges(data):
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
                extended_ranges[i] = data.ranges[i+1]
            elif not math.isnan(data.ranges[i-1]) and ((data.range_min < data.ranges[i-1] < data.range_max)):
                # ranges[i] = data.ranges[i-1]
                ranges[i] = 10
                extended_ranges[i] = data.ranges[i-1]
            # elif ((not math.isnan(data.ranges[i-1])) or (not math.isnan(data.ranges[i+1]))):
            #     ranges[i] = 10

    print("ranges2: ", ranges)

    disparities = np.diff(ranges)

    for i in range(len(disparities)):
        # For dealing with nan values -- we may need to tweak this
        # if math.isnan(ranges[i]):
            # continue
        # If there is a gap defined after the right
        if disparities[i] > gap_threshold:
            r = ranges[i]
            if not (data.range_min <= r <= data.range_max):
                extended_ranges[i] = np.NAN
                continue
            # print("atan: ", math.atan((car_half_width + extender_padding) / (r)))
            # print("angles to overwrite: ", math.atan((car_half_width + extender_padding) / (r)) / data.angle_increment, (r))
            extension_length = int(math.atan((car_half_width + extender_padding) / (r)) / data.angle_increment) + 1
            if i+extension_length > len(ranges):
                extension_length = len(ranges)-i-1
            if np.nanmin(ranges[i: i+extension_length]) >= r:
                extended_ranges[i: i+extension_length] = np.linspace(r+.0001, r+.001, num=extension_length)

        # If there is a gap defined after the left
        elif disparities[i] < -gap_threshold:
            r = ranges[i+1]
            if not (data.range_min <= r <= data.range_max):
                extended_ranges[i] = np.NAN
                continue
            # print("atan: ", math.atan((car_half_width + extender_padding) / (r)))
            # print("angles to overwrite: ", math.atan((car_half_width + extender_padding) / (r)) / data.angle_increment, (r))
            extension_length = int(math.atan((car_half_width + extender_padding) / (r)) / data.angle_increment) + 1
            # extension_length = max(0, i+1-extension_length)
            if i-extension_length < 0:
                extension_length = i
            if np.nanmin(ranges[i-extension_length+1: i+1]) >= r:
                extended_ranges[i-extension_length+1: i+1] = np.linspace(r+.001, r+.0001, num=extension_length)
    # extended_ranges = data.ranges
    msg = data
    msg.ranges = extended_ranges
    extended_ranges_pub.publish(msg)
    return extended_ranges

def find_gap(data):
	ranges = np.array(data)[60:-60]
	#diffs = np.diff(ranges)
	#diffs = np.absolute(diffs)
	deepest = 0
	deepest_ind = -1
	# end_ind = -1
	for i in range(1,len(ranges)):
		if math.isnan(ranges[i]):
			continue
		if (ranges[i] > deepest) and (not math.isinf(ranges[i])) and (not math.isnan(ranges[i])):  # and data[i] < 4:
			deepest = ranges[i]
			deepest_ind = i
	return (deepest_ind + 60)# + end_ind) //2

def callback(data):
    ranges = computeExtendedRanges(data)
    ranges_ind = find_gap(ranges)
    # global last_goal
    # if last_goal != -1 and abs(last_goal - ranges_ind) > 40:
    #     ranges_ind = (last_goal + ranges_ind) //2
    # last_goal = ranges_ind
    # print(ranges_ind)
    global prev_error
    global goals 
    goals_arr = np.array(goals)
    valid_goals = goals_arr[goals_arr != -1]
    average_goal = np.average(valid_goals)
    goals.append(ranges_ind)

    goal = ranges[ranges_ind]
    goal = (math.sin(data.angle_min + (data.angle_increment * ranges_ind))*goal, math.cos(data.angle_min + (data.angle_increment * ranges_ind))*goal)
    sphere_marker = Marker()
    sphere_marker.header.frame_id = 'car_3_laser'
    sphere_marker.type = 2
    sphere_marker.id = 0
    sphere_marker.scale.x = 0.2
    sphere_marker.scale.y = 0.2
    sphere_marker.scale.z = 0.2
    sphere_marker.color.r = 0.0
    sphere_marker.color.r = 0.0
    sphere_marker.color.g = 1.0
    sphere_marker.color.b = 0.0
    sphere_marker.color.a = 1.0
    sphere_marker.pose.position.x = goal[1]
    sphere_marker.pose.position.y = goal[0]
    sphere_marker.pose.position.z = 0.0
    sphere_marker.pose.orientation.x = 0.0
    sphere_marker.pose.orientation.y = 0.0
    sphere_marker.pose.orientation.z = 0.0
    sphere_marker.pose.orientation.w = 0.0
    sphere_marker_pub.publish(sphere_marker)

    middle_index = len(ranges) // 2
    gap_offset = ranges_ind - middle_index
    error = gap_offset * data.angle_increment 
    angle = (kp * error) + (-kd * (prev_error - error))
    command = AckermannDrive()
    command.steering_angle = angle
    MAX_SPEED = 30
    command.speed = 5 * MAX_SPEED / (1 + .1 * abs(angle))
    if command.speed > MAX_SPEED:
        command.speed = MAX_SPEED
    print(command.speed)
    pub.publish(command)

    prev_error = error

if __name__ == '__main__':
	rospy.init_node('follow_the_gap',anonymous = True)
	rospy.Subscriber("/car_3/scan",LaserScan,callback)
	rospy.spin()
