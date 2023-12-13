#!/usr/bin/env python

import rospy
import numpy as np
import math
from sensor_msgs.msg import LaserScan
from ackermann_msgs.msg import AckermannDrive
from std_msgs.msg import Int64
from visualization_msgs.msg import Marker
from collections import deque
import time

# Some useful variable declarations.
angle_range = 240	# Hokuyo 4LX has 240 degrees FoV for scan

follow_the_gap_pub = rospy.Publisher('ftg', AckermannDrive, queue_size=1)
sphere_marker_pub = rospy.Publisher('/sphere_marker', Marker, queue_size=2)

goals = deque(maxlen=40)
goals.append(363)

heading_index = 0
prev_error = 0
kp = 90.0 #80
kd = 80.0 #120
ki = 0
global last_goal
last_goal = -1

def get_heading_index(data):
	global heading_index
	heading_index = data.data

def find_gap(rangs):
	global heading_index

	start = max(0, (heading_index-100))
	end = min(heading_index+100, len(rangs))
	ranges = np.array(rangs)[start:end]
	#diffs = np.diff(ranges)
	#diffs = np.absolute(diffs)
	# deepest = 0
	# deepest_ind = -1
	# # end_ind = -1
	# for i in range(1,len(ranges)):
	# 	if math.isnan(ranges[i]):
	# 		continue
	# 	if (ranges[i] > deepest) and (not math.isinf(ranges[i])) and (not math.isnan(ranges[i])):  # and data[i] < 4:
	# 		deepest = ranges[i]
	# 		deepest_ind = i
	# return (deepest_ind + 100)# + end_ind) //2

	return np.nanargmax(ranges) + start
    ### Start of new implementation
    # rangs = np.where(np.isnan(rangs),5,rangs)
    # shoulder_threshold = 2
    # shoulder_indices = np.where(rangs > shoulder_threshold)[0].tolist()
    # # print("Shoulder indicies: ", shoulder_indices)
    # maxAvg = 0
    # startI = -1
    # for i in range(1, len(shoulder_indices)):
    #     start = shoulder_indices[i-1]
    #     end = shoulder_indices[i]
    #     if (len(rangs[start:end]) > 0):
    #         avg = np.nanmean(rangs[start:end])#*(end-start)
    #         if (avg > maxAvg): #+140
    #             maxAvg = avg
    #             startI = i-1
    
    # mid_index = (shoulder_indices[startI] + shoulder_indices[startI+1])/2
    # # print("MID_INDEX: ", mid_index)
    # return mid_index


def callback(data):
    start_time = time.time()
    ranges = data.ranges
    ranges_ind = find_gap(ranges)
    # global last_goal
    # if last_goal != -1 and abs(last_goal - ranges_ind) > 40:
    #     ranges_ind = (last_goal + ranges_ind) //2
    # last_goal = ranges_ind
    # print(ranges_ind)

    global prev_error
    global goals 
    global heading_index
    goals_arr = np.array(goals)
    valid_goals = goals_arr[goals_arr != -1]
    diffs_from_pp_heading = np.absolute(valid_goals - heading_index)
    average_goal = valid_goals[np.argmin(diffs_from_pp_heading)]
    # print(valid_goals)
    # average_goal = np.median(valid_goals)
    if (math.isnan(ranges[ranges_ind])):
        goals.append(-1)
    else:
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
    MAX_SPEED = 38
    # command.speed = 5 * MAX_SPEED / (1 + .14 * abs(angle))
    command.speed = 5 * MAX_SPEED / (1 + .08 * abs(angle))
    if command.speed > MAX_SPEED:
        command.speed = MAX_SPEED
    # print(command.speed)
    follow_the_gap_pub.publish(command)

    prev_error = error
    # print("ftg takes: ", time.time() - start_time)

if __name__ == '__main__':
	rospy.init_node('follow_the_gap',anonymous = True)
	rospy.Subscriber("extended_ranges",LaserScan,callback)
	rospy.Subscriber("heading_index", Int64, get_heading_index)
	rospy.spin()
