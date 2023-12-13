#!/usr/bin/env python

# Import necessary libraries
import rospy
import os
import sys
import csv
import math
from std_msgs.msg import Int64
from ackermann_msgs.msg import AckermannDrive
from geometry_msgs.msg import PolygonStamped
from geometry_msgs.msg import Point32
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import LaserScan
from sensor_msgs.msg import PointCloud
from visualization_msgs.msg import Marker
import numpy as np
import tf
import time

# Global variables for storing the path, path resolution, frame ID, and car details
plan                = []
path_resolution     = []
frame_id            = 'map'
car_name            = 'car_3'
# trajectory_name     = 'raceline_2' # str(sys.argv[1])
# trajectory_name     = 'comp_1' # str(sys.argv[1])
# trajectory_name     = str(sys.argv[1])
trajectory_name      = str(rospy.get_param('/{}/f1tenth_purepursuit/raceline'.format(car_name), 'comp_7_smooth'))
last_bpi = 0
plan_cloud          = PointCloud()

# Publishers for sending driving commands and visualizing the control polygon
command_pub         = rospy.Publisher('/{}/offboard/command'.format(car_name), AckermannDrive, queue_size = 1)
polygon_pub         = rospy.Publisher('/{}/purepursuit_control/visualize'.format(car_name), PolygonStamped, queue_size = 1)
# plan_cloud_pub      = rospy.Publisher('plan_cloud', PointCloud, queue_size=1)
heading_index_pub   = rospy.Publisher('heading_index', Int64, queue_size=1)
# heading_marker_pub = rospy.Publisher('heading_sphere', Marker, queue_size=2)

# TODO: Set the distance between points in LIDAR scan and path which is picked up as an obstacle on the path...
INTERSECTION_THRESHOLD = 0.07

FTG_TAKEOVER = 4

# Global variables for waypoint sequence and current polygon
global wp_seq
global curr_polygon
global kd
kd = 0.05

wp_seq          = 0
control_polygon = PolygonStamped()
# point_cloud = np.array([[]])
extended_ranges = np.array([])
range_min = 0
range_max = 0
angle_increment = 0
angle_min = 0
ftg_drive = AckermannDrive()

def get_ftg(data):
    global ftg_drive
    ftg_drive = data
    # print("getting ftg")

def get_extended_ranges(data):
    # global point_cloud
    
    # ranges = np.array(data.ranges)
    # print(data.range_min, data.range_max)
    # # print(data.range_min <= ranges and ranges <= data.range_max)
    # ranges = np.where(data.range_min <= ranges, ranges, np.NAN)
    # print(ranges)
    # # ranges = np.where(data.ranges <= data.range_max, ranges, np.NAN)
    # angles = data.angle_min + (data.angle_increment * np.arange(len(ranges)))
    # xc = np.sin(angles)
    # yc = np.cos(angles)

    # point_cloud = np.array((xc*ranges, yc*ranges)).T
    # print(point_cloud)

    global extended_ranges
    global angle_min
    global range_min
    global range_max
    global angle_increment
    ranges = np.array(data.ranges)
    extended_ranges = np.where(data.range_min <= ranges, ranges, np.NAN)
    extended_ranges = np.where(ranges <= data.range_max, ranges, np.NAN)
    # print(extended_ranges)
    angle_min = data.angle_min
    range_min = data.range_min
    range_max = data.range_max
    angle_increment = data.angle_increment
    # print("getting extended_ranges")


def construct_path():
    # Function to construct the path from a CSV file
    # TODO: Modify this path to match the folder where the csv file containing the path is located.
    global plan
    global plan_cloud

    print("TRAJECTORY NAME: ", trajectory_name)
    file_path = os.path.expanduser('~/depend_ws/src/f1tenth_purepursuit/path/{}.csv'.format(trajectory_name))
    with open(file_path) as csv_file:
        csv_reader = csv.reader(csv_file, delimiter = ',')
        for waypoint in csv_reader:
            plan.append(waypoint)

    # Convert string coordinates to floats and calculate path resolution
    for index in range(0, len(plan)):
        for point in range(0, len(plan[index])):
            plan[index][point] = float(plan[index][point])

    for index in range(1, len(plan)):
         dx = plan[index][0] - plan[index-1][0]
         dy = plan[index][1] - plan[index-1][1]
         path_resolution.append(math.sqrt(dx*dx + dy*dy))

    # pts = []
    # for p in plan:
    #     pts.append(Point32(x=p[0], y=p[1], z=p[2]))
    # plan_cloud.points = pts
    # plan_cloud.header.frame_id = 'map'

    # plan = np.array(plan)


def dist(a, b):
    dx = a[0] - b[0]
    dy = a[1] - b[1]
    return math.sqrt(dx*dx + dy*dy)

# Steering Range from -100.0 to 100.0
STEERING_RANGE = 100.0

# vehicle physical parameters
WHEELBASE_LEN       = 0.325

# previous speed
global obst_count
obst_count = 0
global prev_speed
prev_speed = 30
global last_angle
last_angle = 0

def purepursuit_control_node(data):
    start_time = time.time()
    MAX_SPEED = 50
    STEERING_SCALE = 6
    # Main control function for pure pursuit algorithm

    # global plan_cloud
    # plan_cloud_pub.publish(plan_cloud)

    # Create an empty ackermann drive message that we will populate later with the desired steering angle and speed.
    command = AckermannDrive()

    global wp_seq
    global curr_polygon
    global ftg_drive
    global obst_count

    # Obstacle detection vars
    global extended_ranges
    global range_min
    global range_max
    global angle_min
    global angle_increment

    # Obtain the current position of the race car from the inferred_pose message
    odom_x = data.pose.position.x
    odom_y = data.pose.position.y

    # The reference path is stored in the 'plan' array.
    # Your task is to find the base projection of the car on this reference path.
    # The base projection is defined as the closest point on the reference path to the car's current position.
    # Calculate the index and position of this base projection on the reference path.
    
    base_proj_ind = 0
    min_dist = last_dist = 6
    # global last_bpi
    for i in range(last_bpi, len(plan) + last_bpi):
        d = dist((odom_x,odom_y), plan[i%len(plan)])
        if d < min_dist:
            base_proj_ind = i%len(plan)
            min_dist = d
        # # TODO: might need to tune this value
        # elif d > last_dist and d - last_dist > .005:
        #     break
        # last_dist = d

    # print(plan[:, 0:1]**2)
    # xy = plan[:, [0,1]]
    # print(xy.shape)
    # print(point_cloud.shape)

    # Calculate the distance between the point_cloud and all points in the current path plan...
    # x2 = np.sum(xy**2, axis=1)  # shape of (m)
    # y2 = np.sum(point_cloud**2, axis=1)  # shape of (n)
    # # print(x2, y2)
    # m = np.matmul(xy, point_cloud.T)
    # x2 = x2.reshape(-1, 1)
    # dists = np.sqrt(x2 - 2 * m + y2)
    # print(dists)

    # argmin = np.unravel_index(np.nanargmin(dists), dists.shape)
    # if dists[argmin] < INTERSECTION_THRESHOLD:
    #     # Current plan intersects with point_cloud
    #     command_pub.publish(ftg_drive)
    #     print("OBSTACLE DETECTED")
    #     return

    pose_x = plan[base_proj_ind][0]
    pose_y = plan[base_proj_ind][1]
    # last_bpi = base_proj_ind
    # Calculate heading angle of the car (in radians)
    heading = tf.transformations.euler_from_quaternion((data.pose.orientation.x,
                                                        data.pose.orientation.y,
                                                        data.pose.orientation.z,
                                                        data.pose.orientation.w))[2]
    

    global prev_speed
    # TODO: Tune this value
    # lookahead_distance = 2.5
    lookahead_distance = 1.7 * (prev_speed+50)/(MAX_SPEED+50)
    # print(lookahead_distance)


    # Utilizing the base projection, your next task is to identify the goal or target point for the car.
    # This target point should be determined based on the path and the base projection you have already calculated.
    # The target point is a specific point on the reference path that the car should aim towards - lookahead distance ahead of the base projection on the reference path.
    # Calculate the position of this goal/target point along the path.

    target_point_ind = -1
    cumm_dist = 0
    for i in range(base_proj_ind, len(plan)-1 + base_proj_ind):
        if cumm_dist >= lookahead_distance:
            target_point_ind = i%len(plan)
            break
        cumm_dist += path_resolution[i%len(path_resolution)]

    # print(cumm_dist)


    # Implement the pure pursuit algorithm to compute the steering angle given the pose of the car, target point, and lookahead distance.

    if target_point_ind == -1:
        return
    target_point = plan[target_point_ind]
    target_x = target_point[0]
    target_y = target_point[1]
    yt = math.cos(heading)*(odom_y-target_point[1]) - math.sin(heading)*(odom_x-target_point[0])
    # print('yt: {}'.format(yt))
    sin_r = yt/lookahead_distance
    if sin_r > 1:
        sin_r = 1
    elif sin_r < -1:
        sin_r = -1
    alpha = math.asin(sin_r)
    wheel_angle_rad = -math.atan2((2*WHEELBASE_LEN*math.sin(alpha)), lookahead_distance)

    # Ensure that the calculated steering angle is within the STEERING_RANGE and assign it to command.steering_angle

    base_link    = Point32()
    nearest_pose = Point32()
    nearest_goal = Point32()
    base_link.x    = odom_x
    base_link.y    = odom_y
    nearest_pose.x = pose_x
    nearest_pose.y = pose_y
    nearest_goal.x = target_x
    nearest_goal.y = target_y
    control_polygon.header.frame_id = frame_id
    control_polygon.polygon.points  = [nearest_pose, base_link, nearest_goal]
    control_polygon.header.seq      = wp_seq
    control_polygon.header.stamp    = rospy.Time.now()
    wp_seq = wp_seq + 1
    polygon_pub.publish(control_polygon)

    # Find obstacles not-dumb way:
    # angles = extended_ranges.angle_min + (extended_ranges.angle_increment * np.arange(len(ranges)))
    from_center_index = -alpha/angle_increment

    # scan_pose = data
    # scan_heading = tf.transformations.quaternion_from_euler(0,0,alpha)
    # scan_pose.pose.orientation.x = scan_heading[0]
    # scan_pose.pose.orientation.y = scan_heading[1]
    # scan_pose.pose.orientation.z = scan_heading[2]
    # scan_pose.pose.orientation.w = scan_heading[3]
    # scan_orientation_pub.publish(scan_pose)

    dist_from_line = dist((pose_x, pose_y), (odom_x, odom_y))
    heading_index = int(len(extended_ranges)/2 + from_center_index)

    # goal = dist((odom_x, odom_y), target_point)
    # goal = (math.sin((angle_increment * from_center_index))*goal, math.cos((angle_increment * from_center_index))*goal)
    # sphere_marker = Marker()
    # sphere_marker.header.frame_id = 'car_3_laser'
    # sphere_marker.type = 2
    # sphere_marker.id = 0
    # sphere_marker.scale.x = 0.2
    # sphere_marker.scale.y = 0.2
    # sphere_marker.scale.z = 0.2
    # sphere_marker.color.r = 0.0
    # sphere_marker.color.r = 0.0
    # sphere_marker.color.g = 0.0
    # sphere_marker.color.b = 1.0
    # sphere_marker.color.a = 1.0
    # sphere_marker.pose.position.x = goal[1]
    # sphere_marker.pose.position.y = goal[0]
    # sphere_marker.pose.position.z = 0.0
    # sphere_marker.pose.orientation.x = 0.0
    # sphere_marker.pose.orientation.y = 0.0
    # sphere_marker.pose.orientation.z = 0.0
    # sphere_marker.pose.orientation.w = 0.0
    # heading_marker_pub.publish(sphere_marker)

    heading_index_pub.publish(Int64(heading_index))
    scan_at_heading = np.nanmean(extended_ranges[heading_index-2 : heading_index+2])
    # print(heading_index, 
    #         scan_at_heading, 
    #         dist((odom_x, odom_y), target_point), 
    #         dist_from_line,
    #         (odom_x, odom_y) )
    # print(np.isnan(extended_ranges[heading_index-2 : heading_index+2]))
    if ((abs(alpha) < (2*math.pi / 3)) and 
        (np.sum(np.isnan(extended_ranges[heading_index-2 : heading_index+2])) == 0) and 
        dist_from_line < 1.5):

        if scan_at_heading < dist((odom_x, odom_y), target_point) and scan_at_heading < 1:
            print("OBSTACLE DETECTED", time.time())
            obst_count+=1
            # print("pure_pursuit took: ", time.time() - start_time)
        else :
            obst_count = 0
    else :
        obst_count = 0

    if (obst_count > FTG_TAKEOVER):
        command_pub.publish(ftg_drive)
        print("FTG")
        return


    # might need some tuning
    wheel_angle = STEERING_SCALE * math.degrees(wheel_angle_rad)
    if wheel_angle > STEERING_RANGE:
        wheel_angle = STEERING_RANGE
    elif wheel_angle < -STEERING_RANGE:
        wheel_angle = -STEERING_RANGE
    command.steering_angle = wheel_angle

    # Implement Dynamic Velocity Scaling instead of a constant speed

    global last_angle
    global kd
    # TODO: tune this
    # speed = 10 * (MAX_SPEED/(1 + abs(wheel_angle))) + 5
    speed = (5 * MAX_SPEED /
            (1 + (.14 * abs(wheel_angle))) - (kd * abs(wheel_angle-last_angle)))
    if speed > MAX_SPEED:
        speed = MAX_SPEED
    
    # print(speed)

    command.speed = speed
    command_pub.publish(command)

    # Setting previous speed to speed
    prev_speed = speed
    last_angle = wheel_angle

    # Visualization code
    # Make sure the following variables are properly defined in your TODOs above:
    # - odom_x, odom_y: Current position of the car
    # - pose_x, pose_y: Position of the base projection on the reference path
    # - target_x, target_y: Position of the goal/target point


    # print("pure_pursuit took: ", time.time() - start_time)

if __name__ == '__main__':

    try:

        rospy.init_node('pure_pursuit', anonymous = True)
        if not plan:
            rospy.loginfo('obtaining trajectory')
            construct_path()

        # This node subsribes to the pose estimate provided by the Particle Filter. 
        # The message type of that pose message is PoseStamped which belongs to the geometry_msgs ROS package.
        rospy.Subscriber('ftg', AckermannDrive, get_ftg)
        rospy.Subscriber('extended_ranges', LaserScan, get_extended_ranges)

        rospy.wait_for_message('extended_ranges', LaserScan, timeout=10)

        rospy.Subscriber('/{}/particle_filter/viz/inferred_pose'.format(car_name), PoseStamped, purepursuit_control_node)
        rospy.spin()

    except rospy.ROSInterruptException:

        pass
