#!/usr/bin/env python

# Import necessary libraries
import rospy
import os
import sys
import csv
import math
from ackermann_msgs.msg import AckermannDrive
from geometry_msgs.msg import PolygonStamped
from geometry_msgs.msg import Point32
from geometry_msgs.msg import PoseStamped
import tf

# Global variables for storing the path, path resolution, frame ID, and car details
plan                = []
path_resolution     = []
frame_id            = 'map'
car_name            = 'car_3'
# trajectory_name     = 'raceline_2' # str(sys.argv[1])
# trajectory_name     = 'comp_1' # str(sys.argv[1])
trajectory_name     = str(rospy.get_param('/{}/f1tenth_purepursuit/raceline'.format(car_name), 'comp_7_smooth'))
last_bpi = 0

# Publishers for sending driving commands and visualizing the control polygon
command_pub         = rospy.Publisher('/{}/offboard/command'.format(car_name), AckermannDrive, queue_size = 1)
polygon_pub         = rospy.Publisher('/{}/purepursuit_control/visualize'.format(car_name), PolygonStamped, queue_size = 1)

# Global variables for waypoint sequence and current polygon
global wp_seq
global curr_polygon
global kd
kd = 0.05

wp_seq          = 0
control_polygon = PolygonStamped()

def construct_path():
    # Function to construct the path from a CSV file
    # TODO: Modify this path to match the folder where the csv file containing the path is located.
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


def dist(a, b):
    dx = a[0] - b[0]
    dy = a[1] - b[1]
    return math.sqrt(dx*dx + dy*dy)

# Steering Range from -100.0 to 100.0
STEERING_RANGE = 100.0

# vehicle physical parameters
WHEELBASE_LEN       = 0.325

# previous speed
global prev_speed
prev_speed = 30
global last_angle
last_angle = 0
def purepursuit_control_node(data):      
    MAX_SPEED = 65
    # Main control function for pure pursuit algorithm

    # Create an empty ackermann drive message that we will populate later with the desired steering angle and speed.
    command = AckermannDrive()

    global wp_seq
    global curr_polygon

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
    lookahead_distance = 2.75 * (prev_speed+50)/(MAX_SPEED+50)
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

    # might need some tuning
    wheel_angle = 6 * math.degrees(wheel_angle_rad)
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
    speed = 5 * MAX_SPEED / (1 + .14 * abs(wheel_angle)) - kd * abs(wheel_angle - last_angle)
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

if __name__ == '__main__':

    try:

        rospy.init_node('pure_pursuit', anonymous = True)
        if not plan:
            rospy.loginfo('obtaining trajectory')
            construct_path()

        # This node subsribes to the pose estimate provided by the Particle Filter. 
        # The message type of that pose message is PoseStamped which belongs to the geometry_msgs ROS package.
        rospy.Subscriber('/{}/particle_filter/viz/inferred_pose'.format(car_name), PoseStamped, purepursuit_control_node)
        rospy.spin()

    except rospy.ROSInterruptException:

        pass
