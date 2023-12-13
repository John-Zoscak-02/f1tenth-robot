#!/usr/bin/env python
import math
import rospy
from race.msg import pid_input
from ackermann_msgs.msg import AckermannDrive
from collections import deque

# PID Control Params
kp = 80.0 #30
kd = 120 #75.0 #135
ki = 0 #0
#speed 35
servo_offset = 0.0	# zero correction offset in case servo is misaligned and has a bias in turning.
prev_error = 0.0
errors = deque(maxlen=40)
errors.append(0)

# This code can input desired velocity from the user.
# velocity must be between [move_base.launch0,100] to move forward.
# The following velocity values correspond to different speed profiles.
# 15: Very Slow (Good for debug mode)
# 25: Slow and steady
# 35: Nice Autonomous Pace
# > 40: Careful, what you do here. Only use this if your autonomous steering is very reliable.
vel_input = 35	#TODO
prev_angle = 0.0

# Publisher for moving the car.
# TODO: Use the coorect topic /car_x/offboard/command. The multiplexer listens to this topic
command_pub = rospy.Publisher('/car_3/offboard/command', AckermannDrive, queue_size = 1)

def control(data):
	global prev_error
	global vel_input
	global kp
	global kd
	global angle
	global prev_angle
	global errors

	#print("PID Control Node is Listening to error")0.144.35.1

	## Your PID code goes here
	#TODO: Use kp, ki & kd to implement a PID controller 
        angle = (kp * data.pid_error) + (ki * (sum(errors)/len(errors))) + (-kd * (prev_error - data.pid_error))
	errors.append(data.pid_error)
    # 1. Scale the error
	# 2. Apply the PID equation on error to compute steering

	# An empty AckermannDrive message is created. You will populate the steering_angle and the speed fields.
	command = AckermannDrive()
        
        if angle < -100 :
            angle = -100
        elif angle > 100 :
            angle = 100
	command.steering_angle = (angle) if not math.isnan(angle) else 0	
	print('desired angle {}'.format(command.steering_angle))

        if vel_input < 0 :
            vel_input = 0
        elif vel_input > 100 :
            vel_input = 100
    # command.speed = vel_input * (2-abs(data.pid_error))
	# if command.steering_angle > 60 or command.steering_angle < -60:
	# 	command.speed = 10
	# elif command.steering_angle > 40 or command.steering_angle < -40:
	# 	command.speed = 15
	# else:
	#command.speed = vel_input / (1 + 0.5*math.sqrt(abs(command.steering_angle)))
		# command.speed = vel_input - (vel_input/120)*abs(command.steering_angle)  #/(1+abs(command.steering_angle))
	command.speed = vel_input * (1.4 / (1 + abs(data.pid_error)))#**2)
	# command.speed = (-vel_input/2) * abs(data.pid_error) + vel_input
	# print('PID VEL: {}'.format(data.pid_vel))
	# if math.isnan(data.pid_vel) or data.pid_vel==0:
	# 	command.speed = vel_input
	# else:
	# command.speed = -vel_input * (1.4 / (1 + 2*math.sqrt(data.pid_vel)))+vel_input*5/4#**2)
	# 	command.speed = vel_input*data.pid_vel/(1+data.pid_vel) + 10
	# command.speed = min(10+vel_input, 10+vel_input*data.pid_vel)
	print('Desired speed: {}'.format(command.speed))
	# command.speed = vel_input #if data.pid_vel == 1 else 0
	# Move the car autonomously
	command_pub.publish(command)
	prev_error = data.pid_error
	prev_angle = angle

if __name__ == '__main__':

    # This code tempalte asks for the values for the gains from the user upon start, but you are free to set them as ROS parameters as well.
	global kp
	global kd
	global ki
	global vel_input
	# Used during testing pids, current best values are above
	# kp = input("Enter Kp Value: ")
	# kd = input("Enter Kd Value: ")
	# ki = input("Enter Ki Value: ")
	# vel_input = input("Enter desired velocity: ")
	rospy.init_node('pid_controller', anonymous=True)
    # subscribe to the error topic
	rospy.Subscriber("error", pid_input, control)
	rospy.spin()
