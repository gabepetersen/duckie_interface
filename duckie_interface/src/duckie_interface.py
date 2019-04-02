#!/usr/bin/env python

# Duckie Interface Node
# Gabe Petersen - 28 Mar 2019
# Create a interface to translate angled velocities to inverse_kinematics_node.py
# Also create interface for QT

import rospy
import math
from duckietown_msgs.msg import Twist2DStamped
from geometry_msgs.msg import Pose2D

class DuckieInterface(object):
	def __init__(self):
		# setup subscriber and publisher
		self.sub_pose = rospy.Subscriber("car1/pose", Pose2D, self.cc_cb)
		self.pub_pred_pose = rospy.Publisher("car1/predicted_pose", Pose2D, queue_size=1)
		self.pub_car_cmd = rospy.Publisher("~car_cmd", Twist2DStamped, queue_size=1)

		self.curr_pose = Pose2D()
		self.start_cond = 0
	
	def cc_cb(self, cc_msg):
		# get start pose if first iteration		
		if start_cond == 0:
			start_pose = cc_msg
			start_cond = 1
			rospy.loginfo("Read Start Pose")

		# initialize variables
		car_vel = Twist2DStamped()
		car_vel.v = 0
		car_vel.omega = 0
		# get user input
		# need to test angle gains, velocity gains, and direction
		# gather user input
		direction = raw_input('Enter direction(a,w,s,d)--> ')
		
		if direction == 'w':
      	# update forward direction
			# velocity starting at 3.0 and omega at 0 degrees
			car_vel.v += 3.0
			car_vel.omega = 0.0
			# ------ Will Change this Equation ------
			curr_pose.x = 1.5 * cos(curr_pose.theta)
			curr_pose.y = 1.5 * sin(curr_pose.theta)
		elif direction == 's':
         # update reverse direction
			# velocity starting at -3.0 and omega at 0 degrees	
			car_vel.v += -3.0
			car_vel.omega = 0.0
			# ------ Will Change this Equation ------
			curr_pose.x = -1.5 * cos(curr_pose.theta)
			curr_pose.y = -1.5 * sin(curr_pose.theta)
		elif direction == 'd':
         # update right direction
			# velocity starts at 1.0 and omega at -0.785 (-45 deg)
			car_vel.v += 1.0
			car_vel.omega += -0.785
			# ------ Will Change this Equation ------
			cur_pose.theta += -0.5
			curr_pose.x = 1.5 * cos(curr_pose.theta)
			curr_pose.y = 1.5 * sin(curr_pose.theta)
		elif direction == 'a':
         # update left direction
			# velocity starts at 1.0 and omega at 0.785 (45 deg)
			car_vel.v += 1.0
			car_vel.omega += 0.785
			# ------ Will Change this Equation ------
 			cur_pose.theta += 0.5
			curr_pose.x = 1.5 * cos(curr_pose.theta)
			curr_pose.y = 1.5 * sin(curr_pose.theta)

		# latch for a lil bit
		rospy.sleep(0.5)
		# publish current pose
		pub_pred_pose.publish(curr_pose)
		# publish car velocity commands
		pub_car_cmd.publish(car_vel)
		
	
if __name__ == "__main__":
    rospy.init_node("duckie_interface",anonymous=False)
    Duckie_Interface = DuckieInterface()
    rospy.spin()	
		
