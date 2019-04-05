#!/usr/bin/env python

# Duckie Interface Node
# Gabe Petersen - 28 Mar 2019
# Updated - 4 Apr 2019
# Create a interface to translate angled velocities to inverse_kinematics_node.py
# Also eventually create interface for QT

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
		if self.start_cond == 0:
			self.curr_pose = cc_msg
			self.start_cond = 1
			rospy.loginfo("Read Start Pose")

		# initialize velocity variables at each callback to 0 
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
			# ------ Will Change the gains for this Equation ------
			self.curr_pose.x += 0.5 * math.cos(self.curr_pose.theta)
			self.curr_pose.y += 0.5 * math.sin(self.curr_pose.theta)
			rospy.loginfo("duckie goes up")
		elif direction == 's':
         # update reverse direction
			# velocity starting at -3.0 and omega at 0 degrees	
			car_vel.v += -3.0
			car_vel.omega = 0.0
			# ------ Will Change this Equation ------
			self.curr_pose.x += -0.5 * math.cos(self.curr_pose.theta)
			self.curr_pose.y += -0.5 * math.sin(self.curr_pose.theta)
			rospy.loginfo("duckie goes down")
		elif direction == 'd':
         # update right direction
			# velocity starts at 1.0 and omega at -0.785 (-45 deg)
			car_vel.v += 1.0
			car_vel.omega += -0.785
			# ------ Will Change this Equation ------
			self.curr_pose.theta += -0.5
			self.curr_pose.x += 0.5 * math.cos(self.curr_pose.theta)
			self.curr_pose.y += 0.5 * math.sin(self.curr_pose.theta)
			rospy.loginfo("duckie goes right")
		elif direction == 'a':
         # update left direction
			# velocity starts at 1.0 and omega at 0.785 (45 deg)
			car_vel.v += 1.0
			car_vel.omega += 0.785
			# ------ Will Change this Equation ------
 			self.curr_pose.theta += 0.5
			self.curr_pose.x += 0.5 * math.cos(self.curr_pose.theta)
			self.curr_pose.y += 0.5 * math.sin(self.curr_pose.theta)
			rospy.loginfo("duckie goes left")

		# publish current pose
		self.pub_pred_pose.publish(self.curr_pose)
		# publish car velocity commands
		self.pub_car_cmd.publish(car_vel)
		# latch for a lil bit
		rospy.sleep(0.5)
		
	
if __name__ == "__main__":
    rospy.init_node("duckie_interface",anonymous=False)
    Duckie_Interface = DuckieInterface()
    rospy.spin()	
