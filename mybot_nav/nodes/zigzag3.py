#!/usr/bin/env python
#
# """ nav_square.py - Version 1.1 2013-12-20
#
#     A basic demo of the using odometry data to move the robot
#     along a square trajectory.
#
#     Created for the Pi Robot Project: http://www.pirobot.org
#     Copyright (c) 2012 Patrick Goebel.  All rights reserved.
#
#     This program is free software; you can redistribute it and/or modify
#     it under the terms of the GNU General Public License as published by
#     the Free Software Foundation; either version 2 of the License, or
#     (at your option) any later version.5
#
#     This program is distributed in the hope that it will be useful,
#     but WITHOUT ANY WARRANTY; without even the implied warranty of
#     MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
#     GNU General Public License for more details at:
#
#     http://www.gnu.org/licenses/gpl.html
#
# """
#
# import rospy
# from geometry_msgs.msg import Twist, Point, Quaternion
# import tf
# from mybot_nav.transform_utils import quat_to_angle, normalize_angle
# from math import radians, copysign, sqrt, pow, pi
#
# class NavSquare():
#     def __init__(self):
#         # Give the node a name
#         rospy.init_node('nav_square', anonymous=False)
#
#         # Set rospy to execute a shutdown function when terminating the script
#         rospy.on_shutdown(self.shutdown)
#
#         # How fast will we check the odometry values?
#         rate = 20
#
#         # Set the equivalent ROS rate variable
#         r = rospy.Rate(rate)
#
#         # Set the parameters for the target square
#         goal_distance = rospy.get_param("~goal_distance", 1.0)      # meters
#         goal_angle = radians(rospy.get_param("~goal_angle", 90))    # degrees converted to radians
#         linear_speed = rospy.get_param("~linear_speed", 0.2)        # meters per second
#         angular_speed = rospy.get_param("~angular_speed", 0.7)      # radians per second
#         angular_tolerance = radians(rospy.get_param("~angular_tolerance", 2)) # degrees to radians
#
#         # Publisher to control the robot's speed
#         self.cmd_vel = rospy.Publisher('/cmd_vel', Twist, queue_size=5)
#
#         # The base frame is base_footprint for the TurtleBot but base_link for Pi Robot
#         self.base_frame = rospy.get_param('~base_frame', '/base_link')
#
#         # The odom frame is usually just /odom
#         self.odom_frame = rospy.get_param('~odom_frame', '/odom')
#
#         # Initialize the tf listener
#         self.tf_listener = tf.TransformListener()
#
#         # Give tf some time to fill its buffer
#         rospy.sleep(2)
#
#         # Set the odom frame
#         self.odom_frame = '/odom'
#
#         # Find out if the robot uses /base_link or /base_footprint
#         try:
#             self.tf_listener.waitForTransform(self.odom_frame, '/base_footprint', rospy.Time(), rospy.Duration(1.0))
#             self.base_frame = '/base_footprint'
#         except (tf.Exception, tf.ConnectivityException, tf.LookupException):
#             try:
#                 self.tf_listener.waitForTransform(self.odom_frame, '/base_link', rospy.Time(), rospy.Duration(1.0))
#                 self.base_frame = '/base_link'
#             except (tf.Exception, tf.ConnectivityException, tf.LookupException):
#                 rospy.loginfo("Cannot find transform between /odom and /base_link or /base_footprint")
#                 rospy.signal_shutdown("tf Exception")
#
#         # Initialize the position variable as a Point type
#         position = Point()
#
#         # Cycle through the four sides of the square
#         for i in range(4):
#             # Initialize the movement command
#             move_cmd = Twist()
#
#             # Set the movement command to forward motion
#             move_cmd.linear.x = linear_speed
#
#             # Get the starting position values
#             (position, rotation) = self.get_odom()
#
#             x_start = position.x
#             y_start = position.y
#
#             # Keep track of the distance traveled
#             distance = 0
#
#             # Enter the loop to move along a side
#             while distance < goal_distance and not rospy.is_shutdown():
#                 # Publish the Twist message and sleep 1 cycle
#                 self.cmd_vel.publish(move_cmd)
#
#                 r.sleep()
#
#                 # Get the current position
#                 (position, rotation) = self.get_odom()
#
#                 # Compute the Euclidean distance from the start
#                 distance = sqrt(pow((position.x - x_start), 2) +
#                                 pow((position.y - y_start), 2))
#
#             # Stop the robot before rotating
#             move_cmd = Twist()
#             self.cmd_vel.publish(move_cmd)
#             rospy.sleep(1.0)
#
#             # Set the movement command to a rotation
#             move_cmd.angular.z = angular_speed
#
#             # Track the last angle measured
#             last_angle = rotation
#
#             # Track how far we have turned
#             turn_angle = 0
#
#             # Begin the rotation
#             while abs(turn_angle + angular_tolerance) < abs(goal_angle) and not rospy.is_shutdown():
#                 # Publish the Twist message and sleep 1 cycle
#                 self.cmd_vel.publish(move_cmd)
#
#                 r.sleep()
#
#                 # Get the current rotation
#                 (position, rotation) = self.get_odom()
#
#                 # Compute the amount of rotation since the last lopp
#                 delta_angle = normalize_angle(rotation - last_angle)
#
#                 turn_angle += delta_angle
#                 last_angle = rotation
#
#             move_cmd = Twist()
#             self.cmd_vel.publish(move_cmd)
#             rospy.sleep(1.0)
#
#         # Stop the robot when we are done
#         self.cmd_vel.publish(Twist())
#
#     def get_odom(self):
#         # Get the current transform between the odom and base frames
#         try:
#             (trans, rot)  = self.tf_listener.lookupTransform(self.odom_frame, self.base_frame, rospy.Time(0))
#         except (tf.Exception, tf.ConnectivityException, tf.LookupException):
#             rospy.loginfo("TF Exception")
#             return
#
#         return (Point(*trans), quat_to_angle(Quaternion(*rot)))
#
#     def shutdown(self):
#         # Always stop the robot when shutting down the node
#         rospy.loginfo("Stopping the robot...")
#         self.cmd_vel.publish(Twist())
#         rospy.sleep(1)
#
# if __name__ == '__main__':
#     try:
#         NavSquare()
#     except rospy.ROSInterruptException:
#         rospy.loginfo("Navigation terminated.")

import rospy
from geometry_msgs.msg import Twist, Point, Quaternion, PoseStamped
from nav_msgs.msg import OccupancyGrid
import tf
import collections
from mybot_nav.transform_utils import quat_to_angle, normalize_angle
from math import radians, copysign, sqrt, pow, pi

step = 0.5
map_2d_step = [[0 for i in range(0,101)] for j in range(0,101)]
class Point_2D():
	x = 0.0
	y = 0.0
	def __init__(self):
		self.x = 0.0
		self.y = 0.0

def get_step(num):
	global step
	if num % step > step/2:
		return num // step * step + step
	else:
		return num // step * step

def get_map_callback(msg):
	print "get map!!!"
	map_2d_2048 = [[0 for i in range(0,2048)] for j in range(0,2048)]
	i = 0
	j = 0
	for msg_index in range(0,2048 * 2048):
		map_2d_2048[msg_index//2048][msg_index%2048] = msg.data[msg_index]
	for i in range(14,2034):
		for j in range(14,2034):
			if map_2d_2048[i][j] == 0:
				map_2d_step[(i-14)//20][(j-14)//20] += 1
	del map_2d_2048


class NavSquare():
	global step
	global map_2d_step
	def __init__(self):
		rospy.init_node('nav_square', anonymous=False)

		rospy.on_shutdown(self.shutdown)
		rate = 50
		r = rospy.Rate(rate)
		self.cmd_vel = rospy.Publisher('/cmd_vel', Twist, queue_size=5)
		self.move_base_simple_goal = rospy.Publisher('/move_base_simple/goal', PoseStamped, queue_size=1)

		self.get_map = rospy.Subscriber('/map', OccupancyGrid, get_map_callback)

		self.base_frame = rospy.get_param('~base_frame', '/base_link')
		self.odom_frame = rospy.get_param('~odom_frame', '/odom')
		self.map_frame = rospy.get_param('~map_frame', '/map')

		self.tf_listener = tf.TransformListener()

		rospy.sleep(2)

		self.odom_frame = '/odom'

		try:
			self.tf_listener.waitForTransform(self.odom_frame, '/base_footprint', rospy.Time(), rospy.Duration(1.0))
			self.base_frame = '/base_footprint'
		except (tf.Exception, tf.ConnectivityException, tf.LookupException):
			try:
				self.tf_listener.waitForTransform(self.odom_frame, '/base_link', rospy.Time(), rospy.Duration(1.0))
				self.base_frame = '/base_link'
			except (tf.Exception, tf.ConnectivityException, tf.LookupException):
				rospy.loginfo("Cannot find transform between /odom and /base_link or /base_footprint")
				rospy.signal_shutdown("tf Exception")

		position = Point()
		position0 = Point()
		move_goal = PoseStamped()
		mapsize = 12
		goalmap_dict = collections.OrderedDict()
		offmap_dict = dict()
		check_point = Point_2D()
		goal_point = Point_2D()
		Last_Point = Point_2D()
		i=0
		Flag_New_Goal = 0
		Flag_Last_Goal = 1
		Flag_Run = False

		(position, rotation) = self.get_odom()
		position.x = get_step(position.x)
		position.y = get_step(position.y)
		print position
		start_x = position.x
		start_y = position.y
		Last_Point.x = position.x
		Last_Point.y = position.y
		for i in range(0,101):
			for j in range(0,101):
				if map_2d_step[i][j] < 200:
					offmap_dict[(i-50, j-50)] = 1
		'''for a in range(-mapsize,mapsize+1):
			check_point.x = step * mapsize + start_x
			check_point.y = step * a + start_y
			offmap_dict[(check_point.x/step, check_point.y/step)] = 1
		for a in range(-mapsize,mapsize+1):
			check_point.x = -step * mapsize + start_x
			check_point.y = step * a + start_y
			offmap_dict[(check_point.x/step, check_point.y/step)] = 1
		for a in range(1-mapsize,mapsize):
			check_point.x = step * a + start_x
			check_point.y = step * mapsize + start_y
			offmap_dict[(check_point.x/step, check_point.y/step)] = 1
		for a in range(1-mapsize,mapsize):
			check_point.x = step * a + start_x
			check_point.y = -step * mapsize + start_y
			offmap_dict[(check_point.x/step, check_point.y/step)] = 1'''
		#offmap_dict[2.0,2.0] = 1
		move_goal.header.frame_id = 'map'
		move_goal.pose.orientation.x = 0.0
		move_goal.pose.orientation.y = 0.0

		while not rospy.is_shutdown():
			while not rospy.is_shutdown():
				check_point.x = position.x
				check_point.y = position.y
				offmap_dict[(check_point.x/step, check_point.y/step)] = 1
				if (check_point.x/step, check_point.y/step) in goalmap_dict:
					del goalmap_dict[(check_point.x/step, check_point.y/step)]

				check_point.x = position.x + step
				check_point.y = position.y
				if (check_point.x/step, check_point.y/step) not in offmap_dict:
					if Flag_Last_Goal == 1:
						Flag_New_Goal = 1
						goal_point.x = check_point.x
						goal_point.y = check_point.y
					elif Flag_Last_Goal != 2:
						Flag_New_Goal = 1

				else:
					if Flag_Last_Goal == 1:
						goal_point.x = position.x
						goal_point.y = position.y
						Flag_Run = True

				check_point.x = position.x - step
				check_point.y = position.y
				if (check_point.x/step, check_point.y/step) not in offmap_dict:
					if Flag_Last_Goal == 2:
						Flag_New_Goal = 2
						goal_point.x = check_point.x
						goal_point.y = check_point.y
					else:
						if Flag_New_Goal == 0:
							Flag_New_Goal = 2
							goal_point.x = check_point.x
							goal_point.y = check_point.y
				else:
					if Flag_Last_Goal == 2:
						goal_point.x = position.x
						goal_point.y = position.y
						Flag_Run = True

				check_point.x = position.x
				check_point.y = position.y + step
				if (check_point.x/step, check_point.y/step) not in offmap_dict:
					if Flag_New_Goal == 0:
						Flag_New_Goal = 3
						goal_point.x = check_point.x
						goal_point.y = check_point.y
						Flag_Run = True

				check_point.x = position.x
				check_point.y = position.y - step
				if (check_point.x/step, check_point.y/step) not in offmap_dict:
					if Flag_New_Goal == 0:
						Flag_New_Goal = 4
						goal_point.x = check_point.x
						goal_point.y = check_point.y
						Flag_Run = True
					else:
						goalmap_dict[(check_point.x/step, check_point.y/step)] = Point_2D()
						goalmap_dict[(check_point.x/step, check_point.y/step)].x = check_point.x
						goalmap_dict[(check_point.x/step, check_point.y/step)].y = check_point.y

				if Flag_New_Goal == 0:
					if len(goalmap_dict) == 0:
						Flag_New_Goal = 9
						break
					else:
						(key, goal_point) = goalmap_dict.popitem()
						Flag_New_Goal = 5
						Flag_Run = True

				if Flag_Run == True:
					break

				position.x = goal_point.x
				position.y = goal_point.y
				Flag_Last_Goal = Flag_New_Goal

			if Flag_New_Goal == 9:
				break
			elif Flag_New_Goal == 1:
				move_goal.pose.orientation.z = 0.0
				move_goal.pose.orientation.w = 1.0
				move_goal.pose.position.x = Last_Point.x
				move_goal.pose.position.y = Last_Point.y
				if rotation > 0.1 or rotation < -0.1:
					self.move_base_simple_goal.publish(move_goal)
					while (rotation > 0.1 or rotation < -0.1) and not rospy.is_shutdown():
						(position0, rotation) = self.get_odom()
			elif Flag_New_Goal == 2:
				move_goal.pose.orientation.z = 1.0
				move_goal.pose.orientation.w = 0.0
				if rotation > 0.1-pi or rotation < pi-0.1:
					move_goal.pose.position.x = Last_Point.x
					move_goal.pose.position.y = Last_Point.y
					self.move_base_simple_goal.publish(move_goal)
					while ((rotation > 0.1-pi and rotation < 0) or (rotation < pi-0.1 and rotation > 0)) and not rospy.is_shutdown():
						(position0, rotation) = self.get_odom()
			elif Flag_New_Goal == 3:
				move_goal.pose.orientation.z = 1.0
				move_goal.pose.orientation.w = 1.0
				move_goal.pose.position.x = Last_Point.x
				move_goal.pose.position.y = Last_Point.y
				if rotation > pi/2+0.1 or rotation < pi/2-0.1:
					self.move_base_simple_goal.publish(move_goal)
					while (rotation > pi/2+0.1 or rotation < pi/2-0.1) and not rospy.is_shutdown():
						(position0, rotation) = self.get_odom()
			elif Flag_New_Goal == 4:
				move_goal.pose.orientation.z = -1.0
				move_goal.pose.orientation.w = 1.0
				move_goal.pose.position.x = Last_Point.x
				move_goal.pose.position.y = Last_Point.y
				if rotation > (-pi/2)+0.1 or rotation < (-pi/2)-0.1:
					self.move_base_simple_goal.publish(move_goal)
					while (rotation > -pi/2+0.1 or rotation < -pi/2-0.1) and not rospy.is_shutdown():
						(position0, rotation) = self.get_odom()
			move_goal.pose.position.x = goal_point.x
			move_goal.pose.position.y = goal_point.y
			self.move_base_simple_goal.publish(move_goal)
			print move_goal
			(position, rotation) = self.get_odom()
			position.x = get_step(position.x)
			position.y = get_step(position.y)
			distance = sqrt(pow((position.x - goal_point.x), 2) + pow((position.y - goal_point.y), 2))
			while distance > (step/500) and not rospy.is_shutdown():
				(position, rotation) = self.get_odom()
				position.x = get_step(position.x)
				position.y = get_step(position.y)
				distance = sqrt(pow((position.x - goal_point.x), 2) + pow((position.y - goal_point.y), 2))
			Flag_Run = False
			Flag_Last_Goal = Flag_New_Goal
			Flag_New_Goal = 0
			Last_Point.x = goal_point.x
			Last_Point.y = goal_point.y


	def get_odom(self):
		#Get the current transform between the odom and base frames
		try:
			(trans, rot)  = self.tf_listener.lookupTransform(self.map_frame, self.base_frame, rospy.Time(0))
		except (tf.Exception, tf.ConnectivityException, tf.LookupException):
			rospy.loginfo("TF Exception")
			return

		return (Point(*trans), quat_to_angle(Quaternion(*rot)))

	def shutdown(self):
		#Always stop the robot when shutting down the node
		rospy.loginfo("Stopping the robot...")
		self.cmd_vel.publish(Twist())
		rospy.sleep(1)


if __name__ == '__main__':
	try:
		NavSquare()
	except rospy.ROSInterruptException:
		rospy.loginfo("Navigation terminated.")

