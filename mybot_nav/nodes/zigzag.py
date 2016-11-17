#!/usr/bin/env python

""" nav_square.py - Version 1.1 2013-12-20

    A basic demo of the using odometry data to move the robot
    along a square trajectory.

    Created for the Pi Robot Project: http://www.pirobot.org
    Copyright (c) 2012 Patrick Goebel.  All rights reserved.

    This program is free software; you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation; either version 2 of the License, or
    (at your option) any later version.5
    
    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details at:
    
    http://www.gnu.org/licenses/gpl.html
      
"""

import rospy
from geometry_msgs.msg import Twist, Point, Quaternion, PoseStamped
from nav_msgs.msg import OccupancyGrid
import tf
import collections
from mybot_nav.transform_utils import quat_to_angle, normalize_angle
from math import radians, copysign, sqrt, pow, pi
from enum import Enum

from ctypes import *
import os


class Direction(Enum):
    Init = 0
    Up = 1
    Down = 2
    Left = 3
    Right = 4
    Free = 5
    End = 9


step = 0.5
map_2d_step = [[0 for i in range(0, 101)] for j in range(0, 101)]


class Point_2D():
    x = 0.0
    y = 0.0

    def __init__(self):
        self.x = 0.0
        self.y = 0.0


def get_step(num):
    global step
    if num % step > step / 2:
        return num // step * step + step
    else:
        return num // step * step


class NavSquare():
    global step
    global map_2d_step

    def __init__(self):
        # Give the node a name
        rospy.init_node('nav_square', anonymous=False)

        # Set rospy to execute a shutdown function when terminating the script
        rospy.on_shutdown(self.shutdown)

        # How fast will we check the odometry values?
        rate = 50

        # Set the equivalent ROS rate variable
        r = rospy.Rate(rate)
        # self.field_shape = False

        self.goal = PoseStamped()
        self.goal.header.frame_id = 'map'

        self.cmd_vel = rospy.Publisher('/cmd_vel', Twist, queue_size=5)
        self.move_base_simple_goal = rospy.Publisher('/move_base_simple/goal', PoseStamped, queue_size=1)

        # The base frame is base_footprint for the TurtleBot but base_link for Pi Robot
        self.base_frame = rospy.get_param('~base_frame', '/base_link')

        # The odom frame is usually just /odom
        self.odom_frame = rospy.get_param('~odom_frame', '/odom')

        # The map frame is usually just /map
        self.map_frame = rospy.get_param('~map_frame', '/map')

        # Initialize the tf listener
        self.tf_listener = tf.TransformListener()

        self.get_map = rospy.Subscriber('/map', OccupancyGrid, self.get_map_callback)
        self.goal_point = Point_2D

        # Give tf some time to fill its buffer
        rospy.sleep(1)

        # Find out if the robot uses /base_link or /base_footprint
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

        self.map_bound = dict()

        while not rospy.is_shutdown():
            rospy.sleep(1)
            # self.draw_polygon()

            # self.make_plan()

    def make_plan(self):

        new_direction = Direction.Up
        old_direction = Direction.Up
        flag_run = False
        move_goal = PoseStamped()
        move_goal.header.frame_id = 'map'
        move_goal.pose.orientation.x = 0.0
        move_goal.pose.orientation.y = 0.0
        next = Point_2D()
        map = Point_2D()
        goal = Point_2D()
        last_point = Point_2D()
        (current, rotation) = self.get_odom()
        current.x = get_step(current.x)
        current.y = get_step(current.y)
        last_point.x = current.x
        last_point.y = current.y
        goalmap_dict = collections.OrderedDict()

        # while self.field_shape == False:
        #     # Check to make sure ROS is ok still
        #     if rospy.is_shutdown(): return
        #     # Print message about the waiting
        #     msg = "Qualification: waiting on the field shape."
        #     rospy.loginfo(msg)

        # msg = "add field_shape"
        # rospy.loginfo(msg)

        while not rospy.is_shutdown():
            while not rospy.is_shutdown():

                next.x = current.x
                next.y = current.y
                map.x = next.x / step
                map.y = next.y / step
                self.map_bound[(map.x, map.y)] = 1

                if (map.x, map.y) in goalmap_dict:
                    del goalmap_dict[(map.x, map.y)]
                # up

                next.x = current.x + step
                next.y = current.y
                map.x = next.x / step
                map.y = next.y / step

                if (map.x, map.y) not in self.map_bound:
                    new_direction = Direction.Up
                    goal.x = next.x
                    goal.y = next.y
                else:
                    if old_direction == Direction.Up:
                        goal.x = current.x
                        goal.y = current.y
                        flag_run = True
                # down
                next.x = current.x - step
                next.y = current.y
                map.x = next.x / step
                map.y = next.y / step

                if (map.x, map.y) not in self.map_bound:
                    if old_direction == Direction.Down:
                        new_direction = Direction.Down
                        goal.x = next.x
                        goal.y = next.y
                    else:
                        if new_direction == Direction.Init:
                            new_direction = Direction.Down
                            goal.x = next.x
                            goal.y = next.y
                else:
                    if old_direction == Direction.Down:
                        goal.x = current.x
                        goal.y = current.y
                        flag_run = True
                # left
                next.x = current.x
                next.y = current.y + step
                map.x = next.x / step
                map.y = next.y / step

                if (map.x, map.y) not in self.map_bound:
                    if new_direction == Direction.Init:
                        new_direction = Direction.Left
                        goal.x = next.x
                        goal.y = next.y
                        flag_run = True
                # right
                next.x = current.x
                next.y = current.y - step
                map.x = next.x / step
                map.y = next.y / step
                if (map.x, map.y) not in self.map_bound:
                    if new_direction == Direction.Init:
                        new_direction = Direction.Right
                        goal.x = next.x
                        goal.y = next.y
                        flag_run = True
                    else:
                        goalmap_dict[(next.x / step, next.y / step)] = Point_2D()
                        goalmap_dict[(next.x / step, next.y / step)].x = next.x
                        goalmap_dict[(next.x / step, next.y / step)].y = next.y

                if new_direction == Direction.Init:
                    if len(goalmap_dict) == 0:
                        new_direction = Direction.End
                        break
                    else:
                        (key, goal) = goalmap_dict.popitem()
                        new_direction = Direction.Free
                        flag_run = True

                if flag_run == True:
                    break

                current.x = goal.x
                current.y = goal.y
                old_direction = new_direction

            if new_direction == Direction.End:
                break
            elif new_direction == Direction.Up:
                move_goal.pose.orientation.z = 0.0
                move_goal.pose.orientation.w = 1.0
                move_goal.pose.position.x = last_point.x
                move_goal.pose.position.y = last_point.y
                if rotation > 0.1 or rotation < -0.1:
                    self.move_base_simple_goal.publish(move_goal)
                    while (rotation > 0.1 or rotation < -0.1) and not rospy.is_shutdown():
                        (position0, rotation) = self.get_odom()
            elif new_direction == Direction.Down:
                move_goal.pose.orientation.z = 1.0
                move_goal.pose.orientation.w = 0.0
                if rotation > 0.1 - pi or rotation < pi - 0.1:
                    move_goal.pose.position.x = last_point.x
                    move_goal.pose.position.y = last_point.y
                    self.move_base_simple_goal.publish(move_goal)
                    while ((rotation > 0.1 - pi and rotation < 0) or (
                                    rotation < pi - 0.1 and rotation > 0)) and not rospy.is_shutdown():
                        (position0, rotation) = self.get_odom()
            elif new_direction == Direction.Left:
                move_goal.pose.orientation.z = 1.0
                move_goal.pose.orientation.w = 1.0
                move_goal.pose.position.x = last_point.x
                move_goal.pose.position.y = last_point.y
                if rotation > pi / 2 + 0.1 or rotation < pi / 2 - 0.1:
                    self.move_base_simple_goal.publish(move_goal)
                    while (rotation > pi / 2 + 0.1 or rotation < pi / 2 - 0.1) and not rospy.is_shutdown():
                        (position0, rotation) = self.get_odom()
            elif new_direction == Direction.Right:
                move_goal.pose.orientation.z = -1.0
                move_goal.pose.orientation.w = 1.0
                move_goal.pose.position.x = last_point.x
                move_goal.pose.position.y = last_point.y
                if rotation > (-pi / 2) + 0.1 or rotation < (-pi / 2) - 0.1:
                    self.move_base_simple_goal.publish(move_goal)
                    while (rotation > -pi / 2 + 0.1 or rotation < -pi / 2 - 0.1) and not rospy.is_shutdown():
                        (position0, rotation) = self.get_odom()

            move_goal.pose.position.x = goal.x
            move_goal.pose.position.y = goal.y
            self.move_base_simple_goal.publish(move_goal)
            # print move_goal
            (current, rotation) = self.get_odom()
            current.x = get_step(current.x)
            current.y = get_step(current.y)
            distance = sqrt(pow((current.x - goal.x), 2) + pow((current.y - goal.y), 2))
            while distance > (step / 500) and not rospy.is_shutdown():
                (current, rotation) = self.get_odom()
                current.x = get_step(current.x)
                current.y = get_step(current.y)
                distance = sqrt(pow((current.x - goal.x), 2) + pow((current.y - goal.y), 2))

            flag_run = False
            old_direction = new_direction
            new_direction = Direction.Init
            last_point.x = goal.x
            last_point.y = goal.y

    def draw_polygon(self):
        map_size = 3
        (position, rotation) = self.get_odom()
        start_x = position.x
        start_y = position.y
        check_point = Point_2D()
        check_point.x = step * map_size + start_x
        for y in range(-map_size, map_size + 1):
            check_point.y = step * y + start_y
            self.map_bound[(check_point.x / step, check_point.y / step)] = 1

        check_point.x = -step * map_size + start_x
        for y in range(-map_size, map_size + 1):
            check_point.y = step * y + start_y
            self.map_bound[(check_point.x / step, check_point.y / step)] = 1

        check_point.y = step * map_size + start_y
        for x in range(1 - map_size, map_size):
            check_point.x = step * x + start_x
            self.map_bound[(check_point.x / step, check_point.y / step)] = 1

        check_point.y = -step * map_size + start_y
        for x in range(1 - map_size, map_size):
            check_point.x = step * x + start_x
            self.map_bound[(check_point.x / step, check_point.y / step)] = 1

    def get_odom(self):
        # Get the current transform between the odom and base frames
        try:
            (trans, rot) = self.tf_listener.lookupTransform(self.map_frame, self.base_frame, rospy.Time(0))
        except (tf.Exception, tf.ConnectivityException, tf.LookupException):
            rospy.loginfo("TF Exception")
            return

        return (Point(*trans), quat_to_angle(Quaternion(*rot)))

    def shutdown(self):
        # Always stop the robot when shutting down the node
        rospy.loginfo("Stopping the robot...")
        self.cmd_vel.publish(Twist())
        rospy.sleep(1)

    def get_map_callback(self, msg):
        global goal_mx
        resolution = msg.info.resolution
        x_size = msg.info.width
        y_size = msg.info.height
        x_origin = msg.info.origin.position.x
        y_origin = msg.info.origin.position.y
        # rospy.logdebug("subscript map:", resolution, x_size, y_size)
        print ("subscript map:", resolution, x_size, y_size)
        print x_origin
        print y_origin

        rospy.sleep(1)
        (current, rotation) = self.get_odom()
        # current = Point_2D()
        print current.x, current.y

        mx = int((current.x - x_origin) // resolution)
        my = int((current.y - y_origin) // resolution)

        print mx
        print my
        for goal_mx in range(mx, x_size):

            # print x_size * my + mx
            # for i in range(0, 20):
            #     print \
            #         msg.data[x_size * i + 0], msg.data[x_size * i + 1], msg.data[x_size * i + 2], msg.data[x_size * i + 3], msg.data[x_size * i + 4], \
            #         msg.data[x_size * i + 5], msg.data[x_size * i + 6], msg.data[x_size * i + 7], msg.data[x_size * i + 8], msg.data[x_size * i + 9], ',', \
            #         msg.data[x_size * i + 10],msg.data[x_size * i + 11],msg.data[x_size * i + 12],msg.data[x_size * i + 13],msg.data[x_size * i + 14], \
            #         msg.data[x_size * i + 15],msg.data[x_size * i + 16],msg.data[x_size * i + 17],msg.data[x_size * i + 18],msg.data[x_size * i + 19], ','
            #
            #     if (i+1)%10 == 0:
            #         print

            # print "msg.data[y_size * my + goal_mx] =", msg.data[x_size * mx + goal_mx]
            if msg.data[x_size * my + goal_mx] != 0:
                break

        # libtest = cdll.LoadLibrary("/home/lsy563193/catkin_ws/devel/lib/libcostmap_2d.so")
        # libtest.worldToMap((c_double)(goal_point.x),(c_double)(goal_point.y),mx,my)

        self.goal_point.x = goal_mx
        self.goal_point.y = my
        rospy.loginfo("goal_point.x = %d, goal_point.y = %d", self.goal_point.x, self.goal_point.y)

        # self.field_shape = True
        rospy.sleep(1)

    # def worldToWorld(self, mx, my, wx, wy):
    #     if wx < or
if __name__ == '__main__':
    try:
        NavSquare()
    except rospy.ROSInterruptException:
        rospy.loginfo("Navigation terminated.")
