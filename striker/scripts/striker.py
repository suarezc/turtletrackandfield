#!/usr/bin/env python3
import rospy, cv2, cv_bridge
from sensor_msgs.msg import Image, LaserScan
from geometry_msgs.msg import Twist, PoseStamped
import numpy as np
import moveit_commander
import os
import sys

path_prefix = os.path.dirname(__file__) + "/action_states/"

class Striker(object):
    def __init__(self, lane_number=0):
        # rospy.init_node("movement")
        self.initialized = False

        self.state = ""

        # Set up subscribers and publishers
        self.lidar_sub = rospy.Subscriber('scan', LaserScan, self.lidar_callback)
        self.cmd_vel_pub = rospy.Publisher(f'/robot{lane_number}/cmd_vel',
                Twist, queue_size=1)

        self.front_distance = 4
        self.initialized = True 
       
    def set_velocity(self, linear, angular):
        """
        Set the velocity of the robot
        """
        vel_msg = Twist()
        vel_msg.linear.x = linear
        vel_msg.angular.z = angular
        self.cmd_vel_pub.publish(vel_msg)

    def stop(self):
        self.set_velocity(0, 0)

    def bowl(self, v, t):
        """
        Really basic bowling behavior, with speed and time
        """
        print("bowling", v, t)
        self.set_velocity(v, 0)
        rospy.sleep(t)
        self.stop()

    def lidar_callback(self, data):
        """
        Average the LIDAR distances for a 10 angle
        spead in front of the robot.
        """
        if not self.initialized:
            return 
        front_angles = [355 + x for x in range(5)] + [x for x in range(5)]
        distances = []
        for angle in front_angles:
            if data.ranges[angle] < data.range_max:
                 distances.append(data.ranges[angle])
        if len(distances) == 0:
            self.front_distance = data.range_max
        else:
            self.front_distance = sum(distances)/len(distances)

