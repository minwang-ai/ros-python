#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan

class WallFinder:
    def __init__(self):
        self.pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
        self.sub = rospy.Subscriber('/scan', LaserScan, self.laser_callback)

        self.rate = rospy.Rate(10)
        self.move_cmd = Twist()  

        self.laser_data = []
        self.num_ray = None
        self.front_index = None
        self.angle_increment = None


    def laser_callback(self, msg):
        self.laser_data = msg.ranges
        if self.front_index is None:
            self.calculate_laser_indices(msg)

    def calculate_laser_indices(self, msg):
         # Determine the indices for the key directions
        self.angle_increment = msg.angle_increment

        # Calculate indices 
        self.num_ray =len(msg.ranges)
        self.front_index = int((0 - msg.angle_min) / msg.angle_increment)
        #self.right_index = int(self.front_index / 2)
        #self.left_index = self.front_index + int((self.num_ray - self.front_index) / 2)

    def find_wall(self):
        rospy.loginfo("Finding the nearest wall...")

        # Step 1: Identify the shortest laser ray . Assume the closest obstacle is the wall (simplied case, only in the simulation/real enviroment in this course).
        while not self.laser_data:
            self.rate.sleep()

        min_distance = min(self.laser_data)
        min_index = self.laser_data.index(min_distance)
        rospy.loginfo(f"The min distance: {min_distance} m at index {min_index}")

        # Step 2: Rotate the robot until the front of it is facing the wall (front laser ray is the smallest one). 
        angle_to_rotate = (min_index - self.front_index) * self.angle_increment
        
        rospy.loginfo(f"Rotating by {angle_to_rotate} radians to face the nearest wall")
        self.rotate(angle_to_rotate)

        # Step 3: Move forward until the front ray is smaller than 30 cm
        rospy.loginfo("Move forward to the closest wall ...")
        while self.laser_data[self.front_index] > 0.3:
            self.move_cmd.angular.z = 0
            self.move_cmd.linear.x = 0.2
            self.pub.publish(self.move_cmd)
            self.rate.sleep()

        # Step 4: Rotate left by 90 degrees to align with the wall
        rospy.loginfo("Rotating left by 90 degrees to align with the wall")
        self.rotate(1.5708)  # Rotate 90 degrees left (π/2 radians)


        # Stop the robot after positioning
        self.stop_robot()
        return True

    def rotate(self, angle):
        # Rotate by a specific angle
        if angle < 0:
            angular_speed = -0.2  # radians per second
        else:
            angular_speed = 0.2
        duration = angle / angular_speed
        self.move_cmd.linear.x = 0
        self.move_cmd.angular.z = angular_speed

        end_time = rospy.Time.now() + rospy.Duration(duration)
        while rospy.Time.now() < end_time and not rospy.is_shutdown():
            self.pub.publish(self.move_cmd)
            self.rate.sleep()

        # Stop rotation
        self.move_cmd.angular.z = 0
        self.pub.publish(self.move_cmd)

    def stop_robot(self):
        rospy.loginfo("Stopping the robot")
        self.move_cmd.linear.x = 0
        self.move_cmd.angular.z = 0
        self.pub.publish(self.move_cmd)
