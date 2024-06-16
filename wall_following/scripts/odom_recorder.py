#!/usr/bin/env python
import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point32
import math

class OdomRecorder:
    def __init__(self):
        self._odom_sub = rospy.Subscriber('/odom', Odometry, self._odom_callback)
        self._start_position = None
        self._current_position = Point32()
        self._previous_position = None
        self._total_distance = 0.0
        self._record = False
        self._odometry_list = []

    def _odom_callback(self, msg):
        if self._record:

            self._current_position.x = msg.pose.pose.position.x
            self._current_position.y = msg.pose.pose.position.y
            self._current_position.z = 0  # Ignoring z axis for 2D odometry

            if self._previous_position is not None:
                distance = math.sqrt(
                    (self._current_position.x - self._previous_position.x) ** 2 +
                    (self._current_position.y - self._previous_position.y) ** 2
                )
                self._total_distance += distance

            if self._start_position is None:
                self._start_position = Point32(self._current_position.x, self._current_position.y, 0)
                rospy.loginfo(f"start position x: {self._start_position.x}; y: {self._start_position.y} ")

            self._previous_position = Point32(self._current_position.x, self._current_position.y, 0)
            self._odometry_list.append(Point32(self._current_position.x, self._current_position.y, 0))

    def start_recording(self):
        self._record = True
        self._total_distance = 0.0
        self._odometry_list = []

    def stop_recording(self):
        self._record = False

    def get_total_distance(self):
        return self._total_distance

    def get_odometry_list(self):
        return self._odometry_list

    def has_completed_lap(self, tolerance=0.1):
        if self._start_position is None:
            rospy.logwarn("start position not set yet")
            return False

        distance_to_start = math.sqrt(
            (self._current_position.x - self._start_position.x) ** 2 +
            (self._current_position.y - self._start_position.y) ** 2
        )
        rospy.loginfo(f"Current distance to start point: {distance_to_start} meters.")
        return distance_to_start <= tolerance
