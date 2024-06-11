#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan

from find_wall_service_client import FindWallServiceClient  # Import the service client
from record_odom_action_client import RecordOdomActionClient  # Import the action client

import threading

class WallFollower():
    def __init__(self):
    
        self.pub = rospy.Publisher("/cmd_vel", Twist, queue_size=10)
    
        self.move_cmd = Twist()

        self.num_ray = None
        self.front_index = None
        self.right_index = None
        self.angle_increment = None
        self.laser_front_list = []
        self.laser_right_list = []
        self.front_distance = 0
        self.right_distance = 0


        # Register the shutdown hook
        rospy.on_shutdown(self.stop)

    
    def start(self):
        rospy.loginfo("Wall following behavior started.")
        self.sub = rospy.Subscriber("/scan", LaserScan, self.laser_callback)

    def calculate_laser_indices(self, msg):
        # get info about laser data
        #rospy.loginfo(f"laser angle_min: {msg.angle_min}")
        #rospy.loginfo(f"laser angle_max: {msg.angle_max}")
        #rospy.loginfo(f"laser beam number: {len(msg.ranges)}")
        #rospy.loginfo(f"laser angle increment: {msg.angle_increment}")

        # Calculate indices 
        self.num_ray =len(msg.ranges)
        self.front_index = int((0 - msg.angle_min) / msg.angle_increment)
        self.right_index = int(self.front_index / 2)
        #self.left_index = self.front_index + int((self.num_ray - self.front_index) / 2)



    def laser_callback(self, msg):

        if self.front_index is None:
            self.calculate_laser_indices(msg)

        # Use multiple laser readings to make decisions more reliable
        self.laser_front_list = msg.ranges[(self.front_index-3):(self.front_index+3)]  # Average the readings in front
        self.laser_right_list = msg.ranges[(self.right_index-3):(self.right_index+3)]   # Average the readings on the right

        # Calculate the average distances
        self.laser_front = sum(self.laser_front_list) / len(self.laser_front_list)
        self.laser_right = sum(self.laser_right_list) / len(self.laser_right_list)

        # Log distances for debugging
        rospy.loginfo(f"Front: {self.front_distance:.2f}, Right: {self.right_distance:.2f}")

        # # Control logic based on laser readings: use the values of the laser to decide the com_vel
        
        # reset the location to desire path 
        if self.laser_front <= 0.3:
            rospy.loginfo("avoid to hit the wall -> back")
            self.move_cmd.linear.x = -0.3
            self.move_cmd.angular.z = 0
        # desire path
        elif self.laser_front <= 0.5 and self.laser_right > 0.2:
            rospy.loginfo("Closed to the next wall -> turn fast to the left (moving forward at the same time).")
            self.move_cmd.linear.x = 0.1
            self.move_cmd.angular.z = 1.2
        elif self.laser_right > 0.3:
            rospy.loginfo("Away from the wall -> turn right")
            self.move_cmd.linear.x = 0.1
            self.move_cmd.angular.z = -0.2
        elif self.laser_right < 0.2:
            rospy.loginfo("Too close to the wall -> turn left")
            self.move_cmd.linear.x = 0.1
            self.move_cmd.angular.z = 0.2
        else:
            rospy.loginfo("within desired distance -> move forward")
            self.move_cmd.linear.x = 0.1
            self.move_cmd.angular.z = 0



    def move(self):
        rate = rospy.Rate(15)
        while not rospy.is_shutdown():
            self.pub.publish(self.move_cmd)
            rate.sleep()

    def stop(self):
        rospy.loginfo("Stopping the robot...")
        self.move_cmd.linear.x = 0
        self.move_cmd.angular.z = 0
        self.pub.publish(self.move_cmd)  # Publish the stop command

        
    def odom_monitor_thread(self, action_client, wall_follower):
        rate = rospy.Rate(1)
        while not rospy.is_shutdown():
            if action_client.is_goal_successful():
                rospy.loginfo("Goal has been completed for a lap, stop the robot...")
                wall_follower.stop()
                break
            rate.sleep()

        result = action_client.check_goal_status()
        rospy.loginfo(f"Recording finished with result: {result}")



if __name__ =='__main__':
    rospy.init_node("wall_follower_node", anonymous=True)
    wall_follow = WallFollower()

    record_odom_action_client = RecordOdomActionClient()
    find_wall_service_client = FindWallServiceClient()

    try:
        # Call the service before starting wall-following behavior
        wall_found = find_wall_service_client.send_request()
        rospy.loginfo(f"Wall found: {wall_found} ...")

        if wall_found:
            rospy.loginfo("Start to record odom ...")
            goal_sent = record_odom_action_client.send_goal()

            if goal_sent:
                while not record_odom_action_client.is_goal_active():
                    rospy.sleep(0.1)

                # Start monitoring odometry in a separate thread
                odom_thread = threading.Thread(target=wall_follow.odom_monitor_thread, args=(record_odom_action_client,))
                odom_thread.start()

                # Start wall-following behavior in the main thread

                rospy.loginfo("Recording odometry started successfully, starting wall-following behavior")
                wall_follow.start()
                wall_follow.move() 
                

            else:
                rospy.logerr("Failed to start odometry recording. Shutting down.")
                rospy.signal_shutdown("Failed to start odometry recording")

        else:
            rospy.logerr("Failed to find the wall. Shutting down.")
            rospy.signal_shutdown("Failed to find wall")
        

    except rospy.ROSInterruptException:
        wall_follow.stop()
    except Exception as e:
        rospy.logerr(f"An unexpected error occurred: {e}")
        wall_follow.stop()