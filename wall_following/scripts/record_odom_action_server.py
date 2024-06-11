#!/usr/bin/env python
import rospy
import actionlib
from wall_following.msg import RecordOdomFeedback, RecordOdomResult, RecordOdomAction
from odom_recorder import OdomRecorder

class OdomRecordActionServer:
    def __init__(self):
        # Initialize the action server
        self.name = "record_odom"
        self._as = actionlib.SimpleActionServer(self.name, RecordOdomAction, self.goal_callback, False)
        self._feedback = RecordOdomFeedback()  
        self._result = RecordOdomResult()      

        self._as.start()
        self._recorder = OdomRecorder()
        self.rate = rospy.Rate(1)  # Record odometry at 1 Hz
        rospy.loginfo(f"Action server {self.name} has been initialized")

    def goal_callback(self, goal):
        rospy.loginfo(f"Action server {self.name} has been called")
        
        self._recorder.start_recording()
        success = True

        while not rospy.is_shutdown():
            if self._as.is_preempt_requested():
                rospy.loginfo(f"Action server {self.name} is preempted")
                self._as.set_preempted()
                success = False
                self._recorder.stop_recording()
                return

            self._feedback.current_total = self._recorder.get_total_distance()
            self._as.publish_feedback(self._feedback)

            # Check if the robot has completed a lap
            if self._recorder.has_completed_lap():
                rospy.loginfo("Lap completed, need to stop the robot")
                break

            self.rate.sleep()
        
        
        if success:
            rospy.loginfo(f" Calling action server {self.name} is completed")
            
            self._recorder.stop_recording()
            self._result.list_of_odoms = self._recorder.get_odometry_list()
            self._as.set_succeeded(self._result)

if __name__ == '__main__':
    rospy.init_node('record_odom_action_server_node')
    server = OdomRecordActionServer()
    rospy.spin()
