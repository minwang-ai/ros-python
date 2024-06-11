#!/usr/bin/env python

import rospy
import actionlib
from wall_following.msg import RecordOdomAction, RecordOdomGoal, RecordOdomFeedback, RecordOdomResult
from actionlib_msgs.msg import GoalStatus

class RecordOdomActionClient:
    def __init__(self):
        self.action_server_name = 'record_odom'
        self.create_action_client()

    def create_action_client(self):
        self.client = actionlib.SimpleActionClient(self.action_server_name, RecordOdomAction)
        rospy.loginfo('Waiting for action Server'+ self.action_server_name + "to start")
        self.client.wait_for_server()
        rospy.loginfo('Action Server '+ self.action_server_name + "started")

    def feedback_callback(self, feedback):
        # The feedback callback will be called whenever feedback is received.
        rospy.loginfo(f'[Feedback] the robot has moved so far: {feedback.current_total} meters.' )

    def send_goal(self):
        # Create and send the goal
        goal = RecordOdomGoal()
        self.client.send_goal(goal, feedback_cb = self.feedback_callback)
        rospy.loginfo("Goal sent to action server.")
        return self.is_goal_active()
        
    def is_goal_active(self):

        state_result = self.client.get_state()

        rate = rospy.Rate(10)

        while state_result == GoalStatus.PENDING:
            rate.sleep()
            state_result = self.client.get_state()
            rospy.loginfo("Goal is currently Pending")
        if state_result in [GoalStatus.ACTIVE]:
            rospy.loginfo("Goal is now active!")
            return True
        else:    
            return False

        
    def is_goal_successful(self):

        state_result = self.client.get_state()

        rate = rospy.Rate(10)

        if state_result == GoalStatus.SUCCEEDED:
            rospy.loginfo("Goal is completed successfully")
            return True
        else:    
            return False

    def check_goal_status(self):
        state_result = self.client.get_state()
        rospy.loginfo("[Result] State: " + str(state_result))
        if state_result == GoalStatus.SUCCEEDED:
            rospy.loginfo("Goal succeeded!")
            return True
        elif state_result == GoalStatus.PREEMPTING:
            rospy.loginfo("Goal has been preempted")
            return False
        elif state_result == GoalStatus.ABORTED:
            rospy.logerr("Something went wrong in the Server Side")
            return False
        elif state_result == GoalStatus.REJECTED:
            rospy.logwarn("The goal was rejected by the Server")
            return False


        

if __name__ == '__main__':
    try:
        rospy.init_node('record_odom_action_client_node')
        client = RecordOdomActionClient()
        client.send_goal()
    except rospy.ROSInterruptException:
        pass
