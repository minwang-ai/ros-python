#!/usr/bin/env python

import rospy
from wall_following.srv import FindWall, FindWallResponse
from find_wall_movement import FindWallMove 


class FindWallServiceServer:
    def __init__(self):
        self.service_name = "/find_wall"
        
        self.find_wall = FindWallMove()
        self.service = rospy.Service(self.service_name, FindWall, self.callback)
        
        rospy.on_shutdown(self.find_wall.stop_robot)
        
        rospy.loginfo(f"Service {self.service_name} is ready")

    def callback(self, request):
        rospy.loginfo(f"Service {self.service_name} has been called")
        
        wall_found = self.find_wall.find_wall()

        rospy.loginfo(f"Finished service {self.service_name}.")
        
        return FindWallResponse(wallfound=wall_found)

    def spin(self):
        rospy.spin()

if __name__ == '__main__':
    rospy.init_node('find_wall_service_server')
    service_server = FindWallServiceServer()
    try:
        service_server.spin()
    except rospy.ROSInterruptException:
        pass  # Normal shutdown, no additional action needed because on_shutdown will handle stop_robot
    except Exception as e:
        rospy.logerr(f"An unexpected error occurred: {e}")
        service_server.find_wall.stop_robot()
