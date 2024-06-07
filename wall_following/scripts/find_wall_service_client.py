#!/usr/bin/env python

import rospy
from wall_following.srv import FindWall, FindWallRequest
import time

class FindWallServiceClient:
    def __init__(self):
        self.service_name = '/find_wall'

        # Wait for the service client to be running
        rospy.wait_for_service(self.service_name)

        # Create the connection object to the service
        self.service = rospy.ServiceProxy(self.service_name, FindWall)

        # Create a MsgRequest object
        self.request = FindWallRequest()

    def send_request(self):
        time.sleep(2)  # Add a delay to ensure the service server is ready

        try:
            # Send the MsgRequest object through the connection object to be executed by the robot
            result = self.service(self.request)
            # Print the result given by the service called
            rospy.loginfo(f"service call result: wallfound {result}")
            return result.wallfound
        except rospy.ServiceException as e:
            rospy.logerr(f"Service call failed: {e}")
            return False


if __name__ == '__main__':
    rospy.init_node('find_wall_service_client', anonymous=True)
    service_client = FindWallServiceClient()
    try:
        service_client.send_request()
    except rospy.ROSInterruptException:
        pass
