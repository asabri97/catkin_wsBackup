#! /usr/bin/env python

import rospkg
import rospy
from grip_srv_msg_pkg.srv import MyCustomServiceMessage, MyCustomServiceMessageResponse # you import the service message python classes generated from Empty.srv.

rospy.init_node('grip_srv_msg_client') # Initialise a ROS node with the name service_client
rospy.wait_for_service('/my_service') # Wait for the service client /move_bb8_in_circle to be running
grip_service_client = rospy.ServiceProxy('/my_service', MyCustomServiceMessage) # Create the connection to the service
grip_request_object = MyCustomServiceMessageRequest() # Create an object of type EmptyRequest

result = grip_service_client(grip_request_object) # Send through the connection the path to the trajectory file to be executed
print result # Print the result given by the service called