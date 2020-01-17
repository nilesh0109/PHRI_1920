#!/usr/bin/env python

from vision1920.srv import CountResources, CountResourcesResponse
import rospy
import sys
def handle_resources_request():
    num_of_objects = 0
    #print s
    return CountResourcesResponse(num_of_objects)



def count_resources_server(robot_name=''):
    rospy.init_node('count_resources_allocated')
    s = rospy.Service(robot_name+'/count_objects', CountResources, handle_resources_request)
    #print s
    rospy.spin()



if __name__ == "__main__":
    if len(sys.argv)>1:
        count_resources_server(sys.argv[1])
    else:
        count_resources_server()
