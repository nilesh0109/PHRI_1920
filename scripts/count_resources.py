#!/usr/bin/env python

from vision1920.srv import CountResources, CountResourcesResponse
import rospy

def handle_resources_request(s):
    num_of_objects = 0
    print s
    return CountResourcesResponse(num_of_objects)



def count_resources_server():
    rospy.init_node('count_resources_allocated')
    s = rospy.Service('count_objects', CountResources, handle_resources_request)
    #print s
    rospy.spin()



if __name__ == "__main__":
    count_resources_server()
