#!/usr/bin/env python

from vision1920.srv import CountResources, CountResourcesResponse
import rospy
import sys
from CubeDetection import cube_detect
import time
import numpy as np

def handle_resources_request(participant_num):
    #num_of_objects = 0
    success = True
    #success = cube_detect(participant_num)
    if success:
        return CountResourcesResponse(success)


def check_table_empty():
    table1 = []
    table2 = []
#    while True:
#        object1, object2 = cube_detect()
#        table1.append(object1)
#        table2.append(object2)
#        time.sleep(0.2)
#        
#        if len(table1) >= 3:
#            if np.average(table1[-3:]) == 0 and np.average(table2[-3:]) == 0:
#                break
#            return CountResourcesResponse(True)
    return CountResourcesResponse(True)
    

def count_resources_server(robot_name=''):
    rospy.init_node('count_resources_allocated')
    s = rospy.Service(robot_name+'/count_objects', CountResources, handle_resources_request)
    e = rospy.Service(robot_name+'/check_empty', CountResources, check_table_empty)
    #print s
    rospy.spin()



if __name__ == "__main__":
    if len(sys.argv)>1:
        count_resources_server(sys.argv[1])
    else:
        count_resources_server()
