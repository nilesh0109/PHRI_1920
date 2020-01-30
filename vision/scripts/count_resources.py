#!/usr/bin/env python

from vision.srv import CountResources, CountResourcesResponse #, CheckEmpty, CheckEmptyResponse
import rospy
import sys
from CubeDetection import cube_detect
import time
import numpy as np

def handle_resources_request(req):
    success = False
    scene_num = req.scene_number
    while success == False:
        success = cube_detect(scene_num, 1) 
        
        if success:
            return CountResourcesResponse(success)
            
        

def check_table_empty(req):
    table1 = []
    table2 = []
    start = time.time()
    timeout = 30
    scene_num = req.scene_number
    
    rospy.loginfo("Automatic timeout of %d seconds started" % (timeout))
    
    while True:         
        object1, object2 = cube_detect(scene_num, 0) #checks empty table
        table1.append(object1)
        table2.append(object2)
        time.sleep(0.2)
#        print table1, table2
        if len(table1) > 3:
            if np.average(table1[-3:]) == 0 and np.average(table2[-3:]) == 0:
                break
            
        if time.time() - start > timeout:
            break
    rospy.loginfo("Table is now empty")    
    return CountResourcesResponse(True)
    

def count_resources_server(robot_name=''):
    rospy.init_node('count_resources_allocated')

    s = rospy.Service(robot_name+'/count_objects', CountResources, handle_resources_request)
    
    e = rospy.Service(robot_name+'/check_empty', CountResources, check_table_empty)

    rospy.loginfo("VISION services launched")

    rospy.spin()



if __name__ == "__main__":
    if len(sys.argv)>1:
        count_resources_server(sys.argv[1])
    else:
        count_resources_server()
