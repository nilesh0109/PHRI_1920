#!/usr/bin/env python2
# -*- coding: utf-8 -*-
"""
Created on Tue Jan 14 15:13:41 2020

@author: 8bhatia
"""

from vision1920.srv import CountResources
import rospy
import sys

def count_resources(s):
    rospy.wait_for_service('count_objects')
    try:
        count_objects = rospy.ServiceProxy('count_objects', CountResources)
        print "in client 1"
        print(count_objects(s))
        #return resp1.sum
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e
    



if __name__ == "__main__":
    s = sys.argv[1]
    rospy.init_node('vision1920', anonymous=True)
    print(count_resources(s))
    print "in client 2"
