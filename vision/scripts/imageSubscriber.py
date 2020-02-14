#!/usr/bin/env python2
# -*- coding: utf-8 -*-
"""
Created on Thu Feb 13 17:49:51 2020

@author: 8bhatia
"""

import rospy

from sensor_msgs.msg import Image

from cv_bridge import CvBridge, CvBridgeError
import cv2
import numpy as np

def callback(image):
    rospy.loginfo(rospy.get_caller_id() + " - We got the image")

#    img = image.data
    print image
    try:
        cv_image = br.imgmsg_to_cv2(image, 'bgr8')
        #cv_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
        cv2.imshow("Image window", cv_image)
        cv2.waitKey(3)
        input()
    except CvBridgeError as e:
        print e


br = CvBridge()

rospy.init_node('viewer', anonymous=False)
rospy.Subscriber('/A/send_image', Image, callback)
rospy.spin()
