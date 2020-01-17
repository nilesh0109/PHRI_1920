#!/usr/bin/env python2
# -*- coding: utf-8 -*-
"""
Created on Fri Jan 17 16:17:23 2020

@author: 8bhatia
"""

import cv2
import os
import datetime 

def cube_detect(participant_num=0):
    
    #take an image
    
    #crop
    img1[img1 < 225] = 0 #left
    img2[img2 < 225] = 0 #right
  
    params = cv2.SimpleBlobDetector_Params()

    params.filterByArea = True
    params.minArea = 200
    params.maxArea = 1000
    blob_detector = cv2.SimpleBlobDetector_create(params)

    kp1 = blob_detector.detect(img1)
    kp2 = blob_detector.detect(img2)
    if participant_num != 0:
        response = save_images(participant_num, img1, img2, len(kp1), len(kp2))
        if response:
            return True
    else:
        return len(kp1), len(kp2)


def save_images(participant_num, img1, img2, objects1, objects2):
    participant_id = "Participant_"+participant_num
    os.mkdir(participant_id)
    os.chdir(participant_id)
    os.mkdir("images")
    os.chdir("images") 
    img_name = participant_id + "robotA_" + objects1 + datetime.datetime.today().isoformat() + '.png'
    cv2.imwrite(img_name, img1)
    img_name = participant_id + "robotB_" + objects2 + datetime.datetime.today().isoformat() + '.png'
    cv2.imwrite(img_name, img2)
    
    return True

