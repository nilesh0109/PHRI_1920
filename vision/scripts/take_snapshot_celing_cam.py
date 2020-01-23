#!/usr/bin/env python2
# -*- coding: utf-8 -*-
"""
Created on Tue Jan 21 12:25:07 2020

@author: 8bhatia
"""

from nicovision.VideoDevice import VideoDevice
import cv2
import datetime
import time
from cubeCounting import preprocess, count_cubes
import os
import numpy as np

def take_image():
    cam_path = VideoDevice.get_all_devices()

    for i in range(len(cam_path)):
        if cam_path[i][-1:] == "0":
            cam = cam_path[i]
            print cam
            cam = 1
        else:
            cam = 0
        
    cam = cv2.VideoCapture(cam)
    print cam
    
    s, img = cam.read()
    print s
    if s:    # frame captured without any errors
        #cv2.namedWindow("cam-test",CV_WINDOW_AUTOSIZE)
        #cv2.imshow("cam-test",img)
        time.sleep(0.1)
        #cv2.destroyWindow("cam-test")
        cv2.imwrite("filename.jpg",img) #save image
        return img
    
    
im = take_image()
cv2.imshow("test",im)
cv2.waitKey(0)

#def cube_detect(participant_num=0):
#
#    #take an image
##    full_image = take_image() #cv2.imread('vision_data/Webcam/1.jpg')
#    full_image = cv2.imread('vision_data/Webcam/1.jpg')
##    cv2.imshow("img", full_image)
##    cv2.waitKey(0)
#    P_img, left_robot_img, right_robot_img = preprocess(full_image)
#    
#    P_cubes = count_cubes(P_img)
#    left_robot_cubes = count_cubes(left_robot_img)
#    right_robot_cubes = count_cubes(right_robot_img)
#    
#    if participant_num != 0:
#        response = save_images(participant_num, P_img, left_robot_img, right_robot_img, P_cubes, left_robot_cubes, right_robot_cubes)
#        if response:
#            return True
#    else:
#        return left_robot_cubes, right_robot_cubes
#
#
#def save_images(participant_num, P_img, left_robot_img, right_robot_img, P_cubes, left_robot_cubes, right_robot_cubes):
#
#    participant_id = "Participant_" + str(participant_num) #+ "_" + datetime.datetime.today().isoformat()
#    #print participant_id
#    #try:    
#    if os.path.isdir(participant_id):
#        participant_id = participant_id + "_" + datetime.datetime.today().isoformat()
#        
#    os.mkdir(participant_id)
#    os.chdir(participant_id)
#    os.mkdir("images")
#    os.chdir("images") 
#    img_name = participant_id + "_robotA_cubes_" + str(left_robot_cubes) + "_" + datetime.datetime.today().isoformat() + '.png'
#    cv2.imwrite(img_name, left_robot_img)
#    img_name = participant_id + "_robotB_cubes_" + str(right_robot_cubes) + "_" + datetime.datetime.today().isoformat() + '.png'
#    cv2.imwrite(img_name, right_robot_img)
#    img_name = participant_id + "_cubes_" + str(P_cubes) + "_" + datetime.datetime.today().isoformat() + '.png'
#    cv2.imwrite(img_name, P_img)
#    
#    return True
##    except:
##        print "in exc"
##        return False
#
#
#def take_image():
#    cam_path = VideoDevice.get_all_devices()
#
#    for i in range(len(cam_path)):
#        if cam_path[i][-1:] == "0":
#            cam = cam_path[i]
#            print cam
#            cam = 1
#        else:
#            cam = 0
#        
#    cam = cv2.VideoCapture(cam)
#    #print cam
#    
#    s, img = cam.read()
#    #print s
#    if s:    # frame captured without any errors
#        #cv2.imshow("cam-test",img)
#        time.sleep(0.1)
#        #cv2.destroyWindow("cam-test")
#        #cv2.imwrite("filename.jpg",img) #save image
#        return img
#
#
#def check_table_empty():
#    table1 = []
#    table2 = []
#    start = time.time()
#    while True:
#        object1, object2 = cube_detect()
#        table1.append(object1)
#        table2.append(object2)
#        print table1, table2
#        time.sleep(0.2)
#        
#        if len(table1) > 3:
#            print np.average(table1[-3:])
#            if np.average(table1[-3:]) == 0 and np.average(table2[-3:]) == 0:
#                break
#        if time.time() - start > 60:
#            break
#    return True
#
##print cube_detect(1)
#
#check_table_empty()
#        