#!/usr/bin/env python2
# -*- coding: utf-8 -*-
"""
Created on Fri Jan 17 16:17:23 2020

@author: 8bhatia
"""

import cv2
import os
import datetime 
from nicovision.VideoDevice import VideoDevice
import time
from cubeCounting import preprocess, count_cubes


def cube_detect(participant_num=0):

    #take an image
    full_image = take_image()
    P_img, left_robot_img, right_robot_img = preprocess(full_image)
    
    P_cubes = count_cubes(P_img)
    left_robot_cubes = count_cubes(left_robot_img)
    right_robot_cubes = count_cubes(right_robot_img)
    
    if participant_num != 0:
        response = save_images(P_img, left_robot_img, right_robot_img, P_cubes, left_robot_cubes, right_robot_cubes)
        if response:
            return True
    else:
        return left_robot_cubes, right_robot_cubes


def save_images(P_img, left_robot_img, right_robot_img, P_cubes, left_robot_cubes, right_robot_cubes):
    #participant_id = "Participant_" + str(participant_num) #+ "_" + datetime.datetime.today().isoformat()
    
    directory = "./"
    p_nr = len(os.listdir(directory))
    if p_nr:
        participant_id = directory + "participant_%s" % str(p_nr)
        os.makedirs(participant_id)
    else:
        participant_id = directory + "participant_/%s" % str(1)
        os.makedirs(participant_id)
    
    #if os.path.isdir(participant_id):
     #   participant_id = participant_id + "_" + datetime.datetime.today().isoformat()
        
    #os.mkdir(participant_id)
    os.chdir(participant_id)
    os.mkdir("images")
    os.chdir("images") 
    img_name = participant_id + "_robotA_cubes_" + str(left_robot_cubes) + "_" + datetime.datetime.today().isoformat() + '.png'
    cv2.imwrite(img_name, left_robot_img)
    img_name = participant_id + "_robotB_cubes_" + str(right_robot_cubes) + "_" + datetime.datetime.today().isoformat() + '.png'
    cv2.imwrite(img_name, right_robot_img)
    img_name = participant_id + "_cubes_" + str(P_cubes) + "_" + datetime.datetime.today().isoformat() + '.png'
    cv2.imwrite(img_name, P_img)
    
    return True


def take_image():
    cam_path = VideoDevice.get_all_devices()
#    print cam_path
    for i in range(len(cam_path)):
        if cam_path[i][-1:] == "0":
            cam = cam_path[i]
#            print cam
            cam = 0
        else:
            cam = 1
        
    cam = cv2.VideoCapture(cam)
    #print cam
    
    s, img = cam.read()
    #print s
    if s:    # frame captured without any errors
        #cv2.imshow("cam-test",img)
        time.sleep(0.1)
        #cv2.destroyWindow("cam-test")
        #cv2.imwrite("filename.jpg",img) #save image
        return img