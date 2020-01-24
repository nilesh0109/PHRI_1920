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


def cube_detect(scene_num, participant_num=0):

    #take an image
#    full_image = take_image()
    full_image= cv2.imread('/informatik2/students/home/8bhatia/PHRI1920/vision-phri1920/cube_count/test.jpg')
#    cv2.imwrite("img5.jpg", full_image)
#    cv2.waitKey(0)
    P_img, left_robot_img, right_robot_img = preprocess(full_image)
    
    P_cubes = count_cubes(P_img)
    left_robot_cubes = count_cubes(left_robot_img)
    right_robot_cubes = count_cubes(right_robot_img)
    
    if participant_num != 0:
        response = save_images(scene_num, P_img, left_robot_img, right_robot_img, P_cubes, left_robot_cubes, right_robot_cubes, full_image)
        if response:
            return True
    else:
        return left_robot_cubes, right_robot_cubes


def save_images(scene_num, P_img, left_robot_img, right_robot_img, P_cubes, left_robot_cubes, right_robot_cubes, full_image):
    
    directory = "/informatik2/students/home/8bhatia/catkin_ws/test"

    scene_ = "scene_" + str(scene_num)
    if scene_num == 0:
        os.chdir(directory)
        p_nr = len(os.walk(directory).next()[1])
#        print p_nr
        if p_nr:
            participant_id = "participant_" + str(p_nr+1)
            os.mkdir(participant_id)
            os.chdir(participant_id)
            os.mkdir(scene_)
            os.chdir(scene_)
#            print os.getcwd()
#            print"------------------------------------------"
        else:
            participant_id = "participant_" + "1"
            os.mkdir(participant_id)
            os.chdir(participant_id)
            os.mkdir(scene_)
            os.chdir(scene_)
#            print os.getcwd()
#            print"------------------------------------------"
   
    else:
#        print os.getcwd()
        os.chdir("../../..")
#        print os.getcwd()
        direc = os.getcwd()
        directories = os.walk(direc).next()[1]
#        participant_id = os.listdir(directory)[-1:][0]
        participant_id = sorted(directories)
#        print participant_id
        participant_id = participant_id[-1]
        os.chdir(participant_id)
        os.mkdir(scene_)
        os.chdir(scene_)
        
    

    os.mkdir("images")
    os.chdir("images")
    
    img_name = participant_id + "_robotA_cubes_" + str(left_robot_cubes) + "_" + datetime.datetime.today().isoformat() + '.png'
    cv2.imwrite(img_name, left_robot_img)
    img_name = participant_id + "_robotB_cubes_" + str(right_robot_cubes) + "_" + datetime.datetime.today().isoformat() + '.png'
    cv2.imwrite(img_name, right_robot_img)
    img_name = participant_id + "_cubes_" + str(P_cubes) + "_" + datetime.datetime.today().isoformat() + '.png'
    cv2.imwrite(img_name, P_img)
    img_name = participant_id + "_FullImage_" + datetime.datetime.today().isoformat() + '.png'
    cv2.imwrite(img_name, full_image)
    
        
#    p_nr = len(os.walk(directory).next()[1])
#    if p_nr:
#        participant_id = directory + "participant_%s" % str(p_nr)
#        os.makedirs(participant_id)
#    else:
#        participant_id = directory + "participant_/%s" % str(1)
#        os.makedirs(participant_id)
#    
#    #if os.path.isdir(participant_id):
#     #   participant_id = participant_id + "_" + datetime.datetime.today().isoformat()
#        
#    #os.mkdir(participant_id)
#    os.chdir(participant_id)
#    os.mkdir("images")
#    os.chdir("images") 
#    img_name = participant_id + "_robotA_cubes_" + str(left_robot_cubes) + "_" + datetime.datetime.today().isoformat() + '.png'
#    cv2.imwrite(img_name, left_robot_img)
#    img_name = participant_id + "_robotB_cubes_" + str(right_robot_cubes) + "_" + datetime.datetime.today().isoformat() + '.png'
#    cv2.imwrite(img_name, right_robot_img)
#    img_name = participant_id + "_cubes_" + str(P_cubes) + "_" + datetime.datetime.today().isoformat() + '.png'
#    cv2.imwrite(img_name, P_img)
    
    return True


def take_image():
    cam_path = VideoDevice.get_all_devices()
    print cam_path
    for i in range(len(cam_path)):
        if cam_path[i][-1:] == "0":
            cam = cam_path[i]
            print cam
            cam = 0
            print cam
            break
        else:
            cam = 1
            print cam
        
    cam = cv2.VideoCapture(0)
    #print cam
    
    s, img = cam.read()
    #print s
    if s:    # frame captured without any errors
        #cv2.imshow("cam-test",img)
        time.sleep(0.1)
        #cv2.destroyWindow("cam-test")
        #cv2.imwrite("filename.jpg",img) #save image
        return img