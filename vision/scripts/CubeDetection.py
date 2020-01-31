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
from os.path import dirname, abspath
import rospy

def cube_detect(scene_num, participant_num=0):

    #take an image
    full_image = take_image()
#    full_image= cv2.imread('/informatik2/students/home/8bhatia/PHRI1920/vision-phri1920/cube_count/participant_8_FullImage_2020-01-30T12:14:00.445908.png')
    
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
    
    directory = dirname(abspath(__file__)) + "/../../../data"
    
    if not os.path.exists(directory):
        os.mkdir(directory)
        rospy.loginfo("Created directory Data")

    scene_ = "scene_" + str(scene_num)
    if scene_num == 0:
        os.chdir(directory)
        p_nr = len(os.walk(directory).next()[1])
#        print p_nr
        if p_nr:
            participant_id = "participant_" + str(p_nr+1)
            os.mkdir(participant_id)
            rospy.loginfo("Created directory %s" %(participant_id))
            os.chdir(participant_id)
            os.mkdir(scene_)
            rospy.loginfo("Created directory %s" %(scene_))
            os.chdir(scene_)

        else:
            participant_id = "participant_" + "1"
            os.mkdir(participant_id)
            rospy.loginfo("Created directory %s" %(participant_id))
            os.chdir(participant_id)
            os.mkdir(scene_)
            rospy.loginfo("Created directory %s" %(scene_))
            os.chdir(scene_)
  
    else:
#        print os.getcwd()
        os.chdir(directory)
#        print os.getcwd()
        direc = os.getcwd()
        directories = os.walk(direc).next()[1]
#        participant_id = os.listdir(directory)[-1:][0]
        participant_id = sorted(directories)
#        print participant_id
        participant_id = participant_id[-1]
        os.chdir(participant_id)
        os.mkdir(scene_)
        rospy.loginfo("Created directory %s" %(scene_))
        os.chdir(scene_)
        
    

    os.mkdir("images")
    rospy.loginfo("Created directory images")
    os.chdir("images")
    
    img_name = participant_id + "_robotA_cubes_" + str(left_robot_cubes) + "_" + datetime.datetime.today().isoformat() + '.png'
    cv2.imwrite(img_name, left_robot_img)
    img_name = participant_id + "_robotB_cubes_" + str(right_robot_cubes) + "_" + datetime.datetime.today().isoformat() + '.png'
    cv2.imwrite(img_name, right_robot_img)
    img_name = participant_id + "_cubes_" + str(P_cubes) + "_" + datetime.datetime.today().isoformat() + '.png'
    cv2.imwrite(img_name, P_img)
    img_name = participant_id + "_FullImage_" + datetime.datetime.today().isoformat() + '.png'
    cv2.imwrite(img_name, full_image)
    
    rospy.loginfo("saved images")
    
    return True


def take_image():
    cam_path = VideoDevice.get_all_devices()
#    print cam_path
    cam_resolution = [2304,1536]
    
    for i in range(len(cam_path)):
        if cam_path[i][-1:] == "0":
            cam = cam_path[i]
#            print cam
            cam = 0
#            print cam
            break
        else:
            cam = 1
        
    cam = cv2.VideoCapture(0)
    #print cam
    cam.set(3, cam_resolution[0])
    cam.set(4, cam_resolution[1])
#    print cam.get(3)
#    print cam.get(4)
    
    time.sleep(0.1)
    s, img = cam.read()

    if s:    # frame captured without any errors
#        cv2.imshow("cam-test",img)
#        cv2.waitKey(0)
        time.sleep(0.1)
        return img