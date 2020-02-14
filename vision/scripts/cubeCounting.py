#!/usr/bin/env python2
# -*- coding: utf-8 -*-
"""
Created on Tue Jan 21 12:11:43 2020

@author: 8bhatia
"""

#import matplotlib.pyplot as plt
import numpy as np
import cv2

def rotateImage(image, angle):
    image_center = tuple(np.array(image.shape[1::-1]) / 2)
    rot_mat = cv2.getRotationMatrix2D(image_center, angle, 1.0)
    result = cv2.warpAffine(image, rot_mat, image.shape[1::-1], flags=cv2.INTER_LINEAR)
    return result


def preprocess(img):
    
    left_robot_img = rotateImage(img, 35).copy()
    left_robot_img = left_robot_img[730:950, 950:1300]
    
    right_robot_img = rotateImage(img, 117).copy()
    right_robot_img = right_robot_img[340:575, 800:1050]
    
    participant = rotateImage(img, -15).copy()
    participant = participant[320:575, 1190:1440].copy()
    return participant , left_robot_img , right_robot_img

#Place the cubes in distance of each other
#this algorithms works fine with shortly distant cubes and empty surface
#input: cropped surface, output: number of cubes
def count_cubes(image):
    image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
    Z = image.reshape((-1,3))

    # convert to np.float32
    Z = np.float32(Z)

    # define criteria, number of clusters(K) and apply kmeans()
    criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 10, 1.0)
    K = 7
    
    ret,label,center=cv2.kmeans(Z,K,None,criteria,10,cv2.KMEANS_RANDOM_CENTERS)
    # Now convert back into uint8, and make original image
    center = np.uint8(center)
    idx = np.where(center[:,0]==center[:,0].max())
    res = center[label.flatten()]
    res = res.reshape((image.shape))
    if center[idx][0,0] > 25:
        thr = res.copy()
        
        thr[np.where((thr!=center[idx]).any(axis=2))] = [0,0,0]
        thr = cv2.medianBlur(thr, 7)
                
        ret, labels = cv2.connectedComponents(thr[:,:,0])
        return 7 - (ret-1)
    else:
        return 7
    