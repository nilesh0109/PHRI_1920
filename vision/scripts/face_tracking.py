#!/usr/bin/env python2
# -*- coding: utf-8 -*-
"""
Created on Thu Jan 16 11:03:07 2020

@author: 8bhatia
"""

import argparse
import logging
import sys
import time
from os.path import abspath, dirname

from nicoemotionrecognition import EmotionRecognition
#from nicoface.FaceExpression import faceExpression
from nicomotion import Motion
from nicovision.VideoDevice import VideoDevice
import rospy

def get_devices():
    nico_eyes = VideoDevice.get_all_devices()
    print nico_eyes[1]
    return nico_eyes[1]




if __name__ == "__main__":
    get_devices()