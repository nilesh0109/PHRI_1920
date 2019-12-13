#!/usr/bin/env python2
# -*- coding: utf-8 -*-
"""
Created on Thu Dec 12 09:24:46 2019

@author: motion team 

Defining poses and modifying gestures of nico
"""

import time
import rospy
from nicomsg.msg import sff, empty, s

class Poses:
    ''' Required modules'''
    seta = None
    dt = None
    oph = None
    face = None

    ''' init: sets up publishers and inits node '''
    def __init__(self):
        self.seta = rospy.Publisher('/nico/motion/setAngle', sff, queue_size=30)
        self.dt = rospy.Publisher('/nico/motion/disableTorqueAll', empty, queue_size=2)
        self.oph = rospy.Publisher('/nico/motion/openHand', s, queue_size=2)
        self.face = rospy.Publisher('/nico/faceExpression/sendFaceExpression', s, queue_size=30)
        rospy.init_node('pose', anonymous=True)
        return

    ''' Moves head to indicate no '''
    def no(self, speed = 0.08):
        #rate = rospy.Rate(10)  # 10hz
        position = -20
        #speed = 0.08
        self.seta.publish('head_z', 0, speed)
        self.seta.publish('head_y', 0, speed)
        time.sleep(.5)
        for i in range(2):
            position = position * -1
            self.seta.publish('head_z', position, speed)
            time.sleep(.6)
        self.seta.publish('head_z', 0, speed)
        self.seta.publish('head_y', 0, speed)
        self.relax()
        return

    ''' Brings hand up and smiles to greet '''
    def introduce(self, speed = 0.05):
        self.oph.publish('RHand')
        self.oph.publish('LHand')
        self.face.publish('happiness')
        self.seta.publish('l_shoulder_y', -100, speed)
        self.seta.publish('l_arm_x', 10, speed)
        self.seta.publish('l_elbow_y', 130 , speed)
        self.seta.publish('l_wrist_z', 120 , speed)
        time.sleep(3)
        self.seta.publish('l_shoulder_y', 0, speed)
        self.seta.publish('l_shoulder_z', 0, speed)
        self.seta.publish('l_arm_x', 10, speed)
        self.seta.publish('l_elbow_y', 0 , speed)
        self.seta.publish('l_wrist_z', 0 , speed)
        self.face.publish('neutral')
        time.sleep(2)
        self.relax()
        return

    ''' Open hand and arm gesture '''
    def show_path(self, speed = 0.05):
        self.seta.publish('head_y' , 0, speed)
        self.seta.publish('r_shoulder_z', 0, speed)
        self.seta.publish('r_shoulder_y', 0, speed)
        self.seta.publish('r_wrist_x', 0 , speed)
        self.seta.publish('r_arm_x', -10, speed)
        self.seta.publish('r_elbow_y', -25, speed)
        self.oph.publish('RHand')
        time.sleep(.2)
        # Step one
        self.seta.publish('r_elbow_y', -25, speed)
        self.seta.publish('r_wrist_z', 90 , speed+.02)
        self.seta.publish('r_shoulder_y', 30, speed)
        self.seta.publish('r_arm_x', -40, speed)
        time.sleep(.2)
        # Step two - top
        self.seta.publish('r_elbow_y', -25, speed)
        self.seta.publish('r_wrist_z', 120 , speed+.02)
        self.seta.publish('r_shoulder_y', 70, speed)
        self.seta.publish('r_arm_x', -100, speed)
        time.sleep(3)
        # repeat step one - come back
        self.seta.publish('r_elbow_y', -25, speed)
        self.seta.publish('r_wrist_z', 90 , speed+.02)
        self.seta.publish('r_shoulder_y', 30, speed)
        self.seta.publish('r_arm_x', -40, speed)
        time.sleep(.4)
        # Step three - relax
        self.seta.publish('r_wrist_z', 0 , speed+.02)
        self.seta.publish('r_elbow_y', 0, speed)
        self.seta.publish('r_arm_x', -10, speed)
        self.seta.publish('r_shoulder_y', 0, speed)
        time.sleep(1)
        self.relax()
        return

    ''' moves finger to indicate a speech of belief '''
    def believe(self, speed = 0.07):
        # move forward
        self.seta.publish('head_y' , 0, speed)
        self.seta.publish('r_wrist_x', -25 , speed)
        self.seta.publish('r_indexfinger_x', -150, speed)
        self.seta.publish('r_middlefingers_x', 150, speed +.02) # upper 180 lower -150
        self.seta.publish('r_thumb_z', 150, speed)
        self.seta.publish('r_thumb_x', 0, speed)
        self.seta.publish('r_shoulder_y', 50, speed)
        self.seta.publish('r_arm_x', -15, speed)
        self.seta.publish('r_elbow_y', -100 , speed)
        time.sleep(1.5)
        # back
        self.seta.publish('r_elbow_y', -90 , speed)
        time.sleep(.7)
        # forward
        self.seta.publish('r_elbow_y', -100 , speed)
        time.sleep(.7)
        # back
        self.seta.publish('r_elbow_y', -90 , speed)
        time.sleep(.7)
        self.seta.publish('r_shoulder_y', 0, speed)
        self.seta.publish('r_shoulder_z', 0, speed)
        self.seta.publish('r_arm_x', -15, speed)
        self.seta.publish('r_elbow_y', 0 , speed)
        self.seta.publish('r_wrist_x', -0 , speed)
        time.sleep(1)
        self.relax()
        return

    ''' relax the torque and open hands after move'''
    def relax(self):
        self.oph.publish('RHand')
        self.oph.publish('LHand')
        time.sleep(1)
        self.dta.publish()
        return
