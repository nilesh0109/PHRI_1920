#!/usr/bin/env python

import time
import rospy
from nicomsg.msg import sff,empty, s

def move():
    pub = rospy.Publisher('/nico/motion/setAngle', sff, queue_size=10)
    dt = rospy.Publisher('/nico/motion/disableTorqueAll', empty, queue_size=10)
    oph = rospy.Publisher('/nico/motion/openHand', s, queue_size=10)
    face = rospy.Publisher('/nico/faceExpression', s, queue_size=10)
    rospy.init_node('mover', anonymous=True)
    rate = rospy.Rate(10)  # 10hz
    speed = 0.05
    oph.publish('RHand')
    oph.publish('LHand')
    face.publish("happiness")
    pub.publish("l_shoulder_y", -100, speed)
    pub.publish("l_arm_x", 10, speed)
    pub.publish("l_elbow_y", 130 , speed)
    pub.publish("l_wrist_z", 120 , speed)
    time.sleep(3)
    pub.publish("l_shoulder_y", 0, speed)
    pub.publish("l_shoulder_z", 0, speed)
    pub.publish("l_arm_x", 10, speed)
    pub.publish("l_elbow_y", 0 , speed)
    pub.publish("l_wrist_z", 0 , speed)
    face.publish("neutral")
    time.sleep(2)
    dt.publish()
    return

if __name__ == '__main__':
    try:
        move()
    except rospy.ROSInterruptException:
        pass
