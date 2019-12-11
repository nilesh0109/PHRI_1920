#!/usr/bin/env python

import time
import rospy
from nicomsg.msg import sff, empty, s


def move():
    pub = rospy.Publisher('/nico/motion/setAngle', sff, queue_size=10)
    dta = rospy.Publisher('/nico/motion/disableTorqueAll', empty, queue_size=10)
    oph = rospy.Publisher('/nico/motion/openHand', s, queue_size=10)
    rospy.init_node('mover', anonymous=True)
    rate = rospy.Rate(10)  # 10hz
    speed=0.07
    # move
    pub.publish('head_y' , 0, speed)
    pub.publish("r_wrist_x", -25 , speed)
    pub.publish("r_indexfinger_x", -150, speed)
    pub.publish("r_middlefingers_x", 150, speed +.02) # upper 180 lower -150
    pub.publish("r_thumb_z", 150, speed)
    pub.publish("r_thumb_x", 0, speed)
    pub.publish("r_shoulder_y", 50, speed)
    pub.publish("r_arm_x", -15, speed)
    pub.publish("r_elbow_y", -100 , speed)
    time.sleep(1.5)
    # back
    pub.publish("r_elbow_y", -90 , speed)
    time.sleep(.7)
    # forward
    pub.publish("r_elbow_y", -100 , speed)
    time.sleep(.7)
    # back
    pub.publish("r_elbow_y", -90 , speed)
    time.sleep(.7)
    pub.publish("r_shoulder_y", 0, speed)
    pub.publish("r_shoulder_z", 0, speed)
    pub.publish("r_arm_x", -15, speed)
    pub.publish("r_elbow_y", 0 , speed)
    pub.publish("r_wrist_x", -0 , speed)
    time.sleep(1)
    oph.publish('RHand')
    oph.publish('LHand')
    time.sleep(1)
    dta.publish()
    return


if __name__ == '__main__':
    try:
        move()
    except rospy.ROSInterruptException:
        pass
