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
    speed=0.05
    pub.publish('head_y' , 0, speed)
    pub.publish('r_shoulder_z', 0, speed)
    pub.publish('r_shoulder_y', 0, speed)
    pub.publish('r_wrist_x', 0 , speed)
    pub.publish('r_arm_x', -10, speed)
    pub.publish('r_elbow_y', -25, speed)
    oph.publish('RHand')
    time.sleep(.2)
    # Step one
    pub.publish('r_elbow_y', -25, speed)
    pub.publish('r_wrist_z', 90 , speed+.02)
    pub.publish('r_shoulder_y', 30, speed)
    pub.publish('r_arm_x', -40, speed)
    time.sleep(.2)
    # Step two
    pub.publish('r_elbow_y', -25, speed)
    pub.publish('r_wrist_z', 120 , speed+.02)
    pub.publish('r_shoulder_y', 70, speed)
    pub.publish('r_arm_x', -100, speed)
    time.sleep(3)
    # repeat step one
    pub.publish('r_elbow_y', -25, speed)
    pub.publish('r_wrist_z', 90 , speed+.02)
    pub.publish('r_shoulder_y', 30, speed)
    pub.publish('r_arm_x', -40, speed)
    time.sleep(.4)
    # Step three
    pub.publish('r_wrist_z', 0 , speed+.02)
    pub.publish('r_elbow_y', 0, speed)
    pub.publish('r_arm_x', -10, speed)
    pub.publish('r_shoulder_y', 0, speed)
    time.sleep(1)
    dta.publish()
pass

if __name__ == '__main__':
    try:
        move()
    except rospy.ROSInterruptException:
        pass
