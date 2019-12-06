#!/usr/bin/env python

import time
import rospy
from nicomsg.msg import sff

def move():
    pub = rospy.Publisher('/nico/motion/setAngle', sff, queue_size=10)
    rospy.init_node('mover', anonymous=True)
    rate = rospy.Rate(10)  # 10hz

    pub.publish("r_arm_x", 90, 0.1)

    pub.publish("r_wrist_x", -90 , 0.1)
    pub.publish("r_indexfingers_x", 40,.1)
    pub.publish("r_middlefingers_x", -20., .1)
    pub.publish("r_thumb_x", -60., .1)
    time.sleep(6)

    pub.publish("r_arm_x", 0, 0.1)
    pub.publish("r_elbow_y", 0, 0.1)
    pub.publish("r_shoulder_y", 0, 0.1)
    pub.publish("r_shoulder_z", 0, 0.1)
    pub.publish("r_indexfingers_x", 0,.1)
    pub.publish("r_middlefingers_x", 0., .1)
    pub.publish("r_thumb_x", 0, .1)
    time.sleep(1)
pass

if __name__ == '__main__':
    try:
        move()
    except rospy.ROSInterruptException:
        pass
