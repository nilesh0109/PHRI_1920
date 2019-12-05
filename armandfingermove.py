#!/usr/bin/env python

import time
import rospy
from nicomsg.msg import sff

def move():
    pub = rospy.Publisher('/nico/motion/setAngle', sff, queue_size=10)
    rospy.init_node('mover', anonymous=True)
    rate = rospy.Rate(10)  # 10hz
    position = -20
    position = position * -1
    #pub.publish("r_arm_x", 50 + position, 0.1)
    pub.publish("r_shoulder_y", 25, 0.1)
    pub.publish("r_shoulder_z", 25, 0.1)
    pub.publish("r_arm_x", 25, 0.1)
    pub.publish("r_elbow_y", -100 , 0.1)
    time.sleep(4)
    pub.publish("r_arm_x", 0, 0.1)
    pub.publish("r_elbow_y", 0, 0.1)
    pub.publish("r_shoulder_y", 0, 0.1)
    pub.publish("r_shoulder_z", 0, 0.1)
    time.sleep(1)
pass

if __name__ == '__main__':
    try:
        move()
    except rospy.ROSInterruptException:
        pass
