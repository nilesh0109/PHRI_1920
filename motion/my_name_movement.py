#!/usr/bin/env python

import time
import rospy
from nicomsg.msg import sff

def move():
    pub = rospy.Publisher('/nico/motion/setAngle', sff, queue_size=10)
    rospy.init_node('mover', anonymous=True)
    rate = rospy.Rate(10)  # 10hz
    pub.publish("r_shoulder_y", 50, 0.3)
    #pub.publish("r_shoulder_z", 50, 0.1)
    pub.publish("r_arm_x", 10, 0.3)
    pub.publish("r_elbow_y", -140 , 0.3)
    pub.publish("r_wrist_x", -25 , 0.3)
    time.sleep(5)
    pub.publish("r_shoulder_y", 0, 0.1)
    pub.publish("r_shoulder_z", 0, 0.1)
    pub.publish("r_arm_x", 0, 0.1)
    pub.publish("r_elbow_y", 0 , 0.1)
    pub.publish("r_wrist_x", -0 , 0.1)
    time.sleep(1)
    return


if __name__ == '__main__':
    try:
        move()
    except rospy.ROSInterruptException:
        pass
