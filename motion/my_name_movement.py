#!/usr/bin/env python

import time
import rospy
from nicomsg.msg import sff,empty

def move():
    pub = rospy.Publisher('/nico/motion/setAngle', sff, queue_size=10)
    dt = rospy.Publisher('/nico/motion/disableTorqueAll', empty, queue_size=10)
    rospy.init_node('mover', anonymous=True)
    rate = rospy.Rate(10)  # 10hz
    speed = 0.02
    pub.publish("r_shoulder_y", 50, speed)
    #pub.publish("r_shoulder_z", 50, speed)
    pub.publish("r_arm_x", -10, speed)
    pub.publish("r_elbow_y", -140 , speed)
    pub.publish("r_wrist_x", -25 , speed)
    time.sleep(5)
    pub.publish("r_shoulder_y", 0, speed)
    pub.publish("r_shoulder_z", 0, speed)
    pub.publish("r_arm_x", 0, speed)
    pub.publish("r_elbow_y", 0 , speed)
    pub.publish("r_wrist_x", -0 , speed)
    time.sleep(2)
    dt.publish()
    return


if __name__ == '__main__':
    try:
        move()
    except rospy.ROSInterruptException:
        pass
