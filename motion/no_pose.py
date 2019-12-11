#!/usr/bin/env python

# USAGE: Use script with yes or no as parameter
# python yesno.py yes
# python yesno.py no

# license removed for brevity
import time

import rospy
from nicomsg.msg import sff


def talker():
    pub = rospy.Publisher('/nico/motion/setAngle', sff, queue_size=10)
    rospy.init_node('talker', anonymous=True)
    rate = rospy.Rate(10)  # 10hz
    position = -20
    speed = 0.08
    pub.publish("head_z", 0, speed)
    pub.publish("head_y", 0, speed)
    time.sleep(.5)
    for i in range(2):
        position = position * -1
        pub.publish("head_z", position, speed)
        time.sleep(.6)
    pub.publish("head_z", 0, speed)
    pub.publish("head_y", 0, speed)


if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
