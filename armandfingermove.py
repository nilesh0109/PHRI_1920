#!/usr/bin/env python

import time
import rospy
from nicomsg.msg import sff

'''
# "I believe" movement

pub.publish("r_shoulder_y", 50, 0.1)
pub.publish("r_shoulder_z", 30, 0.1)
pub.publish("r_arm_x", 25, 0.1)
pub.publish("r_elbow_y", -140 , 0.1)
'''

'''
# "Me" movement
pub.publish("r_shoulder_y", 10, 0.1)
pub.publish("r_shoulder_z", 30, 0.1)
pub.publish("r_arm_x", 25, 0.1)
pub.publish("r_elbow_y", -140 , 0.1)
pub.publish("r_wrist_x", -90 , 0.1)
pub.publish("r_indexfingers_x", -40,.1)#-170., .1)
pub.publish("r_middlefingers_x", -40., .1)
pub.publish("r_indexfinger_x", -40,.1)
pub.publish("r_thumb_z", 150, .1)
pub.publish("r_thumb_x", -40, .1)
'''

def move():
    pub = rospy.Publisher('/nico/motion/setAngle', sff, queue_size=10)
    rospy.init_node('mover', anonymous=True)
    rate = rospy.Rate(10)  # 10hz
    # arm
    pub.publish("r_shoulder_y", 10, 0.1)
    pub.publish("r_shoulder_z", 30, 0.1)
    pub.publish("r_arm_x", 25, 0.1)
    pub.publish("r_elbow_y", -140 , 0.1)
    #pub.publish("r_wrist_x", -90 , 0.1)
    #fingers
    pub.publish("r_indexfingers_x", -40,.1)#-170., .1)
    #pub.publish("r_middlefingers_x", -40., .1)
    #pub.publish("r_indexfinger_x", -40,.1)
    #pub.publish("r_thumb_z", 150, .1)
    pub.publish("r_thumb_x", -40, .1)
    #wait
    time.sleep(4)
    #return to normal
    pub.publish("r_indexfingers_x", 0., .1)
    pub.publish("r_thumb_x", 0., .1)
    pub.publish("r_shoulder_y", 0, 0.1)
    pub.publish("r_shoulder_z", 0, 0.1)
    pub.publish("r_arm_x", 0, 0.1)
    pub.publish("r_elbow_y", -0 , 0.1)
    time.sleep(1)
    return


if __name__ == '__main__':
    try:
        move()
    except rospy.ROSInterruptException:
        pass
