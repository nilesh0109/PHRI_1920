#!/usr/bin/env python

import rospy
from speech.msg import SpeechProgress

def progress():
    pub = rospy.Publisher('speech_progress', SpeechProgress, queue_size=25)
    rospy.init_node('speech_progress_node')
    rate = rospy.Rate(1.0)
    i=1
    while not rospy.is_shutdown():
        t = rospy.Time.now()
        default_msg = SpeechProgress(t, 'Hello NICO',i%100)
        rospy.loginfo('Hello NICO %s' % t)
        pub.publish(default_msg)
        rate.sleep()
        i+=1

if __name__ == '__main__':
    try:
        progress()
    except rospy.ROSInterruptException:
        print('exception')
        pass
