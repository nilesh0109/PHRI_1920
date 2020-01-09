#!/usr/bin/env python

import rospy
from speech.msg import SpeechProgress

def callback(data):
    rospy.loginfo(rospy.get_caller_id() + "I heard %d %s %d", data.stamp.secs, data.sentence, data.progress)

def subscribe():
    rospy.init_node('subscriber_node')
    sub = rospy.Subscriber('speech_progress', SpeechProgress, callback)
    rospy.spin()

if __name__ == '__main__':
    subscribe()
