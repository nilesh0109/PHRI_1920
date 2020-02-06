#!/usr/bin/env python

import smach
import rospy


class Speak(smach.State):
    def __init__(self):
        smach.State.__init__(
            self, outcomes=["speech_done"], input_keys=["speaker", "audio", "delay"]
        )

    def execute(self, userdata):
        rospy.loginfo("Executing state SPEAK")
        # rospy.sleep(userdata.delay)
        rospy.loginfo(userdata.speaker)
        rospy.loginfo(userdata.audio)
        return "speech_done"
