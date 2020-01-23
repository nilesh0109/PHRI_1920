#!/usr/bin/env python

import smach
import rospy


class NVC(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=["speech_done"], input_keys=["nvc_id"])

    def execute(self, userdata):
        rospy.loginfo("Executing state NVC")
        rospy.loginfo(userdata.nvc_id)
        return "nvc_done"
