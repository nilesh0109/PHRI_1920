#!/usr/bin/env python

import rospy
import smach
from smach_ros import ServiceState
from speech.srv import SpeechSynthesis
from nicopose.srv import Pose

class MakeUtterance(smach.Concurrence):
    """docstring for MakeUtterance."""

    def __init__(self, topic_name):
        super(MakeUtterance, self).__init__(
            input_keys=["speaker", "audio", "delay", "param"],
            outcomes=["utterance_done", "utterance_failed"],
            default_outcome="utterance_failed",
            # outcome_map={"utterance_done": {"SPEAK": "speech_done", "NVC": "nvc_done"}},
            outcome_map={"utterance_done": {"SPEAK": "succeeded"}},
        )
        # Open the container
        with self:
            smach.Concurrence.add("NVCA",
                                  ServiceState("/A/pose", Pose, request_slots=["param"]),
                                  )

            smach.Concurrence.add("NVCB",
                                  ServiceState("/B/pose", Pose, request_slots=["param"]),
                                  )

            smach.Concurrence.add("SPEAK",
                                  ServiceState(topic_name, SpeechSynthesis, request_slots=["audio", "speaker", "delay"]),
                                  )


