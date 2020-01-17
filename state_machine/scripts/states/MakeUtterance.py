#!/usr/bin/env python

import rospy
import smach
from smach_ros import ServiceState
from speech.srv import SpeechSynthesis

from Speak import Speak
from NVC import NVC


class MakeUtterance(smach.Concurrence):
    """docstring for MakeUtterance."""

    def __init__(self):
        super(MakeUtterance, self).__init__(
            input_keys=["speaker", "audio", "delay", "nvc_id"],
            outcomes=["utterance_done", "utterance_failed"],
            default_outcome="utterance_failed",
            # outcome_map={"utterance_done": {"SPEAK": "speech_done", "NVC": "nvc_done"}},
            outcome_map={"utterance_done": {"SPEAK": "succeeded"}},
        )
        # Open the container
        with self:

            # smach.Concurrence.add(
            #     "NVC",
            #     ServiceState(
            #         "/question_recognition", SpeechSynthesis, input_keys=["nvc_id"]
            #     ),
            #     transitions={"succeeded": "nvc_done"},
            # )
            #

             smach.Concurrence.add(
                 "SPEAK",
                 ServiceState(
                     "/speech_synthesis",
                     SpeechSynthesis,
                     request_slots=["audio", "delay"],
                 ),
             )
            # smach.Concurrence.add("SPEAK2", Speak())
            # smach.Concurrence.add("NVC", NVC())

