#!/usr/bin/env python

import rospy
import smach
from smach_ros import ServiceState
from state_machine.srv import SpeechRecognition

from Speak import Speak
from NVC import NVC


class MakeUtterance(smach.Concurrence):
    """docstring for MakeUtterance."""

    def __init__(self):
        super(MakeUtterance, self).__init__(
            input_keys=["speaker", "audio", "delay", "nvc_id"],
            outcomes=["utterance_done", "utterance_failed"],
            default_outcome="utterance_failed",
            outcome_map={"utterance_done": {"SPEAK": "speech_done", "NVC": "nvc_done"}},
        )
        # Open the container
        with self:

            # smach.Concurrence.add(
            #     "NVC",
            #     ServiceState(
            #         "/question_recognition", SpeechRecognition, input_keys=["nvc_id"]
            #     ),
            #     transitions={"succeeded": "nvc_done"},
            # )
            #
            # smach.StateMachine.add(
            #     "SPEAK",
            #     ServiceState(
            #         "/question_recognition",
            #         SpeechRecognition,
            #         input_keys=["audio", "speaker", "delay"],
            #         response_slots=["question"],
            #     ),
            #     transitions={"succeeded": "speech_done"},
            # )
            smach.Concurrence.add("SPEAK", Speak())
            smach.Concurrence.add("NVC", NVC())


def main():
    rospy.init_node("smach_example_state_machine")

    # Create the top level SMACH state machine
    sm = smach.StateMachine(outcomes=["done"])
    sm.userdata.speaker = "a"
    sm.userdata.audio = "Hello_World"
    sm.userdata.delay = 0
    sm.userdata.nvc_id = 0

    with sm:
        smach.StateMachine.add(
            "UT_ONE",
            MakeUtterance(),
            transitions={"utterance_done": "UT_TWO", "utterance_failed": "UT_ONE"},
        )
        smach.StateMachine.add(
            "UT_TWO",
            MakeUtterance(),
            transitions={"utterance_done": "done", "utterance_failed": "UT_TWO"},
        )
    # Execute SMACH plan
    outcome = sm.execute()


if __name__ == "__main__":
    main()
