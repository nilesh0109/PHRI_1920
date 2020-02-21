#!/usr/bin/env python

import rospy
import smach
from smach_ros import ServiceState
from speech.srv import SpeechSynthesis, SpeechSynthesisRequest
from vision.srv import CountResources
import datetime


class CountCubes(smach.Concurrence):
    """docstring for CubeFeedback."""

    def __init__(self):
        super(CountCubes, self).__init__(
            input_keys=["scene_number"],
            output_keys=["A_image_path", "B_image_path", "A_cubes", "B_cubes"],
            outcomes=["counting_done", "counting_failed"],
            default_outcome="counting_failed",
            # outcome_map={"utterance_done": {"SPEAK": "speech_done", "NVC": "nvc_done"}},
            outcome_map={"counting_done": {"SPEAK": "succeeded", "CUBE_COUNT": "succeeded"}},
        )
        # Open the container
        with self:
            smach.Concurrence.add("SPEAK",
                                  ServiceState("/S/speech_synthesis", SpeechSynthesis, request=SpeechSynthesisRequest("scene_0_S_line_1", "S", 0)),
                                  )

            smach.Concurrence.add(
                "CUBE_COUNT",
                ServiceState(
                    "/count_objects",
                    CountResources,
                    request_slots=["scene_number"],
                    response_slots=["A_image_path", "B_image_path", "A_cubes", "B_cubes"],
                )
            )


