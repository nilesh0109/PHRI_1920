#!/usr/bin/env python

import rospy
import smach
from smach_ros import ServiceState
from speech.srv import SpeechSynthesis
from video.srv import PlayVideo

from Speak import Speak
from NVC import NVC

class PlayVideoWithSound(smach.Concurrence):
    """docstring for MakeUtterance."""

    def __init__(self):
        super(PlayVideoWithSound, self).__init__(
            input_keys=["speaker", "audio", "delay", "scene_number"],
            outcomes=["video_done"],
            default_outcome="video_done",
            outcome_map={"video_done": {"PLAY_SOUND": "succeeded", "PLAY_VIDEO": "succeeded"}},
        )
        # Open the container
        with self:

            smach.Concurrence.add(
                "PLAY_VIDEO",
                ServiceState("/play_video", PlayVideo, request_slots=["scene_number"]),
            )

            smach.Concurrence.add(
                "PLAY_SOUND",
                ServiceState("S/speech_synthesis",
                             SpeechSynthesis,
                             request_slots=["audio", "speaker", "delay"]),
            )


