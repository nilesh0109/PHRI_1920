#!/usr/bin/env python

import argparse
import rospy
import smach
from smach_ros import ServiceState
from speech.srv import SpeechRecognition
from video.srv import PlayVideo
from vision.srv import CountResources
from states import SceneFlow, SayResponses, MakeUtterance

# main
def main():

    parser = argparse.ArgumentParser(description="Experiment state machine.")
    parser.add_argument(
        "--scene_file",
        type=str,
        default="scene.yml",
        help="YAML file which contains the event order",
    )
    parser.add_argument("--scene", type=int, default=0, help="Index of the start scene")
    parser.add_argument(
        "--entry", type=int, default=0, help="Script entry in scene to start at"
    )

    args = parser.parse_args()

    rospy.init_node("smach_example_state_machine")

    # Create a SMACH state machine
    sm = smach.StateMachine(outcomes=["experiment_ended", "preempted", "aborted"])

    sm.userdata.speaker = ""
    sm.userdata.audio = ""
    sm.userdata.param = ""
    sm.userdata.last_ship_line = ""
    sm.userdata.scene = "scene_0"
    sm.userdata.scene_number = 0
    sm.userdata.sentence = ""
    sm.userdata.qa_once = False
    sm.userdata.delay = 0

    # Open the container
    with sm:
        # Add states to the container
        smach.StateMachine.add(
            "SCENE",
            SceneFlow.SceneFlow(args.scene_file, args.scene, args.entry),
            transitions={
                "say_robot_line_a": "SPEAKA",
                "say_robot_line_b": "SPEAKB",
                "say_ship_line": "SPEAKS",
                "participant_input": "GET_QUESTION",
                "ressource_allocation": "GET_QUESTION",
                "return_cubes": "RETURN_CUBES",
                "play_video": "PLAY_VIDEO",
                "pause": "SCENE",
                "unknown_event": "aborted",
                "scenes_finished": "experiment_ended",
            },
        )
        smach.StateMachine.add(
            "SPEAKA",
            MakeUtterance.MakeUtterance("/A/speech_synthesis"),
            transitions={"utterance_done": "SCENE", "utterance_failed": "SCENE"},
        )

        smach.StateMachine.add(
            "SPEAKB",
            MakeUtterance.MakeUtterance("/B/speech_synthesis"),
            transitions={"utterance_done": "SCENE", "utterance_failed": "SCENE"},
        )

        smach.StateMachine.add(
            "SPEAKS",
            MakeUtterance.MakeUtterance("/S/speech_synthesis"),
            transitions={"utterance_done": "SCENE", "utterance_failed": "SCENE"},
        )

        smach.StateMachine.add(
            "GET_QUESTION",
            ServiceState(
                "/speech_recognition",
                SpeechRecognition,
                request_slots=["context"],
                response_slots=["sentence"],
            ),
            transitions={"succeeded": "RESOLVE_QUESTION"},
            remapping={"context": "scene"},
        )

        smach.StateMachine.add(
            "RESOLVE_QUESTION",
            SayResponses.SayResponses(),
            transitions={
                "say_robot_line_a": "ANSWERA",
                "say_robot_line_b": "ANSWERB",
                "say_ship_line": "ANSWERS",
                "participant_input": "GET_QUESTION",
                "questions_done": "SCENE",
                "done_confirmation": "CUBE_COUNT",
            },
            remapping={"question": "sentence"},
        )

        smach.StateMachine.add(
            "CUBE_COUNT",
            ServiceState(
                "/count_objects", CountResources, request_slots=["scene_number"]
            ),
            transitions={"succeeded": "SCENE"},
        )

        smach.StateMachine.add(
            "RETURN_CUBES",
            ServiceState(
                "/check_empty", CountResources, request_slots=["scene_number"]
            ),
            transitions={"succeeded": "SCENE"},
        )

        smach.StateMachine.add(
            "PLAY_VIDEO",
            ServiceState("/play_video", PlayVideo, request_slots=["scene_number"]),
            transitions={"succeeded": "SCENE"},
        )

        smach.StateMachine.add(
            "ANSWERA",
            MakeUtterance.MakeUtterance("/A/speech_synthesis"),
            transitions={
                "utterance_done": "RESOLVE_QUESTION",
                "utterance_failed": "RESOLVE_QUESTION",
            },
        )

        smach.StateMachine.add(
            "ANSWERB",
            MakeUtterance.MakeUtterance("/B/speech_synthesis"),
            transitions={
                "utterance_done": "RESOLVE_QUESTION",
                "utterance_failed": "RESOLVE_QUESTION",
            },
        )

        smach.StateMachine.add(
            "ANSWERS",
            MakeUtterance.MakeUtterance("/S/speech_synthesis"),
            transitions={
                "utterance_done": "RESOLVE_QUESTION",
                "utterance_failed": "RESOLVE_QUESTION",
            },
        )
        # Callback for service response
        # def add_two_result_cb(userdata, response):
        #    rospy.loginfo("Result: %i" % response.sum)
        #    return 'succeeded'

        # ServiceState(.., response_cb=add_two_result_cb, ..)

    # Execute SMACH plan
    outcome = sm.execute()

    rospy.loginfo("StateMachine ended with outcome %s", outcome)


if __name__ == "__main__":
    main()
