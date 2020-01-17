#!/usr/bin/env python

import argparse
import rospy
import smach
from smach_ros import ServiceState
from speech.srv import SpeechRecognition, SpeechRecognitionRequest
from vision.srv import CountResources
from states import SceneFlow, SayResponses, Speak

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
    sm.userdata.scene = "scene_0"  # TODO adjust assignment for dynamic scene entry
    sm.userdata.sentence = ""
    sm.userdata.qa_once = False
    sm.userdata.no_of_objects = 69420

    # Open the container
    with sm:
        # Add states to the container
        smach.StateMachine.add(
            "SCENE",
            SceneFlow.SceneFlow(args.scene_file, args.scene, args.entry),
            transitions={
                "say_robot_line": "SPEAK",
                "say_ship_line": "SPEAK",
                "participant_input": "GET_QUESTION",
                "ressource_allocation": "WAIT_FOR_CUBES",
                "unknown_event": "aborted",
                "scenes_finished": "experiment_ended",
            },
        )
        smach.StateMachine.add(
            "SPEAK", Speak.Speak(), transitions={"speech_done": "SCENE"}
        )

        smach.StateMachine.add(
            "SPEAK2", Speak.Speak(), transitions={"speech_done": "RESOLVE_QUESTION"}
        )

        smach.StateMachine.add(
            "GET_QUESTION",
            ServiceState(
                "/question_recognition", SpeechRecognition, response_slots=["question"]
            ),
            transitions={"succeeded": "RESOLVE_QUESTION"},
        )

        smach.StateMachine.add(
            "RESOLVE_QUESTION",
            SayResponses.SayResponses(),
            transitions={
                "say_robot_line": "SPEAK2",
                "say_ship_line": "SPEAK2",
                "participant_input": "GET_QUESTION",
                "questions_done": "SCENE",
            },
        )

        smach.StateMachine.add(
            "WAIT_FOR_CUBES",
            ServiceState(
                "/speech_recognition",
                SpeechRecognition,
                request=SpeechRecognitionRequest("done"),
                response_slots=["sentence"],
            ),
            transitions={"succeeded": "CUBE_COUNT"},
        )

        smach.StateMachine.add(
            "CUBE_COUNT",
            ServiceState(
                "/count_objects", CountResources, response_slots=["no_of_objects"]
            ),
            transitions={"succeeded": "SCENE"},
        )
        # Callback for service response
        # def add_two_result_cb(userdata, response):
        #    rospy.loginfo("Result: %i" % response.sum)
        #    return 'succeeded'

        # ServiceState(.., response_cb=add_two_result_cb, ..)

    # Execute SMACH plan
    outcome = sm.execute()

    rospy.loginfo("nof_cubes = %i", sm.userdata.no_of_objects)
    rospy.loginfo("recognized sentence = %s", sm.userdata.sentence)
    rospy.loginfo("StateMachine ended with outcome %s", outcome)


if __name__ == "__main__":
    main()
