#!/usr/bin/env python

import argparse
import rospy
import smach
from smach_ros import ServiceState
from state_machine.srv import SpeechRecognition
from states import SceneFlow, SayResponses, Speak

# main
def main():

    parser = argparse.ArgumentParser(description="Experiment state machine.")
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
    sm.userdata.scene = "scene1"
    sm.userdata.question = ""
    sm.userdata.qa_once = False

    # Open the container
    with sm:
        # Add states to the container
        smach.StateMachine.add(
            "SCENE",
            SceneFlow.SceneFlow(),
            transitions={
                "say_robot_line": "SPEAK",
                "say_ship_line": "SPEAK",
                "participant_input": "GET_QUESTION",
                "ressource_allocation": "SCENE",
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

        # Callback for service response
        # def add_two_result_cb(userdata, response):
        #    rospy.loginfo("Result: %i" % response.sum)
        #    return 'succeeded'

        # ServiceState(.., response_cb=add_two_result_cb, ..)

    # Execute SMACH plan
    outcome = sm.execute()


if __name__ == "__main__":
    main()
