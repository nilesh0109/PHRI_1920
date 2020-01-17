#!/usr/bin/env python

import argparse
import rospy
import smach
from smach_ros import ServiceState
from state_machine.srv import SpeechRecognition
from states import SceneFlow, SayResponses, Speak, MakeUtterance

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
    sm.userdata.delay = 0
    sm.userdata.nvc_id = 0

    # Open the container
    with sm:
        # Add states to the container
        smach.StateMachine.add(
            "SCENE",
            SceneFlow.SceneFlow(),
            transitions={
                "say_robot_line_a": "SPEAKA",
                "say_robot_line_b": "SPEAKB",
                "say_ship_line": "SPEAKS",
                "participant_input": "GET_QUESTION",
                "ressource_allocation": "SCENE",
                "scenes_finished": "experiment_ended",
            },
        )
        smach.StateMachine.add(
            "SPEAKA", MakeUtterance.MakeUtterance("/speech_synthesis"), transitions={"utterance_done": "SCENE", "utterance_failed": "SCENE"}
        )

        smach.StateMachine.add(
            "SPEAKB", MakeUtterance.MakeUtterance("/speech_synthesis"), transitions={"utterance_done": "SCENE", "utterance_failed": "SCENE"}
        )

        smach.StateMachine.add(
            "SPEAKS", MakeUtterance.MakeUtterance("/speech_synthesis"), transitions={"utterance_done": "SCENE", "utterance_failed": "SCENE"}
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
                "say_robot_line_a": "ANSWERA",
                "say_robot_line_b": "ANSWERB",
                "say_ship_line": "ANSWERS",
                "participant_input": "GET_QUESTION",
                "questions_done": "SCENE",
            },
        )

        smach.StateMachine.add(
            "ANSWERA", MakeUtterance.MakeUtterance("/speech_synthesis"), transitions={"utterance_done": "RESOLVE_QUESTION", "utterance_failed": "SCENE"}
        )

        smach.StateMachine.add(
            "ANSWERB", MakeUtterance.MakeUtterance("/speech_synthesis"), transitions={"utterance_done": "RESOLVE_QUESTION", "utterance_failed": "SCENE"}
        )

        smach.StateMachine.add(
            "ANSWERS", MakeUtterance.MakeUtterance("/speech_synthesis"), transitions={"utterance_done": "RESOLVE_QUESTION", "utterance_failed": "SCENE"}
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
