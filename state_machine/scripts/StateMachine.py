#!/usr/bin/env python

import argparse
import rospy
import smach
from smach_ros import ServiceState
from speech.srv import (
    SpeechRecognition,
    SpeechSynthesis,
    SpeechSynthesisRequest,
    CubeCounting,
)
from vision.srv import CountResources, CheckEmpty
from lighting.srv import LightControl
from states import (
    SceneFlow,
    SayResponses,
    MakeUtterance,
    RecognitionFallback,
    PlayVideo,
    VisionFallback,
    EvaluateCubes,
    CountCubes
)

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
    sm.userdata.light_setting = "blackout"
    sm.userdata.A_image_path = ""
    sm.userdata.B_image_path = ""
    sm.userdata.A_cubes = 999
    sm.userdata.B_cubes = 999

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
                "lift_off": "GET_QUESTION",
                "return_cubes": "RETURN_CUBES",
                "play_video": "PLAY_VIDEO",
                "set_lights": "SET_LIGHTS",
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
            "SOUND",
            ServiceState(
                "S/speech_synthesis",
                SpeechSynthesis,
                request=SpeechSynthesisRequest("video/beep_quieter", "S", 0.0),
            ),
            transitions={"succeeded": "GET_QUESTION"},
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
            "GET_QUESTION_FALLBACK",
            RecognitionFallback.RecognitionFallback(),
            transitions={"manual_answer": "RESOLVE_QUESTION"},
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
                "lift_off_confirmation": "SCENE",
                "recognition_fallback": "GET_QUESTION_FALLBACK",
            },
            remapping={"question": "sentence"},
        )

        smach.StateMachine.add(
            "CUBE_COUNT",
            CountCubes.CountCubes(),
            transitions={
                "counting_done": "CUBE_FEEDBACK",  # NOTE change back to "CONFIRM_CUBES" to reenable gui
                "counting_failed": "CONFIRM_CUBES"
            },
        )

        smach.StateMachine.add(
            "CONFIRM_CUBES",
            VisionFallback.VisionFallback(),
            transitions={"confirmed": "CUBE_FEEDBACK"},
        )

        smach.StateMachine.add(
            "CUBE_FEEDBACK",
            EvaluateCubes.EvaluateCubes(),
            transitions={"evaluation_done": "SCENE", "evaluation_failed": "SCENE"},
            remapping={
                "scene_id": "scene_number",
                "num_cubes_A": "A_cubes",
                "num_cubes_B": "B_cubes",
            },
        )

        smach.StateMachine.add(
            "RETURN_CUBES",
            ServiceState("/check_empty", CheckEmpty, request_slots=["scene_number"]),
            transitions={"succeeded": "SCENE"},
        )

        smach.StateMachine.add(
            "PLAY_VIDEO",
            PlayVideo.PlayVideoWithSound(),
            transitions={"video_done": "SCENE"},
        )

        smach.StateMachine.add(
            "SET_LIGHTS",
            ServiceState("/light_control", LightControl, request_slots=["setting"]),
            transitions={"succeeded": "SCENE"},
            remapping={"setting": "light_setting"},
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

    # Execute SMACH plan
    outcome = sm.execute()

    rospy.loginfo("StateMachine ended with outcome %s", outcome)


if __name__ == "__main__":
    main()
