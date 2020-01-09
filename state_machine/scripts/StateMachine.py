#!/usr/bin/env python

import argparse
import rospy
import smach
import yaml
from os.path import dirname, abspath
from smach_ros import ServiceState
from state_machine.srv import SpeechRecognition, SpeechRecognitionResponse


class SceneFlow(smach.State):
    def __init__(self):
        smach.State.__init__(
            self,
            outcomes=[
                "say_robot_line",
                "say_ship_line",
                "participant_input",
                "ressource_allocation",
                "scenes_finished",
            ],
            output_keys=["speaker", "audio", "scene", "qa_once"],
        )
        self.scene_index = 0
        self.load_dialog_script()
        self.scenes = sorted(self.script.keys())

    def execute(self, userdata):
        rospy.loginfo("Executing state SCENE")
        if self.dialog < len(self.script[self.scenes[self.scene_index]]):
            speaker = self.script[self.scenes[self.scene_index]][self.dialog]["speaker"]
            if speaker == "p":
                userdata.qa_once = self.script[self.scenes[self.scene_index]][
                    self.dialog
                ]["once"]
                self.dialog += 1
                return "participant_input"
            else:
                userdata.speaker = speaker
                userdata.audio = self.script[self.scenes[self.scene_index]][
                    self.dialog
                ]["audio"]
                self.dialog += 1
                if speaker == "s":
                    return "say_ship_line"
                else:
                    return "say_robot_line"
        elif self.scene_index < (len(self.scenes) - 1):
            self.scene_index += 1
            self.dialog = 0
            if self.scene_index < len(self.scenes):
                userdata.scene = self.scenes[self.scene_index]
                # self.load_dialog_script()
            return "ressource_allocation"
        else:
            return "scenes_finished"

    def load_dialog_script(self):
        self.dialog = 0
        with open(dirname(abspath(__file__)) + "/scene.yml", "r") as f:
            self.script = yaml.safe_load(f)


class Speak(smach.State):
    def __init__(self):
        smach.State.__init__(
            self, outcomes=["speech_done"], input_keys=["speaker", "audio"]
        )

    def execute(self, userdata):
        rospy.loginfo("Executing state SPEAK")
        rospy.loginfo(userdata.speaker)
        rospy.loginfo(userdata.audio)
        return "speech_done"


class SayResponses(smach.State):
    def __init__(self):
        smach.State.__init__(
            self,
            outcomes=[
                "say_robot_line",
                "say_ship_line",
                "participant_input",
                "questions_done",
            ],
            input_keys=["question", "scene", "qa_once"],
            output_keys=["speaker", "audio"],
        )
        self.repeat_once = False
        self.answer_index = 0
        with open(dirname(abspath(__file__)) + "/questions.yml", "r") as f:
            self.question_answers = yaml.safe_load(f)

    def execute(self, userdata):
        rospy.loginfo("Executing state RESOLVE_QUESTION")
        answers = self.question_answers[userdata.question]
        if userdata.scene in answers:
            answers = answers[userdata.scene]
        elif userdata.question == "repeat":
            self.repeat_once = True
        if self.answer_index < len(answers):
            answer = answers[self.answer_index]
            userdata.speaker = answer["speaker"]
            userdata.audio = answer["audio"]
            self.answer_index += 1
            if answer["speaker"] == "s":
                return "say_ship_line"
            else:
                return "say_robot_line"
        else:
            self.answer_index = 0
            if self.repeat_once:
                self.repeat_once = False
                return "participant_input"
            elif not userdata.qa_once and userdata.question != "cancel":
                return "participant_input"
            else:
                return "questions_done"


# main
def main():

    parser = argparse.ArgumentParser(description="Experiment state machine.")
    parser.add_argument("--scene", type=int, default=0, help="Index of the start scene")

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
            SceneFlow(),
            transitions={
                "say_robot_line": "SPEAK",
                "say_ship_line": "SPEAK",
                "participant_input": "GET_QUESTION",
                "ressource_allocation": "SCENE",
                "scenes_finished": "experiment_ended",
            },
        )
        smach.StateMachine.add("SPEAK", Speak(), transitions={"speech_done": "SCENE"})

        smach.StateMachine.add(
            "SPEAK2", Speak(), transitions={"speech_done": "RESOLVE_QUESTION"}
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
            SayResponses(),
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
