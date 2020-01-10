import smach
import rospy
from os.path import dirname, abspath
import yaml


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
