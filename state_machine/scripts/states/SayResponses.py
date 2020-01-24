import smach
import rospy
from os.path import dirname, abspath
import yaml


class SayResponses(smach.State):
    def __init__(self):
        smach.State.__init__(
            self,
            outcomes=[
                "say_robot_line_a",
                "say_robot_line_b",
                "say_ship_line",
                "participant_input",
                "questions_done",
                "done_confirmation",
            ],
            input_keys=["question", "last_ship_line"],
            output_keys=["speaker", "audio", "delay", "param"],
        )
        self.repeat_once = False
        self.answer_index = 0
        with open(dirname(abspath(__file__)) + "/questions.yml", "r") as f:
            self.question_answers = yaml.safe_load(f)

    def execute(self, userdata):
        rospy.loginfo("Executing state RESOLVE_QUESTION")
        if userdata.question == "done_confirmation":
            self.answer_index = 0
            return "done_confirmation"
        answers = self.question_answers[userdata.question]
        if userdata.question == "timeout" or userdata.question == "repetition_request":
            self.repeat_once = True
        if self.answer_index < len(answers):
            answer = answers[self.answer_index]
            userdata.speaker = answer["speaker"]
            userdata.audio = (
                userdata.last_ship_line
                if userdata.question == "repetition_request"
                else answer["audio"]
            )
            userdata.param = (
                userdata.last_ship_line
                if userdata.question == "repetition_request"
                else answer["audio"]
            )
            userdata.delay = answer["delay"]
            self.answer_index += 1
            if answer["speaker"] == "S":
                return "say_ship_line"
            elif answer["speaker"] == "A":
                return "say_robot_line_a"
            else:
                return "say_robot_line_b"
        else:
            self.answer_index = 0
            if self.repeat_once:
                self.repeat_once = False
                return "participant_input"
            else:
                return "questions_done"
