import smach
import rospy
from os.path import dirname, abspath
import yaml


class SceneFlow(smach.State):
    def __init__(self, scene_file="scene.yml", scene=0, dialog=0):
        smach.State.__init__(
            self,
            outcomes=[
                "say_robot_line_a",
                "say_robot_line_b",
                "say_ship_line",
                "participant_input",
                "ressource_allocation",
                "unknown_event",
                "scenes_finished",
            ],
            output_keys=[
                "speaker",
                "audio",
                "scene",
                "qa_once",
                "delay",
                "last_ship_line",
            ],
        )
        self.scene_index = scene
        self.dialog = dialog
        self.load_dialog_script(scene_file)
        self.scenes = sorted(self.script.keys())

    def execute(self, userdata):
        rospy.loginfo("Executing state SCENE")
        if self.dialog == len(self.script[self.scenes[self.scene_index]]):
            if self.scene_index < (len(self.scenes) - 1):
                self.scene_index += 1
                self.dialog = 0
            else:
                return "scenes_finished"
        event = self.script[self.scenes[self.scene_index]][self.dialog]["event"]
        outcome = "unknown_event"
        if event == "utterance":
            # print(self.script[self.scenes[self.scene_index]][self.dialog]["parameters"]["speaker"])
            speaker = self.script[self.scenes[self.scene_index]][self.dialog]["speaker"]
            audio = self.script[self.scenes[self.scene_index]][self.dialog]["audio"]
            userdata.speaker = speaker
            userdata.audio = audio
            userdata.delay = self.script[self.scenes[self.scene_index]][self.dialog][
                "delay"
            ]
            if speaker == "s":
                userdata.last_ship_line = audio
                outcome = "say_ship_line"
            elif speaker == "a":
                outcome = "say_robot_line_a"
            else:
                outcome = "say_robot_line_b"
        elif event == "allocation":
            outcome = "ressource_allocation"
        elif event == "question":
            userdata.scene = self.scenes[self.scene_index]
            outcome = "participant_input"
        else:  # unknown event
            rospy.loginfo("Encountered unknown event %s", event)
        self.dialog += 1
        return outcome

    def load_dialog_script(self, scene_file):
        with open(dirname(abspath(__file__)) + "/" + scene_file, "r") as f:
            self.script = yaml.safe_load(f)
