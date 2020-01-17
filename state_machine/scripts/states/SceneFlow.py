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
            output_keys=["speaker", "audio", "scene", "qa_once", "delay"],
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
                if self.scene_index < len(self.scenes):
                    userdata.scene = self.scenes[self.scene_index]
            else:
                return "scenes_finished"
        event = self.script[self.scenes[self.scene_index]][self.dialog]["event"]
        outcome = "unknown_event"
        if event == "utterance":
            # print(self.script[self.scenes[self.scene_index]][self.dialog]["parameters"]["speaker"])
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
                userdata.delay = self.script[self.scenes[self.scene_index]][
                    self.dialog
                ]["delay"]
                self.dialog += 1
                if speaker == "s":
                    return "say_ship_line"
                elif speaker == "a":
                    return "say_robot_line_a"
                else:
                    return "say_robot_line_b"
        elif event == "allocation":
            outcome = "ressource_allocation"
        else:  # unknown event
            rospy.loginfo("Encountered unknown event %s", event)
        self.dialog += 1
        return outcome

    def load_dialog_script(self, scene_file):
        with open(dirname(abspath(__file__)) + "/" + scene_file, "r") as f:
            self.script = yaml.safe_load(f)
