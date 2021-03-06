import smach
import rospy
from os.path import dirname, abspath
import yaml
import random


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
                "return_cubes",
                "play_video",
                "pause",
                "lift_off",
                "set_lights",
                "unknown_event",
                "scenes_finished",
            ],
            output_keys=[
                "speaker",
                "audio",
                "scene",
                "scene_number",
                "qa_once",
                "delay",
                "last_ship_line",
                "param",
                "light_setting",
            ],
        )
        self.scene_index = scene
        self.dialog = dialog
        self.load_dialog_script(scene_file)
        self.scenes = sorted(self.script.keys())
        self.reverse_order = random.random() < 0.5
        self.robot_offset = -1

    def execute(self, userdata):
        rospy.loginfo("Executing state SCENE")
        if self.dialog == len(self.script[self.scenes[self.scene_index]]):
            if self.scene_index < (len(self.scenes) - 1):
                self.scene_index += 1
                self.dialog = 0
            else:
                return "scenes_finished"
        event = self.script[self.scenes[self.scene_index]][self.dialog]["event"]
        rospy.loginfo("Scene: %i, Entry: %i, Event: %s", self.scene_index, self.dialog, event)
        outcome = "unknown_event"
        if event == "utterance":
            # print(self.script[self.scenes[self.scene_index]][self.dialog]["parameters"]["speaker"])
            speaker = self.script[self.scenes[self.scene_index]][self.dialog]["speaker"]
            audio = ""
            delay = 0.0
            if speaker == "S":
                audio = self.script[self.scenes[self.scene_index]][self.dialog]["audio"]
                delay = self.script[self.scenes[self.scene_index]][self.dialog]["delay"]
                userdata.speaker = speaker
                userdata.audio = audio
                userdata.param = audio
                userdata.delay = delay
                userdata.last_ship_line = audio
                self.reverse_order = random.random() < 0.5
                outcome = "say_ship_line"
            else:
                self.robot_offset *= -1
                speaker = self.script[self.scenes[self.scene_index]][self.dialog + self.reverse_order * self.robot_offset]["speaker"]
                audio = self.script[self.scenes[self.scene_index]][self.dialog + self.reverse_order * self.robot_offset]["audio"]
                delay = self.script[self.scenes[self.scene_index]][self.dialog + self.reverse_order * self.robot_offset]["delay"]
                if speaker == "A":
                    outcome = "say_robot_line_a"
                else:
                    outcome = "say_robot_line_b"
            rospy.loginfo("Speaker: %s, Audio: %s, Delay %f", speaker, audio, delay)
            userdata.speaker = speaker
            userdata.audio = audio
            userdata.param = audio
            userdata.delay = delay
        elif event == "question":
            userdata.scene = self.scenes[self.scene_index]
            outcome = "participant_input"
        elif event == "light_setting":
            userdata.light_setting = self.script[self.scenes[self.scene_index]][self.dialog]["setting"]
            outcome = "set_lights"
        elif event == "allocation":
            userdata.scene = "done"
            userdata.scene_number = self.scene_index
            outcome = "ressource_allocation"
        elif event == "unallocation":
            outcome = "return_cubes"
        elif event == "video":
            userdata.scene_number = self.scene_index
            userdata.speaker = "S"
            userdata.audio = self.script[self.scenes[self.scene_index]][self.dialog]["audio"]
            userdata.delay = 0
            outcome = "play_video"
        elif event == "lift_off":
            userdata.scene = "lift_off"
            outcome = "lift_off"
        elif event == "pause":
            raw_input("\033[92mPress enter to continue.\033[0m")
            outcome = "pause"
        else:  # unknown event
            rospy.logwarn("Encountered unknown event %s", event)
        self.dialog += 1
        return outcome

    def load_dialog_script(self, scene_file):
        with open(dirname(abspath(__file__)) + "/" + scene_file, "r") as f:
            self.script = yaml.safe_load(f)
