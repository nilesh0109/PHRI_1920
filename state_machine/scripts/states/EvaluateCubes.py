#!/usr/bin/env python

import rospy
import smach
from smach_ros import ServiceState
from speech.srv import CubeCounting
from nicopose.srv import Pose, PoseRequest
import csv
import datetime

class SaveCSV(smach.State):
    def __init__(self):
        smach.State.__init__(
            self, outcomes=["saved_csv"], input_keys=["scene_id", "num_cubes_A", "num_cubes_B"]
        )
        self.csv_file = "cube_counts/experiment_{}.csv".format(datetime.datetime.now().isoformat())
        with open(self.csv_file, 'w') as csvfile:
            fieldnames = ['scene', 'robot_a', 'robot_b']
            writer = csv.DictWriter(csvfile, fieldnames=fieldnames)

            writer.writeheader()

    def execute(self, userdata):
        rospy.loginfo("Executing state SAVE_CSV")
        with open(self.csv_file, 'a') as csvfile:
            fieldnames = ['scene', 'robot_a', 'robot_b']
            writer = csv.DictWriter(csvfile, fieldnames=fieldnames)

            writer.writerow({'scene': "scene_{}".format(userdata.scene_id), 'robot_a': userdata.num_cubes_A, 'robot_b': userdata.num_cubes_B})
        return "saved_csv"

class EvaluateCubes(smach.Concurrence):
    """docstring for CubeFeedback."""

    def __init__(self):
        super(EvaluateCubes, self).__init__(
            input_keys=["scene_id", "num_cubes_A", "num_cubes_B"],
            outcomes=["evaluation_done", "evaluation_failed"],
            default_outcome="evaluation_failed",
            # outcome_map={"utterance_done": {"SPEAK": "speech_done", "NVC": "nvc_done"}},
            outcome_map={"evaluation_done": {"CUBE_FEEDBACK": "succeeded", "NVCA": "succeeded", "NVCB": "succeeded", "SAVE_CSV": "saved_csv"}},
        )
        # Open the container
        with self:
            smach.Concurrence.add("NVCA",
                                  ServiceState("/A/pose", Pose, request=PoseRequest("scene_0_S_line_1")),
                                  )

            smach.Concurrence.add("NVCB",
                                  ServiceState("/B/pose", Pose, request=PoseRequest("scene_0_S_line_1")),
                                  )

            smach.Concurrence.add("SAVE_CSV",
                                  SaveCSV(),
                                  )

            smach.Concurrence.add(
                "CUBE_FEEDBACK",
                ServiceState(
                    "/cube_counting", CubeCounting, request_slots=["scene_id", "num_cubes_A", "num_cubes_B"]
                ),
            )


