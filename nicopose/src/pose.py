#!/usr/bin/env python
# DO NOT REMOVE THE ABOVE!!!
import time

import rospy
from nicopose.srv import Pose, PoseResponse
from nicomotion import Mover, Motion
from nicoface.FaceExpression import faceExpression
import json
import glob
import re

import threading

TYPE_MOVEMENT = "MOVEMENT"
TYPE_UTTERANCE = "UTTERANCE"
TYPE_EXPRESSION = "EXPRESSION"

class Move():

    def __init__(self):
        self.robot = Motion.Motion('../../../../json/nico_humanoid_upper_rh7d.json', vrep=False)
        self.mov = Mover.Mover(self.robot, stiff_off=True)
        self.fe = faceExpression()
        self.poses = fetch_poses()
        return

    @staticmethod
    def fetch_poses():
        poses = {}

        for filename in glob.glob("../../../../poses/*.json"):
            with open(filename, 'r') as f:
                name = re.search(r"^.*/(.*).json$", filename).group(1)
                poses[name] = json.load(f)

        return poses

    def response(self, p):
        res = PoseResponse()
        print(p.pose_name)

        tasks = self.poses.get(p.pose_name, None)
        if tasks is None:
            print("Pose not found.")
            res.msgback = 0
            return res

        for task in tasks:
            self.task_job(task)

        self.relax()
        res.msgback = 1

        return res

    def relax(self):
        self.robot.disableTorqueAll()
        self.robot.openHand('RHand')
        self.robot.openHand('LHand')
        # Maybe head comes to center?
        return

    def task_job(self, task):
        time.sleep(task["time"])

        if task["type"] == TYPE_MOVEMENT:
            print("EXECUTING MOVEMENT {} at time {} ----- ".format(task["value"], task["time"]))
            self.play_movement(task["value"])

            return
        elif task["type"] == TYPE_EXPRESSION:
            print("EXECUTING EXPRESSION {} at time {} ----- ".format(task["value"], task["time"]))
            return
        elif task["type"] == TYPE_UTTERANCE:
            print("EXECUTING UTTERANCE {} at time {} ----- ".format(task["value"], task["time"]))
        return

    def play_movement(self, movement_name):
        self.mov.play_movement(movement_name, move_speed=0.01)
        self.relax()
        return

    def play_utterance(self, utterance_name):
        # TODO: IMPLEMENTATION OF UTTERANCE
        return

    def play_expression(self, expression_name):
        self.fe.sendFaceExpression(expression_name)
        return


if __name__ == "__main__":
    rospy.init_node('nicopose', anonymous=True)
    m = Move()
    s = rospy.Service('/pose',  Pose, m.response)
    rospy.spin()
