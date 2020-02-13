#!/usr/bin/env python
# DO NOT REMOVE THE ABOVE!!!
import argparse
import json
import time
import rospy
import os
import constants
from poseService import Move
from unipath import Path
import serial.tools.list_ports

from nicoface.FaceExpression import faceExpression
from std_msgs.msg import String


class Fexp:

    def __init__(self, position, label):
        try:
            fex_json = self.create_paths(label)
        except Exception as e:
            Move.lprint("Could not create paths!!")
            Move.lprint(e)
            return
        self.position = position
        self.label = label

        try:
            if self.position == "LEFT":
                ports = serial.tools.list_ports.comports()
                for p in ports:
                    if p.manufacturer and "duino" in p.manufacturer:
                        self.fe = faceExpression(p.device)
                        break
            else:
                self.fe = faceExpression("/dev/ttyACM0")
            self.fe.sendFaceExpression("neutral")
        except Exception as e:
            Move.lprint(e)

        with open(fex_json) as json_file:
            self.explist = json.load(json_file)

        Move.lprint("Fex Subscriber is ready for " + self.label)

    @staticmethod
    def create_paths(label):
        """
        Create paths for mappings, moves and joints specification.
        """

        # Find the path to the file
        file_directory = Path(os.path.dirname(os.path.abspath(__file__)))
        Move.lprint("The path to the file is: %s", file_directory)

        # Create a path to the mappings file
        mappings = os.path.join(file_directory.parent, constants.MAPPINGS_FORMAT_FEX)
        fex_json = mappings.format(label)
        Move.lprint("The fex json file is: "+ fex_json)

        return fex_json

    def play(self, param):
        Move.lprint("Input data: %s", param.data)
        start = time.time()
        if len(self.explist[param.data])<1:
            Move.lprint("not found:" + param.data)
            return
        for i in range(0, len(self.explist[param.data])):
            delay = self.explist[param.data][i][constants.KEY_EXPRESSION_DELAY]
            ex = self.explist[param.data][i][constants.KEY_FACE_EXPRESSION]
            time.sleep(delay)
            Move.lprint("Robot "+ self.label+", Expression to execute is: " + ex)
            self.fe.sendFaceExpression(ex)
        end = time.time()
        elapsed_time = end - start
        Move.lprint("Playing an expression for " + self.label+" took %s seconds", elapsed_time)
        self.relax()
        Move.lprint("Relaxing is done")

    def relax(self):
        self.fe.sendFaceExpression("neutral")


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description='NICO ROS nicopose interface')
    parser.add_argument('--label', dest='robotLabel',
                        help='A for NVC. B non-NVC', type=str,
                        default='A')
    parser.add_argument('--position', dest='robotPosition',
                        help='A for NVC. B non-NVC', type=str,
                        default='A')
    args = parser.parse_known_args()[0]
    if args.robotLabel == 'A':
        args.robotPosition = "LEFT"
    else:
        args.robotPosition = "RIGHT"
    position = args.robotPosition
    time.sleep(1)
    rospy.init_node(constants.NODENAME_NAME_FORMAT.format(args.robotLabel), anonymous=True)
    f = Fexp(position, args.robotLabel)
    rospy.Subscriber(constants.TOPIC_NODE_NAME, String, f.play)
    rospy.spin()
