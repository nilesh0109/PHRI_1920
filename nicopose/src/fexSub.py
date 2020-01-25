#!/usr/bin/env python
# DO NOT REMOVE THE ABOVE!!!
import argparse
import json
import time
import rospy
import os
import constants
from unipath import Path
import serial.tools.list_ports

from nicoface.FaceExpression import faceExpression
from std_msgs.msg import String


class Fexp:

    def __init__(self, position, label):
        fex_json = self.create_paths(label)
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
                self.fe = faceExpression()
        except Exception as e:
            print(e)

        with open(fex_json) as json_file:
            self.explist = json.load(json_file)

        print("Fex Subscriber is ready.")

    @staticmethod
    def create_paths(label):
        """
        Create paths for mappings, moves and joints specification.
        """

        # Find the path to the file
        file_directory = Path(os.path.abspath(__file__))
        print("The path to the file is: %s ", file_directory)

        # Create a path to the mappings file
        mappings = os.path.join(file_directory.parent.parent, constants.MAPPINGS_FORMAT_FEX)
        fex_json = mappings.format(label)
        print("The fex json file is: %s ", fex_json)

        return fex_json

    def play(self, param):
        print("Input data: {}", param.data)
        start = time.time()
        for i in range(0, len(self.explist[param.data])):
            delay = self.explist[param.data][i][constants.KEY_EXPRESSION_DELAY]
            ex = self.explist[param.data][i][constants.KEY_FACE_EXPRESSION]
            time.sleep(delay)
            print("Expression to execute is: {}".format(ex))
            self.fe.sendFaceExpression(ex)
        end = time.time()
        elapsed_time = end - start
        print("Playing an expression took %s seconds", elapsed_time)
        self.relax()
        print("Relaxing is done")

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
    position = args.robotPosition
    rospy.init_node(constants.NODENAME_NAME_FORMAT.format(args.robotLabel), anonymous=True)
    f = Fexp(position, args.robotLabel)
    rospy.Subscriber(constants.TOPIC_NAME_FORMAT.format(args.robotLabel), String, f.play)
    rospy.spin()
