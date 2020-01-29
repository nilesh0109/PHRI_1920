#!/usr/bin/env python
# DO NOT REMOVE THE ABOVE!!!
import argparse
import time
import json
import rospy
import os
import constants
from unipath import Path
from nicopose.srv import Pose, PoseResponse
from nicomotion import Mover, Motion
from std_msgs.msg import String


class Move:
    """
    A class for executing robot's movements.
    """

    def __init__(self, label, position):
        """
        Initialize a Move class.
        :param label: A or B.
        :param position: LEFT or RIGHT
        """
        utm_json, joints_json, moves_path = self.create_paths(label, position)

        self.position = position
        self.label = label
        self.robot = Motion.Motion(joints_json, vrep=False)
        self.mov = Mover.Mover(self.robot, stiff_off=True)
        self.moves_path = moves_path

        try:
            with open(utm_json) as json_file:
                self.utmlist = json.load(json_file)
        except IOError as e:
            self.lprint("The file is not found: %s ", utm_json)
            self.lprint(e)
        except ValueError as e:
            self.lprint("Couldn't parse a json file: %s ", utm_json)
            self.lprint(e)

        self.lprint("Pos service is ready.")

    @staticmethod
    def lprint(*args):
        #rospy.logdebug(">>> motion <<<")
        #rospy.logdebug(args)
        #rospy.logdebug("<<< motion >>>")
        print(args)

    @staticmethod
    def create_paths(label, position):
        """
        Create paths for mappings, moves and joints specification.
        """

        # Find the path to the file
        file_directory = Path(os.path.dirname(os.path.abspath(__file__)))
        Move.lprint("The path to the file is: %s ", file_directory)

        # Create a path to the mappings file
        mappings = os.path.join(file_directory.parent, constants.MAPPINGS_FORMAT_UTMOVE)
        utm_json = mappings.format(label, position)
        Move.lprint("The utm json file is: %s ", utm_json)

        # Create a path to the joints specification file
        joints_json = os.path.join(file_directory.parent, constants.JOINTS_SPECIFICATION_FILE)
        Move.lprint("The utm json file is: %s ", joints_json)

        # Create a path to the moves file
        moves_path= os.path.join(file_directory.parent, constants.MOVES_FOLDER_NAME)

        return utm_json, joints_json, moves_path

    def response(self, uid):
        """
        For the given uid returns the message if a movement was successful or not.
        :param uid: utterance Id.
        :return: 1 (successful) or 0 (failure)
        """
        self.lprint(uid.param)
        topic_name = constants.TOPIC_NAME_FORMAT.format(self.label)
        pub = rospy.Publisher(topic_name, String, queue_size=10)
        pub.publish(uid.param)
        res = PoseResponse()
        try:
            filename = self.moves_path + self.utmlist[uid.param][constants.KEY_POSE_FILENAME]
            sp = self.utmlist[uid.param][constants.KEY_SPEED]
            self.lprint("Gesture delay is ", constants.KEY_GESTURE_DELAY)
            time.sleep(self.utmlist[uid.param][constants.KEY_GESTURE_DELAY])

            start = time.time()
            self.mov.play_movement(filename, move_speed=sp)
            end = time.time()
            elapsed_time = end - start
            self.lprint("Playing a movement took %s seconds", elapsed_time)

            self.relax()
            res.msgback = 1
        except Exception as e:
            self.lprint(e)
            res.msgback = 0
        return res

    def relax(self):
        """
        Relax a robot.
        """
        start = time.time()
        self.disable_torque_body()
        self.robot.openHand("RHand")
        self.robot.openHand("LHand")
        end = time.time()
        elapsed_time = end - start
        self.lprint("Relaxing took %s seconds", elapsed_time)

    def disable_torque_body(self):
        """
        Disable torque in all motors except the head.
        """
        for motor in self.robot._robot.motors:
            if motor.name == "head_z" or motor.name == "head_y":
                continue
            motor.compliant = True


        


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="NICO ROS nicopose arguments")
    parser.add_argument(
        "--label",
        dest="robotLabel",
        help="A for NVC. B for non-NVC",
        type=str,
        default="A",
    )
    parser.add_argument(
        "--position",
        dest="robotPosition",
        help="LEFT or RIGHT",
        type=str,
        default="LEFT",
    )

    args = parser.parse_known_args()[0]
    Move.lprint("Position is %s; label is %s", args.robotPosition, args.robotLabel)
    node_name = constants.NODENAME_NAME_FORMAT.format(args.robotLabel)
    rospy.init_node(node_name, anonymous=True)
    m = Move(args.robotLabel, args.robotPosition)
    s = rospy.Service(constants.SERVICE_NAME_FORMAT.format(args.robotLabel), Pose, m.response)
    rospy.spin()
