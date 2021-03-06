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

    def __init__(self, label, sm):
        """
        Initialize a Move class.
        :param label: A or B.
        :param sm: S or M
        """
        try:
            utm_json, joints_json, moves_path = self.create_paths(label, sm)
        except Exception as e:
            Move.lprint("Could not create paths!!")
            Move.lprint(e)
            return

        self.label = label
        self.robot = Motion.Motion(joints_json, vrep=False)
        self.mov = Mover.Mover(self.robot, stiff_off=True)
        self.moves_path = moves_path

        try:
            with open(utm_json) as json_file:
                self.utmlist = json.load(json_file)
        except IOError as e:
            self.lprint("The file is not found:: ", utm_json)
            self.lprint(e)
        except ValueError as e:
            self.lprint("Couldn't parse a json file:: ", utm_json)
            self.lprint(e)
        self.lprint(utm_json)
        self.mov.play_movement(self.moves_path + "/" +"look.csv", move_speed=0.05)
        self.relax()
        self.lprint("Pos service is ready for "+self.label)

    @staticmethod
    def lprint(*params):
        rospy.loginfo("<--------------------------------->")
        rospy.loginfo(params)
        rospy.loginfo("<--------------------------------->")
        #print(args)
        return

    @staticmethod
    def create_paths(label, sm):
        """
        Create paths for mappings, moves and joints specification.
        """
        rospy.loginfo(os.path.abspath(__file__))
        # Find the path to the file
        file_directory = Path(os.path.dirname(os.path.abspath(__file__)))
        #Move.lprint("The path to the file is: %s", file_directory)

        # Create a path to the mappings file
        mappings = os.path.join(file_directory.parent, constants.MAPPINGS_FORMAT_UTMOVE)
        utm_json = mappings.format(label, sm)
        Move.lprint("Utm json file: "+utm_json)

        # Create a path to the joints specification file
        joints_json = os.path.join(file_directory.parent, constants.JOINTS_SPECIFICATION_FILE)
        #Move.lprint("The joints json file is: %s", joints_json)

        # Create a path to the moves file
        moves_path = os.path.join(file_directory.parent, constants.MOVES_FOLDER_NAME)
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
            filename = self.moves_path + "/" +self.utmlist[uid.param][constants.KEY_POSE_FILENAME]

            sp = self.utmlist[uid.param][constants.KEY_SPEED]
            gesture_delay = self.utmlist[uid.param][constants.KEY_GESTURE_DELAY]
            #self.lprint("Gesture delay is %s", gesture_delay)
            time.sleep(gesture_delay)

            start = time.time()
            #Move.lprint("PLAYING MOVEMENT: %s", filename)
            self.mov.play_movement(filename, move_speed=sp)
            end = time.time()
            elapsed_time = end - start
            self.lprint(">>> Playing a movement {} for {} took {} seconds".format(uid.param, self.label, elapsed_time))

            '''
            Relax assures that motors do not overload (in theory), but then
            a robot does occasionally weird reflexes when it goes to a normal (base)
            position. Therefore, we have commented it out to provide smoother movements.
            '''
            # self.relax()
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
        #self.lprint("Relaxing for " + self.label+" took %s seconds", elapsed_time)

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
        "--SM",
        dest="SM",
        help="S for Still. M for Moving",
        type=str,
        default="S",
    )

    args = parser.parse_known_args()[0]
    node_name = constants.NODENAME_NAME_FORMAT.format(args.robotLabel)
    rospy.init_node(node_name, anonymous=True)
    m = Move(args.robotLabel, args.SM)
    s = rospy.Service(constants.SERVICE_NODE_NAME, Pose, m.response)
    rospy.spin()
