#!/usr/bin/env python
# DO NOT REMOVE THE ABOVE!!!
import argparse
import time
import json
import rospy
from nicopose.srv import Pose, PoseResponse
from nicomotion import Mover, Motion
from std_msgs.msg import String

class Move():
    robot = None
    mov = None
    mover_path = "../../../../moves_and_positions/"
    utmlist = None
    utm_json_format = "../mappings/utmove_{}_{}.json"
    def __init__(self, label, position):
        self.robot = Motion.Motion('../../../../json/nico_humanoid_upper_rh7d.json', vrep=False)
        self.mov = Mover.Mover(self.robot, stiff_off=True)
        self.position = position
        self.label = label

        utm_json = self.utm_json_format.format(label, position)

        with open(utm_json) as json_file:
            self.utmlist = json.load(json_file)
        print("Pos service ready...")
        return

    def response(self, uid):
        print(uid.param)
        pub = rospy.Publisher('fex', String, queue_size=10)
        pub.publish(uid.param)
        try:
            res = PoseResponse()
            fname = self.mover_path + self.utmlist[uid.param]['pose_filename']
            sp = self.utmlist[uid.param]['speed']
            time.sleep(self.utmlist[uid.param]['gesture_delay'])
            self.mov.play_movement(fname, move_speed=sp)
            self.relax()
            res.msgback = 1
        except:
            print("Pose not found.")
            res.msgback = 0
        return res

    def relax(self):
        self.disableTorqueBody()
        self.robot.openHand('RHand')
        self.robot.openHand('LHand')
        return

    def disableTorqueBody(self):
        for motor in self.robot.motors:
            if motor.name == "head_z" or motor.name == "head_y":
                continue
            motor.compliant = True




if __name__ == "__main__":
    parser = argparse.ArgumentParser(description='NICO ROS nicopose interface')
    parser.add_argument('--label', dest='robotLabel',
                        help='A for NVC. B non-NVC', type=str,
                        default='A')
    parser.add_argument('--position', dest='robotPosition',
                        help='LEFT or RIGHT', type=str,
                        default='LEFT')

    args = parser.parse_known_args()[0]
    label = args.robotLabel
    print(args.robotPosition)
    node_name = "nicopose_{}".format(label)
    rospy.init_node(node_name, anonymous=True)
    m = Move(args.robotLabel, args.robotPosition)
    s = rospy.Service('{}/pose'.format(label),  Pose, m.response)
    rospy.spin()
