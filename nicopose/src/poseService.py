#!/usr/bin/env python
# DO NOT REMOVE THE ABOVE!!!

import time
import json
import rospy
from nicopose.srv import Pose, PoseResponse, Fex, FexRequest
from nicomotion import Mover, Motion

class Move():
    utm_json = None
    robot = None
    mov = None
    mover_path = "../../../../moves_and_positions/"
    utmlist = None
    utm_json = "utmove.json"
    def __init__(self):
        self.robot = Motion.Motion('../../../../json/nico_humanoid_upper_rh7d.json', vrep=False)
        self.mov = Mover.Mover(self.robot, stiff_off=True)
        with open(self.utm_json) as json_file:
            self.utmlist = json.load(json_file)
        print("Pos service ready...")
        return

    def response(self, uid):
        print(uid.param)
        grr = rospy.ServiceProxy('/fex', Fex)
        frq = FexRequest()
        frq.param = str(uid)
        try:
            rrr = grr(frq)
        except rospy.ServiceException:
            print(rospy.ServiceException)
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
        self.robot.disableTorqueAll()
        self.robot.openHand('RHand')
        self.robot.openHand('LHand')
        return


if __name__ == "__main__":
    rospy.init_node('nicopose', anonymous=True)
    m = Move()
    s = rospy.Service('/pose',  Pose, m.response)
    rospy.spin()
