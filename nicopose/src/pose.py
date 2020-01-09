#!/usr/bin/env python
# DO NOT REMOVE THE ABOVE!!!

import rospy
from nicopose.srv import Pose, PoseResponse
from nicomotion import Mover, Motion

class Move():
    pos = None
    function_dict = None
    robot = None
    mov = None
    mover_path = "../../../../moves_and_positions/"
    def __init__(self):
        self.robot = Motion.Motion('../../../../json/nico_humanoid_upper_rh7d.json', vrep=False)
        self.mov = Mover.Mover(self.robot, stiff_off=True)
        self.function_dict = {
            'hi': 'hi.csv',
            'rand' : 'pos_head_look_down.csv',
        }
        return

    def response(self, p):
        print(p.pose_name)
        res = PoseResponse()
        if (p.pose_name in self.function_dict):
            fname = self.mover_path + self.function_dict[p.pose_name]
            self.mov.play_movement(fname, move_speed=0.04)
            self.relax()
            res.msgback = 1
        else:
            print("Pose not found.")
            res.msgback = 0
        return res

    def relax(self):
        self.robot.disableTorqueAll()
        self.robot.openHand('RHand')
        self.robot.openHand('LHand')
        # Maybe head comes to center?
        return


if __name__ == "__main__":
    rospy.init_node('nicopose', anonymous=True)
    m = Move()
    s = rospy.Service('/pose',  Pose, m.response)
    rospy.spin()
