#!/usr/bin/env python
# DO NOT REMOVE THE ABOVE!!!

import rospy
from nicopose.srv import Pose, PoseResponse
import poses

class Move():
    pos = None
    function_dict = None
    def __init__(self):
        self.pos = poses.Poses()
        self.function_dict = {
            'no': self.pos.no, 
            'hi': self.pos.introduce, 
            'show_path': self.pos.show_path,
            'believe': self.pos.believe
        }
        return
        
    def response(self, p):
        print(p.pose_name)
        res = PoseResponse()
        if (p.pose_name in self.function_dict):
            self.function_dict[p.pose_name]
            res.msgback = 1
        else:
            print("Pose not found.")
            res.msgback = 0
        return res


if __name__ == "__main__":
    rospy.init_node('nicopose', anonymous=True)
    m = Move()
    s = rospy.Service('/pose',  Pose, m.response)
    rospy.spin()
