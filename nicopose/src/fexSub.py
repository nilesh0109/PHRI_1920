#!/usr/bin/env python
# DO NOT REMOVE THE ABOVE!!!

import json
import time
import rospy
from nicoface.FaceExpression import faceExpression
from std_msgs.msg import String


class Fexp():
    fe = None
    fex_json = "../mappings/fex.json"
    explist = None

    def __init__(self):
        self.fe = faceExpression()
        with open(self.fex_json) as json_file:
            self.explist = json.load(json_file)
        print("Fex Subscriber ready...")
        return

    def play(self, param):
        print(param.data)
        for i in range(0, len(self.explist[param.data])):
            delay = self.explist[param.data][i]['expression_delay']
            ex = self.explist[param.data][i]['fe']
            time.sleep(delay)
            self.fe.sendFaceExpression(ex)
            print(time.time())
        self.relax()
        print("done")
        return

    def relax(self):
        self.fe.sendFaceExpression("neutral")
        return


if __name__ == "__main__":
    rospy.init_node('nicopose', anonymous=True)
    f = Fexp()
    rospy.Subscriber('fex', String, f.play)
    rospy.spin()
