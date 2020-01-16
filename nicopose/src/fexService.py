#!/usr/bin/env python
# DO NOT REMOVE THE ABOVE!!!

import rospy
from nicopose.srv import Fex, FexResponse
from nicoface.FaceExpression import faceExpression
import json
import time


class Fexp():
    fe = None
    fex_json = "fex.json"
    explist = None
    def __init__(self):
        #self.fe = faceExpression()
        with open(self.fex_json) as json_file:
            self.explist = json.load(json_file)
        print("Fex service ready...")
        return

    def response(self, uid):
        #print(uid.param)
        try:
            res = FexResponse()
            for i in range(0, len(self.explist[uid.param])):
                delay = self.explist[uid.param][i]['expression_delay']
                ex = self.explist[uid.param][i]['fe']
                time.sleep(delay)
                self.fe.sendFaceExpression(ex)
            self.relax()
            res.msgback = 1
        except:
            print("Fex failed.")
            res.msgback = 0
        return res

    def relax(self):
        self.fe.sendFaceExpression("neutral")
        return


if __name__ == "__main__":
    rospy.init_node('nicofex', anonymous=True)
    f = Fexp()
    s = rospy.Service('/fex',  Fex, f.response)
    rospy.spin()
