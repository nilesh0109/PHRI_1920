#!/usr/bin/env python
# DO NOT REMOVE THE ABOVE!!!
import argparse
import json
import time
import rospy
from nicoface.FaceExpression import faceExpression
from std_msgs.msg import String
import serial.tools.list_ports


class Fexp():
    fe = None
    fex_json = "../mappings/fex.json"
    explist = None

    def __init__(self, position):
        self.position = position
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
            print("ex: {}".format(ex))
            self.fe.sendFaceExpression(ex)
            print(time.time())
        self.relax()
        print("done")
        return

    def relax(self):
        self.fe.sendFaceExpression("neutral")
        return


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description='NICO ROS nicopose interface')
    parser.add_argument('--label', dest='robotLabel',
                        help='A for NVC. B non-NVC', type=str,
                        default='A')
    parser.add_argument('--position', dest='robotPosition',
                        help='A for NVC. B non-NVC', type=str,
                        default='A')

    args = parser.parse_known_args()[0]
    label = args.robotLabel
    position = args.robotPosition
    rospy.init_node("nicopose_{}".format(label), anonymous=True)
    f = Fexp(position)
    rospy.Subscriber("{}/fex".format(label), String, f.play)
    rospy.spin()
