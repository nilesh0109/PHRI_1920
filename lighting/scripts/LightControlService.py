#!/usr/bin/env python
import sys
from LightMIDIController import LightController

import rospy
from lighting.srv import LightControl, LightControlResponse

lights = LightController()

def handle_light_request(req):
    rospy.loginfo("Setting lights to \'%s\'", req.setting)
    lights.set_lights(req.setting)
    return LightControlResponse(True)

def light_control_server():
    rospy.init_node('light_control_server', anonymous=True)
    rospy.Service('light_control', LightControl, handle_light_request)
    rospy.loginfo("Light control service launched.")
    rospy.spin()

if __name__ == "__main__":
    light_control_server()
