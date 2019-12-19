#!/usr/bin/env python
# DO NOT REMOVE THE ABOVE!!!

import rospy
from nicopose.srv import Pose, PoseResponse
from std_srvs.srv import Trigger, TriggerResponse

"""
class Pose():
    srv = None
    ''''''
    def __init__(self):
        rospy.init_node('nicopose', anonymous=True)
        self.srv = rospy.Service('/pose',  Trigger, resp)
        pass
    ''''''
    def resp(msg):
        return msg
    pass
"""

def resp(msg):
    print(msg)
    res = PoseResponse()
    res.msgback = "we got {" + msg.pose_name + "} as message"
    return res
    '''
    TriggerResponse(
        success=True,
        message="Something something"
    )
    '''


if __name__ == "__main__":
    rospy.init_node('nicopose', anonymous=True)
    s = rospy.Service('/pose',  Pose, resp)
    rospy.spin()
