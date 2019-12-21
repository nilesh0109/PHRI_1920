#!/usr/bin/env python

from speech.srv import SpeechSynthesis, SpeechSynthesisResponse
import random
import rospy

def handle_synthesis_request(req):

    print "Sentence {} done.".format(req.sentence)
    return SpeechSynthesisResponse(True)

def speech_synthesis_server():
    rospy.init_node('speech_synthesis_server')
    s = rospy.Service('speech_synthesis', SpeechSynthesis, handle_synthesis_request)
    print "Speech synthesis service launched."
    rospy.spin()

if __name__ == "__main__":
    speech_synthesis_server()
