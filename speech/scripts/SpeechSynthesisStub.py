#!/usr/bin/env python

from speech.srv import SpeechSynthesis, SpeechSynthesisResponse
import random
import rospy
from AudioPlayer import AudioPlayer
from os.path import dirname, abspath

def handle_synthesis_request(req):
    player = AudioPlayer()
    filepath = dirname(dirname(abspath(__file__))) + '/generated_sounds/{}.wav'.format(req.audio)
    if req.delay:
        rospy.sleep(req.delay)
    player.play(filepath, req.audio)
    print 'Sentence {} done.'.format(req.audio)
    print '<--------------------------------------------->'
    return SpeechSynthesisResponse(True)

def speech_synthesis_server():
    rospy.init_node('speech_synthesis_server')
    s = rospy.Service('speech_synthesis', SpeechSynthesis, handle_synthesis_request)
    print "Speech synthesis service launched."
    rospy.spin()

if __name__ == "__main__":
    speech_synthesis_server()
