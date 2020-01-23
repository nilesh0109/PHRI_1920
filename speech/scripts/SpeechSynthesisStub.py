#!/usr/bin/env python

from speech.srv import SpeechSynthesis, SpeechSynthesisResponse
import random
import rospy
from AudioPlayer import AudioPlayer
from os.path import dirname, abspath
import sys
def handle_synthesis_request(req):
    player = AudioPlayer(req.speaker)
    filepath = dirname(dirname(abspath(__file__))) + '/generated_sounds/{}.wav'.format(req.audio)
    if req.delay:
        rospy.sleep(req.delay)
    player.play(filepath, req.audio)
    print 'Sentence {} done.'.format(req.audio)
    print '<--------------------------------------------->'
    return SpeechSynthesisResponse(True)

def speech_synthesis_server(robot_name=''):
    rospy.init_node(robot_name+'/speech_synthesis_server')
    s = rospy.Service(robot_name+'/speech_synthesis', SpeechSynthesis, handle_synthesis_request)
    print "Speech synthesis service launched."
    rospy.spin()

if __name__ == "__main__":
    if len(sys.argv)>1:
        speech_synthesis_server(sys.argv[1])
    else:
        speech_synthesis_server()
