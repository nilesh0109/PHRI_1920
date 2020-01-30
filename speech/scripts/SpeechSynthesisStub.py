#!/usr/bin/env python
import sys
from os.path import dirname, abspath

import rospy
from speech.srv import SpeechSynthesis, SpeechSynthesisResponse
from AudioPlayer import AudioPlayer

def handle_synthesis_request(req):
    player = AudioPlayer(req.speaker)
    filepath = dirname(dirname(abspath(__file__))) + '/generated_sounds/{}.wav'.format(req.audio)
    if req.delay:
        rospy.sleep(req.delay)
    rospy.loginfo("Playing speech %s", req.audio)
    player.play(filepath, req.audio)
    rospy.loginfo("Finished speech")
    return SpeechSynthesisResponse(True)

def speech_synthesis_server(robot_name=''):
    rospy.init_node('speech_synthesis_server', anonymous=True)
    rospy.Service('speech_synthesis', SpeechSynthesis, handle_synthesis_request)
    rospy.loginfo("Speech synthesis for %s launched.", robot_name)
    rospy.spin()

if __name__ == "__main__":
    if len(sys.argv) > 1:
        speech_synthesis_server(sys.argv[1])
    else:
        speech_synthesis_server()
