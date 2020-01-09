#!/usr/bin/env python

from speech.srv import SpeechSynthesis, SpeechSynthesisResponse
import random
import rospy
from AudioPlayer import AudioPlayer

def handle_synthesis_request(req):

    print "Sentence {} done.".format(req.sentence)
    player = AudioPlayer(' ')
    player.play()
    return SpeechSynthesisResponse(True)

def share_progress(player, sentence):
    pub = rospy.Publisher('speech_progress', SpeechProgress, queue_size=25)
    while player.isPlaying:
        default_msg = SpeechProgress(player.progress, sentence, player.dur)
        pub.publish(default_msg)

def speech_synthesis_server():
    rospy.init_node('speech_synthesis_server')
    s = rospy.Service('speech_synthesis', SpeechSynthesis, handle_synthesis_request)
    print "Speech synthesis service launched."
    rospy.spin()

if __name__ == "__main__":
    speech_synthesis_server()
