#!/usr/bin/env python

from speech.srv import SpeechSynthesis, SpeechSynthesisResponse
#from nicoaudio.AudioPlayer import AudioPlayer
import random
import rospy
from AudioPlayer import AudioPlayer
from os.path import dirname, abspath

sound_dict = {"hello_commander": "scene_0_line_0_S.mp3",
              "systems_clear":   "scene_0_line_1_S.mp3"}

def play_audio(file_name, sounds_dir="generated_sounds/"):
    # load file
    playback = AudioPlayer(sounds_dir + file_name)
    # start playback
    playback.play()
    # wait until playback is finished
    time.sleep(playback.duration - playback.position)

def handle_synthesis_request(req):
    print 'Sentence {} done.'.format(req.sentence)
    player = AudioPlayer()
    filepath = dirname(dirname(abspath(__file__))) + '/generated_sounds/sample.wav'
    player.play(filepath)
    return SpeechSynthesisResponse(True)


def speech_synthesis_server():
    rospy.init_node('speech_synthesis_server')
    s = rospy.Service('speech_synthesis', SpeechSynthesis, handle_synthesis_request)
    print "Speech synthesis service launched."
    rospy.spin()

if __name__ == "__main__":
    speech_synthesis_server()
