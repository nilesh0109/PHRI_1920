#!/usr/bin/env python

from speech.srv import SpeechRecognition, SpeechRecognitionResponse
import rospy

def handle_recognition_request(req):
    sentence = "scene_42"
    print 'Recognized: \'{}\''.format(sentence)
    print '<--------------------------------------------->'
    return SpeechRecognitionResponse(sentence)

def speech_recognition_server():
    rospy.init_node('speech_recognition_server')
    s = rospy.Service('speech_recognition', SpeechRecognition, handle_recognition_request)
    print "Speech recognition service launched."
    rospy.spin()

if __name__ == "__main__":
    speech_recognition_server()
