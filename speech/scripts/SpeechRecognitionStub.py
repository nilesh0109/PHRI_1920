#!/usr/bin/env python

from speech.srv import SpeechRecognition, SpeechRecognitionResponse
import rospy
import sentencelist as sl

def handle_recognition_request(req):
    sentence = sl.recognize(req.context)
    print('Recognized: \'{}\''.format(sentence))
    return SpeechRecognitionResponse(sentence)

def speech_recognition_server():
    sl.initialize()
    rospy.init_node('speech_recognition_server')
    s = rospy.Service('speech_recognition', SpeechRecognition, handle_recognition_request)
    print("Speech recognition service launched.")
    rospy.spin()

if __name__ == "__main__":
    speech_recognition_server()
