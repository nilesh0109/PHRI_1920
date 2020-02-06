#!/usr/bin/env python

from speech.srv import SpeechRecognition, SpeechRecognitionResponse
import random
import rospy

def handle_question_recognition(req):
    response = "fallback"
    print "Returning '{}'".format(response)
    return SpeechRecognitionResponse(response)

def speech_recognition_server():
    rospy.init_node('question_recognition_server')
    s = rospy.Service('speech_recognition_placeholder', SpeechRecognition, handle_question_recognition)
    print "Standin question recognition launched."
    rospy.spin()

if __name__ == "__main__":
    speech_recognition_server()
