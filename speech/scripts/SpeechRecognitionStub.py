#!/usr/bin/env python

import rospy
from speech.srv import SpeechRecognition, SpeechRecognitionResponse

import sentencelist as sl

processor = sl.SentenceList()

def handle_recognition_request(req):
    rospy.loginfo("Received context: %s", req.context)
    processor.configure(req.context)
    sentence = processor.recognize(req.context)
    rospy.loginfo("Recognized: %s", sentence)
    return SpeechRecognitionResponse(sentence)

def speech_recognition_server():
    rospy.init_node('speech_recognition_server')
    rospy.Service('speech_recognition', SpeechRecognition, handle_recognition_request)
    rospy.loginfo("Speech recognition service launched.")
    rospy.spin()

if __name__ == "__main__":
    speech_recognition_server()
