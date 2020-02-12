#!/usr/bin/env python

import rospy
from speech.srv import SpeechRecognition, SpeechRecognitionResponse

import sentencelist as sl

processor = sl.SentenceList()

def handle_recognition_request(req):
    # Calibrate the microphone
    if req.context == "calibrate":
        return processor.calibrate()
    try:
        rospy.loginfo("Received context: %s", req.context)
        processor.configure(req.context)
        docks_hypotheses, confidence = processor.recognize()
        sentence = processor.match_sentence(docks_hypotheses, confidence)
        rospy.loginfo("Recognized: %s", sentence)
        return SpeechRecognitionResponse(sentence)
    except BaseException as e:
        rospy.loginfo("Error: %s", e)
        return SpeechRecognitionResponse("fallback")


def speech_recognition_server():
    rospy.init_node('speech_recognition_server')
    rospy.Service('speech_recognition', SpeechRecognition, handle_recognition_request)
    processor.initialize()
    rospy.loginfo("Speech recognition service launched.")
    rospy.spin()

if __name__ == "__main__":
    speech_recognition_server()
