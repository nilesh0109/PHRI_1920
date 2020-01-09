#!/usr/bin/env python

from state_machine.srv import SpeechRecognition, SpeechRecognitionResponse
import random
import rospy


def handle_question_recognition(req):
    response = random.choice(["repeat", "question0", "cancel"])
    print("Returning '{}'".format(response))
    return SpeechRecognitionResponse(response)


def speech_recognition_server():
    rospy.init_node("question_recognition_server")
    s = rospy.Service(
        "question_recognition", SpeechRecognition, handle_question_recognition
    )
    print("Standin question recognition launched.")
    rospy.spin()


if __name__ == "__main__":
    speech_recognition_server()
