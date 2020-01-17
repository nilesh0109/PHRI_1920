#!/usr/bin/env python

from video.srv import PlayVideo, PlayVideoResponse
import threading
import time
import os
import rospy
from pynput.keyboard import Key, Controller

exitFlag = 0
cliplength = [0, 30, 30, 30, 30]
counter = 0


class myVideoThread(threading.Thread):
    def __init__(self, threadID, filename):
        threading.Thread.__init__(self)
        self.threadID = threadID
        self.filename = filename

    def run(self):
        startvideo(self.filename)


class myPauseThread(threading.Thread):
    def __init__(self, threadID, length):
        threading.Thread.__init__(self)
        self.threadID = threadID
        self.length = length
        self.keyboard = Controller()

    def run(self):
        pausevideoafter(self.keyboard, self.length)


def startvideo(filename):
    os.system(
        str("DISPLAY=:0.1 vlc --fullscreen --global-key-play-pause=a" + str(filename))
    )


def pausevideoafter(keyboard, seconds):
    time.sleep(seconds)
    keyboard.press("a")
    keyboard.release("a")


def handle_video_request(req):
    thread2 = myPauseThread(2, cliplength[req.scene_number])
    counter += 1


def video_server():
    rospy.init_node("play_video_server")
    s = rospy.Service("play_video", PlayVideo, handle_video_request)
    thread1 = myVideoThread(1, "filename.mp4")
    thread2 = myPauseThread(2, 0)
    print("Video service launched")
    rospy.spin()


if __name__ == "__main__":
    video_server()
