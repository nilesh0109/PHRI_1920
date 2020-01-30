#!/usr/bin/env python

from video.srv import PlayVideo, PlayVideoResponse
import threading
import time
import os
import rospy
from os.path import dirname, abspath




class myVideoThread(threading.Thread):
    def __init__(self, threadID, filename):
        threading.Thread.__init__(self)
        self.threadID = threadID
        self.filename = dirname(abspath(__file__)) + '/hrivid.mp4'

    def run(self):
        startvideo(self.filename)

def startvideo(filename):
    os.system(
        str("DISPLAY=:0.1 vlc --fullscreen " + str(filename))
    )


def pause_trigger():
    os.system(
        "dbus-send --type=method_call --dest=org.mpris.MediaPlayer2.vlc /org/mpris/MediaPlayer2 org.mpris.MediaPlayer2.Player.PlayPause")

def play_video(seconds):
    print("playing video for %i seconds" % seconds)
    pause_trigger()
    time.sleep(seconds)
    pause_trigger()
    print("Video stopped")


def handle_video_request(req):
    play_video(cliplength[req.scene_number])
    return True


def video_server():
    rospy.init_node("play_video_server")
    print("Video service launched")
    thread1 = myVideoThread(1, dirname(abspath(__file__)) + '/hrivid.mp4')
    thread1.start()
    time.sleep(1)
    pause_trigger()
    s = rospy.Service("play_video", PlayVideo, handle_video_request)
    rospy.spin()


if __name__ == "__main__":
    exitFlag = 0
    cliplength = [0, 26, 28, 34, 34, 52]
    video_server()

