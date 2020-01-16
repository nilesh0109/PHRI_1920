#!/usr/bin/env python

### requires sudo apt-get install libsndfile1 on linux (For soundfile)
### requires sudo apt-get install libportaudio2 on linux (For sounddevice)
### requires pip install SoundFile
### requires pip install sounddevice
import soundfile as sf
import sounddevice as sd
import time
import threading
from speech.msg import SpeechProgress
import rospy
import math

class AudioPlayer():
    def __init__(self):
        self.progress = 0
        TOPIC_NAME = 'speech_progress'
        self.pub = rospy.Publisher(TOPIC_NAME, SpeechProgress, queue_size=25)
        self.rate = rospy.Rate(10)
        #self.dur = 30 #in secs
        self.isPlaying = False

    def __play(self, data,fs):
        self.isPlaying = True
        try:
            print('starting Speech')
            sd.play(data, fs)
            print('Speech playiing')
            #time.sleep(self.dur)
            sd.wait()
            self.isPlaying = False
            print('Speech finished')
        except:
            self.isPlaying = False
            print('EXCEPTION !')

    def __share_progress(self):
        print('publishing started ')
        start = rospy.Time.now()
        while self.isPlaying:
            t = rospy.Time.now()
            self.progress = ((t.secs - start.secs + (t.nsecs - start.nsecs)*1e-9) * 100 / self.dur)
            if self.progress > 99: self.progress=100
            default_msg = SpeechProgress(t, self.audiofile, self.progress)
            self.pub.publish(default_msg)
            self.rate.sleep()
            #time.sleep(0.1)

    
    def play(self, filepath, filename):
        self.audiofile=filename
        data, fs = sf.read(filepath, dtype='float32')
        self.dur = math.floor(len(data)/fs)+1

        play_thread = threading.Thread(target= self.__play, args=( data,fs))
        progress_thread = threading.Thread(target= self.__share_progress)

        threads = [play_thread, progress_thread]

        play_thread.start()
        progress_thread.start()

        for thread in threads:
            thread.join()

if __name__ == '__main__':
    AudioPlayer('').play()
