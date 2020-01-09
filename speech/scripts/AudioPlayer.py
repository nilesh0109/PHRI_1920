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
    def __init__(self, filename):
        self.audiofile = filename
        self.progress = 0
        #self.dur = 30 #in secs
        self.isPlaying = False

    def __play(self, data,fs):
        #global isPlaying
        self.isPlaying = True
        #print(len(data), fs)
        try:
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
        pub = rospy.Publisher('speech_progress', SpeechProgress, queue_size=25)
        rate = rospy.Rate(5)
        print('publishing started')
        start = rospy.Time.now()
        print('duration is', self.dur)
        while self.isPlaying:
            t = rospy.Time.now()
            self.progress = ((t.secs - start.secs) * 100 / self.dur)
            #print('start ==> ',start.secs, 'current ====>',t.secs, 'progress==>', self.progress)
            default_msg = SpeechProgress(t, 'Hello', self.progress)
            pub.publish(default_msg)
            rate.sleep()
            #print(self.progress)
            #time.sleep(0.1)

    
    def play(self):
        data, fs = sf.read('/informatik2/students/home/8vijayra/Desktop/sample.wav', dtype='float32')
        self.dur = math.ceil(len(data)/fs)

        play_thread = threading.Thread(target= self.__play, args=( data,fs))
        progress_thread = threading.Thread(target= self.__share_progress)

        threads = [play_thread, progress_thread]

        play_thread.start()
        progress_thread.start()

        for thread in threads:
            thread.join()
        print('finished')

if __name__ == '__main__':
    AudioPlayer('').play()



# isPlaying = True
# def get_progress():
#     start = time.time()
#     while isPlaying:
#         print(time.time() - start)
#         time.sleep(0.1)

# def player_Callback():
#     global isPlaying
#     isPlaying = True
#     try:
#         #sd.play(data, fs)
#         print('Speech playiing')
#         time.sleep(10)
#         #sd.wait()
#         isPlaying = False
#         print('Speech finished')
#     except:
#         isPlaying = False
#         print('EXCEPTION !')      
# threads = []

# #data, fs = sf.read('/speech.wav', dtype='float32')
# #data, fs = sf.read('/long_audio.wav', dtype='float32')
# thread1 = threading.Thread(target= player_Callback)
# thread2 = threading.Thread(target= get_progress)
# threads += [thread1]
# thread1.start()
# thread2.start()

# for thread in threads:
#     thread.join()

# print('finished')