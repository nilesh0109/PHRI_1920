#!/usr/bin/env python
# requires:
# - sudo apt-get install libsndfile1 on linux (For soundfile)
# - sudo apt-get install libportaudio2 on linux (For sounddevice)

import alsaaudio
import math
import rospy
import soundfile as sf
import sounddevice as sd
from speech.msg import SpeechProgress
import threading
import wave

class AudioPlayer:
    def __init__(self, robot_name=''):
        self.progress = 0
        self.robot_name = robot_name
        topic_name = robot_name+'/speech_progress'
        self.pub = rospy.Publisher(topic_name, SpeechProgress, queue_size=25)
        self.rate = rospy.Rate(10)
        self.isPlaying = False

    def __play(self, data, fs):
        self.isPlaying = True
        try:
            print('Starting Speech')
            sd.play(data, fs)
            print('Playing Speech')
            sd.wait()
            self.isPlaying = False
            print('Finished Speech')
        except:
            self.isPlaying = False
            print('EXCEPTION !')

    def __share_progress(self):
        print('Publishing progress')
        start = rospy.Time.now()
        while self.isPlaying:
            t = rospy.Time.now()
            time = t.secs - start.secs + (t.nsecs - start.nsecs)*1e-9
            self.progress = time * 100 / self.dur
            if self.progress > 99:
                self.progress = 100
            default_msg = SpeechProgress(self.audiofile, self.progress)
            self.pub.publish(default_msg)
            self.rate.sleep()

    def play(self, filepath, filename):
        # special case when on the wtmhri2 lab computer:
        if self.robot_name == 'S':
            threads = []
            for channel in range(1, 17):  # play on all 16 channels simultaneously
                st = threading.Thread(target=play_file, args=(filepath, channel))
                threads.append(st)
                st.start()
            for thread in threads:
                thread.join()
        else:
            self.audiofile = filename
            data, fs = sf.read(filepath, dtype='float32')
            self.dur = math.floor(len(data)/fs)+1

            play_thread = threading.Thread(target=self.__play, args=(data, fs))
            progress_thread = threading.Thread(target=self.__share_progress)

            threads = [play_thread, progress_thread]
            play_thread.start()
            progress_thread.start()

            for thread in threads:
                thread.join()

def play_file(filepath, channel):
    wav_file = wave.open(filepath, 'rb')
    device = alsaaudio.PCM(device=("mono" + str(channel)))
    play_alsa(device, wav_file)
    wav_file.close()

def play_alsa(device, f):
    # Set attributes
    device.setchannels(f.getnchannels())
    device.setrate(f.getframerate())
    # 8bit is unsigned in wav files
    if f.getsampwidth() == 1:
        device.setformat(alsaaudio.PCM_FORMAT_U8)
    # Otherwise we assume signed data, little endian
    elif f.getsampwidth() == 2:
        device.setformat(alsaaudio.PCM_FORMAT_S16_LE)
    elif f.getsampwidth() == 3:
        device.setformat(alsaaudio.PCM_FORMAT_S24_LE)
    elif f.getsampwidth() == 4:
        device.setformat(alsaaudio.PCM_FORMAT_S32_LE)
    else:
        raise ValueError('Unsupported format')
    device.setperiodsize(320)
    data = f.readframes(320)
    while data:
        # Read data from stdin
        device.write(data)
        data = f.readframes(320)

if __name__ == '__main__':
    AudioPlayer('').play()
